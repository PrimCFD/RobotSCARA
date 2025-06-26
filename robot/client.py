import socket
import struct
from typing import List, Dict
from robot.misc import load_config

def _build_header(mode: str,
                  sensor_dev: str = '',
                  arduino_dev: str = '') -> bytes:

    """
    Build the variable-length handshake understood by the unified SIL / HIL
    server.  Layout (little-endian):

        [0] uint8  mode            – ord('S') or ord('H')
        [1] uint8  len(sensor)     – 0-255
        [2] char[] sensor_dev
        [?] uint8  len(arduino)    – 0-255
        [?] char[] arduino_dev
    """
    if mode not in ('S', 'H'):
        raise ValueError("mode must be 'S' or 'H'")

    if len(sensor_dev) > 255 or len(arduino_dev) > 255:
        raise ValueError("Device-path strings must be ≤ 255 bytes")

    buf = bytearray([ord(mode)])

      # Variable-length device paths are **only** needed in HIL mode

    if mode == 'H':
            buf.append(len(sensor_dev))
            buf.extend(sensor_dev.encode())
            buf.append(len(arduino_dev))
            buf.extend(arduino_dev.encode())

    return bytes(buf)

def request_sil_simulation(trajectory: List[Dict],
                           DynamicsThread) -> List[List[Dict]]:
    """
    Sends trajectory via binary protocol and receives simulated results,
    emitting progress (0–100%) separately for simulation then ideal data.
    """
    host = 'localhost'
    port = 5555

    def recv_exact(sock: socket.socket, size: int) -> bytes:
        """
        Read exactly `size` bytes from the socket or raise ConnectionError.
        """
        data = bytearray()
        while len(data) < size:
            chunk = sock.recv(size - len(data))
            if not chunk:
                raise ConnectionError(f"Expected {size} bytes, got {len(data)} bytes before EOF")
            data.extend(chunk)
        return bytes(data)


    with socket.create_connection((host, port)) as sock:
        # Load configuration and pack trajectory message
        sock.sendall(_build_header('S'))
        config = load_config()
        elbow = config["elbow"]
        l_arm_proth = config["arm"]["l_arm_proth"]
        n = len(trajectory)
        msg = struct.pack('@i', n)
        msg += struct.pack('<3d', elbow['x'], elbow['y'], elbow['z'])
        msg += struct.pack('<d', l_arm_proth)
        for wp in trajectory:
            msg += struct.pack(
                '<10d',
                wp['t'],
                *wp['x'],
                *wp['x_dot'],
                *wp['x_ddot']
            )
        sock.sendall(msg)

        # Read header: first an 8-byte double end_time, then a 4-byte int waypoint count
        raw_end = recv_exact(sock, 8)
        end_time = struct.unpack('<d', raw_end)[0]

        raw_k = recv_exact(sock, 4)
        k = struct.unpack('@i', raw_k)[0]

        # -- Phase 1: Read simulation frames until timestamp >= end_time --
        FRAME_BYTES = 16 * 8  # 16 doubles per frame
        simulation_frames: List[Dict] = []
        DynamicsThread.status_signal.emit('Computing simulation ... ')
        DynamicsThread.progress_signal.emit(0)

        while True:
            frame_chunk = recv_exact(sock, FRAME_BYTES)
            vals = struct.unpack('<16d', frame_chunk)
            frame = {
                't': vals[0],
                'x': list(vals[1:4]),
                'x_dot': list(vals[4:7]),
                'theta': list(vals[7:10]),
                'theta_dot': list(vals[10:13]),
                'tau': list(vals[13:16])
            }
            simulation_frames.append(frame)

            pct_sim = int(frame['t'] / end_time * 100)
            DynamicsThread.progress_signal.emit(min(pct_sim, 100))

            if frame['t'] >= end_time:
                simulation_frames.pop()
                break

        # -- Phase 2: Read ideal data --
        IDEAL_BYTES = k * 10 * 8  # 10 doubles per waypoint
        ideal_data = bytearray()

        DynamicsThread.status_signal.emit('Computing ideal trajectory ... ')
        DynamicsThread.progress_signal.emit(0)

        while len(ideal_data) < IDEAL_BYTES:
            chunk = sock.recv(min(4096, IDEAL_BYTES - len(ideal_data)))
            if not chunk:
                raise ConnectionError("Incomplete ideal data")
            ideal_data.extend(chunk)
            pct_ideal = int(len(ideal_data) / IDEAL_BYTES * 100)
            DynamicsThread.progress_signal.emit(min(pct_ideal, 100))

        ideal_points: List[Dict] = []
        for i in range(k):
            offset = i * 10 * 8
            vals = struct.unpack_from('<10d', ideal_data, offset)
            ideal_points.append({
                't': vals[0],
                'theta': list(vals[1:4]),
                'theta_dot': list(vals[4:7]),
                'tau_ideal': list(vals[7:10])
            })

        DynamicsThread.progress_signal.emit(100)
        return [simulation_frames, ideal_points]

def request_hil(trajectory: List[Dict],
                           sensor_dev: str,
                           arduino_dev: str,
                           DynamicsThread) -> List[List[Dict]]:

    host = 'localhost'
    port = 5555

    def recv_exact(sock: socket.socket, size: int) -> bytes:
        data = bytearray()
        while len(data) < size:
            chunk = sock.recv(size - len(data))
            if not chunk:
                raise ConnectionError(f"EOF after {len(data)} / {size} bytes")
            data.extend(chunk)
        return bytes(data)

    end_time = trajectory[-1]['t']

    with socket.create_connection((host, port)) as sock:
        # --- 1) handshake  -------------------------------------------------
        sock.sendall(_build_header('H', sensor_dev, arduino_dev))

        # --- 2) legacy trajectory payload (unchanged) ----------------------
        config = load_config()
        elbow = config["elbow"]
        l_arm_proth = config["arm"]["l_arm_proth"]
        n = len(trajectory)

        msg = struct.pack('@i', n)
        msg += struct.pack('<3d', elbow['x'], elbow['y'], elbow['z'])
        msg += struct.pack('<d', l_arm_proth)
        for wp in trajectory:
            msg += struct.pack(
                '<10d',
                wp['t'],
                *wp['x'],
                *wp['x_dot'],
                *wp['x_ddot']
            )
        sock.sendall(msg)

        # --- 3) streaming frames (same 16-double struct) -------------------
        FRAME_BYTES = 16 * 8
        frames: List[Dict] = []

        DynamicsThread.status_signal.emit('Running HIL …')
        DynamicsThread.progress_signal.emit(0)

        while True:
            try:
                chunk = recv_exact(sock, FRAME_BYTES)
            except ConnectionError:
                break      # server closed connection

            vals = struct.unpack('<16d', chunk)
            frame = {
                't': vals[0],
                'x':          list(vals[1:4]),
                'x_dot':      list(vals[4:7]),
                'theta':      list(vals[7:10]),
                'theta_dot':  list(vals[10:13]),
                'tau':        list(vals[13:16])
            }
            frames.append(frame)

            DynamicsThread.progress_signal.emit(
                min(int(frame['t'] / end_time * 100), 100))

            if frame['t'] >= end_time:
                break

            # Drain the socket so server sees EOF, avoid EPIPE

        while sock.recv(4096):
            pass

        DynamicsThread.progress_signal.emit(100)
        return [frames, []]          # no “ideal” block for HIL
