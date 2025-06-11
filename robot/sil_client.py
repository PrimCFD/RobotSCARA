import socket
import struct
from typing import List, Dict
from robot.misc import load_config

def request_sil_simulation(trajectory: List[Dict], DynamicsThread) -> List[Dict]:
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
        config = load_config()
        elbow = config["elbow"]
        l_arm_proth = config["arm"]["l_arm_proth"]
        n = len(trajectory)
        msg = struct.pack('<i', n)
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
        k = struct.unpack('<i', raw_k)[0]

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

        DynamicsThread.status_signal.emit('Computing ideal trajectory dynamics ... ')
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
