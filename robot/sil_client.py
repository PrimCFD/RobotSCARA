import socket
import struct
from typing import List, Dict
from robot.misc import load_config


def request_sil_simulation(trajectory: List[Dict]) -> List[Dict]:
    """
    Sends trajectory via binary protocol and receives simulated results.

    Parameters:
        trajectory: list of {"t": float, "x": [x,y,z], "x_dot": [vx,vy,vz], "x_ddot": [ax,ay,az]}

    Returns:
        list of frames with keys: t, x, x_dot, theta, theta_dot, tau
    """
    host = 'localhost'
    port = 5555

    with socket.create_connection((host, port)) as sock:

        # Steady position for Ziegler Nichols
        # trajectory = [{"t": 0.0, "x": [0.15,0.0,0.2], "x_dot": [0.0,0.0,0.0], "x_ddot": [0.002,0.0,0.0]}, {"t": 2.5, "x": [0.15, 0.0, 0.2], "x_dot": [0.0,0.0,0.0], "x_ddot": [0.0,0.0,0.0]}]

        config = load_config()

        # Extract parameters
        elbow = config["elbow"]
        l_arm_proth = config["arm"]["l_arm_proth"]

        # Build binary message
        n = len(trajectory)
        message = struct.pack('<i', n)
        # Add elbow position and arm parameters
        message += struct.pack('<3d', elbow['x'], elbow['y'], elbow['z'])
        message += struct.pack('<1d', l_arm_proth)

        for wp in trajectory:
            message += struct.pack('<10d',
                                   wp['t'],
                                   *wp['x'],
                                   *wp['x_dot'],
                                   *wp['x_ddot']
                                   )
        sock.sendall(message)

        # Read new header (two 4-byte integers)
        header = sock.recv(8)
        if len(header) != 8:
            raise ConnectionError("Incomplete header")
        m, k = struct.unpack('<ii', header)  # m=sim frames, k=ideal points

        # Read all frame data
        frame_data = b''
        remaining = m * 128  # 16 doubles * 8 bytes
        while remaining > 0:
            chunk = sock.recv(min(4096, remaining))
            if not chunk:
                raise ConnectionError("Incomplete frame data")
            frame_data += chunk
            remaining -= len(chunk)

        # Parse frames
        simulation_frames = []
        for i in range(m):
            start = i * 128
            values = struct.unpack_from('<16d', frame_data, start)
            simulation_frames.append({
                't': values[0],
                'x': list(values[1:4]),
                'x_dot': list(values[4:7]),
                'theta': list(values[7:10]),
                'theta_dot': list(values[10:13]),
                'tau': list(values[13:16])
            })

        # Read ideal data points (now including positions and velocities)
        ideal_data = b''
        remaining = k * 80  # 10 doubles * 8 bytes (t + 3 theta + 3 theta_dot + 3 tau)
        while remaining > 0:
            chunk = sock.recv(min(4096, remaining))
            if not chunk:
                raise ConnectionError("Incomplete ideal data")
            ideal_data += chunk
            remaining -= len(chunk)

        # Parse ideal data points
        ideal_points = []
        for i in range(k):
            start = i * 80
            values = struct.unpack_from('<10d', ideal_data, start)
            ideal_points.append({
                't': values[0],
                'theta': list(values[1:4]),
                'theta_dot': list(values[4:7]),
                'tau_ideal': list(values[7:10])
            })

        return [simulation_frames, ideal_points]