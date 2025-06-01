import socket
import struct
from typing import List, Dict


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
        # Build binary message
        n = len(trajectory)
        message = struct.pack('<i', n)
        for wp in trajectory:
            message += struct.pack('<10d',
                                   wp['t'],
                                   *wp['x'],
                                   *wp['x_dot'],
                                   *wp['x_ddot']
                                   )
        sock.sendall(message)

        # Read response header (number of frames)
        header = sock.recv(4)
        if len(header) != 4:
            raise ConnectionError("Incomplete header")
        m = struct.unpack('<i', header)[0]

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
        frames = []
        for i in range(m):
            start = i * 128
            values = struct.unpack_from('<16d', frame_data, start)
            frames.append({
                't': values[0],
                'x': list(values[1:4]),
                'x_dot': list(values[4:7]),
                'theta': list(values[7:10]),
                'theta_dot': list(values[10:13]),
                'tau': list(values[13:16])
            })
        return frames