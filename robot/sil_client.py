import socket
import json
from typing import List, Dict


def request_sil_simulation(trajectory: List[Dict]) -> List[Dict]:
    """
    Sends a full trajectory to the SIL server and receives back the simulated results.

    Parameters:
        trajectory: list of {"t": float, "x": [x, y, z]} waypoints

    Returns:
        list of {"t": float, "x": [x, y, z], "x_dot": [...], "theta": [...], "theta_dot": [...],"theta_ddot": [...], "tau": [...]} frames
    """
    host = 'localhost'
    port = 5555

    with socket.create_connection((host, port)) as sock:
        # Prepare request message
        message = json.dumps({"trajectory": trajectory}) + "\n"
        sock.sendall(message.encode("utf-8"))

        # Read entire response until newline
        response = b""
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                raise ConnectionError("Disconnected before receiving complete response.")
            response += chunk
            if b"\n" in response:
                response, _, _ = response.partition(b"\n")
                break

        # Decode JSON
        try:
            results = json.loads(response.decode())
            assert isinstance(results, list), "Server response must be a list"
            for frame in results:
                assert all(key in frame for key in ("t", "x", "x_dot", "theta", "theta_dot", "tau")), "Missing keys in frame"
            return results
        except json.JSONDecodeError as e:
            raise ValueError(f"Invalid JSON received: {e}")
