class HilClient:
    def __init__(self, method="socket"):
        self.sock = socket.create_connection(("localhost", 5555))

    def get_torque(self, state_dict):
        self.sock.send(json.dumps(state_dict).encode())
        return json.loads(self.sock.recv(512))