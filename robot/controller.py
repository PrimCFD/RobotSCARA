class Controller:
    def __init__(self, mode="SIL"):
        self.backend = HilClient() if mode == "HIL" else HilClient()  # both use C++ core

    def compute_torque(self, x, x_dot, x_d, x_dot_d):
        return self.backend.get_torque({...})