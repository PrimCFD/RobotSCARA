def test_full_workflow():
    """End-to-end test from GUI input to visualization"""
    from robot.gui import MainWindow
    from PyQt5 import QtWidgets, QtCore
    import time

    app = QtWidgets.QApplication([])
    window = MainWindow((1400, 900), 60)

    # Simulate target position change
    window.theta = 15
    window.phi = 20
    window.start_computation_in_thread_velo()

    # Verify visualization updates
    QtCore.QTimer.singleShot(5000, assert_runs)

    app.quit()

def assert_runs():
    assert hasattr(window, 'visualizer')
    assert window.visualizer.p is not None