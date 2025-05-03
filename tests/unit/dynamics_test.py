import pytest
import numpy as np
from PyQt5 import QtWidgets
from robot.analytic import Plot_velo_accel
from robot.kinematics import Compute_kine_traj
from robot.misc import constants


@pytest.fixture(scope="session")
def qt_app():
    """Session-wide Qt application fixture"""
    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication([])
    yield app
    # Cleanup after all tests complete
    app.quit()


def test_torque_calculation(qt_app):  # Add qt_app fixture parameter
    """Verify torque calculation produces physical values"""
    # Initialize Qt application context
    qt_app.processEvents()

    # Create plotter within Qt context
    plotter = Plot_velo_accel(1.0, 0.5, 0.15, 0.0, 0.2, 0.1, 0.1)

    # Get sample trajectory data
    (d, v_1, v_2, v_3,
     h_1, h_2, h_3,
     l_11, l_21, l_31,
     l_12, l_22, l_32,
     vec_elbow, vec_shoulder,
     l_arm_proth, l_humerus,
     m_1, m_2, m_3, m_d) = constants()

    # Perform calculation
    results = plotter.compute_inv_dynamics(
        h_1, h_2, h_3, l_11, l_21, l_31,
        m_1, m_2, m_3, m_d, l_arm_proth
    )

    # Validate results
    torques = results[7:]
    for tau in torques:
        assert np.all(np.isfinite(tau)), "Non-finite torque values detected"
        assert np.all(np.abs(tau) < 30), f"Torque exceeds 30Nm limit: {max(tau)}"

    # Cleanup Qt objects
    del plotter
    qt_app.processEvents()