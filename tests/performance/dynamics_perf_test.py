# tests/performance/test_dynamics.py
import pytest
from PyQt5 import QtWidgets, QtCore
from robot.analytic import Plot_velo_accel
from robot.kinematics import Compute_kine_traj
from robot.misc import constants
import numpy as np

@pytest.fixture(scope="session")
def qt_app():
    """Session-wide Qt application fixture"""
    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication([])
        yield app
        app.quit()
    else:
        yield app

@pytest.mark.benchmark(group="dynamics-calculation")
def test_real_time_performance(benchmark, qt_app):
    """Verify dynamics calculations meet real-time requirements"""
    # 1. Setup trajectory
    theta = np.deg2rad(90)
    phi = np.deg2rad(180)

    # 2. Generate trajectory
    try:
        result = Compute_kine_traj(0.15, 0.0, 0.2, theta, phi, True, True)
        p_0, phi_arc, theta_arc, p, *_ = result
        num_points = len(p)
        print(f"\nGenerated {num_points} trajectory points")
        assert num_points > 10
    except Exception as e:
        pytest.fail(f"Trajectory setup failed: {str(e)}")

    # 3. Initialize plotter safely
    plotter = None
    try:
        plotter = Plot_velo_accel(1.0, 2.0, 0.15, 0.0, 0.2, 0.5, 1.0)
        plotter.setAttribute(QtCore.Qt.WA_DontShowOnScreen)  # Prevent window flashing
    except Exception as e:
        pytest.fail(f"Plotter init failed: {str(e)}")

    # 4. Benchmark with proper iteration handling
    def benchmark_wrapper():
        qt_app.processEvents()
        try:
            (d, v_1, v_2, v_3,
             h_1, h_2, h_3,
             l_11, l_21, l_31,
             l_12, l_22, l_32,
             vec_elbow, vec_shoulder,
             l_arm_proth, l_humerus,
             m_1, m_2, m_3, m_d) = constants()
            # Single iteration of the actual computation
            return plotter.compute_inv_dynamics(
                h_1, h_2, h_3,
                l_11, l_21, l_31,
                m_1, m_2, m_3,
                m_d, l_arm_proth
            )
        except Exception as e:
            pytest.fail(f"Computation failed: {str(e)}")
            return None

    # Run benchmark with:
    # - 100 rounds (repeated executions)
    # - 1 iteration per round (to allow setup-free operation)
    # - No explicit setup function
    result = benchmark.pedantic(benchmark_wrapper,
                              rounds=100,  # Increased from 10
                              iterations=1,  # Required when not using setup
                              warmup_rounds=5)

    # 5. Cleanup
    del plotter
    qt_app.processEvents()

    # 6. Performance validation
    assert benchmark.stats['mean'] < 0.015, \
        f"Mean time {benchmark.stats['mean']*1000:.2f}ms exceeds 15ms budget"