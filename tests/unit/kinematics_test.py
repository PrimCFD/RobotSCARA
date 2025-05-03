import pytest
import numpy as np
from robot.kinematics import inv_kine, Compute_kine_traj, Compute_Workspace
from robot.misc import constants


def test_inv_kine_singularity():
    """Test inverse kinematics at home position"""
    # Known home position coordinates
    x, y, z = 0.15, 0.0, 0.20

    (
        d, v_1, v_2, v_3,
        h_1, h_2, h_3,
        l_11, l_21, l_31,
        l_12, l_22, l_32,
        vec_elbow, vec_shoulder,
        l_arm_proth, l_humerus,
        m_1, m_2, m_3, m_d
    ) = constants()

    y_vec = np.array([0, 1, 0])
    z_vec = np.array([0, 0, 1])

    m_1_point = np.array([0, 0, h_1])
    m_2_point = np.array([0, 0, h_2])
    m_3_point = np.array([0, 0, h_3])

    results = inv_kine(x, y, z, m_1_point, m_2_point, m_3_point, y_vec, z_vec, l_11, l_12, l_21, l_22, l_31, l_32, d, v_1, v_2, v_3, h_1, h_2, h_3,
             1, False)

    # Check all solutions are valid (non-NaN)
    assert not np.isnan(results).any()


def test_workspace_bounds():
    """Verify workspace computation rejects invalid points"""
    grid, nx, ny, nz, voxels = Compute_Workspace(
        (-0.5, 0.5), (-0.5, 0.5), (0, 0.5), 10, 10, 10
    )

    # Check out-of-bounds points are marked invalid
    invalid_point = np.array([0.6, 0.6, 0.6])
    idx = np.argmin(np.linalg.norm(grid - invalid_point, axis=1))
    assert voxels.ravel()[idx] == 0


def test_trajectory_continuity():
    """Verify generated trajectory is smooth and continuous"""
    p_0, phi_arc, theta_arc, p, *_ = Compute_kine_traj(
        0.15, 0.0, 0.2, np.deg2rad(15), np.deg2rad(20), True, True
    )

    # Check position derivatives
    dp = np.diff(p, axis=0)
    assert np.all(np.abs(dp) < 0.1)  # Max step size