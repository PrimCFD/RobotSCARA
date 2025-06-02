import numpy as np
import vtk
import sys
from robot.misc import Spherical_to_cartesian, Cartesian_to_spherical, Rz, Ry, angular_dist, constants, set_config
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull


def inv_kine(x, y, z, m_1, m_2, m_3, y_vec, z_vec, l_11, l_12, l_21, l_22, l_31, l_32, d, v_1, v_2, v_3, h_1, h_2, h_3,
             N_points, cart):
    p_temp = np.array([x, y, z])  # Position vector
    p = p_temp.T  # Dimension adjustment

    # A_i coefficients
    A_1 = 2 * (y)
    A_2 = 2 * (y)
    A_3 = 2 * (-h_3 + (z + d - v_3))

    # B_i coefficients
    B_1 = 2 * (x)
    B_2 = 2 * (x)
    B_3 = -2 * (x)

    # C_i coefficients

    if N_points == 1:
        C_1 = (l_12 ** 2 - l_11 ** 2 - np.linalg.norm((m_1 - (p + (d - v_1) * z_vec)), axis=0) ** 2) / l_11
        C_2 = (l_22 ** 2 - l_21 ** 2 - np.linalg.norm((m_2 - (p + (d - v_2) * z_vec)), axis=0) ** 2) / l_21
        C_3 = (l_32 ** 2 - l_31 ** 2 - np.linalg.norm((m_3 - (p + (d - v_3) * z_vec)), axis=0) ** 2) / l_31

    else:
        C_1 = (l_12 ** 2 - l_11 ** 2 - np.linalg.norm((m_1 - (p + (d - v_1) * z_vec)), axis=1) ** 2) / l_11
        C_2 = (l_22 ** 2 - l_21 ** 2 - np.linalg.norm((m_2 - (p + (d - v_2) * z_vec)), axis=1) ** 2) / l_21
        C_3 = (l_32 ** 2 - l_31 ** 2 - np.linalg.norm((m_3 - (p + (d - v_3) * z_vec)), axis=1) ** 2) / l_31

    # t_i solutions (2 per equation)
    t_1a = (B_1 + np.sqrt(B_1 ** 2 - (A_1 + C_1) * (C_1 - A_1))) / (A_1 + C_1)
    t_1b = (B_1 - np.sqrt(B_1 ** 2 - (A_1 + C_1) * (C_1 - A_1))) / (A_1 + C_1)
    t_2a = (B_2 + np.sqrt(B_2 ** 2 - (A_2 + C_2) * (C_2 - A_2))) / (A_2 + C_2)
    t_2b = (B_2 - np.sqrt(B_2 ** 2 - (A_2 + C_2) * (C_2 - A_2))) / (A_2 + C_2)
    t_3a = (B_3 + np.sqrt(B_3 ** 2 - (A_3 + C_3) * (C_3 - A_3))) / (A_3 + C_3)
    t_3b = (B_3 - np.sqrt(B_3 ** 2 - (A_3 + C_3) * (C_3 - A_3))) / (A_3 + C_3)

    # Theta_i (2 per equation)
    theta_1a = 2 * np.arctan(t_1a)
    theta_1b = 2 * np.arctan(t_1b)
    theta_2a = 2 * np.arctan(t_2a)
    theta_2b = 2 * np.arctan(t_2b)
    theta_3a = 2 * np.arctan(t_3a)
    theta_3b = 2 * np.arctan(t_3b)

    # Points as vectors

    # S joints

    s_1a = h_1 * z_vec - l_11 * Rz(theta_1a, N_points) @ y_vec
    s_1b = h_1 * z_vec - l_11 * Rz(theta_1b, N_points) @ y_vec
    s_2a = h_2 * z_vec - l_21 * Rz(theta_2a, N_points) @ y_vec
    s_2b = h_2 * z_vec - l_21 * Rz(theta_2b, N_points) @ y_vec
    s_3a = h_3 * z_vec - l_31 * Ry(-theta_3a, N_points) @ z_vec
    s_3b = h_3 * z_vec - l_31 * Ry(-theta_3b, N_points) @ z_vec

    if cart:

        return s_1a, s_1b, s_2a, s_2b, s_3a, s_3b

    else:

        return theta_1a, theta_1b, theta_2a, theta_2b, theta_3a, theta_3b


def arc(x_0, y_0, z_0, theta, phi, kine):
    p_0 = np.array([x_0, y_0, z_0])

    set_config("temp")
    d, v_1, v_2, v_3, h_1, h_2, h_3, l_11, l_21, l_31, l_12, l_22, l_32, vec_elbow, vec_shoulder, l_arm_proth, l_humerus, m_1, m_2, m_3, m_d = constants()

    r_0, theta_0, phi_0 = Cartesian_to_spherical(x_0, y_0, z_0, vec_elbow, 1)

    if kine:
        N_per_decideg = 10
        N_points = int(angular_dist(theta_0, theta_0 + theta, phi_0,
                                    phi_0 + phi) * 180 / np.pi * N_per_decideg * 10)  # 2 points per 0.1 degree of arc

        theta_arc = np.linspace(theta_0, theta_0 + theta, N_points)
        phi_arc = np.linspace(phi_0, phi_0 + phi, N_points)

    else:

        theta_arc = theta
        phi_arc = phi
        N_points = len(theta_arc.tolist())

    x_arc, y_arc, z_arc = Spherical_to_cartesian(r_0, theta_arc, phi_arc, vec_elbow)

    return p_0, x_arc, y_arc, z_arc, N_points, theta_arc, phi_arc


def Compute_kine_traj(x, y, z, theta, phi, cart, kine, arc_traj):
    # Window dimensions

    ### Biomechanics ###

    set_config("temp")
    d, v_1, v_2, v_3, h_1, h_2, h_3, l_11, l_21, l_31, l_12, l_22, l_32, vec_elbow, vec_shoulder, l_arm_proth, l_humerus, m_1, m_2, m_3, m_d = constants()

    if arc_traj:

        p_0, x_arc, y_arc, z_arc, N_points, theta_arc, phi_arc = arc(x, y, z, theta, phi, kine)

    else:

        x_arc, y_arc, z_arc = x, y, z
        N_points = len(x_arc.tolist())
        p_0 = np.array([x_arc[0], y_arc[0], z_arc[0]])
        r, theta_arc, phi_arc = Cartesian_to_spherical(x_arc, y_arc, z_arc, vec_elbow, N_points)

    p_temp = np.array([x_arc, y_arc, z_arc])  # Position vector
    p = p_temp.T  # Dimension adjustment

    y_vec = np.array([0, 1, 0])  # Y vector
    z_vec = np.array([0, 0, 1])  # Z vector

    m_1 = np.array([0, 0, h_1])
    m_2 = np.array([0, 0, h_2])
    m_3 = np.array([0, 0, h_3])

    # Inverse kinematics
    s_1a, s_1b, s_2a, s_2b, s_3a, s_3b \
        = inv_kine(x_arc, y_arc, z_arc, m_1, m_2, m_3, y_vec, z_vec,
                   l_11, l_12, l_21, l_22, l_31, l_32, d, v_1, v_2, v_3, h_1, h_2, h_3, N_points, cart)


    return p_0, phi_arc, theta_arc, p, vec_elbow, vec_shoulder, \
           s_1a, s_1b, s_2a, s_2b, s_3a, s_3b, d, v_1, v_2, v_3, m_1, m_2, m_3, z_vec, N_points


def Compute_kine_point(x, y, z, cart, N_points):
    ### Inverse Kinematics and Trajectory ###

    y_vec = np.array([0, 1, 0])  # Y vector
    z_vec = np.array([0, 0, 1])  # Z vector

    set_config("temp")
    d, v_1, v_2, v_3, h_1, h_2, h_3, l_11, l_21, l_31, l_12, l_22, l_32, vec_elbow, vec_shoulder, l_arm_proth, l_humerus, m_1, m_2, m_3, m_d = constants()

    m_1 = np.array([0, 0, h_1])
    m_2 = np.array([0, 0, h_2])
    m_3 = np.array([0, 0, h_3])

    theta_1a, theta_1b, theta_2a, theta_2b, theta_3a, theta_3b \
        = inv_kine(x, y, z, m_1, m_2, m_3, y_vec, z_vec, l_11, l_12, l_21, l_22,
                   l_31, l_32, d, v_1, v_2, v_3, h_1, h_2, h_3, N_points, cart)

    return theta_1a, theta_1b, theta_2a, theta_2b, theta_3a, theta_3b


def Compute_Workspace(x_limits, y_limits, z_limits, nx, ny, nz):
    xs = np.linspace(x_limits[0], x_limits[1], nx)
    ys = np.linspace(y_limits[0], y_limits[1], ny)
    zs = np.linspace(z_limits[0], z_limits[1], nz)

    grid = np.array(np.meshgrid(xs, ys, zs, indexing='ij')).reshape(3, -1).T  # shape (N, 3)
    N_points = nx * ny * nz

    with np.errstate(invalid='ignore', divide='ignore', over='ignore'):
        # Compute all joint solutions
        theta_1a, theta_1b, theta_2a, theta_2b, theta_3a, theta_3b = Compute_kine_point(
            grid[:, 0], grid[:, 1], grid[:, 2], False, N_points
        )

        all_thetas = np.stack([theta_1a, theta_1b, theta_2a, theta_2b, theta_3a, theta_3b], axis=1)
        valid_mask = ~np.isnan(all_thetas).any(axis=1)

        # Define bounds per joint solution
        theta1a_mask = (theta_1a >= -np.pi) & (theta_1a <= 0)
        theta1b_mask = (theta_1b >= -np.pi) & (theta_1b <= 0)

        theta2a_mask = (theta_2a >= -np.pi) & (theta_2a <= 0)
        theta2b_mask = (theta_2b >= -np.pi) & (theta_2b <= 0)

        theta3a_mask = (theta_3a >= -np.pi) & (theta_3a <= 0)
        theta3b_mask = (theta_3b >= 0) & (theta_3b <= np.pi)

        # Combine all angle constraints
        angle_mask = theta1a_mask & theta1b_mask & theta2a_mask & theta2b_mask & theta3a_mask & theta3b_mask
        valid_mask &= angle_mask

        # Voxels
        voxels_full = np.zeros((nx, ny, nz), dtype=np.uint8)
        flat_indices = np.argwhere(valid_mask).flatten()

        # Vectorized mapping of flat indices to 3D indices
        indices_3d = np.unravel_index(flat_indices, (nx, ny, nz))

        # Update the voxels array using the 3D indices
        voxels_full[indices_3d] = 1

    return grid, nx, ny, nz, voxels_full


def Compute_Workspace_Sphere(theta_limits, phi_limits, n_theta, n_phi, r, vec_elbow):
    thetas = np.linspace(theta_limits[0], theta_limits[1], n_theta)
    phis = np.linspace(phi_limits[0], phi_limits[1], n_phi)

    theta_grid, phi_grid = np.meshgrid(thetas, phis, indexing='ij')
    N_points = n_theta * n_phi

    theta_flat = theta_grid.ravel()
    phi_flat = phi_grid.ravel()

    with np.errstate(invalid='ignore', divide='ignore', over='ignore'):
        # Convert spherical coordinates to cartesian coordinates
        x, y, z = Spherical_to_cartesian(r, theta_flat, phi_flat, vec_elbow)  # shape (N, 3)
        xyz = np.array([x, y, z]).T
        # Get the inverse kinematics solutions (unpacking the six solutions)
        theta_1a, theta_1b, theta_2a, theta_2b, theta_3a, theta_3b = Compute_kine_point(xyz[:, 0], xyz[:, 1], xyz[:, 2],
                                                                                        False, N_points)

        # Stack all solutions into a single array for easy processing
        all_thetas = np.stack([theta_1a, theta_1b, theta_2a, theta_2b, theta_3a, theta_3b], axis=1)

        # Find valid solutions by checking for NaN values across all the theta arrays
        valid_mask = ~np.isnan(all_thetas).any(axis=1)

        # Filter out invalid points based on the mask
        reachable_angles = np.stack((theta_flat, phi_flat), axis=-1)[valid_mask]
        reachable_cart = xyz[valid_mask]

    return reachable_angles, reachable_cart, xyz


def Compute_trajectory_from_data(r, theta_arc, phi_arc, vec_elbow):
    x, y, z = Spherical_to_cartesian(r, theta_arc, phi_arc, vec_elbow)
    x_arr = np.array(x)
    y_arr = np.array(y)
    z_arr = np.array(z)
    p = np.array([x_arr, y_arr, z_arr])
    p_0 = np.array([x_arr[0], y_arr[0], z_arr[0]])
    return p, p_0
