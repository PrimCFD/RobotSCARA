import numpy as np
import matplotlib.pyplot as plt
from robot.kinematics import Compute_kine_traj
from robot.misc import Spherical_to_cartesian_velocity, Spherical_to_cartesian_accel, Cartesian_to_spherical, angular_dist
from robot.misc import batch_matmul, batch_matvecmul, Cart_velocity_ramp
from PyQt5.QtWidgets import QVBoxLayout, QWidget
from PyQt5.QtCore import pyqtSignal
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar


class Plot_velo_accel(QWidget):
    """
    Widget for visualizing motor velocities, accelerations and torques.

    Uses inverse dynamics calculations to plot:
    - Joint angular velocities
    - Joint angular accelerations
    - Required motor torques

    Args:
        omega_max (float): Maximum angular velocity constraint
        accel (float): Angular acceleration constraint
        x_0, y_0, z_0 (float): Initial end-effector position
        theta, phi (float): Target angles in spherical coordinates
        parent: Parent widget
    """

    closed = pyqtSignal()

    def __init__(self, omega_max, accel, x_0, y_0, z_0, theta, phi, parent=None):

        super().__init__(parent)

        # Create a Matplotlib figure
        self.figure, self.axs = plt.subplots(3, 3, figsize=(12, 6), sharex=True)
        self.canvas = FigureCanvas(self.figure)

        # Add the navigation toolbar and the canvas to the layout
        layout = QVBoxLayout(self)
        layout.addWidget(NavigationToolbar(self.canvas, self))
        layout.addWidget(self.canvas)

        self.omega_max = omega_max
        self.accel = accel
        self.x_0 = x_0
        self.y_0 = y_0
        self.z_0 = z_0
        self.theta = theta
        self.phi = phi

    def K(self, x, y, z, theta_1, theta_2, theta_3, d, v_3, h_3, N):
        if N > 1:
            P = np.array([[y * np.sin(theta_1) - x * np.cos(theta_1), 0 * np.ones(N), 0 * np.ones(N)],
                          [0 * np.ones(N), y * np.sin(theta_2) - x * np.cos(theta_2), 0 * np.ones(N)],
                          [0 * np.ones(N), 0 * np.ones(N),
                           (z + d - v_3 - h_3) * np.sin(theta_3) + x * np.cos(theta_3)]])
            return np.einsum('kji', P)
        else:
            return np.array([[y * np.sin(theta_1) - x * np.cos(theta_1), 0, 0],
                             [0, y * np.sin(theta_2) - x * np.cos(theta_2), 0],
                             [0, 0, (z + d - v_3 - h_3) * np.sin(theta_3) + x * np.cos(theta_3)]])

    def J(self, x, y, z, theta_1, theta_2, theta_3, d, v_1, v_2, v_3, h_1, h_2, h_3, l_11, l_21, l_31, N):
        if N > 1:
            P = np.array([[x + l_11 * np.sin(theta_1), y + l_11 * np.cos(theta_1), z + d - v_1 - h_1],
                          [x + l_21 * np.sin(theta_2), y + l_21 * np.cos(theta_2), z + d - v_2 - h_2],
                          [x + l_31 * np.sin(theta_3), y, z + d - v_3 - h_3 + l_31 * np.cos(theta_3)]])
            return np.einsum('kji', P)
        else:
            return np.array([[x + l_11 * np.sin(theta_1), y + l_11 * np.cos(theta_1), z + d - v_1 - h_1],
                             [x + l_21 * np.sin(theta_2), y + l_21 * np.cos(theta_2), z + d - v_2 - h_2],
                             [x + l_31 * np.sin(theta_3), y, z + d - v_3 - h_3 + l_31 * np.cos(theta_3)]])

    def K_dot(self, x, y, z, xp, yp, zp, theta_1, theta_2, theta_3, theta_1p, theta_2p, theta_3p, d, v_3, h_3, N):
        if N > 1:
            P = np.array([[yp * np.sin(theta_1) - xp * np.cos(theta_1) + theta_1p * (
                    y * np.cos(theta_1) + x * np.sin(theta_1)), 0 * np.ones(N), 0 * np.ones(N)],
                          [0 * np.ones(N), yp * np.sin(theta_2) - xp * np.cos(theta_2) + theta_2p * (
                                  y * np.cos(theta_2) + x * np.sin(theta_2)), 0 * np.ones(N)],
                          [0 * np.ones(N), 0 * np.ones(N), zp * np.sin(theta_3) + xp * np.cos(theta_3) + theta_3p * (
                                  (z + d - v_3 - h_3) * np.cos(theta_3) - x * np.sin(theta_3))]])
            return np.einsum('kji', P)
        else:
            return np.array([[yp * np.sin(theta_1) - xp * np.cos(theta_1) + theta_1p * (
                    y * np.cos(theta_1) + x * np.sin(theta_1)), 0, 0],
                             [0, yp * np.sin(theta_2) - xp * np.cos(theta_2) + theta_2p * (
                                     y * np.cos(theta_2) + x * np.sin(theta_2)), 0],
                             [0, 0, zp * np.sin(theta_3) + xp * np.cos(theta_3) + theta_3p * (
                                     (z + d - v_3 - h_3) * np.cos(theta_3) - x * np.sin(theta_3))]])

    def J_dot(self, xp, yp, zp, theta_1, theta_2, theta_3, theta_1p, theta_2p, theta_3p, l_11, l_21, l_31, N):
        if N > 1:

            P = np.array([[xp + l_11 * theta_1p * np.cos(theta_1), yp - l_11 * theta_1p * np.cos(theta_1), zp],
                          [xp + l_21 * theta_2p * np.cos(theta_2), yp - l_21 * theta_2p * np.cos(theta_2), zp],
                          [xp + l_31 * theta_3p * np.cos(theta_3), yp, zp - l_31 * theta_3p * np.sin(theta_3)]])
            return np.einsum('kji', P)
        else:
            return np.array([[xp + l_11 * theta_1p * np.cos(theta_1), yp - l_11 * theta_1p * np.cos(theta_1), zp],
                             [xp + l_21 * theta_2p * np.cos(theta_2), yp - l_21 * theta_2p * np.cos(theta_2), zp],
                             [xp + l_31 * theta_3p * np.cos(theta_3), yp, zp - l_31 * theta_3p * np.sin(theta_3)]])

    def Compute_motor_speed(self, p, p_dot, theta_1, theta_2, theta_3, d, v_1, v_2, v_3, h_1, h_2, h_3, l_11, l_21,
                            l_31):
        N_points = len(p.tolist())

        if N_points > 1:
            x = p[:, 0]
            y = p[:, 1]
            z = p[:, 2]

        else:
            x = p[0]
            y = p[1]
            z = p[2]

        K_temp = self.K(x, y, z, theta_1, theta_2, theta_3, d, v_3, h_3, N_points)
        eps = 1e-8
        diag_vals = np.einsum('nii->ni', K_temp)  # shape (N, 3) – extrait les diagonales
        safe_vals = np.where(np.abs(diag_vals) < eps, eps, diag_vals)
        inv_diag = 1 / safe_vals
        K_inv_batch = np.zeros_like(K_temp)
        np.einsum('ni, nij -> nij', inv_diag, np.eye(K_temp.shape[1])[None, :, :], out=K_inv_batch)

        Jacobian_prod = batch_matmul(K_inv_batch,
                                     self.J(x, y, z, theta_1, theta_2, theta_3, d, v_1, v_2, v_3, h_1, h_2, h_3, l_11,
                                            l_21,
                                            l_31,
                                            N_points))
        theta_p = batch_matvecmul(Jacobian_prod, p_dot.T)

        if N_points > 1:

            return theta_p[:, 0], theta_p[:, 1], theta_p[:, 2]

        else:

            return theta_p[0], theta_p[1], theta_p[2]

    def Compute_motor_accel(self, p, p_dot, p_dotdot, theta_1, theta_2, theta_3, theta_1p, theta_2p, theta_3p, d, v_1,
                            v_2, v_3,
                            h_1, h_2, h_3, l_11, l_21, l_31):
        p_dot = p_dot.T
        N_points = len(p_dot.tolist())

        if N_points > 1:
            x = p[:, 0]
            y = p[:, 1]
            z = p[:, 2]
            xp = p_dot[:, 0]
            yp = p_dot[:, 1]
            zp = p_dot[:, 2]

        else:
            x = p[0]
            y = p[1]
            z = p[2]
            xp = p_dot[0]
            yp = p_dot[1]
            zp = p_dot[2]

        theta_p_temp = np.array([theta_1p, theta_2p, theta_3p])
        theta_p = theta_p_temp.T

        p_dotdot = p_dotdot.T
        J_1 = batch_matvecmul(
            self.J(x, y, z, theta_1, theta_2, theta_3, d, v_1, v_2, v_3, h_1, h_2, h_3, l_11, l_21, l_31, N_points),
            p_dotdot)
        J_2 = batch_matvecmul(
            self.J_dot(xp, yp, zp, theta_1, theta_2, theta_3, theta_1p, theta_2p, theta_3p, l_11, l_21, l_31, N_points),
            p_dotdot)
        J_3 = batch_matvecmul(
            self.K_dot(x, y, z, xp, yp, zp, theta_1, theta_2, theta_3, theta_1p, theta_2p, theta_3p, d, v_3, h_3,
                       N_points),
            theta_p)
        J_sum = J_1 + J_2 + J_3

        K_temp = self.K(x, y, z, theta_1, theta_2, theta_3, d, v_3, h_3, N_points)
        eps = 1e-8
        diag_vals = np.einsum('nii->ni', K_temp)  # shape (N, 3) – extrait les diagonales
        safe_vals = np.where(np.abs(diag_vals) < eps, eps, diag_vals)
        inv_diag = 1 / safe_vals
        K_inv_batch = np.zeros_like(K_temp)
        np.einsum('ni, nij -> nij', inv_diag, np.eye(K_temp.shape[1])[None, :, :], out=K_inv_batch)

        theta_pp = batch_matvecmul(K_inv_batch, J_sum)

        if N_points > 1:

            return theta_pp[:, 0], theta_pp[:, 1], theta_pp[:, 2]

        else:

            return theta_pp[0], theta_pp[1], theta_pp[2]

    def compute_torque(self, x, y, z, theta_1, theta_2, theta_3, d, v_1, v_2, v_3, h_1, h_2, h_3, l_11, l_21, l_31,
                       N_points, m_11, m_21, m_31, m_12, m_22, m_32, m_d, x_p, y_p, z_p, theta_1p, theta_2p, theta_3p,
                       x_pp, y_pp, z_pp, theta_1pp, theta_2pp, theta_3pp,
                       l_arm_proth):

        I_b_11 = 1 / 3 * m_11 * l_11 ** 2
        I_b_21 = 1 / 3 * m_21 * l_21 ** 2
        I_b_31 = 1 / 3 * m_31 * l_31 ** 2
        I_b_12 = 1 / 3 * m_12 * l_11 ** 2
        I_b_22 = 1 / 3 * m_22 * l_21 ** 2
        I_b_32 = 1 / 3 * m_32 * l_31 ** 2

        I_arm = 1 / 3 * m_d * l_arm_proth ** 2

        tau_e_1 = (I_b_11 + I_b_12) * theta_1pp + m_12 / 6 * l_11 * (
                y_p * np.sin(theta_1) - x_p * np.cos(theta_1)) * theta_1p
        tau_e_2 = (I_b_21 + I_b_22) * theta_2pp + m_22 / 6 * l_21 * (
                y_p * np.sin(theta_2) - x_p * np.cos(theta_2)) * theta_2p
        tau_e_3 = (I_b_31 + I_b_32) * theta_3pp + m_32 / 6 * l_31 * (
                z_p * np.sin(theta_3) + x_p * np.cos(theta_3)) * theta_3p + m_31 * 9.81 * l_31 * np.sin(theta_3)

        f_e_1 = (m_d + I_arm / l_arm_proth ** 2 + m_12 / 3) * x_pp + m_12 * l_11 * theta_1p ** 2 / 6 * (
            - np.cos(theta_1))
        f_e_2 = (m_d + I_arm / l_arm_proth ** 2 + m_22 / 3) * y_pp + m_22 * l_21 * theta_2p ** 2 / 6 * (
            - np.sin(theta_2))
        f_e_3 = (m_d + I_arm / l_arm_proth ** 2 + m_32 / 3) * z_pp + m_32 * l_31 * theta_3p ** 2 / 6 * (
            - np.sin(theta_3)) + 9.81 * ((m_12 + m_22 + m_32) + m_d) * 1 / 2

        J_temp = self.J(x, y, z, theta_1, theta_2, theta_3, d, v_1, v_2, v_3, h_1, h_2, h_3, l_11, l_21, l_31, N_points)

        # choose a small damping factor
        lam = 1e-3

        # build a batch of 3×3 identity matrices
        I3 = np.eye(3)
        I_batch = np.broadcast_to(I3, J_temp.shape)

        # compute J·Jᵀ + λ²I for each batch entry
        JJT = J_temp @ np.swapaxes(J_temp, 1, 2)  # shape (N,3,3)
        JJT_damped = JJT + (lam ** 2) * I_batch

        # invert that (always well‑conditioned thanks to λ)
        inv_JJT = np.linalg.inv(JJT_damped)  # shape (N,3,3)

        # form the damped pseudoinverse: J⁺ = Jᵀ · (J·Jᵀ + λ²I)⁻¹
        J_inv = np.swapaxes(J_temp, 1, 2) @ inv_JJT  # shape (N,3,3)

        K_temp = self.K(x, y, z, theta_1, theta_2, theta_3, d, v_3, h_3, N_points)
        Jacob = batch_matmul(J_inv, K_temp)
        Jacob_transposed = np.transpose(Jacob, (0, 2, 1))

        f_e = np.array([f_e_1, f_e_2, f_e_3])
        tau_e = np.array([tau_e_1, tau_e_2, tau_e_3])
        tau = tau_e.T + batch_matvecmul(Jacob_transposed, f_e.T)
        tau_1, tau_2, tau_3 = tau[:, 0], tau[:, 1], tau[:, 2]

        return tau_1, tau_2, tau_3

    def compute_inv_dynamics(self, h_1, h_2, h_3, l_11, l_21, l_31, m_1, m_2, m_3, m_d, l_arm_proth):
        p_0, phi_arc, theta_arc, p, vec_elbow, vec_shoulder, \
        theta_1bis, theta_1, theta_2bis, theta_2, theta_3bis, theta_3, d, v_1, v_2, v_3, m_1_point, m_2_point, m_3_point, z_vec, N_per_decideg = Compute_kine_traj(
            self.x_0, self.y_0, self.z_0, self.theta, self.phi, False, True)
        # /!\ m_1, m_2, m_3 from Compute_kine_traj = Points/vectors not masses/!\

        p_dot, p_dotdot, t = Cart_velocity_ramp(l_arm_proth, self.theta, self.phi, theta_arc, phi_arc,
                                                self.omega_max,
                                                self.accel, p)

        theta_1p, theta_2p, theta_3p = self.Compute_motor_speed(p, p_dot, theta_1, theta_2, theta_3, d, v_1, v_2, v_3,
                                                                h_1, h_2,
                                                                h_3,
                                                                l_11, l_21, l_31)

        theta_1pp, theta_2pp, theta_3pp = self.Compute_motor_accel(p, p_dot, p_dotdot, theta_1, theta_2, theta_3,
                                                                   theta_1p,
                                                                   theta_2p,
                                                                   theta_3p, d, v_1, v_2, v_3,
                                                                   h_1, h_2, h_3, l_11, l_21, l_31)

        N_points = len(p.tolist())

        if N_points > 1:
            x = p[:, 0]
            y = p[:, 1]
            z = p[:, 2]
            x_p = p_dot.T[:, 0]
            y_p = p_dot.T[:, 1]
            z_p = p_dot.T[:, 2]
            x_pp = p_dotdot.T[:, 0]
            y_pp = p_dotdot.T[:, 1]
            z_pp = p_dotdot.T[:, 2]

        else:
            x = p[0]
            y = p[1]
            z = p[2]
            x_p = p_dot.T[0]
            y_p = p_dot.T[1]
            z_p = p_dot.T[2]
            x_pp = p_dotdot.T[0]
            y_pp = p_dotdot.T[1]
            z_pp = p_dotdot.T[2]

        tau_1, tau_2, tau_3 = self.compute_torque(x, y, z, theta_1, theta_2, theta_3, d, v_1, v_2, v_3, h_1, h_2, h_3,
                                                  l_11, l_21, l_31,
                                                  N_points, m_1, m_2, m_3, m_1, m_2, m_3, m_d, x_p, y_p, z_p, theta_1p,
                                                  theta_2p, theta_3p,
                                                  x_pp, y_pp, z_pp, theta_1pp, theta_2pp, theta_3pp,
                                                  l_arm_proth)

        return theta_1p, theta_2p, theta_3p, theta_1pp, theta_2pp, theta_3pp, t, tau_1, tau_2, tau_3

    def plot_inv_dynamics(self, theta_1p, theta_2p, theta_3p, theta_1pp, theta_2pp, theta_3pp, t, tau_1, tau_2, tau_3):

        # Colors: one per joint
        joint_colors = ['#1f77b4', '#2ca02c', '#d62728']  # Blue, Green, Red

        # Reorganize data: [velocities, accelerations, torques]
        data = [
            [np.degrees(theta_1p), np.degrees(theta_2p), np.degrees(theta_3p)],  # Column 1: velocities
            [np.degrees(theta_1pp), np.degrees(theta_2pp), np.degrees(theta_3pp)],  # Column 2: accelerations
            [tau_1, tau_2, tau_3]  # Column 3: torques
        ]

        labels = [
            [r'$\dot{\theta}_1$', r'$\dot{\theta}_2$', r'$\dot{\theta}_3$'],
            [r'$\ddot{\theta}_1$', r'$\ddot{\theta}_2$', r'$\ddot{\theta}_3$'],
            [r'$\tau_1$', r'$\tau_2$', r'$\tau_3$']
        ]

        units = ['[deg/s]', '[deg/s²]', '[Nm]']  # One per column

        for col in range(3):  # 0: vel, 1: acc, 2: torque
            for row in range(3):  # 0: joint 1, 1: joint 2, 2: joint 3

                # Plotting the data
                ax = self.axs[row, col]
                ax.plot(t, data[col][row], color=joint_colors[row], linewidth=2)
                ax.set_title(labels[col][row], fontsize=12)

                if row == 2:  # Set xlabel only for the last row
                    ax.set_xlabel('Time [s]')

                ax.set_ylabel(units[col])
                ax.grid(True)

        self.figure.tight_layout()
        self.canvas.draw()  # Assuming you have a Matplotlib canvas to update the plot

    def closeEvent(self, event):
        self.closed.emit()  # Emit when window is closing
        super().closeEvent(event)
