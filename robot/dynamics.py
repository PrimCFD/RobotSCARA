import numpy as np
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import QVBoxLayout, QWidget
from PyQt5.QtCore import pyqtSignal
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar

class Plot_velo_accel(QWidget):
    closed = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        # Create a 3x3 grid of subplots
        self.figure, self.axs = plt.subplots(3, 3, figsize=(15, 10), sharex=True)
        self.canvas = FigureCanvas(self.figure)

        layout = QVBoxLayout(self)
        layout.addWidget(NavigationToolbar(self.canvas, self))
        layout.addWidget(self.canvas)

    def plot_comparisons(self, t,
                         theta_sim, theta_dot_sim, tau_sim,
                         theta_ideal, theta_dot_ideal, tau_ideal):
        """
        Plot comparisons for positions, velocities, and torques

        Args:
            t: Time vector (N,)
            theta_sim: Simulated positions (N, 3)
            theta_dot_sim: Simulated velocities (N, 3)
            tau_sim: Simulated torques (N, 3)
            theta_ideal: Ideal positions (N, 3)
            theta_dot_ideal: Ideal velocities (N, 3)
            tau_ideal: Ideal torques (N, 3)
        """
        # Clear previous plots
        for ax_row in self.axs:
            for ax in ax_row:
                ax.clear()

        joint_names = ['Joint 1', 'Joint 2', 'Joint 3']
        plot_types = ['Position [rad]', 'Velocity [rad/s]', 'Torque [Nm]']

        for i in range(3):  # For each joint
            # Position plot
            ax = self.axs[i, 0]
            ax.plot(t, theta_sim[:, i], 'b-', linewidth=2, label='Simulated')
            ax.plot(t, theta_ideal[:, i], 'r--', linewidth=2, label='Ideal')
            ax.set_title(f'{joint_names[i]} Position')
            ax.set_ylabel(plot_types[0])
            ax.grid(True)
            ax.legend()

            # Velocity plot
            ax = self.axs[i, 1]
            ax.plot(t, theta_dot_sim[:, i], 'b-', linewidth=2, label='Simulated')
            ax.plot(t, theta_dot_ideal[:, i], 'r--', linewidth=2, label='Ideal')
            ax.set_title(f'{joint_names[i]} Velocity')
            ax.set_ylabel(plot_types[1])
            ax.grid(True)
            ax.legend()

            # Torque plot
            ax = self.axs[i, 2]
            ax.plot(t, tau_sim[:, i], 'b-', linewidth=2, label='Simulated')
            ax.plot(t, tau_ideal[:, i], 'r--', linewidth=2, label='Ideal')
            ax.set_title(f'{joint_names[i]} Torque')
            ax.set_ylabel(plot_types[2])
            ax.grid(True)
            ax.legend()

            if i == 2:
                for j in range(3):
                    self.axs[i, j].set_xlabel('Time [s]')

        self.figure.tight_layout()
        self.canvas.draw()