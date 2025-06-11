import sys
import traceback
import numpy as np
import pyvista as pv

from robot.kinematics import Compute_kine_traj, Compute_kine_point, Compute_Workspace, Compute_Workspace_Sphere, arc, Compute_trajectory_from_data
from robot.scene import Pyvista3DScene
from robot.dynamics import Plot_velo_accel
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtGui import QDoubleValidator, QIcon
from PyQt5.QtWidgets import QPushButton, QStyle
from robot.misc import constants, Cart_velocity_ramp, Cartesian_to_spherical, set_config, save_config, load_config, parse_simulation_result, Spherical_to_cartesian
from scipy.spatial import ConvexHull, Delaunay
from scipy.interpolate import interp1d
from skimage import measure
from robot.measurement import TrajectoryLogger, SimulationLogger, FileBrowser
from robot.sil_process import SILServerProcess
from robot.sil_client import request_sil_simulation


# Button styles
STYLE_ACTION_NEEDED = """
        QPushButton {
            background-color: #4CAF50;  /* Green */
            color: white;
            border: 2px solid #45a049;
            border-radius: 5px;
            padding: 8px;
            font-weight: bold;
        }
        QPushButton:hover {
            background-color: #45a049;
            border: 2px solid #3d8b40;
        }
        QPushButton:pressed {
            background-color: #3d8b40;
        }
    """

STYLE_CHANGED = """
        QPushButton {
            background-color: #f44336;  /* Red */
            color: white;
            border: 2px solid #d32f2f;
            border-radius: 5px;
            padding: 8px;
            font-weight: bold;
        }
        QPushButton:hover {
            background-color: #d32f2f;
        }
    """

STYLE_NORMAL = """
        QPushButton {
            background-color: #f0f0f0;
            color: #333;
            border: 2px solid #ccc;
            border-radius: 5px;
            padding: 8px;
        }
        QPushButton:hover {
            background-color: #e0e0e0;
        }
    """


class WorkerThreadPlot(QtCore.QThread):
    result_signal = QtCore.pyqtSignal(object, object, object, object, object, object, object, object, object, object)

    error_signal = QtCore.pyqtSignal(str)

    def __init__(self, plot_window):
        super().__init__()
        self.plot_window = plot_window

        set_config("temp")
        self.d, self.v_1, self.v_2, self.v_3, self.h_1, self.h_2, self.h_3, self.l_11, self.l_21, self.l_31, self.l_12, self.l_22, self.l_32, self.vec_elbow, self.vec_shoulder, self.l_arm_proth, self.l_humerus, \
        self.m_1, self.m_2, self.m_3, self.m_d = constants()

    def run(self):
        try:
            theta_1p, theta_2p, theta_3p, theta_1pp, theta_2pp, theta_3pp, t, tau_1, tau_2, tau_3 = self.plot_window.compute_inv_dynamics(
                self.h_1, self.h_2, self.h_3, self.l_11, self.l_21, self.l_31, self.m_1[0], self.m_2[0], self.m_3[0], self.m_1[1], self.m_2[1], self.m_3[1], self.m_d,
                self.l_arm_proth)
            self.result_signal.emit(theta_1p, theta_2p, theta_3p, theta_1pp, theta_2pp, theta_3pp, t, tau_1, tau_2,
                                    tau_3)

        except Exception as e:
            # Catch exceptions and emit the error signal with the error message
            error_message = f"Error in worker thread: {str(e)}\n{traceback.format_exc()}"
            self.error_signal.emit(error_message)


class WorkerThreadVelo(QtCore.QThread):
    result_signal = QtCore.pyqtSignal(object)
    finished_signal = QtCore.pyqtSignal()
    error_signal = QtCore.pyqtSignal(str)

    def __init__(self, theta, phi, theta_arc, phi_arc, omega_max, accel, p):
        super().__init__()

        set_config("temp")

        self.d, self.v_1, self.v_2, self.v_3, self.h_1, self.h_2, self.h_3, self.l_11, self.l_21, self.l_31, self.l_12, self.l_22, self.l_32, self.vec_elbow, self.vec_shoulder, self.l_arm_proth, self.l_humerus, \
        self.m_1, self.m_2, self.m_3, self.m_d = constants()

        self.theta, self.phi, self.theta_arc, self.phi_arc, self.omega_max, self.accel, self.p = theta, phi, theta_arc, phi_arc, omega_max, accel, p

    def run(self):
        try:
            p_dot, p_dotdot, t = Cart_velocity_ramp(self.l_arm_proth, self.theta, self.phi, self.theta_arc,
                                                    self.phi_arc, self.omega_max, self.accel, self.p)

            self.result_signal.emit(p_dot)
        except Exception as e:
            # Catch exceptions and emit the error signal with the error message
            error_message = f"Error in worker thread: {str(e)}\n{traceback.format_exc()}"
            self.error_signal.emit(error_message)

        finally:
            self.finished_signal.emit()  # Ensure this is always emitted


class WorkerThreadWorkspace(QtCore.QThread):
    result_signal = QtCore.pyqtSignal(object, object, object, object, object)
    finished_signal = QtCore.pyqtSignal()
    error_signal = QtCore.pyqtSignal(str)

    def __init__(self, xlim, ylim, zlim, nx, ny, nz):
        super().__init__()
        self.limits = (xlim, ylim, zlim)
        self.res = (nx, ny, nz)

    def run(self):
        try:
            grid, nx, ny, nz, voxels_full = Compute_Workspace(*self.limits, *self.res)
            self.result_signal.emit(grid, nx, ny, nz, voxels_full)
        except Exception as e:
            # Catch exceptions and emit the error signal with the error message
            error_message = f"Error in worker thread: {str(e)}\n{traceback.format_exc()}"
            self.error_signal.emit(error_message)

        finally:
            self.finished_signal.emit()  # Ensure this is always emitted


class WorkerThreadWorkspaceSphere(QtCore.QThread):
    result_signal = QtCore.pyqtSignal(object, object, object)
    finished_signal = QtCore.pyqtSignal()
    error_signal = QtCore.pyqtSignal(str)

    def __init__(self, theta_limits, phi_limits, n_theta, n_phi):
        super().__init__()
        self.limits = (theta_limits, phi_limits)
        self.res = (n_theta, n_phi)

        set_config("temp")

        self.d, self.v_1, self.v_2, self.v_3, self.h_1, self.h_2, self.h_3, self.l_11, self.l_21, self.l_31, self.l_12, self.l_22, self.l_32, self.vec_elbow, self.vec_shoulder, self.l_arm_proth, self.l_humerus, \
        self.m_1, self.m_2, self.m_3, self.m_d = constants()

    def run(self):
        try:
            angles, points, grid = Compute_Workspace_Sphere(*self.limits, *self.res, self.l_arm_proth, self.vec_elbow)
            self.result_signal.emit(angles, points, grid)
        except Exception as e:
            # Catch exceptions and emit the error signal with the error message
            error_message = f"Error in worker thread: {str(e)}\n{traceback.format_exc()}"
            self.error_signal.emit(error_message)

        finally:
            self.finished_signal.emit()  # Ensure this is always emitted


class AcquisitionThread(QtCore.QThread):
    result_signal = QtCore.pyqtSignal(object, object, object)
    finished_signal = QtCore.pyqtSignal()
    error_signal = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()

    def run(self):
        try:
            self.start_logging()
        except Exception as e:
            # Catch exceptions and emit the error signal with the error message
            error_message = f"Error in worker thread: {str(e)}\n{traceback.format_exc()}"
            self.error_signal.emit(error_message)

        finally:
            self.finished_signal.emit()  # Ensure this is always emitted


class DynamicsThread(QtCore.QThread):
    result_signal = QtCore.pyqtSignal(list)
    finished_signal = QtCore.pyqtSignal()
    error_signal = QtCore.pyqtSignal(str)

    def __init__(self, trajectory_data):
        """
        Args:
            trajectory_data: JSON of {{"t" : t, "x" : [x,y,z]}, ...]
        """
        super().__init__()
        self.trajectory_data = trajectory_data

    def run(self):
        try:
            # Send to SIL server
            results = request_sil_simulation(self.trajectory_data)
            self.result_signal.emit(results)

        except Exception as e:
            error_msg = f"Dynamics simulation failed: {str(e)}"
            self.error_signal.emit(error_msg)

        finally:
            self.finished_signal.emit()  # Ensure this is always emitted


class MainWindow(QtWidgets.QMainWindow):
    """
    Main application window containing:
    - 3D visualization panel
    - Control inputs for target positioning
    - Measurement system interface
    - Workspace analysis tools

    Handles:
    - User interaction
    - Thread management for heavy computations
    - Real-time visualization updates
    - Data logging and replay functionality
    """

    def __init__(self, resolution, fps):
        super().__init__()

        self.setWindowTitle("SCARA Robot")
        self.setWindowIcon(QIcon("./assets/images/scara_icon.svg"))

        self.resolution = resolution
        self.fps = fps

        # Workspace
        self.xlim = (-0.5, 0.5)
        self.ylim = (-0.5, 0.5)
        self.zlim = (0, 0.5)
        self.nx = 100
        self.ny = 100
        self.nz = 100
        self.hull = None

        self.theta_limits = (0, np.pi)
        self.phi_limits = (0, 2 * np.pi)
        self.n_theta = 150
        self.n_phi = 250

        # Target angle movement (initial default values)
        self.theta = 15  # Angle in the sagittal plane deg
        self.phi = 20  # Angle in the horizontal plane deg

        self.elbow_changed = False

        # Initial Geometry
        set_config("initial")
        config = load_config()
        set_config("temp")
        save_config(config)

        self.d, self.v_1, self.v_2, self.v_3, self.h_1, self.h_2, self.h_3, self.l_11, self.l_21, self.l_31, self.l_12, self.l_22, self.l_32, self.vec_elbow, self.vec_shoulder, self.l_arm_proth, self.l_humerus, \
        self.m_1, self.m_2, self.m_3, self.m_d = constants()

        # Origin position
        r, theta, phi = Cartesian_to_spherical(0.15, 0, 0.2, self.vec_elbow, 1)
        self.theta_0 = theta # rad
        self.phi_0 = phi # rad
        self.x_0, self.y_0, self.z_0 = Spherical_to_cartesian(self.l_arm_proth, self.theta_0, self.phi_0, self.vec_elbow)

        theta_1a, theta_1b, theta_2a, theta_2b, theta_3a, theta_3b = Compute_kine_point(self.x_0, self.y_0, self.z_0, False, 1)
        print(theta_1a, theta_1b, theta_2a, theta_2b, theta_3a, theta_3b)

        self.p_0 = np.array([self.x_0, self.y_0, self.z_0])

        # Speed ramp
        self.omega_max = 5 * np.pi / 180  # rad.s^-1
        self.accel = 10 * np.pi / 180  # rad.s^-2

        # Plot Window
        self.plot_window = None

        # PyVista widget
        self.PyVistaCreated = False

        # Tracking change actions
        self.last_action = None

        # Initial kinematic computation
        self.start_computation_in_thread_velo()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.log_sensor_data)

        self.acquisition_freq = 1  # Acquisition freq in ms

        self.trajectory_logger = TrajectoryLogger()
        self.simulation_logger = SimulationLogger()
        self.file_browser = FileBrowser(self.trajectory_logger)
        self.file_browser.traj_constants_ready.connect(self.handle_traj_measure)

        # Start animation with a button
        self.start_btn = QPushButton("Loading Workspace ...")
        self.start_btn.clicked.connect(self.start_PyVista_anim)
        self.start_btn.setStyleSheet("""
        QPushButton:disabled {
            background-color: #f0f0f0;
            color: #a0a0a0;
            border: 1px solid #c0c0c0;
        }
        """)

        self.start_btn.setEnabled(False)

        self.simulate_dynamics_btn = QPushButton("Loading Workspace ...")
        self.simulate_dynamics_btn.setStyleSheet("""
        QPushButton:disabled {
            background-color: #f0f0f0;
            color: #a0a0a0;
            border: 1px solid #c0c0c0;
        }
        """)
        self.simulate_dynamics_btn.clicked.connect(self.run_dynamics_simulation)
        self.simulate_dynamics_btn.setEnabled(False)

        # Button to change target position
        self.target_btn = QPushButton("Change Target Position")
        self.target_btn.clicked.connect(self.change_target_pos)

        # Button to compute and display velocities/accelerations
        self.dyn_plot_btn = QPushButton("Plot Motors")
        self.dyn_plot_btn.clicked.connect(self.open_plot_window)

        # Angle Input
        self.theta_input = QtWidgets.QLineEdit()
        self.theta_input.setPlaceholderText("e.g. 15.0 deg")
        self.theta_input.textEdited.connect(self.on_theta_change)

        self.phi_input = QtWidgets.QLineEdit()
        self.phi_input.setPlaceholderText("e.g. 20.0 deg")
        self.phi_input.textEdited.connect(self.on_phi_change)

        self.theta_validator = QDoubleValidator(-360.0, 360.0, 1, self)
        self.theta_validator.setNotation(QDoubleValidator.StandardNotation)
        self.theta_validator.setLocale(QtCore.QLocale("C"))
        self.theta_input.setValidator(self.theta_validator)

        self.phi_validator = QDoubleValidator(-360.0, 360.0, 1, self)
        self.phi_validator.setNotation(QDoubleValidator.StandardNotation)
        self.phi_validator.setLocale(QtCore.QLocale("C"))
        self.phi_input.setValidator(self.phi_validator)

        # Button to change elbow position
        self.elbow_btn = QPushButton("Change Elbow Position")
        self.elbow_btn.clicked.connect(self.change_elbow_pos)

        # Elbow Input
        self.x_input = QtWidgets.QLineEdit()
        self.x_input.setPlaceholderText("e.g. 0.350 m")
        self.x_input.textEdited.connect(self.on_x_change)

        self.y_input = QtWidgets.QLineEdit()
        self.y_input.setPlaceholderText("e.g. 0.350 m")
        self.y_input.textEdited.connect(self.on_y_change)

        self.z_input = QtWidgets.QLineEdit()
        self.z_input.setPlaceholderText("e.g. 0.000 m")
        self.z_input.textEdited.connect(self.on_z_change)

        self.x_validator = QDoubleValidator(-1.000, 1.000, 3, self)
        self.x_validator.setNotation(QDoubleValidator.StandardNotation)
        self.x_validator.setLocale(QtCore.QLocale("C"))
        self.x_input.setValidator(self.x_validator)

        self.y_validator = QDoubleValidator(-1.000, 1.000, 3, self)
        self.y_validator.setNotation(QDoubleValidator.StandardNotation)
        self.y_validator.setLocale(QtCore.QLocale("C"))
        self.y_input.setValidator(self.y_validator)

        self.z_validator = QDoubleValidator(-1.000, 1.000, 3, self)
        self.z_validator.setNotation(QDoubleValidator.StandardNotation)
        self.z_validator.setLocale(QtCore.QLocale("C"))
        self.z_input.setValidator(self.z_validator)

        self.toggle_button_workspace = QPushButton("Workspace")
        self.toggle_button_workspace.setCheckable(True)
        self.toggle_button_workspace.setChecked(True)  # Visible by default
        self.toggle_button_workspace.clicked.connect(self.toggle_workspace_visibility)

        self.toggle_button_workspace_sphere = QPushButton("Sphere")
        self.toggle_button_workspace_sphere.setCheckable(True)
        self.toggle_button_workspace_sphere.setChecked(True)  # Visible by default
        self.toggle_button_workspace_sphere.clicked.connect(self.toggle_workspace_sphere_visibility)

        self.start_measure_button = QPushButton("Start Measurement")
        self.start_measure_button.setIcon(self.style().standardIcon(QStyle.SP_DialogYesButton))
        self.start_measure_button.clicked.connect(self.start_logging)

        self.stop_measure_button = QPushButton("Stop Measurement")
        self.stop_measure_button.setIcon(self.style().standardIcon(QStyle.SP_DialogNoButton))
        self.stop_measure_button.clicked.connect(self.stop_logging)

        self.replay_measure_button = QPushButton("Replay")
        self.replay_measure_button.setIcon(self.style().standardIcon(QStyle.SP_BrowserReload))
        self.replay_measure_button.clicked.connect(self.handle_buffer)

        self.replay_measure_stop_button = QPushButton("Stop Replay")
        self.replay_measure_stop_button.setIcon(self.style().standardIcon(QStyle.SP_MediaStop))
        self.replay_measure_stop_button.clicked.connect(self.return_scene_to_visible)


        self.buffer_reading = False
        self.replay = False
        self.dynamics_traj = False

        self.setup_ui()

        self.sil_server = SILServerProcess()
        self.sil_server.start(debug=True)

    def setup_ui(self):
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QtWidgets.QVBoxLayout(self.central_widget)

        self.h_layout = QtWidgets.QHBoxLayout()
        self.main_layout.addLayout(self.h_layout)

        side_panel = QtWidgets.QWidget()
        side_panel_layout = QtWidgets.QVBoxLayout(side_panel)
        side_panel.setMaximumWidth(600)

        initial_position_group = QtWidgets.QGroupBox("Position elbow")
        initial_position_layout = QtWidgets.QVBoxLayout(initial_position_group)

        xyz_layout = QtWidgets.QHBoxLayout()

        x_layout = QtWidgets.QHBoxLayout()
        x_layout.addWidget(QtWidgets.QLabel("x (m):"))
        x_layout.addWidget(self.x_input)

        y_layout = QtWidgets.QHBoxLayout()
        y_layout.addWidget(QtWidgets.QLabel("y (m):"))
        y_layout.addWidget(self.y_input)

        z_layout = QtWidgets.QHBoxLayout()
        z_layout.addWidget(QtWidgets.QLabel("z (m):"))
        z_layout.addWidget(self.z_input)

        xyz_layout.addLayout(x_layout)
        xyz_layout.addLayout(y_layout)
        xyz_layout.addLayout(z_layout)

        initial_position_layout.addLayout(xyz_layout)

        initial_position_layout.addWidget(self.elbow_btn)

        position_group = QtWidgets.QGroupBox("Position Target")
        position_layout = QtWidgets.QVBoxLayout(position_group)

        phi_theta_layout = QtWidgets.QHBoxLayout()

        theta_layout = QtWidgets.QHBoxLayout()
        theta_layout.addWidget(QtWidgets.QLabel("θ (deg):"))
        theta_layout.addWidget(self.theta_input)

        phi_layout = QtWidgets.QHBoxLayout()
        phi_layout.addWidget(QtWidgets.QLabel("φ (deg):"))
        phi_layout.addWidget(self.phi_input)

        phi_theta_layout.addLayout(theta_layout)
        phi_theta_layout.addLayout(phi_layout)

        position_layout.addLayout(phi_theta_layout)

        position_layout.addWidget(self.target_btn)

        visual_group = QtWidgets.QGroupBox("Visualization Options")
        visual_layout = QtWidgets.QHBoxLayout(visual_group)
        visual_layout.addWidget(self.toggle_button_workspace)
        visual_layout.addWidget(self.toggle_button_workspace_sphere)

        play_group = QtWidgets.QGroupBox("Preview")
        play_layout = QtWidgets.QHBoxLayout(play_group)
        play_layout.addWidget(self.start_btn)
        play_layout.addWidget(self.simulate_dynamics_btn)

        analysis_group = QtWidgets.QGroupBox("Analysis")
        analysis_layout = QtWidgets.QVBoxLayout(analysis_group)
        analysis_layout.addWidget(self.dyn_plot_btn)

        measure_group = QtWidgets.QGroupBox("Measure")
        measure_layout = QtWidgets.QHBoxLayout(measure_group)
        measure_layout.addWidget(self.start_measure_button)
        measure_layout.addWidget(self.stop_measure_button)
        measure_layout.addWidget(self.replay_measure_button)
        measure_layout.addWidget(self.replay_measure_stop_button)

        save_group = QtWidgets.QGroupBox("Save Trajectory")
        save_layout = QtWidgets.QVBoxLayout(save_group)
        save_layout.addWidget(self.file_browser)

        side_panel_layout.addWidget(initial_position_group)
        side_panel_layout.addWidget(position_group)
        side_panel_layout.addWidget(visual_group)
        side_panel_layout.addWidget(play_group)
        side_panel_layout.addWidget(analysis_group)
        side_panel_layout.addWidget(measure_group)
        side_panel_layout.addWidget(save_group)
        side_panel_layout.addStretch()

        self.h_layout.addWidget(side_panel)

        self.visualizer_container = QtWidgets.QWidget()
        self.visualizer_layout = QtWidgets.QVBoxLayout(self.visualizer_container)
        self.h_layout.addWidget(self.visualizer_container, 1)

    def compute_kinematics(self, x, y, z, arc_traj):
        p_0, phi_arc, theta_arc, p, vec_elbow, vec_shoulder, s_1a, \
        s_1b, s_2a, s_2b, s_3a, s_3b, d, v_1, v_2, v_3, m_1_point, \
        m_2_point, m_3_point, z_vec, N_points = \
            Compute_kine_traj(x, y, z, self.theta * np.pi / 180, self.phi * np.pi / 180, True, True, arc_traj)

        return p_0, phi_arc, theta_arc, p, vec_elbow, vec_shoulder, s_1a, \
        s_1b, s_2a, s_2b, s_3a, s_3b, d, v_1, v_2, v_3, m_1_point, \
        m_2_point, m_3_point, z_vec, N_points

    def compute_and_update_visualizer(self, t=None):
        if not self.PyVistaCreated:
            self.visualizer = Pyvista3DScene(self.p_0, self.phi_arc, self.theta_arc, self.p, self.p_dot,
                                             self.vec_elbow, self.vec_shoulder, self.s_1a, self.s_1b,
                                             self.s_2a, self.s_2b, self.s_3a, self.s_3b, self.d,
                                             self.v_1, self.v_2, self.v_3, self.m_1_point, self.m_2_point,
                                             self.m_3_point,
                                             self.z_vec, self.resolution, self.fps)

            self.visualizer.replay_finished.connect(self.return_scene_to_visible)
            self.visualizer_layout.addWidget(self.visualizer)
            self.PyVistaCreated = True
            self.start_computation_in_thread_workspace()
            self.start_computation_in_thread_workspace_sphere()

        else:
            self.visualizer.stop_anim = True
            self.visualizer.update_scene(self.p_0, self.phi_arc, self.theta_arc, self.p,
                                         self.p_dot, self.vec_elbow, self.vec_shoulder, self.s_1a, self.s_1b,
                                         self.s_2a, self.s_2b, self.s_3a,
                                         self.s_3b, self.d, self.v_1,
                                         self.v_2, self.v_3, self.m_1_point, self.m_2_point, self.m_3_point, self.z_vec,
                                         self.resolution,
                                         self.fps)

            if self.elbow_changed:
                self.visualizer.update_elbow()
                self.start_computation_in_thread_workspace_sphere()
                self.trajectory_logger.delete_buffer()

        self.visualizer.resampling(t=t)

    def on_theta_change(self):
        text = self.theta_input.text()
        try:

            if text and not text.endswith("."):
                val = float(self.theta_input.text())
                self.theta = val
                self.waiting_target_pos_change()
        except ValueError:
            pass

    def on_phi_change(self):
        text = self.phi_input.text()
        try:
            if text and not text.endswith("."):
                val = float(self.phi_input.text())
                self.phi = val
                self.waiting_target_pos_change()
        except ValueError:
            pass

    def on_x_change(self):
        text = self.x_input.text()
        try:
            if text and not text.endswith("."):
                val = float(self.x_input.text())
                self.x_new = val
                self.waiting_elbow_pos_change()
        except ValueError:
            pass

    def on_y_change(self):
        text = self.y_input.text()
        try:
            if text and not text.endswith("."):
                val = float(self.y_input.text())
                self.y_new = val
                self.waiting_elbow_pos_change()
        except ValueError:
            pass

    def on_z_change(self):
        text = self.z_input.text()
        try:
            if text and not text.endswith("."):
                val = float(self.z_input.text())
                self.z_new = val
                self.waiting_elbow_pos_change()
        except ValueError:
            pass

    def change_target_pos(self):
        if not hasattr(self, 'visualizer') or not hasattr(self.visualizer, 'Workspace_mesh'):
            QtWidgets.QMessageBox.warning(self, "Attention", "Workspace mesh not yet available. Please wait.")
            return

        self.last_action = 'target'

        if self.is_trajectory_feasible_mesh():
            self.target_btn.setText("Processing...")
            self.target_btn.setEnabled(False)
            self.start_computation_in_thread_velo()
        else:
            QtWidgets.QMessageBox.warning(self, "Trajectory Error",
                                          f"The trajectory to θ={self.theta}° and φ={self.phi}° is not feasible.")
            self.target_btn.setText("Invalid trajectory")
            self.target_btn.setStyleSheet("QPushButton { color: red; }")
            QtCore.QTimer.singleShot(2000, self.waiting_target_pos_change)

    def change_elbow_pos(self):
        if not hasattr(self, 'visualizer'):
            QtWidgets.QMessageBox.warning(self, "Attention", "Pyvista scene not yet available. Please wait.")
            return

        self.last_action = 'elbow'

        if not hasattr(self, 'x_new') or not self.x_input.text():
            self.x_new = self.vec_elbow[0]
        if not hasattr(self, 'y_new') or not self.y_input.text():
            self.y_new = self.vec_elbow[1]
        if not hasattr(self, 'z_new') or not self.z_input.text():
            self.z_new = self.vec_elbow[2]

        self.x_new = float(self.x_new)
        self.y_new = float(self.y_new)
        self.z_new = float(self.z_new)

        set_config("temp")

        config = load_config()

        config['elbow']['x'] = self.x_new
        config['elbow']['y'] = self.y_new
        config['elbow']['z'] = self.z_new

        save_config(config)

        self.d, self.v_1, self.v_2, self.v_3, self.h_1, self.h_2, self.h_3, self.l_11, self.l_21, self.l_31, self.l_12, self.l_22, self.l_32, self.vec_elbow, self.vec_shoulder, self.l_arm_proth, self.l_humerus, \
        self.m_1, self.m_2, self.m_3, self.m_d = constants()

        self.start_computation_in_thread_velo()

    def is_trajectory_feasible_mesh(self):
        p_0, x_arc, y_arc, z_arc, N_points, theta_arc, phi_arc = arc(self.x_0, self.y_0, self.z_0,
                                                                     self.theta * np.pi / 180, self.phi * np.pi / 180,
                                                                     True)
        traj = np.array([x_arc, y_arc, z_arc]).T
        traj_cloud = pv.PolyData(traj)

        workspace_mesh = self.original_workspace_mesh
        enclosed = traj_cloud.select_enclosed_points(workspace_mesh, tolerance=1e-6)
        mask = enclosed.point_data['SelectedPoints'].astype(bool)
        return np.all(mask)

    def changed_target_anim(self):
        self.target_btn.setStyleSheet(STYLE_NORMAL)

    def changed_elbow_anim(self):
        self.elbow_btn.setStyleSheet(STYLE_NORMAL)

    def toggle_style_target(self):
        self.target_btn.setText("Target position changed")
        self.target_btn.setStyleSheet(STYLE_ACTION_NEEDED)
        QtCore.QTimer.singleShot(1000, self.changed_target_anim)

    def toggle_style_elbow(self):
        self.elbow_changed = False
        self.elbow_btn.setText("Elbow position changed")
        self.elbow_btn.setStyleSheet(STYLE_ACTION_NEEDED)
        QtCore.QTimer.singleShot(1000, self.changed_elbow_anim)

    def waiting_target_pos_change(self):
        self.target_btn.setText("Change Target position")
        self.target_btn.setStyleSheet(STYLE_CHANGED)
        self.target_btn.setToolTip("New target position needs confirmation")

    def waiting_elbow_pos_change(self):
        self.elbow_changed = True
        self.elbow_btn.setText("Change Elbow position")
        self.elbow_btn.setStyleSheet(STYLE_CHANGED)
        self.elbow_btn.setToolTip("New elbow position needs confirmation")

    def toggle_style_start(self):
        # Disable buttons while computations running
        self.start_btn.setText("Play Trajectory")
        self.start_btn.setStyleSheet("""
                QPushButton {
                    font-weight: bold;
                }
                """)
        self.start_btn.setEnabled(True)

        self.simulate_dynamics_btn.setText("Simulate Dynamics")
        self.simulate_dynamics_btn.setStyleSheet("""
                        QPushButton {
                            font-weight: bold;
                        }
                        """)
        self.simulate_dynamics_btn.setEnabled(True)

    def toggle_workspace_visibility(self):
        visible = self.toggle_button_workspace.isChecked()
        self.Workspace.SetVisibility(visible)
        self.visualizer.scene.render()

    def toggle_workspace_sphere_visibility(self):
        visible = self.toggle_button_workspace_sphere.isChecked()
        self.WorkspaceSphere.SetVisibility(visible)
        self.visualizer.scene.render()

    def start_logging(self):
        self.trajectory_logger.start_new_trajectory()
        self.start_measure_button.setText("Logging ...")
        self.start_measure_button.setStyleSheet("""
        QPushButton:disabled {
            background-color: #f0f0f0;
            color: #a0a0a0;
            border: 1px solid #c0c0c0;
        }
        """)
        self.timer.start(self.acquisition_freq)

    def stop_logging(self):
        self.timer.stop()
        self.start_measure_button.setStyleSheet("")
        self.start_measure_button.setText("Start Measurement")
        self.trajectory_logger.save_to_buffer()

    def log_sensor_data(self):
        angles = self.read_sensor()
        self.trajectory_logger.log_frame(angles)

    def read_sensor(self):
        frame = self.visualizer.frame_count
        p = self.visualizer.p[min(frame, self.visualizer.max_frames-1)]
        r, theta, phi = Cartesian_to_spherical(p[0], p[1], p[2], self.vec_elbow, 1)
        return [theta, phi]

    def start_PyVista_anim(self):
        if hasattr(self, 'visualizer'):
            self.visualizer.stop_anim = True
            if self.replay or self.dynamics_traj:
                self.return_scene_to_visible()
            self.visualizer.start_animation()
        else:
            QtWidgets.QMessageBox.critical(self, "Error", "Visualizer not created")

    def handle_thread_error(self, error_message):
        print(f"Received error message from thread: {error_message}")
        QtWidgets.QMessageBox.critical(self, "Thread Error", f"Calculation error: {error_message}")

    def handle_results_velo(self, p_dot):
        self.p_dot = p_dot  # Set it first
        self.compute_and_update_visualizer()  # Now create or update the visualizer
        self.target_btn.setEnabled(True)
        self.last_action = None

    def handle_results_workspace(self, grid, nx, ny, nz, voxels_full):

        verts, faces, normals, _ = measure.marching_cubes(voxels_full, level=0.5)

        grid_coords = grid.reshape((nx, ny, nz, 3))

        verts_world = np.zeros_like(verts)
        for i, (x, y, z) in enumerate(verts):
            ix, iy, iz = np.clip(np.floor([x, y, z]).astype(int), 0, [nx - 1, ny - 1, nz - 1])
            verts_world[i] = grid_coords[ix, iy, iz]

        faces_pv = np.hstack([np.full((faces.shape[0], 1), 3), faces]).astype(np.int32)
        mesh = pv.PolyData(verts_world, faces_pv)
        smoothed = mesh.smooth(n_iter=40, relaxation_factor=0.05)

        self.visualizer.Workspace_mesh = smoothed

        self.original_workspace_mesh = self.visualizer.Workspace_mesh.copy()

        self.Workspace = self.visualizer.scene.add_mesh(self.visualizer.Workspace_mesh,
                                                        color=self.visualizer.color_workspace, opacity=0.05,
                                                        name="WorkspaceMesh", style='Wireframe')

    def handle_worskpace_finished_wait(self, angles, points, grid):
        self.angles_reach, self.points_reach, self.grid_reach = angles, points, grid

    def worskpace_finished(self):
        self.worskpace_finished_bool = True

    def worskpace_sphere_finished(self):
        self.worskpace_sphere_finished_bool = True

    def handle_results_workspace_sphere(self):

        if self.worskpace_finished_bool and self.worskpace_sphere_finished_bool:

            points = self.points_reach
            cloud = pv.PolyData(points)

            mesh = self.visualizer.Workspace_mesh

            # Use select_enclosed_points to mark which points are inside
            enclosed = cloud.select_enclosed_points(mesh, tolerance=1e-6)

            # Extract the mask
            mask = enclosed.point_data['SelectedPoints'].astype(bool)
            valid_points = points[mask]

            new_sphere_arm = pv.PolyData(valid_points)

            if not hasattr(self.visualizer, 'sphere_arm'):
                self.visualizer.sphere_arm = new_sphere_arm
                self.WorkspaceSphere = self.visualizer.scene.add_mesh(self.visualizer.sphere_arm, ambient=0.4,
                                                                      diffuse=0.5, specular=0.8,
                                                                      color=self.visualizer.color_workspace,
                                                                      opacity=0.4, name="WorkspaceSphere",
                                                                      style='wireframe')

                self.visualizer.setup_scene()

            else:
                self.visualizer.stop_anim = True
                if self.visualizer.timer.isActive():
                    self.visualizer.timer.stop()

                self.visualizer.sphere_arm.copy_from(new_sphere_arm)
                self.visualizer.scene.render()

    def handle_traj_measure(self, results):
        self.replay = True
        self.visualizer.stop_anim = True

        self.elbow_btn.setEnabled(False)
        self.target_btn.setEnabled(False)
        self.file_browser.play_btn.setEnabled(False)
        self.replay_measure_button.setEnabled(False)

        if (self.buffer_reading and self.trajectory_logger.empty_buffer) or \
                (not self.buffer_reading and self.trajectory_logger.empty_file):
            QtWidgets.QMessageBox.warning(self, "Empty Data", "No trajectory data to animate.")
            self.return_scene_to_visible()
            return

        # Load file values
        consts, t_measure, phi_arc, theta_arc = results

        d, v_1, v_2, v_3, h_1, h_2, h_3, l_11, l_21, l_31, l_12, l_22, l_32, vec_elbow, vec_shoulder, l_arm_proth, l_humerus, \
        m_1, m_2, m_3, m_d = consts

        p_temp, p_0 = Compute_trajectory_from_data(l_arm_proth, theta_arc, phi_arc, vec_elbow)
        p = p_temp.T

        self.visualizer.update_scene(
            p_0=p_0,
            phi_arc=phi_arc,
            theta_arc=theta_arc,
            p=p,
            p_dot=np.zeros_like(p),
            vec_elbow=vec_elbow,
            vec_shoulder=vec_shoulder,
            s_1a=None, s_1b=None, s_2a=None, s_2b=None,
            s_3a=None, s_3b=None,
            d=d, v_1=v_1, v_2=v_2, v_3=v_3,
            m_1=np.zeros(3), m_2=np.zeros(3), m_3=np.zeros(3),
            z_vec=self.z_vec,
            resolution=self.resolution,
            fps=self.fps
        )

        self.visualizer.update_elbow(fullscene=False)
        self.visualizer.resampling(measure=True, t=t_measure)


        arm = ["Forearm", "Shoulder", "Humerus", "Elbow", "Table", "Trajectory", "Controlled"]

        for name, actor in self.visualizer.scene.renderer.actors.items():
            if name not in arm:
                actor.SetVisibility(False)


        # Play animation
        self.visualizer.start_animation(measure=True)

    def handle_buffer(self):
        self.buffer_reading = True
        self.trajectory_logger.read_buffer()
        if not self.trajectory_logger.empty_buffer:
            results = self.trajectory_logger.results
            self.handle_traj_measure(results)
        else:
            QtWidgets.QMessageBox.warning(self, "Empty Buffer", "No trajectory data to replay.")
            self.buffer_reading = False

    def return_scene_to_visible(self):
        self.buffer_reading = False
        self.elbow_btn.setEnabled(True)
        self.target_btn.setEnabled(True)
        self.file_browser.play_btn.setEnabled(True)
        self.replay_measure_button.setEnabled(True)
        self.visualizer.stop_anim = True

        if self.replay:

            arm = ["Forearm", "Shoulder", "Humerus", "Elbow", "Table", "Trajectory", "Controlled"]

            for name, actor in self.visualizer.scene.renderer.actors.items():
                if name not in arm:
                    actor.SetVisibility(True)


        self.visualizer.update_scene(self.p_0, self.phi_arc, self.theta_arc, self.p,
                                     self.p_dot, self.vec_elbow, self.vec_shoulder, self.s_1a, self.s_1b,
                                     self.s_2a, self.s_2b, self.s_3a,
                                     self.s_3b, self.d, self.v_1,
                                     self.v_2, self.v_3, self.m_1_point, self.m_2_point, self.m_3_point, self.z_vec,
                                     self.resolution,
                                     self.fps)

        self.visualizer.update_elbow(fullscene=True)
        self.replay = False
        self.dynamics_traj = False

    def start_computation_in_thread_workspace(self):
        # Prevent spinning up multiple threads
        if hasattr(self, 'worker_thread_workspace') and self.worker_thread_workspace.isRunning():
            return

        self.worskpace_finished_bool = False

        self.worker_thread_workspace = WorkerThreadWorkspace(self.xlim, self.ylim, self.zlim, self.nx, self.ny, self.nz)
        self.worker_thread_workspace.result_signal.connect(self.handle_results_workspace)
        self.worker_thread_workspace.finished_signal.connect(self.toggle_style_start)
        self.worker_thread_workspace.finished_signal.connect(self.worskpace_finished)
        self.worker_thread_workspace.finished_signal.connect(self.handle_results_workspace_sphere)
        self.worker_thread_workspace.error_signal.connect(self.handle_thread_error)
        self.worker_thread_workspace.start()

    def start_computation_in_thread_workspace_sphere(self):
        # Prevent spinning up multiple threads
        if hasattr(self, 'worker_thread_workspace_sphere') and self.worker_thread_workspace_sphere.isRunning():
            return

        self.worskpace_sphere_finished_bool = False

        self.worker_thread_workspace_sphere = WorkerThreadWorkspaceSphere(self.theta_limits, self.phi_limits,
                                                                          self.n_theta, self.n_phi)
        self.worker_thread_workspace_sphere.result_signal.connect(self.handle_worskpace_finished_wait)
        self.worker_thread_workspace_sphere.finished_signal.connect(self.worskpace_sphere_finished)
        self.worker_thread_workspace_sphere.finished_signal.connect(self.handle_results_workspace_sphere)
        self.worker_thread_workspace_sphere.error_signal.connect(self.handle_thread_error)
        self.worker_thread_workspace_sphere.start()

    def start_computation_in_thread_velo(self):
        if hasattr(self, 'worker_thread_velo') and self.worker_thread_velo.isRunning():
            return  # Still running — don't touch it

        # If old thread exists and is finished, clean it up
        if hasattr(self, 'worker_thread_velo'):
            self.worker_thread_velo.quit()
            self.worker_thread_velo.wait()
            self.worker_thread_velo.deleteLater()
            del self.worker_thread_velo

        self.p_0, self.phi_arc, self.theta_arc, self.p, self.vec_elbow, self.vec_shoulder, self.s_1a, \
        self.s_1b, self.s_2a, self.s_2b, self.s_3a, self.s_3b, self.d, self.v_1, self.v_2, self.v_3, self.m_1_point, \
        self.m_2_point, self.m_3_point, self.z_vec, self.N_points \
         = self.compute_kinematics(self.x_0, self.y_0, self.z_0, True)

        print(self.s_1a[0, :], self.s_1b[0,:], self.s_2a[0, :], self.s_2b[0, :], self.s_3a[0, :], self.s_3b[0, :])

        self.worker_thread_velo = WorkerThreadVelo(
            self.theta, self.phi, self.theta_arc, self.phi_arc,
            self.omega_max, self.accel, self.p
        )
        self.worker_thread_velo.result_signal.connect(self.handle_results_velo)
        self.worker_thread_velo.error_signal.connect(self.handle_thread_error)
        if self.last_action == 'target':
            self.worker_thread_velo.finished_signal.connect(self.toggle_style_target)
        elif self.last_action == 'elbow':
            self.worker_thread_velo.finished_signal.connect(self.toggle_style_elbow)

        self.worker_thread_velo.start()

    def open_plot_window(self):
        # Load cached data
        self.simulation_logger.load_ideal_data()
        ideal_data = self.simulation_logger.ideal_data

        # Extract simulation results
        sim_results = self.simulation_logger.sim_results
        t_sim = [frame['t'] for frame in sim_results]
        theta_sim = np.array([frame['theta'] for frame in sim_results])
        theta_dot_sim = np.array([frame['theta_dot'] for frame in sim_results])
        tau_sim = np.array([frame['tau'] for frame in sim_results])

        # Check if we have ideal data
        if ideal_data is None or len(ideal_data['t']) == 0:
            QtWidgets.QMessageBox.warning(
                self, "Data Missing",
                "No ideal data available for comparison"
            )
            return

        # Extract ideal data
        t_ideal = ideal_data['t']
        theta_ideal = ideal_data['theta']
        theta_dot_ideal = ideal_data['theta_dot']
        tau_ideal = ideal_data['tau_ideal']

        # Interpolate ideal data to simulation time base
        def interpolate_data(t_target, t_source, data_source):
            data_interp = np.zeros((len(t_target), data_source.shape[1]))
            for i in range(data_source.shape[1]):
                interp_fn = interp1d(
                    t_source, data_source[:, i],
                    kind='linear', fill_value="extrapolate"
                )
                data_interp[:, i] = interp_fn(t_target)
            return data_interp

        tau_ideal_interp = interpolate_data(t_sim, t_ideal, tau_ideal)
        theta_ideal_interp = interpolate_data(t_sim, t_ideal, theta_ideal)
        theta_dot_ideal_interp = interpolate_data(t_sim, t_ideal, theta_dot_ideal)

        # Create plot window
        if self.plot_window is None:
            self.plot_window = Plot_velo_accel()
            self.plot_window.closed.connect(self.on_plot_window_closed)
            self.plot_window.setWindowTitle("Dynamics Comparison")
            self.plot_window.setWindowIcon(QIcon("./assets/images/plot_icon.svg"))

        # Plot all comparisons
        self.plot_window.plot_comparisons(
            np.array(t_sim),
            theta_sim, theta_dot_sim, tau_sim,
            theta_ideal_interp, theta_dot_ideal_interp, tau_ideal_interp
        )
        self.plot_window.show()

    def interpolate_torques(self, t_target, t_source, tau_source):
        tau_interp = np.zeros((len(t_target), 3))
        for i in range(3):
            interp_fn = interp1d(
                t_source,
                tau_source[:, i],
                kind='linear',
                fill_value="extrapolate"
            )
            tau_interp[:, i] = interp_fn(t_target)
        return tau_interp

    def on_plot_window_closed(self):
        self.plot_window = None

    def run_dynamics_simulation(self):
        if not hasattr(self, 'simulation_logger'):
            QtWidgets.QMessageBox.warning(self, "No Trajectory", "No simulated trajectory logger object")
            return

        self.simulate_dynamics_btn.setEnabled(False)
        self.simulate_dynamics_btn.setText("Computing...")

        # Get the current trajectory data
        p_kine = self.p
        p_dot_kine, p_dotdot_kine, t_kine = Cart_velocity_ramp(self.l_arm_proth, self.theta, self.phi, self.theta_arc,
                                                self.phi_arc, self.omega_max, self.accel, self.p)

        trajectory_data = [
            {
                "t": float(t_kine[i]),
                "x": [float(x), float(y), float(z)],
                "x_dot": [float(vx), float(vy), float(vz)],
                "x_ddot": [float(ax), float(ay), float(az)]
            }
            for i, ((x, y, z), (vx, vy, vz), (ax, ay, az)) in enumerate(
                zip(p_kine, p_dot_kine.T, p_dotdot_kine.T)
            )
        ]

        hash = self.simulation_logger.compute_trajectory_hash(trajectory_data)

        # Check for simulation cache match
        if not self.simulation_logger.needs_recompute(hash):
            print("✅ Using cached SIL simulation results")
            self.simulation_logger.load_cached_simulation()
            self.simulation_logger.load_ideal_data()
            t_dyn, p_dyn, p_dot_dyn, theta_dyn, theta_dot_dyn, tau_dyn = parse_simulation_result(self.simulation_logger.sim_results)
            p_0, phi_arc, theta_arc, p, vec_elbow, vec_shoulder, s_1a, \
            s_1b, s_2a, s_2b, s_3a, s_3b, d, v_1, v_2, v_3, m_1_point, \
            m_2_point, m_3_point, z_vec, N_points = \
                self.compute_kinematics(p_dyn[:, 0], p_dyn[:, 1], p_dyn[:, 2], False)

            # Update visualizer with simulated data without altering self.p
            self.visualizer.update_scene(
                p_0, phi_arc, theta_arc, p_dyn,  # Use simulated trajectory
                p_dot_dyn.T, vec_elbow, vec_shoulder,
                s_1a, s_1b, s_2a, s_2b, s_3a, s_3b, d,
                v_1, v_2, v_3, m_1_point, m_2_point, m_3_point,
                z_vec, self.resolution, self.fps
            )

            self.visualizer.resampling(t=t_dyn)
            self.start_PyVista_anim()
            self.dynamics_traj = True
            self.simulate_dynamics_btn.setEnabled(True)
            self.simulate_dynamics_btn.setText("Simulate Dynamics")
            return

        print("🌀 Running new SIL simulation (no cache match)...")

        # Define thread and callbacks
        self.worker_thread_dynamics = DynamicsThread(trajectory_data)

        def on_result(results):
            # Save new simulation result to buffer

            simulation_results = results [0]
            ideal_points = results [1]

            # Extract arrays from ideal_points
            t_ideal = [point['t'] for point in ideal_points]
            theta_ideal = [point['theta'] for point in ideal_points]
            theta_dot_ideal = [point['theta_dot'] for point in ideal_points]
            tau_ideal = [point['tau_ideal'] for point in ideal_points]

            # Save full ideal data
            self.simulation_logger.save_ideal_data(
                t_ideal, theta_ideal, theta_dot_ideal, tau_ideal
            )

            sim_hash = self.simulation_logger.compute_trajectory_hash(trajectory_data)
            self.simulation_logger.save_simulation_results(simulation_results, sim_hash)

            t_dyn, p_dyn, p_dot_dyn, theta_dyn, theta_dot_dyn, tau_dyn = parse_simulation_result(self.simulation_logger.sim_results)
            p_0, phi_arc, theta_arc, p, vec_elbow, vec_shoulder, s_1a, \
            s_1b, s_2a, s_2b, s_3a, s_3b, d, v_1, v_2, v_3, m_1_point, \
            m_2_point, m_3_point, z_vec, N_points = \
                self.compute_kinematics(p_dyn[:, 0], p_dyn[:, 1], p_dyn[:, 2], False)

            # Update visualizer with simulated data without altering self.p
            self.visualizer.update_scene(
                p_0, phi_arc, theta_arc, p_dyn,  # Use simulated trajectory
                p_dot_dyn.T, vec_elbow, vec_shoulder,
                s_1a, s_1b, s_2a, s_2b, s_3a, s_3b, d,
                v_1, v_2, v_3, m_1_point, m_2_point, m_3_point,
                z_vec, self.resolution, self.fps
            )

            #self.visualizer.resampling(t=t_dyn)
            #self.start_PyVista_anim()
            self.dynamics_traj = True
            self.simulate_dynamics_btn.setEnabled(True)
            return

        self.worker_thread_dynamics.result_signal.connect(on_result)
        self.worker_thread_dynamics.error_signal.connect(self.handle_thread_error)
        self.worker_thread_dynamics.finished_signal.connect(
            lambda: self.simulate_dynamics_btn.setEnabled(True))
        self.worker_thread_dynamics.finished_signal.connect(
            lambda: self.simulate_dynamics_btn.setText("Simulate Dynamics"))
        self.worker_thread_dynamics.start()

    def closeEvent(self, event):
        if hasattr(self, 'worker_thread_plot') and self.worker_thread_plot.isRunning():
            self.worker_thread_plot.quit()
            self.worker_thread_plot.wait()
        if hasattr(self, 'worker_thread_velo') and self.worker_thread_velo.isRunning():
            self.worker_thread_velo.quit()
            self.worker_thread_velo.wait()
        if hasattr(self, 'worker_thread_workspace') and self.worker_thread_workspace.isRunning():
            self.worker_thread_workspace.quit()
            self.worker_thread_workspace.wait()
        if hasattr(self, 'worker_thread_workspace_sphere') and self.worker_thread_workspace_sphere.isRunning():
            self.worker_thread_workspace_sphere.quit()
            self.worker_thread_workspace_sphere.wait()
        if self.plot_window is not None:
            self.plot_window.close()  # Close the plot window explicitly
        if hasattr(self, 'timer'):
            self.timer.deleteLater()
        if hasattr(self, 'sil_server'):
            self.sil_server.stop()
        event.accept()  # Accept the close event


def create_Main_Window(resolution, fps):
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow(resolution, fps)
    window.resize(resolution[0], resolution[1])
    window.show()
    sys.exit(app.exec_())
