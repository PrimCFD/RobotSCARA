import numpy as np
import pyvista as pv
import vtk
from pyvistaqt import QtInteractor
from PyQt5 import QtWidgets, QtCore
from robot.misc import speed_ramp, angular_dist, t_physical
from scipy.interpolate import CubicSpline


class Pyvista3DScene(QtWidgets.QWidget):
    """
    3D visualization engine using PyVista for robot state display.

    Handles:
    - Real-time kinematic visualization
    - Workspace rendering
    - Trajectory animation
    - Collision/feasibility visualization

    Implements:
    - VTK-based rendering pipeline
    - Frame interpolation for smooth animation
    - GPU-accelerated visualization
    """

    replay_finished = QtCore.pyqtSignal()

    ### Plot PyVista ###

    def __init__(self, p_0, phi_arc, theta_arc, p, p_dot, vec_elbow, vec_shoulder, s_1a, s_1b, s_2a, s_2b, s_3a, s_3b,
                 d, v_1,
                 v_2, v_3, m_1, m_2, m_3, z_vec, resolution, fps):
        super().__init__()

        self.update_scene(p_0, phi_arc, theta_arc, p, p_dot, vec_elbow, vec_shoulder, s_1a, s_1b, s_2a, s_2b, s_3a,
                          s_3b, d, v_1,
                          v_2, v_3, m_1, m_2, m_3, z_vec, resolution, fps)

        N_points = len(p.tolist())
        self.N_points = N_points

        self.h_3 = self.m_3[2]

        self.frame_count = 0
        self.max_frames = self.N_points

        # Timer for animation
        self.timer = QtCore.QTimer(self)

        # Layout
        layout = QtWidgets.QVBoxLayout()
        self.setLayout(layout)

        # 3D plotter widget
        self.scene = QtInteractor(self)
        layout.addWidget(self.scene.interactor)

        self.stop_anim = False

        self.scene.camera_position = (2.5 * self.h_3, 0, 2 * self.h_3)
        self.scene.add_axes()

        self.z_shift = - self.h_3 * 0.4  # Shift in z for plotting m

        self.r_joint = 0.01  # Sphere joint representation radius m
        self.r_robot = 0.002
        self.r_arm = 0.005

        self.tube_resolution = 1  # Default is 12; lower for fewer polygons

        self.color_a = "darkgreen"  # a solution color
        self.color_b = "yellow"  # b solution color
        self.color_frame = "black"
        self.color_rigid_arm = "orange"
        self.color_controlled = "red"
        self.color_arm = "purple"
        self.color_table = "lightgrey"
        self.color_workspace = "green"

    def setup_scene(self):

        # Table
        self.Table = pv.Plane(center=(0, 0, 0), direction=(0, 0, 1), i_size=1, j_size=1)
        self.scene.add_mesh(self.Table, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_table, name="Table")

        # Rigid frame
        self.Rigid_frame = pv.Tube((0, 0, 0), self.m_3, radius=self.r_robot, resolution=self.tube_resolution)
        self.scene.add_mesh(self.Rigid_frame, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_frame)

        # Arm

        # Elbow
        self.Elbow = self.create_sphere(self.vec_elbow)
        self.scene.add_mesh(self.Elbow, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_arm, name="Elbow")

        # Humerus
        self.Humerus = pv.Tube(self.vec_elbow, self.vec_shoulder, radius=self.r_arm, resolution=self.tube_resolution)
        self.scene.add_mesh(self.Humerus, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_arm, name="Humerus")

        # Initial position
        self.Init = self.create_sphere(self.p_0)
        self.scene.add_mesh(self.Init, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_controlled)

        # Motors
        self.M_1 = self.create_sphere(self.m_1)
        self.M_2 = self.create_sphere(self.m_2)
        self.M_3 = self.create_sphere(self.m_3)

        self.scene.add_mesh(self.M_1, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_frame)
        self.scene.add_mesh(self.M_2, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_frame)
        self.scene.add_mesh(self.M_3, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_frame)

        self.light = pv.Light(light_type='scene light', intensity=0.7)  # lumiere venant de la camera

        # Add Lights
        self.scene.add_light(self.light)

        # Initializing meshes

        # Points

        # Controlled point
        self.P = self.create_sphere(self.p[0])
        self.scene.add_mesh(self.P, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_controlled, name="Controlled")

        # Trajectory
        self.Trajectory_arc = pv.PolyData()
        self.Trajectory_arc.points = np.zeros((1, 3))  # Initial dummy point
        # Create a line cell (connectivity)
        self.Trajectory_arc.lines = np.array([1, 0])  # [number_of_points, point_indices...]
        self.trajectory_actor = self.scene.add_mesh(
            self.Trajectory_arc,
            color=self.color_controlled,
            line_width=10,
            name="Trajectory"
        )

        # B joints
        self.b_1 = self.p[0] + (self.d - self.v_1) * self.z_vec
        self.b_2 = self.p[0] + (self.d - self.v_2) * self.z_vec
        self.b_3 = self.p[0] + (self.d - self.v_3) * self.z_vec

        self.B_1 = self.create_sphere(self.b_1)
        self.B_2 = self.create_sphere(self.b_2)
        self.B_3 = self.create_sphere(self.b_3)

        self.scene.add_mesh(self.B_1, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_rigid_arm)
        self.scene.add_mesh(self.B_2, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_rigid_arm)
        self.scene.add_mesh(self.B_3, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_rigid_arm)

        # S points
        self.S_1a = self.create_sphere(self.s_1a[0])
        self.S_1b = self.create_sphere(self.s_1b[0])
        self.S_2a = self.create_sphere(self.s_2a[0])
        self.S_2b = self.create_sphere(self.s_2b[0])
        self.S_3a = self.create_sphere(self.s_3a[0])
        self.S_3b = self.create_sphere(self.s_3b[0])

        self.scene.add_mesh(self.S_1a, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_a)
        self.scene.add_mesh(self.S_1b, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_b)
        self.scene.add_mesh(self.S_2a, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_a)
        self.scene.add_mesh(self.S_2b, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_b)
        self.scene.add_mesh(self.S_3a, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_a)
        self.scene.add_mesh(self.S_3b, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_b)

        # Shoulder
        self.Shoulder = self.create_sphere(self.vec_shoulder)
        self.scene.add_mesh(self.Shoulder, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_arm,
                            name="Shoulder")

        # Rigid arm
        self.Rigid_arm = pv.Tube(self.p[self.frame_count], self.b_3, radius=self.r_robot,
                                 resolution=self.tube_resolution)
        self.scene.add_mesh(self.Rigid_arm, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_rigid_arm)

        # Forearm
        self.Forearm = pv.Tube(self.vec_elbow, self.p[0], radius=self.r_arm, resolution=self.tube_resolution)
        self.scene.add_mesh(self.Forearm, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_arm, name="Forearm")

        # Rigid Links

        # L_LinkIndex_solution_endpoint

        # Motor links
        self.L_11_a_M = pv.Tube(self.m_1, self.s_1a[0], radius=self.r_robot, resolution=self.tube_resolution)
        self.L_11_b_M = pv.Tube(self.m_1, self.s_1b[0], radius=self.r_robot, resolution=self.tube_resolution)
        self.L_21_a_M = pv.Tube(self.m_2, self.s_2a[0], radius=self.r_robot, resolution=self.tube_resolution)
        self.L_21_b_M = pv.Tube(self.m_2, self.s_2b[0], radius=self.r_robot, resolution=self.tube_resolution)
        self.L_31_a_M = pv.Tube(self.m_3, self.s_3a[0], radius=self.r_robot, resolution=self.tube_resolution)
        self.L_31_b_M = pv.Tube(self.m_3, self.s_3b[0], radius=self.r_robot, resolution=self.tube_resolution)

        self.scene.add_mesh(self.L_11_a_M, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_a)
        self.scene.add_mesh(self.L_11_b_M, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_b)
        self.scene.add_mesh(self.L_21_a_M, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_a)
        self.scene.add_mesh(self.L_21_b_M, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_b)
        self.scene.add_mesh(self.L_31_a_M, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_a)
        self.scene.add_mesh(self.L_31_b_M, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_b)

        # B joints links
        self.L_12_a_B = pv.Tube(self.b_1, self.s_1a[0], radius=self.r_robot, resolution=self.tube_resolution)
        self.L_12_b_B = pv.Tube(self.b_1, self.s_1b[0], radius=self.r_robot, resolution=self.tube_resolution)
        self.L_22_a_B = pv.Tube(self.b_2, self.s_2a[0], radius=self.r_robot, resolution=self.tube_resolution)
        self.L_22_b_B = pv.Tube(self.b_2, self.s_2b[0], radius=self.r_robot, resolution=self.tube_resolution)
        self.L_32_a_B = pv.Tube(self.b_3, self.s_3a[0], radius=self.r_robot, resolution=self.tube_resolution)
        self.L_32_b_B = pv.Tube(self.b_3, self.s_3b[0], radius=self.r_robot, resolution=self.tube_resolution)

        self.scene.add_mesh(self.L_12_a_B, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_a)
        self.scene.add_mesh(self.L_12_b_B, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_b)
        self.scene.add_mesh(self.L_22_a_B, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_a)
        self.scene.add_mesh(self.L_22_b_B, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_b)
        self.scene.add_mesh(self.L_32_a_B, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_a)
        self.scene.add_mesh(self.L_32_b_B, ambient=0.4, diffuse=0.5, specular=0.8, color=self.color_b)

        self.translation_vector = self.z_shift * self.z_vec
        self.transform = vtk.vtkTransform()
        self.transform.Translate(*self.translation_vector)

        for actor in self.scene.renderer.actors.values():
            actor.SetUserTransform(self.transform)

    def create_sphere(self, pos):
        pos = np.asarray(pos, dtype=float).flatten()
        return pv.Sphere(radius=self.r_joint, center=tuple(pos), theta_resolution=8, phi_resolution=8)

    def resampling(self, measure=False, t=None):

        if not measure:
            t = t_physical(self.p, self.p_dot)

        t_fps = np.linspace(0, t[- 1], int(self.fps * (t[- 1])))

        # Spline interpolation
        spline_theta = CubicSpline(t, self.theta_arc)
        spline_phi = CubicSpline(t, self.phi_arc)
        spline_p = CubicSpline(t, self.p)

        if not measure:
            spline_p_dot = CubicSpline(t, self.p_dot.T)
            spline_s_1a = CubicSpline(t, self.s_1a)
            spline_s_1b = CubicSpline(t, self.s_1b)
            spline_s_2a = CubicSpline(t, self.s_2a)
            spline_s_2b = CubicSpline(t, self.s_2b)
            spline_s_3a = CubicSpline(t, self.s_3a)
            spline_s_3b = CubicSpline(t, self.s_3b)

        # New arcs
        self.theta_arc = spline_theta(t_fps)
        self.phi_arc = spline_phi(t_fps)
        self.p = spline_p(t_fps)

        if not measure:
            self.p_dot = (spline_p_dot(t_fps)).T
            self.s_1a = spline_s_1a(t_fps)
            self.s_1b = spline_s_1b(t_fps)
            self.s_2a = spline_s_2a(t_fps)
            self.s_2b = spline_s_2b(t_fps)
            self.s_3a = spline_s_3a(t_fps)
            self.s_3b = spline_s_3b(t_fps)

        self.N_points = len(t_fps.tolist())
        self.max_frames = self.N_points

    def start_animation(self, measure=False):
        self.stop_anim = False
        self.frame_count = 0
        self.max_frames = self.N_points

        try:
            self.timer.timeout.disconnect()
        except TypeError:
            pass

        if not measure:
            self.timer.timeout.connect(self.update_animation)
        else:
            self.timer.timeout.connect(self.update_animation_measure)
        self.timer.start(int(1 / self.fps * 1000))  # ms per frame

    def frame(self, frame_count):
        # New meshes

        # Controlled point
        new_P = self.create_sphere(self.p[frame_count])

        # B joints
        self.b_1 = self.p[frame_count] + (self.d - self.v_1) * self.z_vec
        self.b_2 = self.p[frame_count] + (self.d - self.v_2) * self.z_vec
        self.b_3 = self.p[frame_count] + (self.d - self.v_3) * self.z_vec

        new_B_1 = self.create_sphere(self.b_1)
        new_B_2 = self.create_sphere(self.b_2)
        new_B_3 = self.create_sphere(self.b_3)

        # S points
        new_S_1a = self.create_sphere(self.s_1a[frame_count])
        new_S_1b = self.create_sphere(self.s_1b[frame_count])
        new_S_2a = self.create_sphere(self.s_2a[frame_count])
        new_S_2b = self.create_sphere(self.s_2b[frame_count])
        new_S_3a = self.create_sphere(self.s_3a[frame_count])
        new_S_3b = self.create_sphere(self.s_3b[frame_count])

        # Rigid arm
        new_Rigid_arm = pv.Tube(self.p[frame_count], self.b_3, radius=self.r_robot, resolution=self.tube_resolution)

        # Forearm
        new_Forearm = pv.Tube(self.vec_elbow, self.p[frame_count], radius=self.r_arm, resolution=self.tube_resolution)

        # Rigid Links

        # L_LinkIndex_solution_endpoint

        # Motor links
        new_L_11_a_M = pv.Tube(self.m_1, self.s_1a[frame_count], radius=self.r_robot, resolution=self.tube_resolution)
        new_L_11_b_M = pv.Tube(self.m_1, self.s_1b[frame_count], radius=self.r_robot, resolution=self.tube_resolution)
        new_L_21_a_M = pv.Tube(self.m_2, self.s_2a[frame_count], radius=self.r_robot, resolution=self.tube_resolution)
        new_L_21_b_M = pv.Tube(self.m_2, self.s_2b[frame_count], radius=self.r_robot, resolution=self.tube_resolution)
        new_L_31_a_M = pv.Tube(self.m_3, self.s_3a[frame_count], radius=self.r_robot, resolution=self.tube_resolution)
        new_L_31_b_M = pv.Tube(self.m_3, self.s_3b[frame_count], radius=self.r_robot, resolution=self.tube_resolution)

        # B joints links
        new_L_12_a_B = pv.Tube(self.b_1, self.s_1a[frame_count], radius=self.r_robot, resolution=self.tube_resolution)
        new_L_12_b_B = pv.Tube(self.b_1, self.s_1b[frame_count], radius=self.r_robot, resolution=self.tube_resolution)
        new_L_22_a_B = pv.Tube(self.b_2, self.s_2a[frame_count], radius=self.r_robot, resolution=self.tube_resolution)
        new_L_22_b_B = pv.Tube(self.b_2, self.s_2b[frame_count], radius=self.r_robot, resolution=self.tube_resolution)
        new_L_32_a_B = pv.Tube(self.b_3, self.s_3a[frame_count], radius=self.r_robot, resolution=self.tube_resolution)
        new_L_32_b_B = pv.Tube(self.b_3, self.s_3b[frame_count], radius=self.r_robot, resolution=self.tube_resolution)

        # Update meshes

        # Controlled point

        self.P.copy_from(new_P)

        new_points = self.p[:frame_count + 1]

        # Update trajectory points
        self.Trajectory_arc.points = new_points

        # Define line connectivity: [num_points, 0, 1, 2, ..., num_points-1]
        n_points = len(new_points)
        self.Trajectory_arc.lines = np.hstack([[n_points], np.arange(n_points)])

        # Force VTK to update
        self.Trajectory_arc.Modified()  # Update the dataset
        self.trajectory_actor.GetMapper().GetInput().Modified()  # Update mapper

        # B points
        self.B_1.copy_from(new_B_1)
        self.B_2.copy_from(new_B_2)
        self.B_3.copy_from(new_B_3)

        # S points
        self.S_1a.copy_from(new_S_1a)
        self.S_1b.copy_from(new_S_1b)
        self.S_2a.copy_from(new_S_2a)
        self.S_2b.copy_from(new_S_2b)
        self.S_3a.copy_from(new_S_3a)
        self.S_3b.copy_from(new_S_3b)

        # Links

        self.Forearm.copy_from(new_Forearm)
        self.Rigid_arm.copy_from(new_Rigid_arm)

        # Motor links
        self.L_11_a_M.copy_from(new_L_11_a_M)
        self.L_11_b_M.copy_from(new_L_11_b_M)
        self.L_21_a_M.copy_from(new_L_21_a_M)
        self.L_21_b_M.copy_from(new_L_21_b_M)
        self.L_31_a_M.copy_from(new_L_31_a_M)
        self.L_31_b_M.copy_from(new_L_31_b_M)

        # B joints links
        self.L_12_a_B.copy_from(new_L_12_a_B)
        self.L_12_b_B.copy_from(new_L_12_b_B)
        self.L_22_a_B.copy_from(new_L_22_a_B)
        self.L_22_b_B.copy_from(new_L_22_b_B)
        self.L_32_a_B.copy_from(new_L_32_a_B)
        self.L_32_b_B.copy_from(new_L_32_b_B)

    def frame_measure(self, frame_count):
        # New meshes

        # Controlled point
        new_P = self.create_sphere(self.p[frame_count])

        # Trajectory
        new_points = self.p[:frame_count + 1]

        # Update trajectory points
        self.Trajectory_arc.points = new_points

        # Define line connectivity: [num_points, 0, 1, 2, ..., num_points-1]
        n_points = len(new_points)
        self.Trajectory_arc.lines = np.hstack([[n_points], np.arange(n_points)])

        # Force VTK to update
        self.Trajectory_arc.Modified()  # Update the dataset
        self.trajectory_actor.GetMapper().GetInput().Modified()  # Update mapper

        # Forearm
        new_Forearm = pv.Tube(self.vec_elbow, self.p[frame_count], radius=self.r_arm, resolution=self.tube_resolution)

        # Controlled point

        self.P.copy_from(new_P)

        self.Forearm.copy_from(new_Forearm)

    def update_animation(self):

        if self.stop_anim:
            self.timer.stop()
            return

        if self.frame_count >= self.max_frames:
            self.timer.stop()
            self.stop_anim = True
            return

        self.frame(self.frame_count)

        self.scene.render()
        self.frame_count += 1

    def update_animation_measure(self):

        if self.stop_anim:
            self.replay_finished.emit()
            self.timer.stop()
            return

        if self.frame_count >= self.max_frames:
            self.timer.stop()
            self.replay_finished.emit()
            self.stop_anim = True
            return

        self.frame_measure(self.frame_count)

        self.scene.render()
        self.frame_count += 1

    def update_scene(self, p_0, phi_arc, theta_arc, p, p_dot, vec_elbow, vec_shoulder, s_1a, s_1b, s_2a, s_2b, s_3a,
                     s_3b, d, v_1,
                     v_2, v_3, m_1, m_2, m_3, z_vec, resolution, fps):

        self.p_0 = p_0
        self.phi_arc = phi_arc
        self.theta_arc = theta_arc
        self.p = p
        self.vec_elbow = vec_elbow
        self.vec_shoulder = vec_shoulder
        self.s_1a = s_1a
        self.s_1b = s_1b
        self.s_2a = s_2a
        self.s_2b = s_2b
        self.s_3a = s_3a
        self.s_3b = s_3b
        self.d = d
        self.v_1 = v_1
        self.v_2 = v_2
        self.v_3 = v_3
        self.m_1 = m_1
        self.m_2 = m_2
        self.m_3 = m_3
        self.z_vec = z_vec
        self.resolution = resolution
        self.fps = fps
        self.p_dot = p_dot

    def update_elbow(self, fullscene=True):

        self.stop_anim = True
        if self.timer.isActive():
            self.timer.stop()

        # Elbow
        new_Elbow = self.create_sphere(self.vec_elbow)
        self.Elbow.copy_from(new_Elbow)

        # Humerus
        new_Humerus = pv.Tube(self.vec_elbow, self.vec_shoulder, radius=self.r_arm, resolution=self.tube_resolution)
        self.Humerus.copy_from(new_Humerus)

        # Shoulder
        new_Shoulder = self.create_sphere(self.vec_shoulder)
        self.Shoulder.copy_from(new_Shoulder)

        if fullscene:
            self.frame(0)

        else:
            self.frame_measure(0)

        self.scene.render()
