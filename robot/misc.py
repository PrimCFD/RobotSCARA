import os
import json
import threading
import numpy as np
from pathlib import Path

_config_cache = {}
_config_name = "initial"
_config_lock = threading.Lock()


def set_config(name):
    global _config_name
    with _config_lock:
        _config_name = name
        _config_cache.clear()


def load_config():
    with _config_lock:
        if _config_name in _config_cache:
            return _config_cache[_config_name]

        package_root = Path(__file__).parent.parent
        config_path = package_root / "configs" / f"{_config_name}.json"

        if not config_path.exists():
            raise FileNotFoundError(f"Config file not found: {config_path}")

        with open(config_path, 'r') as f:
            cfg = json.load(f)

        _config_cache[_config_name] = cfg
        return cfg


def save_config(new_config=None):
    with _config_lock:
        package_root = Path(__file__).parent.parent
        config_path = package_root / "configs" / f"{_config_name}.json"
        if new_config is None:
            new_config = _config_cache.get(_config_name)
            if new_config is None:
                raise RuntimeError("No config loaded or cached to save.")

        with open(config_path, 'w') as f:
            json.dump(new_config, f, indent=4)

        _config_cache[_config_name] = new_config


def constants(cfg=None):
    if cfg is None:
        cfg = load_config()

    d = cfg["geometry"]["d"]
    v_1, v_2, v_3 = cfg["geometry"]["v"]
    h_1, h_2, h_3 = cfg["geometry"]["h"]
    l_11, l_12 = cfg["geometry"]["l1"]
    l_21, l_22 = cfg["geometry"]["l2"]
    l_31, l_32 = cfg["geometry"]["l3"]

    x_elbow, y_elbow, z_elbow = cfg["elbow"].values()
    vec_elbow = np.array([x_elbow, y_elbow, z_elbow])

    x_0, y_0, z_0 = cfg["initial_pos"].values()
    p_0 = np.array([x_0, y_0, z_0])

    l_arm_proth = np.linalg.norm(p_0 - vec_elbow)

    l_humerus = cfg["arm"]["l_humerus"]
    z_shoulder = cfg["shoulder"]["z"]

    x_shoulder = x_elbow + np.sqrt((l_humerus ** 2 - z_shoulder ** 2) / 2)
    y_shoulder = y_elbow + np.sqrt((l_humerus ** 2 - z_shoulder ** 2) / 2)
    vec_shoulder = np.array([x_shoulder, y_shoulder, z_shoulder])

    m_1 = cfg["mass"]["m_1"]
    m_2 = cfg["mass"]["m_2"]
    m_3 = cfg["mass"]["m_3"]
    m_d_seul = cfg["mass"]["m_d_seul"]
    m_bras = cfg["mass"]["m_bras"]
    m_d = m_d_seul + m_bras

    return (
        d, v_1, v_2, v_3,
        h_1, h_2, h_3,
        l_11, l_21, l_31,
        l_12, l_22, l_32,
        vec_elbow, vec_shoulder,
        l_arm_proth, l_humerus,
        m_1, m_2, m_3, m_d
    )


def angular_dist(theta1, theta2, phi1, phi2):
    return (np.arccos(
        np.sin(theta1) * np.sin(theta2)
        + np.cos(theta1) * np.cos(theta2) * np.cos(
            phi1 - phi2)))


def Spherical_to_cartesian(r, theta, phi, center):
    # Transforms spherical coordinates in cartesian coordinates

    x = r * np.sin(theta) * np.cos(phi) + center[0]  # m
    y = r * np.sin(theta) * np.sin(phi) + center[1]  # m
    z = r * np.cos(theta) + center[2]  # m

    return x, y, z


def speed_ramp(accel, v_max, pos_target, N_points):
    # Time to accelerate to omega_target
    t_accel = v_max / accel
    pos_accel = 0.5 * accel * t_accel ** 2

    if 2 * pos_accel >= pos_target:
        # Triangular profile
        pos_accel = pos_target / 2
        t_accel = np.sqrt(2 * pos_accel / accel)
        v_max = accel * t_accel
        t_total = 2 * t_accel

        t = np.linspace(0, t_total, N_points)
        v = np.piecewise(
            t,
            [t <= t_accel, t > t_accel],
            [lambda t: accel * t,
             lambda t: accel * (t_total - t)]
        )
        a = np.piecewise(
            t,
            [t <= t_accel, t > t_accel],
            [lambda t: accel,
             lambda t: -accel]
        )

    else:
        # Trapezoidal profile
        pos_cruise = pos_target - 2 * pos_accel
        t_cruise = pos_cruise / v_max
        t_total = 2 * t_accel + t_cruise

        t = np.linspace(0, t_total, N_points)
        v = np.piecewise(
            t,
            [t <= t_accel,
             (t > t_accel) & (t <= t_accel + t_cruise),
             t > t_accel + t_cruise],
            [lambda t: accel * t,
             lambda t: v_max,
             lambda t: accel * (t_total - t)]
        )
        a = np.piecewise(
            t,
            [t <= t_accel,
             (t > t_accel) & (t <= t_accel + t_cruise),
             t > t_accel + t_cruise],
            [lambda t: accel,
             lambda t: 0,
             lambda t: -accel]
        )

    return t, v, a


def Proj_velocity_spherical(r, theta, phi, angle_p):
    dtheta_dt = np.gradient(theta)
    dphi_dt = np.gradient(phi)

    norm_spherical = np.sqrt(dtheta_dt ** 2 + (np.sin(theta) * dphi_dt) ** 2)

    u_theta = dtheta_dt / norm_spherical
    u_phi = dphi_dt / norm_spherical

    theta_p = (angle_p / r) * u_theta
    phi_p = (angle_p / (r * np.sin(theta))) * u_phi

    return theta_p, phi_p


def Spherical_to_cartesian_velocity(r, theta, phi, r_p, theta_p, phi_p):
    # Transforms spherical coordinates in cartesian coordinates

    # Local base for readability

    e_rx = np.sin(theta) * np.cos(phi)
    e_ry = np.sin(theta) * np.sin(phi)
    e_rz = np.cos(theta)

    e_thetax = np.cos(theta) * np.cos(phi)
    e_thetay = np.cos(theta) * np.sin(phi)
    e_thetaz = - np.sin(theta)

    e_phix = - np.sin(phi)
    e_phiy = np.cos(phi)

    # Radial
    radial = r_p
    # Meridional
    meridional = r * theta_p
    # Azimutal
    azimutal = r * phi_p * np.sin(theta)

    x_p = radial * e_rx + meridional * e_thetax + azimutal * e_phix  # m.s-1
    y_p = radial * e_ry + meridional * e_thetay + azimutal * e_phiy  # m.s-1
    z_p = radial * e_rz + meridional * e_thetaz  # m.s-1

    return x_p, y_p, z_p


def Spherical_to_cartesian_accel(r, theta, phi, r_p, theta_p, phi_p, r_pp):
    # Transforms spherical coordinates in cartesian coordinates

    # Local base for readability

    e_rx = np.sin(theta) * np.cos(phi)
    e_ry = np.sin(theta) * np.sin(phi)
    e_rz = np.cos(theta)

    e_thetax = np.cos(theta) * np.cos(phi)
    e_thetay = np.cos(theta) * np.sin(phi)
    e_thetaz = - np.sin(theta)

    e_phix = - np.sin(phi)
    e_phiy = np.cos(phi)

    theta_pp = np.gradient(theta_p)
    phi_pp = np.gradient(phi_p)

    # Radial
    radial = r_pp - r * theta_p ** 2 - r * (np.sin(theta)) ** 2 * phi_p ** 2
    # Meridional
    meridional = r * theta_pp + 2 * r_p * theta_p - r * np.sin(
        theta) * np.cos(theta) * phi_p ** 2
    # Azimutal
    azimutal = 2 * r_p * phi_p * np.sin(theta) + 2 * r * theta_p * phi_p * np.cos(
        theta) + r * np.sin(theta) * phi_pp

    x_pp = radial * e_rx + meridional * e_thetax + azimutal * e_phix  # m.s-1
    y_pp = radial * e_ry + meridional * e_thetay + azimutal * e_phiy  # m.s-1
    z_pp = radial * e_rz + meridional * e_thetaz  # m.s-1

    return x_pp, y_pp, z_pp


def t_physical(p, p_dot):
    diffs = np.diff(p, axis=0)
    segment_lengths = np.linalg.norm(diffs, axis=1)

    speeds = np.linalg.norm(p_dot.T, axis=1)
    avg_speeds = (speeds[:-1] + speeds[1:]) / 2
    dt = segment_lengths / avg_speeds
    t = np.concatenate(([0], np.cumsum(dt)))
    return t


def Cart_velocity_ramp(l_arm_proth, theta, phi, theta_arc, phi_arc, omega_max, accel, p):
    N_points = len(theta_arc.tolist())

    pos_target = angular_dist(theta, 0, phi, 0)
    t, vel_profile, accel_profile = speed_ramp(accel, omega_max, pos_target, N_points)

    r = l_arm_proth

    theta_p, phi_p = Proj_velocity_spherical(r, theta_arc, phi_arc, vel_profile)

    x_p, y_p, z_p = Spherical_to_cartesian_velocity(r, theta_arc, phi_arc, 0, theta_p, phi_p)
    x_pp, y_pp, z_pp = Spherical_to_cartesian_accel(r, theta_arc, phi_arc, 0, theta_p, phi_p, 0)

    p_dot = np.array([x_p, y_p, z_p])
    p_dotdot = np.array([x_pp, y_pp, z_pp])

    t = t_physical(p, p_dot)

    return p_dot, p_dotdot, t


def Cartesian_to_spherical(x, y, z, center, N_points):
    # Transforms cartesian coordinates in spherical coordinates
    # /!\ Angles in rad /!\

    x = x - center[0]
    y = y - center[1]
    z = z - center[2]

    r = np.sqrt(x ** 2 + y ** 2 + z ** 2)  # m

    if N_points > 1:
        theta = np.zeros(N_points)
        phi = np.zeros(N_points)
        for i in range(N_points):
            if r[i] != 0:
                theta[i] = np.arctan2(np.sqrt(x[i] ** 2 + y[i] ** 2), z[i])  # rad
            else:
                theta[i] = 0
        for i in range(N_points):
            if y[i] != 0:
                phi[i] = np.arctan2(y[i], x[i])  # rad
            else:
                phi[i] = 0

    else:
        if r != 0:
            theta = np.arctan2(np.sqrt(x ** 2 + y ** 2), z)  # rad
        else:
            theta = 0

        if y != 0:
            phi = np.arctan2(y, x)  # rad
        else:
            phi = 0

    return r, theta, phi


def Rz(theta, N):
    if N > 1:
        P = np.array([[np.cos(theta), -np.sin(theta), 0 * np.ones(N)],
                      [np.sin(theta), np.cos(theta), 0 * np.ones(N)],
                      [0 * np.ones(N), 0 * np.ones(N), 1 * np.ones(N)]])

        return np.einsum('kji', P)
    else:
        return np.array([[np.cos(theta), -np.sin(theta), 0],
                         [np.sin(theta), np.cos(theta), 0],
                         [0, 0, 1]])


def Ry(theta, N):
    if N > 1:
        P = np.array([[np.cos(theta), 0 * np.ones(N), -np.sin(theta)],
                      [0 * np.ones(N), 1 * np.ones(N), 0 * np.ones(N)],
                      [np.sin(theta), 0 * np.ones(N), np.cos(theta)]])
        return np.einsum('kji', P)
    else:
        return np.array([[np.cos(theta), 0, -np.sin(theta)],
                         [0, 1, 0],
                         [np.sin(theta), 0, np.cos(theta)]])


def batch_matmul(A, B):
    return np.einsum('nmp, npq->nmq', A, B, optimize=True)


def batch_matvecmul(A, B):
    return np.einsum('nmp,np->nm', A, B, optimize=True)

def parse_simulation_result(sim_result):
    """
    Parses a list of SIL simulation JSON frames into structured NumPy arrays.

    Args:
        sim_result (List[Dict]): List of frames with keys 't', 'x', 'x_dot', 'theta', 'theta_dot', 'tau'.

    Returns:
        Tuple of numpy arrays:
        - t: shape (N,)
        - x: shape (N, 3)
        - x_dot: shape (N, 3)
        - theta: shape (N, 3)
        - theta_dot: shape (N, 3)
        - tau: shape (N, 3)
    """
    t = np.array([frame['t'] for frame in sim_result])
    x = np.array([frame['x'] for frame in sim_result])
    x_dot = np.array([frame['x_dot'] for frame in sim_result])
    theta = np.array([frame['theta'] for frame in sim_result])
    theta_dot = np.array([frame['theta_dot'] for frame in sim_result])
    tau = np.array([frame['tau'] for frame in sim_result])
    return t, x, x_dot, theta, theta_dot, tau

