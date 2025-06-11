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

    l_arm_proth = cfg["arm"]["l_arm_proth"]

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
    """Convert spherical velocity to Cartesian velocity"""
    # Ensure inputs are arrays for vectorization
    r = np.asarray(r)
    theta = np.asarray(theta)
    phi = np.asarray(phi)
    r_p = np.asarray(r_p)
    theta_p = np.asarray(theta_p)
    phi_p = np.asarray(phi_p)

    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)
    sin_phi = np.sin(phi)
    cos_phi = np.cos(phi)

    # Precompute components
    radial = r_p
    meridional = r * theta_p
    azimutal = r * phi_p * sin_theta

    # Cartesian velocity components
    x_p = (radial * sin_theta * cos_phi +
           meridional * cos_theta * cos_phi -
           azimutal * sin_phi)

    y_p = (radial * sin_theta * sin_phi +
           meridional * cos_theta * sin_phi +
           azimutal * cos_phi)

    z_p = radial * cos_theta - meridional * sin_theta

    return x_p, y_p, z_p


def Spherical_to_cartesian_accel(r, theta, phi, r_p, theta_p, phi_p, r_pp, theta_pp, phi_pp):
    """Convert spherical acceleration to Cartesian acceleration"""
    # Ensure inputs are arrays for vectorization
    r = np.asarray(r)
    theta = np.asarray(theta)
    phi = np.asarray(phi)
    r_p = np.asarray(r_p)
    theta_p = np.asarray(theta_p)
    phi_p = np.asarray(phi_p)
    r_pp = np.asarray(r_pp)
    theta_pp = np.asarray(theta_pp)
    phi_pp = np.asarray(phi_pp)

    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)
    sin_phi = np.sin(phi)
    cos_phi = np.cos(phi)
    sin2_theta = sin_theta ** 2

    # Precompute components
    radial = r_pp - r * theta_p ** 2 - r * sin2_theta * phi_p ** 2
    meridional = (r * theta_pp + 2 * r_p * theta_p -
                  r * sin_theta * cos_theta * phi_p ** 2)
    azimutal = (2 * r_p * phi_p * sin_theta +
                2 * r * theta_p * phi_p * cos_theta +
                r * sin_theta * phi_pp)

    # Cartesian acceleration components
    x_pp = (radial * sin_theta * cos_phi +
            meridional * cos_theta * cos_phi -
            azimutal * sin_phi)

    y_pp = (radial * sin_theta * sin_phi +
            meridional * cos_theta * sin_phi +
            azimutal * cos_phi)

    z_pp = radial * cos_theta - meridional * sin_theta

    return x_pp, y_pp, z_pp


def t_physical(p, p_dot, min_speed=1e-10):
    # Calculate segment lengths between consecutive points
    diffs = np.diff(p, axis=0)
    segment_lengths = np.linalg.norm(diffs, axis=1)

    # Calculate speeds at each point
    speeds = np.linalg.norm(p_dot.T, axis=1)

    # Handle case with only one point
    if len(p) == 1:
        return np.array([0.0])

    # Calculate average speeds between points
    avg_speeds = (speeds[:-1] + speeds[1:]) / 2

    # Handle stationary segments (where average speed is near zero)
    # For zero-length segments, dt=0 regardless of speed
    dt = np.zeros_like(segment_lengths)

    # For non-zero segments with valid speed
    non_zero_mask = (segment_lengths > 0) & (avg_speeds > min_speed)
    dt[non_zero_mask] = segment_lengths[non_zero_mask] / avg_speeds[non_zero_mask]

    # For non-zero segments with near-zero speed
    zero_speed_mask = (segment_lengths > 0) & (avg_speeds <= min_speed)
    dt[zero_speed_mask] = segment_lengths[zero_speed_mask] / min_speed

    # Construct time vector
    t = np.concatenate(([0], np.cumsum(dt)))
    return t


def Cart_velocity_ramp(l_arm_proth, theta, phi, theta_arc, phi_arc, omega_max, accel, p):
    # Convert angular paths to numpy arrays
    theta_arc = np.asarray(theta_arc)
    phi_arc = np.asarray(phi_arc)
    N_points = len(theta_arc)

    # Calculate total angular distance along the path
    total_angular_distance = 0.0
    for i in range(1, N_points):
        dtheta = theta_arc[i] - theta_arc[i - 1]
        dphi = phi_arc[i] - phi_arc[i - 1]
        ds = np.sqrt(dtheta ** 2 + (np.sin(theta_arc[i - 1]) * dphi) ** 2)
        total_angular_distance += ds

    # Generate angular speed profile
    t, omega_profile, _ = speed_ramp(accel, omega_max, total_angular_distance, N_points)

    # Pre-calculate trig functions
    sin_theta_arc = np.sin(theta_arc)
    cos_theta_arc = np.cos(theta_arc)

    # Compute path derivatives using central differences
    dtheta = np.zeros(N_points)
    dphi = np.zeros(N_points)

    # First point (forward difference)
    dtheta[0] = theta_arc[1] - theta_arc[0]
    dphi[0] = phi_arc[1] - phi_arc[0]

    # Interior points (central differences)
    for i in range(1, N_points - 1):
        dtheta[i] = (theta_arc[i + 1] - theta_arc[i - 1]) / 2.0
        dphi[i] = (phi_arc[i + 1] - phi_arc[i - 1]) / 2.0

    # Last point (backward difference)
    dtheta[-1] = theta_arc[-1] - theta_arc[-2]
    dphi[-1] = phi_arc[-1] - phi_arc[-2]

    # Compute arc length derivatives
    ds = np.sqrt(dtheta ** 2 + (sin_theta_arc * dphi) ** 2)

    # Avoid division by zero
    ds[ds == 0] = 1e-10

    # Compute unit tangent components
    u_theta = dtheta / ds
    u_phi = (sin_theta_arc * dphi) / ds

    # Compute angular velocities
    theta_p = omega_profile * u_theta
    phi_p = omega_profile * u_phi / np.where(sin_theta_arc > 1e-10, sin_theta_arc, 1)

    # Compute angular accelerations
    theta_pp = np.gradient(theta_p, t)
    phi_pp = np.gradient(phi_p, t)

    # Convert to Cartesian
    r = l_arm_proth
    zeros = np.zeros(N_points)

    # Cartesian velocity
    x_p, y_p, z_p = Spherical_to_cartesian_velocity(
        r, theta_arc, phi_arc, zeros, theta_p, phi_p
    )

    # Cartesian acceleration
    x_pp, y_pp, z_pp = Spherical_to_cartesian_accel(
        r, theta_arc, phi_arc, zeros, theta_p, phi_p, zeros, theta_pp, phi_pp
    )

    # Return results as 2D arrays
    p_dot = np.vstack([x_p, y_p, z_p])
    p_dotdot = np.vstack([x_pp, y_pp, z_pp])

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

