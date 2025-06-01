import sympy as sp

# Time variable
t = sp.symbols('t')

# Position variables as functions of time
x = sp.Function('x')(t)
y = sp.Function('y')(t)
z = sp.Function('z')(t)
theta1 = sp.Function('theta1')(t)
theta2 = sp.Function('theta2')(t)
theta3 = sp.Function('theta3')(t)

# First derivatives
dx = sp.diff(x, t)
dy = sp.diff(y, t)
dz = sp.diff(z, t)
dtheta1 = sp.diff(theta1, t)
dtheta2 = sp.diff(theta2, t)
dtheta3 = sp.diff(theta3, t)

# Second derivatives
ddx = sp.diff(dx, t)
ddy = sp.diff(dy, t)
ddz = sp.diff(dz, t)
ddtheta1 = sp.diff(dtheta1, t)
ddtheta2 = sp.diff(dtheta2, t)
ddtheta3 = sp.diff(dtheta3, t)

# System parameters
params = {
    'm_plat': sp.symbols('m_p'),
    'm_dist': sp.symbols('m_d'),
    'l11': sp.symbols('l_{11}'),
    'l21': sp.symbols('l_{21}'),
    'l31': sp.symbols('l_{31}'),
    'g': sp.symbols('g'),
    'I_prox': sp.symbols('I_{prox}'),
    'a1x': sp.symbols('a1x'),
    'a1y': sp.symbols('a1y'),
    'a2x': sp.symbols('a2x'),
    'a2y': sp.symbols('a2y'),
    'a3x': sp.symbols('a3x'),
    'a3y': sp.symbols('a3y'),
    'h1': sp.symbols('h1'),
    'h2': sp.symbols('h2'),
    'h3': sp.symbols('h3'),
    'v1': sp.symbols('v1'),
    'v2': sp.symbols('v2'),
    'v3': sp.symbols('v3'),
    # New parameters for forearm
    'm_arm': sp.symbols('m_{arm}'),
    'l_arm': sp.symbols('l_{arm}'),
    'elbow_x': sp.symbols('elbow_x'),
    'elbow_y': sp.symbols('elbow_y'),
    'elbow_z': sp.symbols('elbow_z'),
}

# =========================================================
# FOREARM ROD MODELING USING EPM METHOD
# =========================================================

# End-effector position
p = sp.Matrix([x, y, z])

# Elbow position (fixed)
elbow = sp.Matrix([
    params['elbow_x'],
    params['elbow_y'],
    params['elbow_z']
])

# Vector from elbow to end-effector
r_arm = p - elbow

# Constraint: forearm length must be constant
# This will be used for dynamics but not computed here
forearm_constraint = r_arm.dot(r_arm) - params['l_arm']**2

# Forearm kinetic energy using EPM method
# Mass distribution: 1/3 at elbow (fixed), 2/3 at end-effector (moving)
# This matches the kinetic energy of a slender rod rotating about one end
m_eff_kin = (2/3) * params['m_arm']  # Effective mass at end-effector

# Forearm potential energy
# Center of mass is midway between elbow and end-effector
com_z = (elbow[2] + z)/2
V_arm = params['m_arm'] * params['g'] * com_z

# =========================================================
# ROBOT KINEMATICS
# =========================================================

def compute_J(x, y, z, theta1, theta2, theta3, params):
    """Symbolic computation of J matrix"""
    J = sp.zeros(3, 3)

    # Leg 1
    J[0, 0] = -params['a1x'] - params['l11'] * sp.cos(theta1) + x
    J[0, 1] = -params['a1y'] - params['l11'] * sp.sin(theta1) + y
    J[0, 2] = -params['h1'] + params['v1'] + z

    # Leg 2
    J[1, 0] = -params['a2x'] - params['l21'] * sp.cos(theta2) + x
    J[1, 1] = -params['a2y'] - params['l21'] * sp.sin(theta2) + y
    J[1, 2] = -params['h2'] + params['v2'] + z

    # Leg 3
    J[2, 0] = -params['a3x'] - params['l31'] * sp.cos(theta3) + x
    J[2, 1] = -params['a3y'] + y
    J[2, 2] = -params['h3'] - params['l31'] * sp.sin(theta3) + params['v3'] + z

    return J

def compute_K(x, y, z, theta1, theta2, theta3, params):
    """Symbolic computation of K matrix"""
    K = sp.zeros(3, 3)

    # Leg 1
    term1x = -params['a1x'] - params['l11'] * sp.cos(theta1) + x
    term1y = -params['a1y'] - params['l11'] * sp.sin(theta1) + y
    K[0, 0] = params['l11'] * (term1x * sp.sin(theta1) - term1y * sp.cos(theta1))

    # Leg 2
    term2x = -params['a2x'] - params['l21'] * sp.cos(theta2) + x
    term2y = -params['a2y'] - params['l21'] * sp.sin(theta2) + y
    K[1, 1] = params['l21'] * (term2x * sp.sin(theta2) - term2y * sp.cos(theta2))

    # Leg 3
    term3x = -params['a3x'] - params['l31'] * sp.cos(theta3) + x
    term3z = -params['h3'] - params['l31'] * sp.sin(theta3) + params['v3'] + z
    K[2, 2] = params['l31'] * (term3x * sp.sin(theta3) - term3z * sp.cos(theta3))

    return K

# Compute symbolic matrices
J = compute_J(x, y, z, theta1, theta2, theta3, params)
K = compute_K(x, y, z, theta1, theta2, theta3, params)

# Display results
print("Symbolic J matrix:")
sp.pprint(J)
print("\nSymbolic K matrix:")
sp.pprint(K)

# Proximal endpoint velocities
v_B1 = sp.Matrix([
    -params['l11'] * sp.sin(theta1) * dtheta1,
    params['l11'] * sp.cos(theta1) * dtheta1,
    0
])

v_B2 = sp.Matrix([
    -params['l21'] * sp.sin(theta2) * dtheta2,
    params['l21'] * sp.cos(theta2) * dtheta2,
    0
])

v_B3 = sp.Matrix([
    -params['l31'] * sp.sin(theta3) * dtheta3,
    0,
    params['l31'] * sp.cos(theta3) * dtheta3
])

# Platform velocity
v_C = sp.Matrix([dx, dy, dz])

# =========================================================
# EPM METHOD FOR ROBOT + FOREARM
# =========================================================

# Robot kinetic energy using EPM method
# Each distal link: 1/2 mass at each endpoint
# Distal mass at B contributes to proximal link rotational inertia
I_added = (params['m_dist']/2) * (params['l11']**2 * dtheta1**2 +
                                  params['l21']**2 * dtheta2**2 +
                                  params['l31']**2 * dtheta3**2)

# Proximal kinetic energy (includes original inertia + added point masses)
T_prox = 0.5 * params['I_prox'] * (dtheta1**2 + dtheta2**2 + dtheta3**2) + 0.5 * I_added

# Platform kinetic energy (includes original mass + distal masses + forearm mass)
platform_mass = params['m_plat'] + 3 * (params['m_dist']/2) + m_eff_kin
T_plat = 0.5 * platform_mass * v_C.dot(v_C)

# Total kinetic energy (robot + forearm)
T_total = T_prox + T_plat

# Potential energy (robot + forearm)
V_robot = params['g'] * (params['m_plat'] * z + params['m_dist'] * z)
V_total = V_robot + V_arm

# Generalized coordinates (now includes constraint)
q = sp.Matrix([theta1, theta2, theta3, x, y, z])
dq = sp.Matrix([dtheta1, dtheta2, dtheta3, dx, dy, dz])

# Mass matrix
M = sp.zeros(6, 6)
for i in range(6):
    for j in range(6):
        M[i, j] = sp.diff(sp.diff(T_total, dq[i]), dq[j])

# Simplify mass matrix
M = sp.trigsimp(M)

# Gravity vector
G = sp.Matrix([[sp.diff(V_total, var)] for var in q])
G = sp.trigsimp(G)

# Coriolis matrix
C = sp.zeros(6, 6)
for i in range(6):
    for j in range(6):
        for k in range(6):
            C[i, j] += (sp.diff(M[i, j], q[k]) +
                        sp.diff(M[i, k], q[j]) -
                        sp.diff(M[k, j], q[i])) * dq[k] / 2
C = sp.trigsimp(C)

print("Complete matrices" + '\n')
print("M" + '\n')
sp.pprint(M)
print("C" + '\n')
sp.pprint(C)
print("G" + '\n')
sp.pprint(G)

