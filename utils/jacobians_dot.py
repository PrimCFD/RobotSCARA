import sympy as sp

# Define time-dependent variables
t = sp.symbols('t')
x = sp.Function('x')(t)
y = sp.Function('y')(t)
z = sp.Function('z')(t)
theta1 = sp.Function('theta1')(t)
theta2 = sp.Function('theta2')(t)
theta3 = sp.Function('theta3')(t)

# Define constants
a1x, a1y, a1z = sp.symbols('a1x a1y a1z')
a2x, a2y, a2z = sp.symbols('a2x a2y a2z')
a3x, a3y, a3z = sp.symbols('a3x a3y a3z')
l11, l21, l31 = sp.symbols('l11 l21 l31')  # Proximal link lengths
v_1, v_2, v_3 = sp.symbols('v_1 v_2 v_3')  # Platform z-offsets
d = sp.symbols('d')  # Platform height

# Time derivatives
x_dot = sp.diff(x, t)
y_dot = sp.diff(y, t)
z_dot = sp.diff(z, t)
theta1_dot = sp.diff(theta1, t)
theta2_dot = sp.diff(theta2, t)
theta3_dot = sp.diff(theta3, t)

# =====================
# Proximal Joint Positions (s_i)
# =====================

# Leg 1 (XY-plane motion)
s1x = a1x + l11 * sp.cos(theta1)
s1y = a1y + l11 * sp.sin(theta1)
s1z = a1z  # Fixed Z-position

# Leg 2 (XY-plane motion)
s2x = a2x + l21 * sp.cos(theta2)
s2y = a2y + l21 * sp.sin(theta2)
s2z = a2z  # Fixed Z-position

# Leg 3 (XZ-plane motion)
s3x = a3x + l31 * sp.cos(theta3)
s3y = a3y  # Fixed Y-position
s3z = a3z + l31 * sp.sin(theta3)

# =====================
# Constraint Vectors (g_i = c - s_i)
# =====================
g1x = x - s1x
g1y = y - s1y
g1z = (z + (d - v_1)) - s1z

g2x = x - s2x
g2y = y - s2y
g2z = (z + (d - v_2)) - s2z

g3x = x - s3x
g3y = y - s3y
g3z = (z + (d - v_3)) - s3z

# =====================
# Jacobian Matrix J
# =====================
J = sp.Matrix([
    [g1x, g1y, g1z],
    [g2x, g2y, g2z],
    [g3x, g3y, g3z]
])

# =====================
# Jacobian Matrix K (Diagonal)
# =====================
# Derivatives of proximal joint positions w.r.t joint angles
ds1_dtheta1 = sp.Matrix([-l11*sp.sin(theta1), l11*sp.cos(theta1), 0])
ds2_dtheta2 = sp.Matrix([-l21*sp.sin(theta2), l21*sp.cos(theta2), 0])
ds3_dtheta3 = sp.Matrix([-l31*sp.sin(theta3), 0, l31*sp.cos(theta3)])

# K matrix elements (dot product of g_i and ds_i/dtheta_i)
K11 = sp.Matrix([g1x, g1y, g1z]).dot(ds1_dtheta1)
K22 = sp.Matrix([g2x, g2y, g2z]).dot(ds2_dtheta2)
K33 = sp.Matrix([g3x, g3y, g3z]).dot(ds3_dtheta3)

# Construct diagonal K matrix
K = sp.diag(K11, K22, K33)

# =====================
# Time Derivatives (J_dot and K_dot)
# =====================

# Compute J_dot by differentiating each element
J_dot = sp.Matrix.zeros(3, 3)
for i in range(3):
    for j in range(3):
        J_dot[i, j] = sp.diff(J[i, j], t)

# Compute K_dot by differentiating diagonal elements
K_dot = sp.diag(
    sp.diff(K[0, 0], t),
    sp.diff(K[1, 1], t),
    sp.diff(K[2, 2], t)
)

J_dot_simp = sp.trigsimp(J_dot)
K_dot_simp = sp.trigsimp(K_dot)

# =====================
# Simplify and Display
# =====================
print("Jacobian Matrix J:")
sp.pprint(J)
print("\nJacobian Matrix K:")
sp.pprint(K)
print("\nTime Derivative of J (J_dot):")
sp.pprint(J_dot_simp)
print("\nTime Derivative of K (K_dot):")
sp.pprint(K_dot_simp)