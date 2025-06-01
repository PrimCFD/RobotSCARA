import sympy as sp

# Define symbolic variables
theta1, theta2, theta3 = sp.symbols('theta1 theta2 theta3')
x, y, z = sp.symbols('x y z')

# Base positions (symbolic)
a1x, a1y, a1z = sp.symbols('a1x a1y a1z')
a2x, a2y, a2z = sp.symbols('a2x a2y a2z')
a3x, a3y, a3z = sp.symbols('a3x a3y a3z')

# Link lengths (symbolic)
l11, l12 = sp.symbols('l11 l12')  # Leg 1
l21, l22 = sp.symbols('l21 l22')  # Leg 2
l31, l32 = sp.symbols('l31 l32')  # Leg 3

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

# Distal vectors (g_i = c - s_i)
g1x = x - s1x
g1y = y - s1y
g1z = z - s1z

g2x = x - s2x
g2y = y - s2y
g2z = z - s2z

g3x = x - s3x
g3y = y - s3y
g3z = z - s3z

# =====================
# Jacobian Matrix J
# =====================
J = sp.Matrix([
    [g1x, g1y, g1z],  # Leg 1 constraint
    [g2x, g2y, g2z],  # Leg 2 constraint
    [g3x, g3y, g3z]   # Leg 3 constraint
])

# =====================
# Jacobian Matrix K (Diagonal)
# =====================
# Derivatives of proximal joint positions w.r.t joint angles
ds1_dtheta1 = sp.Matrix([-l11*sp.sin(theta1), l11*sp.cos(theta1), 0])
ds2_dtheta2 = sp.Matrix([-l21*sp.sin(theta2), l21*sp.cos(theta2), 0])
ds3_dtheta3 = sp.Matrix([-l31*sp.sin(theta3), 0, l31*sp.cos(theta3)])

# K matrix elements
K11 = sp.Matrix([g1x, g1y, g1z]).dot(ds1_dtheta1)
K22 = sp.Matrix([g2x, g2y, g2z]).dot(ds2_dtheta2)
K33 = sp.Matrix([g3x, g3y, g3z]).dot(ds3_dtheta3)

# Construct diagonal K matrix
K = sp.diag(K11, K22, K33)

# =====================
# Velocity Relationships
# =====================
theta_dot = sp.Matrix([sp.symbols('dtheta1'), sp.symbols('dtheta2'), sp.symbols('dtheta3')])
c_dot = sp.Matrix([sp.symbols('dx'), sp.symbols('dy'), sp.symbols('dz')])

# Primary velocity equation: J·c_dot = K·theta_dot
velocity_eq = sp.Eq(J * c_dot, K * theta_dot)

# =====================
# Output Results
# =====================
print("Jacobian Matrix J (3x3):")
sp.pprint(J)
print("\nJacobian Matrix K (3x3 diagonal):")
sp.pprint(K)
print("\nVelocity Equation: J·c_dot = K·theta_dot")
sp.pprint(velocity_eq)

# Optional: Compute inverse relationships
print("\nInverse Velocity Relationship (theta_dot in terms of c_dot):")
theta_dot_expr = sp.simplify(K.inv() * J * c_dot)
sp.pprint(sp.Eq(theta_dot, theta_dot_expr))

print("\nForward Velocity Relationship (c_dot in terms of theta_dot):")
c_dot_expr = sp.simplify(J.inv() * K * theta_dot)
sp.pprint(sp.Eq(c_dot, c_dot_expr))