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
a1x, a1y, a2x, a2y, a3x, a3y = sp.symbols('a1x a1y a2x a2y a3x a3y')
l11, l21, l31 = sp.symbols('l11 l21 l31')
v1, v2, v3 = sp.symbols('v1 v2 v3')  # End-effector z-offsets
h1, h2, h3 = sp.symbols('h1 h2 h3')  # Base joint z-positions

# Time derivatives
x_dot = sp.diff(x, t)
y_dot = sp.diff(y, t)
z_dot = sp.diff(z, t)
theta1_dot = sp.diff(theta1, t)
theta2_dot = sp.diff(theta2, t)
theta3_dot = sp.diff(theta3, t)

# =====================
# Constraint Vectors (g_i)
# =====================
g1x = x - a1x - l11*sp.cos(theta1)
g1y = y - a1y - l11*sp.sin(theta1)
g1z = (z + v1) - h1

g2x = x - a2x - l21*sp.cos(theta2)
g2y = y - a2y - l21*sp.sin(theta2)
g2z = (z + v2) - h2

g3x = x - a3x - l31*sp.cos(theta3)
g3y = y - a3y  # Fixed y-position for leg 3
g3z = (z + v3) - (h3 + l31*sp.sin(theta3))

# =====================
# Jacobian Matrix J
# =====================
J = sp.Matrix([
    [g1x, g1y, g1z],
    [g2x, g2y, g2z],
    [g3x, g3y, g3z]
])

# =====================
# Jacobian Matrix K
# =====================
K11 = g1x * (l11*sp.sin(theta1)) + g1y * (-l11*sp.cos(theta1))
K22 = g2x * (l21*sp.sin(theta2)) + g2y * (-l21*sp.cos(theta2))
K33 = g3x * (l31*sp.sin(theta3)) + g3z * (-l31*sp.cos(theta3))

K = sp.diag(K11, K22, K33)

# =====================
# Time Derivatives
# =====================

# Compute J_dot
J_dot = sp.Matrix.zeros(3, 3)
for i in range(3):
    for j in range(3):
        J_dot[i,j] = sp.diff(J[i,j], t).simplify()

# Compute K_dot
K_dot = sp.diag(
    sp.diff(K[0,0], t).simplify(),
    sp.diff(K[1,1], t).simplify(),
    sp.diff(K[2,2], t).simplify()
)

# =====================
# Simplify and Display
# =====================
print("Jacobian Matrix J:")
sp.pprint(J)
print("\nJacobian Matrix K:")
sp.pprint(K)
print("\nTime Derivative of J (J_dot):")
sp.pprint(J_dot)
print("\nTime Derivative of K (K_dot):")
sp.pprint(K_dot)

# Output simplified expressions for C++ implementation
print("\n\n// C++ J_dot Implementation")
print("Eigen::Matrix3d J_dot;")
print(f"J_dot << {sp.ccode(J_dot[0,0])}, {sp.ccode(J_dot[0,1])}, {sp.ccode(J_dot[0,2])},")
print(f"         {sp.ccode(J_dot[1,0])}, {sp.ccode(J_dot[1,1])}, {sp.ccode(J_dot[1,2])},")
print(f"         {sp.ccode(J_dot[2,0])}, {sp.ccode(J_dot[2,1])}, {sp.ccode(J_dot[2,2])};")

print("\n\n// C++ K_dot Implementation")
print("Eigen::Matrix3d K_dot = Eigen::Matrix3d::Zero();")
print(f"K_dot(0,0) = {sp.ccode(K_dot[0,0])};")
print(f"K_dot(1,1) = {sp.ccode(K_dot[1,1])};")
print(f"K_dot(2,2) = {sp.ccode(K_dot[2,2])};")