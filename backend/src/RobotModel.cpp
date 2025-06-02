#include "RobotModel.hpp"
#include "HardcodedParams.hpp"
#include <iostream>
#include <cmath>
#include <tuple>
#include <stdexcept>
#include <limits>

RobotDynamics::RobotDynamics() : initialized_(false) {
}

RobotDynamics::RobotDynamics(const RobotParams& params) 
    : params_(params), initialized_(true) {}

void RobotDynamics::loadHardcodedParams() {
    ::loadHardcodedParams(params_);
    initialized_ = true;
}

void RobotDynamics::setParameters(const RobotParams& params) {
    params_ = params;
    initialized_ = true;
}

Eigen::Matrix3d RobotDynamics::computeK(const Eigen::Vector3d& pos, 
                                        const Eigen::Vector3d& theta) const {

    if (!initialized_) std::cerr<<"Not initialized"<<std::endl;

    
    double x = pos(0), y = pos(1), z = pos(2);
    double th1 = theta(0), th2 = theta(1), th3 = theta(2);
    
    Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
    
    // Leg 1
    double term1x = -params_.a1x - params_.l_11*cos(th1) + x;
    double term1y = -params_.a1y - params_.l_11*sin(th1) + y;
    K(0,0) = params_.l_11 * (term1x*sin(th1) - term1y*cos(th1));
    
    // Leg 2
    double term2x = -params_.a2x - params_.l_21*cos(th2) + x;
    double term2y = -params_.a2y - params_.l_21*sin(th2) + y;
    K(1,1) = params_.l_21 * (term2x*sin(th2) - term2y*cos(th2));
    
    // Leg 3
    double term3x = -params_.a3x - params_.l_31*cos(th3) + x;
    double term3z = -params_.h_3 - params_.l_31*sin(th3) + params_.v_3 + z;
    K(2,2) = params_.l_31 * (term3x*sin(th3) - term3z*cos(th3));
    
    return K;
}

Eigen::Matrix3d RobotDynamics::computeJ(const Eigen::Vector3d& pos, 
                                        const Eigen::Vector3d& theta) const {

    if (!initialized_) std::cerr<<"Not initialized"<<std::endl;

    
    double x = pos(0), y = pos(1), z = pos(2);
    double th1 = theta(0), th2 = theta(1), th3 = theta(2);
    
    Eigen::Matrix3d J;
    
    // Leg 1
    J(0,0) = -params_.a1x - params_.l_11*cos(th1) + x;
    J(0,1) = -params_.a1y - params_.l_11*sin(th1) + y;
    J(0,2) = -params_.h_1 + params_.v_1 + z;
    
    // Leg 2
    J(1,0) = -params_.a2x - params_.l_21*cos(th2) + x;
    J(1,1) = -params_.a2y - params_.l_21*sin(th2) + y;
    J(1,2) = -params_.h_2 + params_.v_2 + z;
    
    // Leg 3
    J(2,0) = -params_.a3x - params_.l_31*cos(th3) + x;
    J(2,1) = -params_.a3y + y;
    J(2,2) = -params_.h_3 - params_.l_31*sin(th3) + params_.v_3 + z;
    
    return J;
}

Eigen::Matrix3d RobotDynamics::computeKDot(const Eigen::Vector3d& pos,
                                           const Eigen::Vector3d& vel,
                                           const Eigen::Vector3d& theta,
                                           const Eigen::Vector3d& theta_dot) const {
    double x = pos(0), y = pos(1), z = pos(2);
    double xd = vel(0), yd = vel(1), zd = vel(2);
    double th1 = theta(0), th2 = theta(1), th3 = theta(2);
    double th1d = theta_dot(0), th2d = theta_dot(1), th3d = theta_dot(2);
    
    Eigen::Matrix3d K_dot = Eigen::Matrix3d::Zero();
    
    // Leg 1
    K_dot(0,0) = params_.l_11 * (
        (-params_.a1x*cos(th1) - params_.a1y*sin(th1) + 
         x*cos(th1) + y*sin(th1)) * th1d +
        sin(th1)*xd - cos(th1)*yd
    );
    
    // Leg 2
    K_dot(1,1) = params_.l_21 * (
        (-params_.a2x*cos(th2) - params_.a2y*sin(th2) + 
         x*cos(th2) + y*sin(th2)) * th2d +
        sin(th2)*xd - cos(th2)*yd
    );
    
    // Leg 3
    K_dot(2,2) = params_.l_31 * (
        (-params_.a3x*cos(th3) - params_.h_3*sin(th3) + params_.v_3*sin(th3) + 
         x*cos(th3) + z*sin(th3)) * th3d +
        sin(th3)*xd - cos(th3)*zd
    );
    
    return K_dot;
}

Eigen::Matrix3d RobotDynamics::computeJDot(const Eigen::Vector3d& vel,
                                           const Eigen::Vector3d& theta,
                                           const Eigen::Vector3d& theta_dot) const {
    double xd = vel(0), yd = vel(1), zd = vel(2);
    double th1 = theta(0), th2 = theta(1), th3 = theta(2);
    double th1d = theta_dot(0), th2d = theta_dot(1), th3d = theta_dot(2);
    
    Eigen::Matrix3d J_dot;
    
    // Leg 1
    J_dot(0,0) = params_.l_11*sin(th1)*th1d + xd;
    J_dot(0,1) = -params_.l_11*cos(th1)*th1d + yd;
    J_dot(0,2) = zd;
    
    // Leg 2
    J_dot(1,0) = params_.l_21*sin(th2)*th2d + xd;
    J_dot(1,1) = -params_.l_21*cos(th2)*th2d + yd;
    J_dot(1,2) = zd;
    
    // Leg 3
    J_dot(2,0) = params_.l_31*sin(th3)*th3d + xd;
    J_dot(2,1) = yd;
    J_dot(2,2) = -params_.l_31*cos(th3)*th3d + zd;
    
    return J_dot;
}

Eigen::Vector3d RobotDynamics::computeMotorSpeed(const Eigen::Vector3d& pos,
                                                const Eigen::Vector3d& vel,
                                                const Eigen::Vector3d& theta) const {
    if (!initialized_) std::cerr<<"Not initialized"<<std::endl;

    
    Eigen::Matrix3d K = computeK(pos, theta);
    Eigen::Matrix3d J = computeJ(pos, theta);
    
    Eigen::Matrix3d K_inv = dampedPseudoInverse(K);
    Eigen::Matrix3d Jacobian_prod = K_inv * J;
    return Jacobian_prod * vel;
}

Eigen::Vector3d RobotDynamics::computeMotorAccel(const Eigen::Vector3d& pos,
                                                const Eigen::Vector3d& vel,
                                                const Eigen::Vector3d& accel,
                                                const Eigen::Vector3d& theta,
                                                const Eigen::Vector3d& theta_dot) const {
    if (!initialized_) std::cerr<<"Not initialized"<<std::endl;

    
    Eigen::Matrix3d J = computeJ(pos, theta);
    Eigen::Matrix3d J_dot = computeJDot(vel, theta, theta_dot);
    Eigen::Matrix3d K = computeK(pos, theta);
    Eigen::Matrix3d K_dot = computeKDot(pos, vel, theta, theta_dot);
    
    Eigen::Vector3d J_1 = J * accel;
    Eigen::Vector3d J_2 = J_dot * vel;
    Eigen::Vector3d J_3 = K_dot * theta_dot;
    Eigen::Vector3d J_sum = J_1 + J_2 - J_3;
    
    Eigen::Matrix3d K_inv = dampedPseudoInverse(K);
    return K_inv * J_sum;
}

Eigen::Matrix3d RobotDynamics::computeMassMatrix(const Eigen::Vector3d& theta, 
                                                 const Eigen::Vector3d& pos) const {
    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    
    // Proximal links with added distal point masses
    M(0,0) = (params_.m_11/4.0 + params_.m_12/2.0) * params_.l_11 * params_.l_11;
    M(1,1) = (params_.m_21/4.0 + params_.m_22/2.0) * params_.l_21 * params_.l_21;
    M(2,2) = (params_.m_31/4.0 + params_.m_32/2.0) * params_.l_31 * params_.l_31;

        // Add forearm inertia using 1/3-2/3 distribution
    double I_arm = (1.0/3.0) * params_.m_arm * params_.l_arm_proth * params_.l_arm_proth;
    Eigen::Matrix3d J = computeJ(pos, theta);
    Eigen::Matrix3d K = computeK(pos, theta);
    Eigen::Matrix3d Jacob = dampedPseudoInverse(J) * K;
    
    // Add platform mass contribution (point mass at end effector)
    M += Jacob.transpose() * (params_.m_d + I_arm/(params_.l_arm_proth * params_.l_arm_proth)) * Jacob;

    return M;
}

Eigen::Vector3d RobotDynamics::computeGravity(const Eigen::Vector3d& theta, const Eigen::Vector3d& pos) const {
    double th1 = theta(0), th2 = theta(1), th3 = theta(2);
    double x = pos(0), y = pos(1), z = pos(2);
    
    // Leg 3 has different orientation (xz-plane motion)
    double G3 = (params_.m_31/2.0 + params_.m_32 + params_.m_12/2.0 + params_.m_12/2.0) * GRAVITY * params_.l_31 * sin(th3);

    // Add forearm gravity (distributed 50/50 between endpoints)
    // Elbow contribution is constant (z_shoulder) so only end effector contributes
    G3 += (params_.m_arm/2.0) * GRAVITY * z;
    
    // Add platform gravity at end effector
    G3 += params_.m_d * GRAVITY * z;
              
    return Eigen::Vector3d(0, 0, G3);
}

Eigen::Vector3d RobotDynamics::computeForwardDynamics(
    const Eigen::Vector3d& theta,
    const Eigen::Vector3d& theta_dot,
    const Eigen::Vector3d& torque,
    const Eigen::Vector3d& pos,
    const Eigen::Vector3d& vel
) const {
    if (!initialized_) std::cerr<<"Not initialized"<<std::endl;


    // Compute dynamics terms
    Eigen::Matrix3d M = computeMassMatrix(theta, pos);
    Eigen::Vector3d G = computeGravity(theta, pos);
    
    // Friction model
    Eigen::Vector3d friction = -0.1 * theta_dot;

    Eigen::Matrix3d M_inv = dampedPseudoInverse(M);

    return M_inv * (torque - G + friction);
}

Eigen::Vector3d RobotDynamics::computeTorque(
    const Eigen::Vector3d& pos,
    const Eigen::Vector3d& vel,
    const Eigen::Vector3d& accel,
    const Eigen::Vector3d& theta,
    const Eigen::Vector3d& theta_dot,
    const Eigen::Vector3d& theta_ddot
) const {
    // Compute basic torques
    Eigen::Matrix3d M = computeMassMatrix(theta, pos);
    Eigen::Vector3d G = computeGravity(theta, pos);
    Eigen::Vector3d tau_e = M * theta_ddot + G;

    // Friction model
    Eigen::Vector3d friction = -0.1 * theta_dot;
    
    return tau_e + friction;
}

RobotDynamics::DynamicsResult RobotDynamics::computeInverseDynamics(
    const Eigen::Vector3d& pos,
    const Eigen::Vector3d& vel,
    const Eigen::Vector3d& accel,
    const Eigen::Vector3d& theta) const {
    
    DynamicsResult result;
    result.success = false;
    
    if (!initialized_) std::cerr<<"Not initialized"<<std::endl;

    
    try {
        result.theta_dot = computeMotorSpeed(pos, vel, theta);
        result.theta_ddot = computeMotorAccel(pos, vel, accel, theta, result.theta_dot);
        result.torque = computeTorque(pos, vel, accel, theta, result.theta_dot, result.theta_ddot);
        result.success = true;
    }
    catch (const std::exception& e) {
        std::cerr<<"Computation error: "<<e.what();
    }
    
    return result;
}

Eigen::Matrix3d RobotDynamics::dampedPseudoInverse(
    const Eigen::Matrix3d& matrix, 
    double lambda_tikhonov,
    double epsilon_diag
) const {
    // Diagonal matrix handling
    bool is_diagonal = true;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (i != j && std::abs(matrix(i, j)) > 1e-12) {
                is_diagonal = false;
                break;
            }
        }
        if (!is_diagonal) break;
    }

    if (is_diagonal) {
        Eigen::Matrix3d inv = Eigen::Matrix3d::Zero();
        for (int i = 0; i < 3; ++i) {
            double val = matrix(i, i);
            inv(i, i) = (std::abs(val) < epsilon_diag) ? 
                        1.0 / (val + epsilon_diag) : 
                        1.0 / val;
        }
        return inv;
    } else {
        Eigen::Matrix3d JJT = matrix * matrix.transpose();
        Eigen::Matrix3d damping = (lambda_tikhonov * lambda_tikhonov) * Eigen::Matrix3d::Identity();
        Eigen::Matrix3d JJT_damped = JJT + damping;
        Eigen::Matrix3d inv_JJT_damped = JJT_damped.inverse();
        return matrix.transpose() * inv_JJT_damped;
    }
}

RobotDynamics::IKSolution RobotDynamics::invKineSinglePoint(
    const Eigen::Vector3d& p, 
    const Eigen::Vector3d& theta_prev) const 
{
    using namespace std;
    if (!initialized_) {
        IKSolution result;
        result.theta = theta_prev;
        result.valid = false;
        return result;
    }
    
    constexpr double PI = 3.14159265358979323846;
    constexpr double JOINT_LIMIT = PI - 0.1;
    constexpr size_t MAX_SOLUTIONS_PER_LEG = 2;
    constexpr size_t MAX_CANDIDATES = 8;  // 2^3 legs

    // Geometric parameters
    const double d = params_.d;
    const double l11 = params_.l_11, l12 = params_.l_12;
    const double l21 = params_.l_21, l22 = params_.l_22;
    const double l31 = params_.l_31, l32 = params_.l_32;
    const double v1 = params_.v_1, v2 = params_.v_2, v3 = params_.v_3;
    const double h1 = params_.h_1, h2 = params_.h_2, h3 = params_.h_3;

    // Anchor points
    const Eigen::Vector3d a1(params_.a1x, params_.a1y, h1);
    const Eigen::Vector3d a2(params_.a2x, params_.a2y, h2);
    const Eigen::Vector3d a3(params_.a3x, params_.a3y, h3);

    // Distal points
    const Eigen::Vector3d b1 = p + Eigen::Vector3d(0, 0, d - v1);
    const Eigen::Vector3d b2 = p + Eigen::Vector3d(0, 0, d - v2);
    const Eigen::Vector3d b3 = p + Eigen::Vector3d(0, 0, d - v3);

    // Calculate coefficients
    auto computeC = [](const Eigen::Vector3d& a, const Eigen::Vector3d& b, double l1, double l2) {
        return (l2*l2 - l1*l1 - (a - b).squaredNorm()) / l1;
    };

    // Leg 1 coefficients
    const double A1 = 2*(b1.y() - a1.y());
    const double B1 = 2*(b1.x() - a1.x());
    const double C1 = computeC(a1, b1, l11, l12);
    
    // Leg 2 coefficients
    const double A2 = 2*(b2.y() - a2.y());
    const double B2 = 2*(b2.x() - a2.x());
    const double C2 = computeC(a2, b2, l21, l22);
    
    // Leg 3 coefficients
    const double A3 = 2*(b3.z() - a3.z());
    const double B3 = 2*(a3.x() - b3.x());
    const double C3 = computeC(a3, b3, l31, l32);

    // Solve for t parameters
    auto solveLeg = [](double A, double B, double C, int leg_num, double solutions[MAX_SOLUTIONS_PER_LEG]) -> size_t {
        const double denom = A + C;
        size_t solution_count = 0;
        
        if (fabs(denom) < 1e-8) {
            return 0;
        }
        
        const double discriminant = B*B - (A + C)*(C - A);
        if (discriminant < 0) {
            return 0;
        }

        const double sqrt_disc = sqrt(discriminant);
        solutions[solution_count++] = (B + sqrt_disc) / denom;
        if (fabs(sqrt_disc) > 1e-8) {
            solutions[solution_count++] = (B - sqrt_disc) / denom;
        }
        return solution_count;
    };

    // Get all possible t solutions using fixed arrays
    double t1_sol[MAX_SOLUTIONS_PER_LEG];
    size_t t1_count = solveLeg(A1, B1, C1, 1, t1_sol);
    
    double t2_sol[MAX_SOLUTIONS_PER_LEG];
    size_t t2_count = solveLeg(A2, B2, C2, 2, t2_sol);
    
    double t3_sol[MAX_SOLUTIONS_PER_LEG];
    size_t t3_count = solveLeg(A3, B3, C3, 3, t3_sol);

    // Prepare solution storage
    IKSolution best_solution;
    best_solution.valid = false;
    double min_joint_dist = numeric_limits<double>::max();
    double min_pos_error = numeric_limits<double>::max();
    
    Eigen::Vector3d candidates[MAX_CANDIDATES];
    size_t candidate_count = 0;

    // Generate all candidate solutions
    for (size_t i1 = 0; i1 < t1_count; i1++) {
        for (size_t i2 = 0; i2 < t2_count; i2++) {
            for (size_t i3 = 0; i3 < t3_count; i3++) {
                double th1 = 2 * atan(t1_sol[i1]);
                double th2 = 2 * atan(t2_sol[i2]);
                double th3 = 2 * atan(t3_sol[i3]);
                
                // Check symmetric joint limits
                bool limits_ok = (th1 >= -JOINT_LIMIT && th1 <= JOINT_LIMIT) &&
                                 (th2 >= -JOINT_LIMIT && th2 <= JOINT_LIMIT) &&
                                 (th3 >= -JOINT_LIMIT && th3 <= JOINT_LIMIT);
                if (!limits_ok) continue;
                
                // Store in fixed array
                if (candidate_count < MAX_CANDIDATES) {
                    candidates[candidate_count++] = Eigen::Vector3d(th1, th2, th3);
                }
            }
        }
    }
    
    // If no solutions, use previous configuration
    if (candidate_count == 0) {
        best_solution.theta = theta_prev;
        best_solution.valid = true;
        return best_solution;
    }
    
    // Configuration type detector
    auto get_config_type = [PI](const Eigen::Vector3d& theta) {
        Eigen::Vector3d config_type;
        for (int i = 0; i < 3; i++) {
            double angle = fmod(theta(i) + 2*PI, 2*PI);
            config_type(i) = (angle > PI) ? 1 : 0;
        }
        return config_type;
    };

    Eigen::Vector3d prev_config_type = get_config_type(theta_prev);

    // Evaluate all candidate solutions
    for (size_t i = 0; i < candidate_count; i++) {
        const Eigen::Vector3d& theta_candidate = candidates[i];
        
        // Compute forward kinematics
        const Eigen::Vector3d p_fk = forwardKinematics(theta_candidate, p);
        const double pos_error = (p_fk - p).norm();
        
        // Skip solutions with large position errors
        if (pos_error > 1e-3) continue;

        // Calculate joint space continuity metric
        const double joint_dist = (theta_candidate - theta_prev).norm();
        
        // Configuration type matching
        Eigen::Vector3d candidate_config = get_config_type(theta_candidate);
        double config_match = (candidate_config - prev_config_type).norm();
        double config_score = 1.0 - min(config_match, 1.0);
        
        // Combined selection metric
        double selection_metric = 0.8*joint_dist - 0.2*config_score;

        // Track best solution
        if (selection_metric < min_joint_dist) {
            min_joint_dist = selection_metric;
            best_solution.theta = theta_candidate;
            best_solution.valid = true;
            min_pos_error = pos_error;
        }
    }

    // Final validation
    if (best_solution.valid) {
        const Eigen::Vector3d p_fk = forwardKinematics(best_solution.theta, p);
        const double final_error = (p_fk - p).norm();
        
        if (final_error > 5e-3) {
            best_solution.valid = false;
        }
    }

    // Fallback to previous configuration if no valid solution
    if (!best_solution.valid) {
        best_solution.theta = theta_prev;
        best_solution.valid = true;
    }

    return best_solution;
}

Eigen::Vector3d circleSphereIntersection(
    const Eigen::Vector3d& A, const Eigen::Vector3d& B, const Eigen::Vector3d& C,
    double rA, double rB, double rC, const Eigen::Vector3d& ref_pos) 
{
    constexpr double epsilon = 1e-8;
    const Eigen::Vector3d d12 = B - A;
    const double d = d12.norm();
    
    if (d > rA + rB || d < std::abs(rA - rB)) {
        //std::cerr << "FK Warning: Spheres don't intersect\n";
        return ref_pos;
    }

    if (d < 1e-8) {
        //std::cerr << "FK Warning: Spheres A and B are concentric\n";
        return ref_pos;
    }
    
    const double a = (rA*rA - rB*rB + d*d) / (2*d);
    const double h = std::sqrt(rA*rA - a*a);
    const Eigen::Vector3d p0 = A + a * d12.normalized();
    
    Eigen::Vector3d u = d12.unitOrthogonal();
    Eigen::Vector3d v = d12.cross(u).normalized();
    
    const double F0 = (p0 - C).squaredNorm();
    const double RHS = (rC*rC - F0 - h*h) / (2*h);
    const double D = (p0 - C).dot(u);
    const double E = (p0 - C).dot(v);
    const double R = std::sqrt(D*D + E*E);
    
    if (std::abs(R) < epsilon || std::abs(RHS) > R) {
        //std::cerr << "FK Warning: No circle-sphere intersection\n";
        return ref_pos;
    }
    
    const double alpha = std::atan2(E, D);
    const double phi0 = std::acos(RHS / R);
    const double phi1 = alpha - phi0;
    const double phi2 = alpha + phi0;
    
    const Eigen::Vector3d P1 = p0 + h*(u*std::cos(phi1) + v*std::sin(phi1));
    const Eigen::Vector3d P2 = p0 + h*(u*std::cos(phi2) + v*std::sin(phi2));
    
    auto validate = [&](const Eigen::Vector3d& p) {
        return std::abs((p - A).norm() - rA) < 1e-3 &&
               std::abs((p - B).norm() - rB) < 1e-3 &&
               std::abs((p - C).norm() - rC) < 1e-3;
    };
    
    if (validate(P1)) return P1;
    if (validate(P2)) return P2;
    
    return (P1 - ref_pos).squaredNorm() < (P2 - ref_pos).squaredNorm() ? P1 : P2;
}

Eigen::Vector3d RobotDynamics::forwardKinematics(
    const Eigen::Vector3d& theta,
    const Eigen::Vector3d& ref_pos) const 
{
    // Normalize angles to [-π, π] using atan2(sin, cos)
    auto normalize_angle = [](double angle) -> double {
        return atan2(sin(angle), cos(angle));
    };

    double adjusted_th1 = normalize_angle(theta(0));
    double adjusted_th2 = normalize_angle(theta(1));
    double adjusted_th3 = normalize_angle(theta(2));

    // Compute S joint positions
    Eigen::Vector3d s1(
        params_.a1x + params_.l_11 * cos(adjusted_th1),
        params_.a1y + params_.l_11 * sin(adjusted_th1),
        params_.h_1
    );
    
    Eigen::Vector3d s2(
        params_.a2x + params_.l_21 * cos(adjusted_th2),
        params_.a2y + params_.l_21 * sin(adjusted_th2),
        params_.h_2
    );
    
    Eigen::Vector3d s3(
        params_.a3x + params_.l_31 * cos(adjusted_th3),
        params_.a3y,
        params_.h_3 + params_.l_31 * sin(adjusted_th3)
    );
    
    // Compute constraint points (C_i)
    const Eigen::Vector3d c1(s1.x(), s1.y(), s1.z() - (params_.d - params_.v_1));
    const Eigen::Vector3d c2(s2.x(), s2.y(), s2.z() - (params_.d - params_.v_2));
    const Eigen::Vector3d c3(s3.x(), s3.y(), s3.z() - (params_.d - params_.v_3));
    const double r1 = params_.l_12;
    const double r2 = params_.l_22;
    const double r3 = params_.l_32;

    // Helper function to validate solution against all spheres
    auto validate_solution = [&](const Eigen::Vector3d& p, double tolerance = 1e-2) {
        return std::abs((p - c1).norm() - r1) < tolerance &&
               std::abs((p - c2).norm() - r2) < tolerance &&
               std::abs((p - c3).norm() - r3) < tolerance;
    };

    // Set reference point A = c1
    const Eigen::Vector3d& A = c1;
    const Eigen::Vector3d& B = c2;
    const Eigen::Vector3d& C = c3;
    const double rA = r1;
    const double rB = r2;
    const double rC = r3;

    // Two-plane method for trilateration
    Eigen::Vector3d v1 = B - A;
    Eigen::Vector3d v2 = C - A;
    
    // Compute constants for plane equations
    const double d1 = rB*rB - rA*rA + A.squaredNorm() - B.squaredNorm();
    const double d2 = rC*rC - rA*rA + A.squaredNorm() - C.squaredNorm();
    
    // Normal vector to the two planes
    Eigen::Vector3d n = v1.cross(v2);
    const double epsilon = 1e-8;
    Eigen::Matrix<double, 2, 3> M;
    M.row(0) = v1.transpose();
    M.row(1) = v2.transpose();
    Eigen::Vector2d d_vec(d1/2, d2/2);
    
    // Find particular solution using pseudoinverse
    Eigen::Matrix2d MMt = M * M.transpose();
    if (std::abs(MMt.determinant()) < epsilon) {
        //std::cerr << "FK Warning: Singular MMt matrix\n";
        return ref_pos;
    }
    
    Eigen::Matrix2d MMt_inv = MMt.inverse();
    Eigen::Vector3d P0 = M.transpose() * MMt_inv * d_vec;
    
    // Solve quadratic for line parameter
    Eigen::Vector3d P0_minus_A = P0 - A;
    const double a_coeff = n.squaredNorm();
    const double b_coeff = 2 * P0_minus_A.dot(n);
    const double c_coeff = P0_minus_A.squaredNorm() - rA*rA;
    const double discriminant = b_coeff*b_coeff - 4*a_coeff*c_coeff;
    
    if (n.norm() < epsilon || discriminant < 0) {
        if (discriminant < 0) {
            //std::cerr << "FK Warning: Negative discriminant - ";
        }
        //std::cerr << "Using circle-sphere fallback\n";
        return circleSphereIntersection(A, B, C, rA, rB, rC, ref_pos);
    }
    
    const double t1 = (-b_coeff + std::sqrt(discriminant)) / (2*a_coeff);
    const double t2 = (-b_coeff - std::sqrt(discriminant)) / (2*a_coeff);
    
    const Eigen::Vector3d P1 = P0 + t1 * n;
    const Eigen::Vector3d P2 = P0 + t2 * n;
    
    // Validate solutions and select best
    const bool valid1 = validate_solution(P1);
    const bool valid2 = validate_solution(P2);
    
    if (valid1 && valid2) {
        return (P1 - ref_pos).squaredNorm() < (P2 - ref_pos).squaredNorm() ? P1 : P2;
    }
    if (valid1) return P1;
    if (valid2) return P2;

    Eigen::Matrix3d J = computeJ(ref_pos, theta);
    Eigen::Matrix3d K = computeK(ref_pos, theta);
    
    Eigen::Vector3d delta_theta = theta - last_theta_;
    Eigen::Vector3d estimated_pos = ref_pos + dampedPseudoInverse(J) * K * delta_theta;
    
    // Use this estimate if it validates reasonably well
    if (validate_solution(estimated_pos, 1e-2)) {
        last_theta_ = theta;
        return estimated_pos;
    }
    
    // Final fallback: use reference position but log error
    std::cerr << "FK Error: Using fallback position at theta: " 
              << theta.transpose() << "\n";
    last_theta_ = theta;
    return estimated_pos;

}