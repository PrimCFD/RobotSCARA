#include "RobotModel.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <cmath>
#include <tuple>

using json = nlohmann::json;

RobotDynamics::RobotDynamics() : initialized_(false) {}

RobotDynamics::RobotDynamics(const RobotParams& params) 
    : params_(params), initialized_(true) {}


bool RobotDynamics::loadFromConfig(const std::string& config_path) {
    try {
        std::ifstream file(config_path);
        if (!file.is_open()) {
            std::cerr << "Failed to open config file: " << config_path << std::endl;
            return false;
        }
        
        json config;
        file >> config;

        // Load Geometry ------------------------------------------------
        auto& geom = config["geometry"];
        params_.d = geom["d"];
        
        // Load arrays
        auto& v = geom["v"];
        params_.v_1 = v[0]; params_.v_2 = v[1]; params_.v_3 = v[2];
        
        auto& h = geom["h"];
        params_.h_1 = h[0]; params_.h_2 = h[1]; params_.h_3 = h[2];
        
        auto& l1 = geom["l1"];
        params_.l_11 = l1[0]; params_.l_12 = l1[1];
        
        auto& l2 = geom["l2"];
        params_.l_21 = l2[0]; params_.l_22 = l2[1];
        
        auto& l3 = geom["l3"];
        params_.l_31 = l3[0]; params_.l_32 = l3[1];

        // Load Elbow/Shoulder Positions --------------------------------
        auto& elbow = config["elbow"];
        params_.vec_elbow << elbow["x"], elbow["y"], elbow["z"];
        
        auto& initial_pos = config["initial_pos"];
        Eigen::Vector3d p_0(
            initial_pos["x"], 
            initial_pos["y"], 
            initial_pos["z"]
        );

        params_.initial_pos = p_0;

        // Compute l_arm_proth (norm between p0 and elbow)
        params_.l_arm_proth = (p_0 - params_.vec_elbow).norm();

        // Compute Shoulder Position ------------------------------------
        double l_humerus = config["arm"]["l_humerus"];
        double z_shoulder = config["shoulder"]["z"];
        
        double xy_shoulder_offset = std::sqrt(
            (l_humerus * l_humerus - z_shoulder * z_shoulder) / 2.0
        );
        
        params_.vec_shoulder << 
            params_.vec_elbow.x() + xy_shoulder_offset,
            params_.vec_elbow.y() + xy_shoulder_offset,
            z_shoulder;

        // Load Mass Parameters -----------------------------------------
        auto& mass = config["mass"];

        auto& m1 = mass["m_1"];
        params_.m_11 = m1[0]; params_.m_12 = m1[1];
        
        auto& m2 = mass["m_2"];
        params_.m_21 = m2[0]; params_.m_22 = m2[1];

        auto& m3 = mass["m_3"];
        params_.m_31 = m3[0]; params_.m_32 = m3[1];
        
        double m_d_seul = mass["m_d_seul"];
        double m_bras = mass["m_bras"];
        params_.m_d = m_d_seul + m_bras;  // Computed parameter

        // Validate critical parameters
        if (params_.l_arm_proth <= 0 || params_.m_d <= 0) {
            throw std::runtime_error("Invalid computed parameters");
        }

        initialized_ = true;
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Config error: " << e.what() << "\n";
        return false;
    }
}

void RobotDynamics::setParameters(const RobotParams& params) {
    params_ = params;
    initialized_ = true;
}

// K matrix computation (based on analytic.py K function)
Eigen::Matrix3d RobotDynamics::computeK(const Eigen::Vector3d& pos, 
                                        const Eigen::Vector3d& theta) const {
    if (!initialized_) {
        throw std::runtime_error("RobotDynamics not initialized");
    }
    
    double x = pos(0), y = pos(1), z = pos(2);
    double theta_1 = theta(0), theta_2 = theta(1), theta_3 = theta(2);
    
    Eigen::Matrix3d K;
    K << y * sin(theta_1) - x * cos(theta_1), 0, 0,
         0, y * sin(theta_2) - x * cos(theta_2), 0,
         0, 0, (z + params_.d - params_.v_3 - params_.h_3) * sin(theta_3) + x * cos(theta_3);
    
    return K;
}

// J matrix computation (based on analytic.py J function)
Eigen::Matrix3d RobotDynamics::computeJ(const Eigen::Vector3d& pos, 
                                        const Eigen::Vector3d& theta) const {
    if (!initialized_) {
        throw std::runtime_error("RobotDynamics not initialized");
    }
    
    double x = pos(0), y = pos(1), z = pos(2);
    double theta_1 = theta(0), theta_2 = theta(1), theta_3 = theta(2);
    
    Eigen::Matrix3d J;
    J << x + params_.l_11 * sin(theta_1), 
         y + params_.l_11 * cos(theta_1), 
         z + params_.d - params_.v_1 - params_.h_1,
         
         x + params_.l_21 * sin(theta_2), 
         y + params_.l_21 * cos(theta_2), 
         z + params_.d - params_.v_2 - params_.h_2,
         
         x + params_.l_31 * sin(theta_3), 
         y, 
         z + params_.d - params_.v_3 - params_.h_3 + params_.l_31 * cos(theta_3);
    
    return J;
}

// K_dot computation (based on analytic.py K_dot function)
Eigen::Matrix3d RobotDynamics::computeKDot(const Eigen::Vector3d& pos,
                                           const Eigen::Vector3d& vel,
                                           const Eigen::Vector3d& theta,
                                           const Eigen::Vector3d& theta_dot) const {
    if (!initialized_) {
        throw std::runtime_error("RobotDynamics not initialized");
    }
    
    double x = pos(0), y = pos(1), z = pos(2);
    double xp = vel(0), yp = vel(1), zp = vel(2);
    double theta_1 = theta(0), theta_2 = theta(1), theta_3 = theta(2);
    double theta_1p = theta_dot(0), theta_2p = theta_dot(1), theta_3p = theta_dot(2);
    
    Eigen::Matrix3d K_dot;
    K_dot << yp * sin(theta_1) - xp * cos(theta_1) + theta_1p * (y * cos(theta_1) + x * sin(theta_1)), 0, 0,
             0, yp * sin(theta_2) - xp * cos(theta_2) + theta_2p * (y * cos(theta_2) + x * sin(theta_2)), 0,
             0, 0, zp * sin(theta_3) + xp * cos(theta_3) + theta_3p * ((z + params_.d - params_.v_3 - params_.h_3) * cos(theta_3) - x * sin(theta_3));
    
    return K_dot;
}

// J_dot computation (based on analytic.py J_dot function)
Eigen::Matrix3d RobotDynamics::computeJDot(const Eigen::Vector3d& vel,
                                           const Eigen::Vector3d& theta,
                                           const Eigen::Vector3d& theta_dot) const {
    if (!initialized_) {
        throw std::runtime_error("RobotDynamics not initialized");
    }
    
    double xp = vel(0), yp = vel(1), zp = vel(2);
    double theta_1 = theta(0), theta_2 = theta(1), theta_3 = theta(2);
    double theta_1p = theta_dot(0), theta_2p = theta_dot(1), theta_3p = theta_dot(2);
    
    Eigen::Matrix3d J_dot;
    J_dot << xp + params_.l_11 * theta_1p * cos(theta_1), 
             yp - params_.l_11 * theta_1p * cos(theta_1), 
             zp,
             
             xp + params_.l_21 * theta_2p * cos(theta_2), 
             yp - params_.l_21 * theta_2p * cos(theta_2), 
             zp,
             
             xp + params_.l_31 * theta_3p * cos(theta_3), 
             yp, 
             zp - params_.l_31 * theta_3p * sin(theta_3);
    
    return J_dot;
}

// Motor speed computation (based on analytic.py Compute_motor_speed)
Eigen::Vector3d RobotDynamics::computeMotorSpeed(const Eigen::Vector3d& pos,
                                                const Eigen::Vector3d& vel,
                                                const Eigen::Vector3d& theta) const {
    if (!initialized_) {
        throw std::runtime_error("RobotDynamics not initialized");
    }
    
    Eigen::Matrix3d K = computeK(pos, theta);
    Eigen::Matrix3d J = computeJ(pos, theta);
    
    // Safe inversion of K
    Eigen::Matrix3d K_inv = dampedPseudoInverse(K);
    
    Eigen::Matrix3d Jacobian_prod = K_inv * J;
    return Jacobian_prod * vel;
}

// Motor acceleration computation (based on analytic.py Compute_motor_accel)
Eigen::Vector3d RobotDynamics::computeMotorAccel(const Eigen::Vector3d& pos,
                                                const Eigen::Vector3d& vel,
                                                const Eigen::Vector3d& accel,
                                                const Eigen::Vector3d& theta,
                                                const Eigen::Vector3d& theta_dot) const {
    if (!initialized_) {
        throw std::runtime_error("RobotDynamics not initialized");
    }
    
    Eigen::Matrix3d J = computeJ(pos, theta);
    Eigen::Matrix3d J_dot = computeJDot(vel, theta, theta_dot);
    Eigen::Matrix3d K = computeK(pos, theta);
    Eigen::Matrix3d K_dot = computeKDot(pos, vel, theta, theta_dot);
    
    Eigen::Vector3d J_1 = J * accel;
    Eigen::Vector3d J_2 = J_dot * vel;
    Eigen::Vector3d J_3 = K_dot * theta_dot;
    Eigen::Vector3d J_sum = J_1 + J_2 - J_3;
    
    // Safe inversion of K
    Eigen::Matrix3d K_inv = dampedPseudoInverse(K);
    
    return K_inv * J_sum;
}

Eigen::Matrix3d RobotDynamics::computeMassMatrix(const Eigen::Vector3d& theta, const Eigen::Vector3d& pos) const {
    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    
    // Proximal links (1/4 term) and distal links (1/3 term) based on slender link method

    M(0,0) = (params_.m_11 / 4.0) * pow(params_.l_11, 2) 
           + (params_.m_12 / 3.0) * pow(params_.l_11, 2);
    
    M(1,1) = (params_.m_21 / 4.0) * pow(params_.l_21, 2) 
           + (params_.m_22 / 3.0) * pow(params_.l_21, 2);
    
    M(2,2) = (params_.m_31 / 4.0) * pow(params_.l_31, 2) 
           + (params_.m_32 / 3.0) * pow(params_.l_31, 2);

    Eigen::Matrix3d J = computeJ(pos, theta);

    // Inertia of the platform + payload (human arm)
    double I_arm = (1.0/3.0) * params_.m_d * pow(params_.l_arm_proth, 2);

    // Effective mass terms (inertial force)
    double m_eff = params_.m_d + I_arm/pow(params_.l_arm_proth, 2);

    M += J.transpose() * m_eff * J;

    return M;
}

Eigen::Vector3d RobotDynamics::computeCoriolis(
    const Eigen::Vector3d& x, 
    const Eigen::Vector3d& x_dot,
    const Eigen::Vector3d& theta, 
    const Eigen::Vector3d& theta_dot
) const {
    // Platform velocity (x_dot = [x_p, y_p, z_p])
    double xp = x_dot(0), yp = x_dot(1), zp = x_dot(2);

    // Joint angles and velocities
    double theta_1 = theta(0), theta_2 = theta(1), theta_3 = theta(2);
    double theta_1p = theta_dot(0), theta_2p = theta_dot(1), theta_3p = theta_dot(2);

    // Coriolis terms for slender link method
    Eigen::Vector3d C;

    // Proximal link contributions
    double C1_prox = (params_.m_12 / 6.0) * params_.l_11 * 
                    (yp * sin(theta_1) - xp * cos(theta_1)) * theta_1p;

    double C2_prox = (params_.m_22 / 6.0) * params_.l_21 * 
                    (yp * sin(theta_2) - xp * cos(theta_2)) * theta_2p;

    double C3_prox = (params_.m_32 / 6.0) * params_.l_31 * 
                    (zp * sin(theta_3) + xp * cos(theta_3)) * theta_3p;

    // Distal link contributions
    Eigen::Matrix3d J = computeJ(x, theta);
    Eigen::Matrix3d J_dot = computeJDot(x_dot, theta, theta_dot);

    Eigen::Vector3d C_distal;
    for (int i = 0; i < 3; i++) {
        double distal_mass;
        switch(i) {
            case 0: distal_mass = params_.m_12; break;
            case 1: distal_mass = params_.m_22; break;
            case 2: distal_mass = params_.m_32; break;
            default: distal_mass = 0;
        }

        C_distal(i) = (distal_mass / 6.0) * (
            J.row(i).dot(J_dot.col(i)) + 
            J_dot.row(i).dot(J.col(i))    
            ) * theta_dot(i);
    }

    // Total Coriolis terms
    C << C1_prox + C_distal(0), 
         C2_prox + C_distal(1), 
         C3_prox + C_distal(2);

    return C;
}

Eigen::Vector3d RobotDynamics::computeGravity(const Eigen::Vector3d& theta) const {
    double theta_3 = theta(2);
    double G3 = params_.m_31 * GRAVITY * params_.l_31 * sin(theta_3);
    return Eigen::Vector3d(0.0, 0.0, G3);
}

Eigen::Vector3d RobotDynamics::computePlatformForces(
    const Eigen::Vector3d& theta,
    const Eigen::Vector3d& theta_dot
) const {
    double theta_1 = theta(0), theta_2 = theta(1), theta_3 = theta(2);
    double theta_1p = theta_dot(0), theta_2p = theta_dot(1), theta_3p = theta_dot(2);

    // Centrifugal terms (joint 1, 2, 3)
    double centrifugal_1 = (params_.m_12 * params_.l_11 / 6.0) * pow(theta_1p, 2) * (-cos(theta_1));
    double centrifugal_2 = (params_.m_22 * params_.l_21 / 6.0) * pow(theta_2p, 2) * (-sin(theta_2));
    double centrifugal_3 = (params_.m_32 * params_.l_31 / 6.0) * pow(theta_3p, 2) * (-sin(theta_3));

    // Gravitational force (z-direction only)
    double gravity = GRAVITY * (params_.m_12 + params_.m_22 + params_.m_32 + params_.m_d) * 0.5;

    return Eigen::Vector3d(centrifugal_1, centrifugal_2, centrifugal_3 + gravity);
}

Eigen::Vector3d RobotDynamics::computeForwardDynamics(
    const Eigen::Vector3d& theta,
    const Eigen::Vector3d& theta_dot,
    const Eigen::Vector3d& torque,
    const Eigen::Vector3d& pos,            // Task-space position
    const Eigen::Vector3d& vel             // Task-space acceleration
) const {
    if (!initialized_) {
        throw std::runtime_error("RobotDynamics not initialized");
    }

    // Compute platform forces
    Eigen::Vector3d f_e = computePlatformForces(theta, theta_dot);

    // Map task-space forces to joint space
    Eigen::Matrix3d J = computeJ(pos, theta);
    Eigen::Vector3d tau_ext = J.transpose() * f_e;

    // Compute dynamics terms

    Eigen::Matrix3d M = computeMassMatrix(theta, pos);
    Eigen::Vector3d C = computeCoriolis(pos, vel, theta, theta_dot);
    Eigen::Vector3d G = computeGravity(theta);

    // Safe inversion with damping for diagonal matrix K
    Eigen::Matrix3d M_inv = dampedPseudoInverse(M);

    // In RobotModel.cpp:
    Eigen::Vector3d friction = 
        -0.3 * theta_dot.normalized()  // Coulomb friction
        -0.1 * theta_dot;              // Viscous friction

    return M_inv * (torque - tau_ext - C - G + friction);
}

Eigen::Vector3d RobotDynamics::computeTorque(
    const Eigen::Vector3d& pos,
    const Eigen::Vector3d& vel,
    const Eigen::Vector3d& accel,
    const Eigen::Vector3d& theta,
    const Eigen::Vector3d& theta_dot,
    const Eigen::Vector3d& theta_ddot
) const {
    if (!initialized_) {
        throw std::runtime_error("RobotDynamics not initialized");
    }

    // Compute motor torques using existing helper functions
    Eigen::Matrix3d M = computeMassMatrix(theta, pos);
    Eigen::Vector3d C = computeCoriolis(pos, vel, theta, theta_dot);
    Eigen::Vector3d G = computeGravity(theta);
    Eigen::Vector3d tau_e = M * theta_ddot + C + G;

    // Compute platform forces (f_e)
    Eigen::Vector3d f_e = computePlatformForces(theta, theta_dot);

    // Compute Jacobian terms using existing functions
    Eigen::Matrix3d J = computeJ(pos, theta);
    Eigen::Matrix3d K = computeK(pos, theta);
    Eigen::Matrix3d J_inv = dampedPseudoInverse(J);
    Eigen::Matrix3d Jacob = J_inv * K;

    return tau_e + Jacob.transpose() * f_e;
}

// Complete inverse dynamics computation
RobotDynamics::DynamicsResult RobotDynamics::computeInverseDynamics(
    const Eigen::Vector3d& pos,
    const Eigen::Vector3d& vel,
    const Eigen::Vector3d& accel,
    const Eigen::Vector3d& theta) const {
    
    DynamicsResult result;
    result.success = false;
    
    if (!initialized_) {
        result.error_msg = "RobotDynamics not initialized";
        return result;
    }
    
    try {
        // Compute motor velocities
        result.theta_dot = computeMotorSpeed(pos, vel, theta);
        
        // Compute motor accelerations
        result.theta_ddot = computeMotorAccel(pos, vel, accel, theta, result.theta_dot);
        
        // Compute torques
        result.torque = computeTorque(pos, vel, accel, theta, result.theta_dot, result.theta_ddot);
        
        result.success = true;
    }
    catch (const std::exception& e) {
        result.error_msg = std::string("Computation error: ") + e.what();
    }
    
    return result;
}

Eigen::Vector3d RobotDynamics::computeRK4Integration(
    const Eigen::Vector3d& theta,
    const Eigen::Vector3d& theta_dot,
    const Eigen::Vector3d& torque,
    const Eigen::Vector3d& pos,
    const Eigen::Vector3d& vel,
    double dt) const 
{
    if (!initialized_) {
        throw std::runtime_error("RobotDynamics not initialized");
    }

    // RK4 integration for θ̈ = M⁻¹(τ - C - G)
    auto computeAccel = [&](const Eigen::Vector3d& th, 
                           const Eigen::Vector3d& th_dot,
                           const Eigen::Vector3d& pos_current,
                           const Eigen::Vector3d& vel_current) {
        return computeForwardDynamics(th, th_dot, torque, pos_current, vel_current);
    };

    // k1
    Eigen::Vector3d k1 = computeAccel(theta, theta_dot, pos, vel);

    // k2
    Eigen::Vector3d theta_k2 = theta + 0.5*dt*theta_dot;
    Eigen::Vector3d theta_dot_k2 = theta_dot + 0.5*dt*k1;
    Eigen::Vector3d k2 = computeAccel(theta_k2, theta_dot_k2, pos, vel);

    // k3
    Eigen::Vector3d theta_k3 = theta + 0.5*dt*theta_dot;
    Eigen::Vector3d theta_dot_k3 = theta_dot + 0.5*dt*k2;
    Eigen::Vector3d k3 = computeAccel(theta_k3, theta_dot_k3, pos, vel);

    // k4
    Eigen::Vector3d theta_k4 = theta + dt*theta_dot;
    Eigen::Vector3d theta_dot_k4 = theta_dot + dt*k3;
    Eigen::Vector3d k4 = computeAccel(theta_k4, theta_dot_k4, pos, vel);

    // Final integration
    return theta_dot + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4);
}

// Utility functions
Eigen::Matrix3d RobotDynamics::dampedPseudoInverse(
    const Eigen::Matrix3d& matrix, 
    double lambda_tikhonov,  // Tikhonov damping parameter (e.g., 1e-3)
    double epsilon_diag      // Diagonal damping parameter (for K matrix)
) const {
    // Check if the matrix is diagonal (for K matrix)
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
        // Diagonal damping: Invert diagonal elements with epsilon
        Eigen::Matrix3d inv = Eigen::Matrix3d::Zero();
        for (int i = 0; i < 3; ++i) {
            double val = matrix(i, i);
            inv(i, i) = (std::abs(val) < epsilon_diag) ? 
                        1.0 / (val + epsilon_diag) : 
                        1.0 / val;
        }
        return inv;
    } else {
        // Tikhonov damping for non-diagonal matrices (e.g., Jacobian)
        // Formula: J⁺ = Jᵀ * (J * Jᵀ + λ²I)^-1
        Eigen::Matrix3d JJT = matrix * matrix.transpose();
        Eigen::Matrix3d damping = (lambda_tikhonov * lambda_tikhonov) * Eigen::Matrix3d::Identity();
        Eigen::Matrix3d JJT_damped = JJT + damping;
        Eigen::Matrix3d inv_JJT_damped = JJT_damped.inverse();
        return matrix.transpose() * inv_JJT_damped;
    }
}


RobotDynamics::IKSolution RobotDynamics::invKineSinglePoint(const Eigen::Vector3d& p, const Eigen::Vector3d& theta_prev) const {
    using std::atan, std::sqrt, std::runtime_error;

    if (!initialized_) throw runtime_error("RobotDynamics not initialized");

    const Eigen::Vector3d z_vec(0, 0, 1);

    // m_i = (0, 0, h_i)
    Eigen::Vector3d m_1(0, 0, params_.h_1);
    Eigen::Vector3d m_2(0, 0, params_.h_2);
    Eigen::Vector3d m_3(0, 0, params_.h_3);

    // A and B
    double A1 = 2 * p.y(), B1 = 2 * p.x();
    double A2 = 2 * p.y(), B2 = 2 * p.x();
    double A3 = 2 * (-params_.h_3 + (p.z() + params_.d - params_.v_3));
    double B3 = -2 * p.x();

    // C helper
    auto computeC = [&](const Eigen::Vector3d& m, double l1, double l2, double v) {
        Eigen::Vector3d offset = (params_.d - v) * z_vec;
        double norm_sq = (m - (p + offset)).squaredNorm();
        return (l2 * l2 - l1 * l1 - norm_sq) / l1;
    };

    double C1 = computeC(m_1, params_.l_11, params_.l_12, params_.v_1);
    double C2 = computeC(m_2, params_.l_21, params_.l_22, params_.v_2);
    double C3 = computeC(m_3, params_.l_31, params_.l_32, params_.v_3);

    // Solve quadratic: returns 0, 1 or 2 roots
    auto solve_t = [](double A, double B, double C, std::vector<double>& out) {
        double disc = B * B - (A + C) * (C - A);
        if (disc < 0 || std::abs(A + C) < 1e-8) return;
        double denom = A + C;
        double sqrt_disc = sqrt(disc);
        out.push_back((B + sqrt_disc) / denom);
        out.push_back((B - sqrt_disc) / denom);
    };

    std::vector<double> t1, t2, t3;
    solve_t(A1, B1, C1, t1);
    solve_t(A2, B2, C2, t2);
    solve_t(A3, B3, C3, t3);

    double best_cost = std::numeric_limits<double>::max();
    IKSolution best_solution;

    for (double t1i : t1) {
        for (double t2i : t2) {
            for (double t3i : t3) {
                double th1 = 2 * atan(t1i);
                double th2 = 2 * atan(t2i);
                double th3 = 2 * atan(t3i);

                Eigen::Vector3d candidate(th1, th2, th3);
                if (!candidate.allFinite()) continue;

                double cost = (candidate - theta_prev).squaredNorm();

                if (cost < best_cost) {
                    best_cost = cost;
                    best_solution.theta = candidate;
                    best_solution.valid = true;
                }
            }
        }
    }

    return best_solution;
}