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

void RobotDynamics::setElbowArm(const Eigen::Vector3d& elbow, 
                                      double l_arm_proth) {
    params_.vec_elbow = elbow;
    params_.l_arm_proth = l_arm_proth;
}

Eigen::Matrix3d RobotDynamics::computeK(const Eigen::Vector3d& pos, 
                                        const Eigen::Vector3d& theta) const {

    if (!initialized_) std::cerr<<"Not initialized"<<std::endl;

    
    double x = pos(0), y = pos(1), z = pos(2);
    double th1 = theta(0), th2 = theta(1), th3 = theta(2);
    double cth1 = cos(th1), cth2 = cos(th2), cth3 = cos(th3);
    double sth1 = sin(th1), sth2 = sin(th2), sth3 = sin(th3);
    
    Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
    
    // Leg 1
    double term1x = -params_.a1x - params_.l_11*cth1 + x;
    double term1y = -params_.a1y - params_.l_11*sth1 + y;
    K(0,0) = params_.l_11 * (-term1x*sth1 + term1y*cth1);
    
    // Leg 2
    double term2x = -params_.a2x - params_.l_21*cth2 + x;
    double term2y = -params_.a2y - params_.l_21*sth2 + y;
    K(1,1) = params_.l_21 * (-term2x*sth2 + term2y*cth2);
    
    // Leg 3
    double term3x = -params_.a3x - params_.l_31*cth3 + x;
    double term3z = -params_.h_3 - params_.l_31*sth3 + params_.d - params_.v_3 + z;
    K(2,2) = params_.l_31 * (-term3x*sth3 + term3z*cth3);
    
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
    J(0,2) = -params_.h_1 + params_.d - params_.v_1 + z;
    
    // Leg 2
    J(1,0) = -params_.a2x - params_.l_21*cos(th2) + x;
    J(1,1) = -params_.a2y - params_.l_21*sin(th2) + y;
    J(1,2) = -params_.h_2 + params_.d - params_.v_2 + z;
    
    // Leg 3
    J(2,0) = -params_.a3x - params_.l_31*cos(th3) + x;
    J(2,1) = -params_.a3y + y;
    J(2,2) = -params_.h_3 - params_.l_31*sin(th3) + params_.d - params_.v_3 + z;
    
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
    double cth1 = cos(th1), cth2 = cos(th2), cth3 = cos(th3);
    double sth1 = sin(th1), sth2 = sin(th2), sth3 = sin(th3);
    
    Eigen::Matrix3d K_dot = Eigen::Matrix3d::Zero();
    
    // Leg 1
    K_dot(0,0) = - params_.l_11 * (
        (-params_.a1x*cth1 - params_.a1y*sth1 + 
         x*cth1 + y*sth1) * th1d +
        sth1*xd - cth1*yd
    );
    
    // Leg 2
    K_dot(1,1) = - params_.l_21 * (
        (-params_.a2x*cth2 - params_.a2y*sth2 + 
         x*cth2 + y*sth2) * th2d +
        sth2*xd - cth2*yd
    );
    
    // Leg 3
    K_dot(2,2) = - params_.l_31 * (
        (-params_.a3x*cth3 + (- params_.h_3 + params_.d - params_.v_3) * sth3 + 
         x*cth3 + z*sth3) * th3d +
        sth3*xd - cth3*zd
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

    // Add platform mass contribution (point mass at end effector)
    Eigen::DiagonalMatrix<double,3> m;
    m.diagonal().setConstant(params_.m_d + params_.m_arm/3.0);
    
    // Proximal links with added distal point masses
    M(0,0) = (params_.m_11/3.0 + params_.m_12/2.0) * params_.l_11 * params_.l_11;
    M(1,1) = (params_.m_21/3.0 + params_.m_22/2.0) * params_.l_21 * params_.l_21;
    M(2,2) = (params_.m_31/3.0 + params_.m_32/2.0) * params_.l_31 * params_.l_31;

    Eigen::Matrix3d J = computeJ(pos, theta);
    Eigen::Matrix3d K = computeK(pos, theta);
    Eigen::Matrix3d Jacob = dampedPseudoInverse(J) * K;

    return M + Jacob.transpose() * m * Jacob;
}

Eigen::Vector3d RobotDynamics::computeGravity(const Eigen::Vector3d& theta, const Eigen::Vector3d& pos) const {
    double th1 = theta(0), th2 = theta(1), th3 = theta(2);
    double x = pos(0), y = pos(1), z = pos(2);
    
    // Leg 3 has different orientation (xz-plane motion)
    double G3 = (params_.m_31/2.0 + params_.m_32/2.0) * GRAVITY * params_.l_31 * sin(th3);

    // Distal links
    double G3_dist = (params_.m_32/2.0 + params_.m_12/2.0 + params_.m_22/2.0) * GRAVITY;

    // Add forearm gravity (distributed 1/3 - 2/3 between endpoints)
    // Elbow contribution is constant (z_shoulder) so only end effector contributes
    G3_dist += (params_.m_arm/2.0) * GRAVITY;
    
    // Add platform gravity at end effector
    G3_dist += params_.m_d * GRAVITY;

    Eigen::Matrix3d J = computeJ(pos, theta);
    Eigen::Matrix3d K = computeK(pos, theta);
    Eigen::Matrix3d Jacob = dampedPseudoInverse(J) * K;

    Eigen::Vector3d G(0, 0, G3);
    Eigen::Vector3d G_dist(0, 0, G3_dist);
              
    return G + Jacob.transpose() * G_dist;
}

Eigen::Vector3d RobotDynamics::computeForwardDynamics(
    const Eigen::Vector3d& theta,
    const Eigen::Vector3d& torque,
    const Eigen::Vector3d& pos
) const {
    if (!initialized_) {
        std::cerr << "Robot model not initialized" << std::endl;
        return Eigen::Vector3d::Zero();
    }

    // Compute dynamics terms
    Eigen::Matrix3d M = computeMassMatrix(theta, pos);
    Eigen::Vector3d G = computeGravity(theta, pos);

    // Regularized matrix inversion
    Eigen::LLT<Eigen::Matrix3d> llt(M);
    if (llt.info() == Eigen::Success) {
        return llt.solve(torque - G);
    } else {
        // Fallback to damped pseudo-inverse
        return dampedPseudoInverse(M) * (torque - G);
    }
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
    
    return tau_e;
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
        const Eigen::Matrix3d& A, double lambda0) const
{
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double s_min = svd.singularValues().minCoeff();
    double lambda = std::max(lambda0, 1e-2 * s_min);   // adaptive damping
    Eigen::Matrix3d D = svd.singularValues().asDiagonal();
    for (int i = 0; i < 3; ++i)
        D(i,i) = D(i,i) / (D(i,i)*D(i,i) + lambda*lambda);
    return svd.matrixV() * D * svd.matrixU().transpose();
}

RobotDynamics::IKSolution RobotDynamics::invKineSinglePoint(
    const Eigen::Vector3d& p, 
    const Eigen::Vector3d& theta_prev) const 
{

    // Iterative solution
    IKSolution sol_iter = iterativeIK(p, theta_prev, 1e-6, 20);
    
    // Compare solutions and choose best
    if(sol_iter.valid) {
        return sol_iter;
    }

    // Return solution anyways marked non valid
    return sol_iter;
}

Eigen::Matrix3d RobotDynamics::computeIKJacobian(
    const Eigen::Vector3d& p, 
    const Eigen::Vector3d& theta) const 
{
    Eigen::Matrix3d J = Eigen::Matrix3d::Zero();

    // Leg 1 (XY plane)
    {
        double th1 = theta(0);
        Eigen::Vector3d s1(
            params_.a1x + params_.l_11 * cos(th1),
            params_.a1y + params_.l_11 * sin(th1),
            params_.h_1
        );
        Eigen::Vector3d b1 = p + Eigen::Vector3d(0,0,params_.d-params_.v_1);
        Eigen::Vector3d vec = b1 - s1;
        Eigen::Vector3d ds_dth1(
            -params_.l_11 * sin(th1),
            params_.l_11 * cos(th1),
            0
        );
        J(0,0) = -2.0 * vec.dot(ds_dth1);  // df/dθ₁
    }

    // Leg 2 (XY plane)
    {
        double th2 = theta(1);
        Eigen::Vector3d s2(
            params_.a2x + params_.l_21 * cos(th2),
            params_.a2y + params_.l_21 * sin(th2),
            params_.h_2
        );
        Eigen::Vector3d b2 = p + Eigen::Vector3d(0,0,params_.d-params_.v_2);
        Eigen::Vector3d vec = b2 - s2;
        Eigen::Vector3d ds_dth2(
            -params_.l_21 * sin(th2),
            params_.l_21 * cos(th2),
            0
        );
        J(1,1) = -2.0 * vec.dot(ds_dth2);  // df/dθ₂
    }

    // Leg 3 (XZ plane)
    {
        double th3 = theta(2);
        Eigen::Vector3d s3(
            params_.a3x + params_.l_31 * cos(th3),
            params_.a3y,
            params_.h_3 + params_.l_31 * sin(th3)
        );
        Eigen::Vector3d b3 = p + Eigen::Vector3d(0,0,params_.d-params_.v_3);
        Eigen::Vector3d vec = b3 - s3;
        Eigen::Vector3d ds_dth3(
            -params_.l_31 * sin(th3),
            0,  // Correct (no y-change)
            params_.l_31 * cos(th3)
        );
        J(2,2) = -2.0 * vec.dot(ds_dth3);  // df/dθ₃
    }

    return J;
}

RobotDynamics::IKSolution RobotDynamics::iterativeIK(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& theta0,
    double tolerance,
    int max_iter) const
{
    IKSolution result;
    Eigen::Vector3d theta = theta0;
    result.position_error = std::numeric_limits<double>::max();
    result.iterations = 0;
    
    const double lambda_init = 0.01;    // LM damping factor
    const double lambda_factor = 2.0;   // LM adjustment factor
    const double min_step_size = 1e-8;  // Minimum step size
    const double max_step_size = 0.05;   // Maximum step size (radians)
    
    double lambda = lambda_init;
    double prev_error = std::numeric_limits<double>::max();

    for(int iter = 0; iter < max_iter; ++iter) {
        result.iterations = iter + 1;
        
        // Compute constraint residuals
        Eigen::Vector3d f = constraintEquations(p, theta);
        double error = f.norm();
        
        // Update best solution found
        if(error < result.position_error) {
            result.theta = theta;
            result.position_error = error;
            result.valid = error < tolerance;
        }
        
        // Check convergence
        if(error < tolerance) {
            result.valid = true;
            return result;
        }
        
        // Compute Jacobian (diagonal matrix for parallel robot)
        Eigen::Matrix3d J_f = computeIKJacobian(p, theta);
        
        // Levenberg-Marquardt adjustment
        Eigen::Matrix3d J_f_damped = J_f;
        for(int i = 0; i < 3; ++i) {
            J_f_damped(i,i) += lambda * std::abs(J_f(i,i));
        }
        
        // Compute step
        Eigen::Vector3d delta_theta = J_f_damped.inverse() * (-f);
        
        // Apply adaptive step size control
        double step_size = delta_theta.norm();
        if(step_size > max_step_size) {
            delta_theta *= max_step_size / step_size;
            step_size = max_step_size;
        } else if(step_size < min_step_size) {
            break;  // Converged as much as possible
        }
        
        // Store candidate update
        Eigen::Vector3d theta_candidate = theta + delta_theta;
        
        // Evaluate candidate
        double candidate_error = constraintEquations(p, theta_candidate).norm();
        
        // Adaptive LM adjustment
        if(candidate_error < error) {
            // Successful step - accept update
            theta = theta_candidate;
            lambda /= lambda_factor;  // Reduce damping
            prev_error = error;
        } else {
            // Failed step - increase damping and try again
            lambda *= lambda_factor;
            if(lambda > 1e6) break;  // Prevent infinite loop
        }
    }
    
    // Final validation
    result.position_error = constraintEquations(p, result.theta).norm();
    result.valid = result.position_error < tolerance;
    return result;
}

Eigen::Vector3d RobotDynamics::forwardKinematics(
        const Eigen::Vector3d& theta,
        const Eigen::Vector3d& x_prev) const
{
    bool ok = false;
    Eigen::Vector3d x = forwardKinematicsAnalytic(theta, x_prev, ok);
    if (ok) {
        return x;
    }

    // Fallback to iterative solver with better initial guess
    FKSolution sol = iterativeFK(theta, x_prev, 1e-6, 100);
    if (sol.valid) {
        return sol.position;
    }

    return x_prev;
}

Eigen::Vector3d RobotDynamics::forwardKinematicsAnalytic(
        const Eigen::Vector3d& theta,
        const Eigen::Vector3d& x_prev,
        bool& ok) const
{
    constexpr double EPS = 1e-10;

    // ---------------------------------------------------------------------
    // 1. Build sphere centres (C_i) and radii (r_i)
    // ---------------------------------------------------------------------
    const double th1 = theta(0);
    const double th2 = theta(1);
    const double th3 = theta(2);

    // Proximal joint end‑points S_i (re‑use existing parameter names)
    const Eigen::Vector3d s1(
        params_.a1x + params_.l_11 * std::cos(th1),
        params_.a1y + params_.l_11 * std::sin(th1),
        params_.h_1);

    const Eigen::Vector3d s2(
        params_.a2x + params_.l_21 * std::cos(th2),
        params_.a2y + params_.l_21 * std::sin(th2),
        params_.h_2);

    const Eigen::Vector3d s3(
        params_.a3x + params_.l_31 * std::cos(th3),
        params_.a3y,
        params_.h_3 + params_.l_31 * std::sin(th3));

    // Offsets along the distal links (platform attachment).
    const double d = params_.d;
    const Eigen::Vector3d C1 = s1 - Eigen::Vector3d(0, 0, d - params_.v_1);
    const Eigen::Vector3d C2 = s2 - Eigen::Vector3d(0, 0, d - params_.v_2);
    const Eigen::Vector3d C3 = s3 - Eigen::Vector3d(0, 0, d - params_.v_3);

    const double r1 = params_.l_12;
    const double r2 = params_.l_22;
    const double r3 = params_.l_32;

    // ---------------------------------------------------------------------
    // 2. Establish local orthonormal basis {e_x, e_y, e_z}
    //    such that C1 == origin, C2 lies on +x axis.
    //    This follows the algorithm in three_sphere_intersection.py.
    // ---------------------------------------------------------------------
    const Eigen::Vector3d diff12 = C2 - C1;
    const double d12 = diff12.norm();
    if (d12 < EPS) {
        ok = false;            // Degenerate – coincident sphere centres
        return x_prev;
    }

    const Eigen::Vector3d e_x = diff12 / d12;

    // Components of C3 in the temporary frame
    const Eigen::Vector3d diff13 = C3 - C1;
    const double i = e_x.dot(diff13);
    Eigen::Vector3d tmp = diff13 - i * e_x;    // reject on e_x to get component orthogonal to e_x
    const double tmp_norm = tmp.norm();
    if (tmp_norm < EPS) {
        // The three centres are (almost) collinear – fall back to previous iterative FK
        ok = false;
        return x_prev;
    }

    const Eigen::Vector3d e_y = tmp / tmp_norm;
    const Eigen::Vector3d e_z = e_x.cross(e_y); // right‑handed basis guaranteed since e_x ⟂ e_y
    const double j = e_y.dot(diff13);           // projection of diff13 on e_y

    // ---------------------------------------------------------------------
    // 3. Solve intersection coordinates (x, y, z) in local frame
    // ---------------------------------------------------------------------
    // Equation system (see StackOverflow answer 1407395)
    const double x = (r1*r1 - r2*r2 + d12*d12) / (2.0 * d12);
    const double y_num = r1*r1 - r3*r3 + i*i + j*j - 2.0 * x * i;
    const double y_den = 2.0 * j;
    if (std::abs(y_den) < EPS) {
        ok = false;            // Numerically unstable (j -> 0): C3 almost on e_x axis
        return x_prev;
    }
    const double y = y_num / y_den;

    const double z_sq = r1*r1 - x*x - y*y;
    if (z_sq < -1e-8) {       // Negative beyond tolerance ⇒ no real intersection
        ok = false;
        return x_prev;
    }
    const double z = (z_sq < 0.0) ? 0.0 : std::sqrt(z_sq);

    // Two possible solutions in 3‑D space (mirror w.r.t. e_x–e_y plane)
    const Eigen::Vector3d P1 = C1 + x * e_x + y * e_y + z * e_z;
    const Eigen::Vector3d P2 = C1 + x * e_x + y * e_y - z * e_z;


    // ---------------------------------------------------------------------
    // 4. Choose the candidate closest to the previous Cartesian estimate
    // ---------------------------------------------------------------------
    const double dist1 = (P1 - x_prev).squaredNorm();
    const double dist2 = (P2 - x_prev).squaredNorm();
    const Eigen::Vector3d candidate = (dist1 <= dist2) ? P1 : P2;

    // ---------------------------------------------------------------------
    // 5. Validate candidate with sphere residuals (robustness)
    // ---------------------------------------------------------------------
    auto sphere_residual = [](const Eigen::Vector3d& p,
                              const Eigen::Vector3d& c,
                              double r) {
        return std::abs((p - c).squaredNorm() - r*r);
    };

    const double err1 = sphere_residual(candidate, C1, r1);
    const double err2 = sphere_residual(candidate, C2, r2);
    const double err3 = sphere_residual(candidate, C3, r3);
    const double max_err = std::max({err1, err2, err3});

    ok = (max_err < 1e-8 * (r1 + r2 + r3));

    double pos_err = (candidate - x_prev).norm();
    if (pos_err > 1e-3) ok = false;

    return ok ? candidate : x_prev;
}


RobotDynamics::FKSolution RobotDynamics::iterativeFK(
    const Eigen::Vector3d& theta,
    const Eigen::Vector3d& ref_pos,
    double tolerance,
    int max_iter) const
{
    FKSolution result;
    Eigen::Vector3d p = ref_pos;
    result.constraint_error = std::numeric_limits<double>::max();
    result.valid = false;
    
    // Tuning parameters
    const double max_cartesian_step = 0.05;  // Max Cartesian step (m)
    const double max_joint_step = 0.1;        // Max joint-space step (rad)
    const double min_det = 1e-12;              // Singularity threshold
    const double tau = 1e-3;                  // Levenberg damping factor

    // Perturbation to break symmetry
    const double perturb_size = 1e-6;
    for (int perturb = 0; perturb < 2; perturb++) {
        Eigen::Vector3d f = constraintEquations(p, theta);
        Eigen::Matrix3d J = computeConstraintJacobian(p, theta);
        Eigen::Vector3d delta_p = -J.transpose() * f;
        double step_norm = delta_p.norm();
        if (step_norm > 0) {
            delta_p = perturb_size * (delta_p / step_norm);
            p += delta_p;
        }
    }

    for(int i = 0; i < max_iter; ++i) {
        // Compute constraint residuals
        const Eigen::Vector3d f = constraintEquations(p, theta);
        const double err_norm = f.norm();
        
        // Update best solution
        if(err_norm < result.constraint_error) {
            result.position = p;
            result.constraint_error = err_norm;
            result.valid = true;
        }

        // Check convergence
        if(err_norm < tolerance) {
            return result;
        }

        // Compute Jacobian
        const Eigen::Matrix3d J = computeConstraintJacobian(p, theta);
        
        // Compute row norms for Levenberg damping
        Eigen::Vector3d row_norms = J.rowwise().norm();
        double max_row_norm = row_norms.maxCoeff();
        double mu = tau * max_row_norm;
        
        // Compute step direction
        Eigen::Vector3d delta_p;
        double detJ = J.determinant();
        
        if (std::abs(detJ) < min_det) {
            // Fallback to gradient descent when near singularity
            delta_p = -J.transpose() * f;
        } else {
            // Levenberg-Marquardt step: (J^T*J + mu*I) * delta_p = -J^T*f
            Eigen::Matrix3d A = J.transpose() * J + mu * Eigen::Matrix3d::Identity();
            Eigen::LDLT<Eigen::Matrix3d> ldlt(A);
            
            if (ldlt.info() == Eigen::Success) {
                delta_p = ldlt.solve(-J.transpose() * f);
            } else {
                // Secondary fallback to gradient
                delta_p = -J.transpose() * f;
            }
        }

        // Limit Cartesian step size
        double cart_step_size = delta_p.norm();
        if(cart_step_size > max_cartesian_step) {
            delta_p *= max_cartesian_step / cart_step_size;
        }

        // Compute joint-space step equivalent
        Eigen::Matrix3d K = computeK(p, theta);
        Eigen::Matrix3d Kinv = dampedPseudoInverse(K);
        Eigen::Vector3d joint_step = Kinv * J * delta_p;
        
        // Limit joint-space step size
        double joint_step_size = joint_step.norm();
        if(joint_step_size > max_joint_step) {
            delta_p *= max_joint_step / joint_step_size;
        }

        // Apply update
        p += delta_p;
    }

    result.valid = (result.constraint_error < tolerance);
    return result;
}

// Constraint equations for FK
Eigen::Vector3d RobotDynamics::constraintEquations(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& theta) const
{
    Eigen::Vector3d residuals;

    // Compute S joint positions
    Eigen::Vector3d s1(
        params_.a1x + params_.l_11 * cos(theta(0)),
        params_.a1y + params_.l_11 * sin(theta(0)),
        params_.h_1
    );
    
    Eigen::Vector3d s2(
        params_.a2x + params_.l_21 * cos(theta(1)),
        params_.a2y + params_.l_21 * sin(theta(1)),
        params_.h_2
    );
    
    Eigen::Vector3d s3(
        params_.a3x + params_.l_31 * cos(theta(2)),
        params_.a3y,
        params_.h_3 + params_.l_31 * sin(theta(2))
    );
    
    Eigen::Vector3d offset1(0, 0, params_.d - params_.v_1);
    Eigen::Vector3d offset2(0, 0, params_.d - params_.v_2);
    Eigen::Vector3d offset3(0, 0, params_.d - params_.v_3);

    // For each leg: ||p + offset - s_i||² - l_i2² = 0
    residuals(0) = (p + offset1 - s1).squaredNorm() - pow(params_.l_12, 2);    
    residuals(1) = (p + offset2 - s2).squaredNorm() - pow(params_.l_22, 2);    
    residuals(2) = (p + offset3 - s3).squaredNorm() - pow(params_.l_32, 2);
    
    return residuals;
}

Eigen::Matrix3d RobotDynamics::computeConstraintJacobian(
    const Eigen::Vector3d& p, 
    const Eigen::Vector3d& theta) const 
{
    Eigen::Matrix3d J_constraint;
    
    // Compute s_i positions (same as constraintEquations)
    Eigen::Vector3d s1(
        params_.a1x + params_.l_11 * cos(theta(0)),
        params_.a1y + params_.l_11 * sin(theta(0)),
        params_.h_1
    );
    
    Eigen::Vector3d s2(
        params_.a2x + params_.l_21 * cos(theta(1)),
        params_.a2y + params_.l_21 * sin(theta(1)),
        params_.h_2
    );
    
    Eigen::Vector3d s3(
        params_.a3x + params_.l_31 * cos(theta(2)),
        params_.a3y,
        params_.h_3 + params_.l_31 * sin(theta(2))
    );

    // Compute b_i = p + offset
    Eigen::Vector3d b1 = p + Eigen::Vector3d(0,0,params_.d-params_.v_1);
    Eigen::Vector3d b2 = p + Eigen::Vector3d(0,0,params_.d-params_.v_2);
    Eigen::Vector3d b3 = p + Eigen::Vector3d(0,0,params_.d-params_.v_3);

    // dF/dp = 2*(b_i - s_i)^T for each leg
    J_constraint.row(0) = 2.0 * (b1 - s1).transpose();  // x-components
    J_constraint.row(1) = 2.0 * (b2 - s2).transpose();  // y-components
    J_constraint.row(2) = 2.0 * (b3 - s3).transpose();  // z-components

    return J_constraint;
}