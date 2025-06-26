#include "Controller.hpp"
#include <algorithm>
#include <stdexcept>
#include <cmath>


TrajectoryPoint Controller::interpolateTrajectory(
    const std::vector<TrajectoryPoint>& traj,
    double t_query
) const {
    if (traj.empty()) {
        throw std::runtime_error("Trajectory is empty.");
    }    

    // Clamp to trajectory bounds
    if (t_query <= traj.front().t) return traj.front();
    if (t_query >= traj.back().t) return traj.back();

    // Binary search for bracketing interval
    auto it = std::lower_bound(traj.begin(), traj.end(), t_query,
        [](const TrajectoryPoint& p, double t) { return p.t < t; });
    
    if (it == traj.begin()) return traj.front();
    const auto& p1 = *it;
    const auto& p0 = *(it - 1);

    const double h = p1.t - p0.t;
    if (h < 1e-10) return p0;  // Avoid division by zero

    const double s = (t_query - p0.t) / h;  // Normalized time [0,1]
    const double s2 = s * s;
    const double s3 = s2 * s;
    const double s4 = s3 * s;
    const double s5 = s4 * s;

    // Quintic Hermite basis functions for position
    const double H00 = 1 - 10*s3 + 15*s4 - 6*s5;
    const double H10 = 10*s3 - 15*s4 + 6*s5;
    const double H01 = s - 6*s3 + 8*s4 - 3*s5;
    const double H11 = -4*s3 + 7*s4 - 3*s5;
    const double H02 = (s2 - 3*s3 + 3*s4 - s5) / 2;
    const double H12 = (s3 - 2*s4 + s5) / 2;

    // Interpolate position
    Eigen::Vector3d x = p0.x * H00 + p1.x * H10 + 
                        h * (p0.x_dot * H01 + p1.x_dot * H11) + 
                        h*h * (p0.x_ddot * H02 + p1.x_ddot * H12);

    // Derivatives for velocity (with respect to s)
    const double dH00 = -30*s2 + 60*s3 - 30*s4;
    const double dH10 = 30*s2 - 60*s3 + 30*s4;
    const double dH01 = 1 - 18*s2 + 32*s3 - 15*s4;
    const double dH11 = -12*s2 + 28*s3 - 15*s4;
    const double dH02 = (2*s - 9*s2 + 12*s3 - 5*s4) / 2;
    const double dH12 = (3*s2 - 8*s3 + 5*s4) / 2;

    // Interpolate velocity (dp/ds) then convert to dp/dt by dividing by h
    Eigen::Vector3d x_dot = (p0.x * dH00 + p1.x * dH10 + 
                             h * (p0.x_dot * dH01 + p1.x_dot * dH11) + 
                             h*h * (p0.x_ddot * dH02 + p1.x_ddot * dH12)) / h;

    // Second derivatives for acceleration (with respect to s)
    const double d2H00 = -60*s + 180*s2 - 120*s3;
    const double d2H10 = 60*s - 180*s2 + 120*s3;
    const double d2H01 = -36*s + 96*s2 - 60*s3;
    const double d2H11 = -24*s + 84*s2 - 60*s3;
    const double d2H02 = 1 - 9*s + 18*s2 - 10*s3;
    const double d2H12 = 3*s - 12*s2 + 10*s3;

    // Interpolate acceleration (d²p/ds²) then convert to d²p/dt² by dividing by h²
    Eigen::Vector3d x_ddot = (p0.x * d2H00 + p1.x * d2H10 + 
                              h * (p0.x_dot * d2H01 + p1.x_dot * d2H11) + 
                              h*h * (p0.x_ddot * d2H02 + p1.x_ddot * d2H12)) / (h*h);

    return {t_query, x, x_dot, x_ddot};
}

Eigen::Vector3d Controller::computeMPCTorque(
    const TrajectoryPoint& desired_state,
    const Eigen::Vector3d& current_pos,
    const Eigen::Vector3d& current_vel,
    const Eigen::Vector3d& current_accel,
    const Eigen::Vector3d& theta,
    const Eigen::Vector3d& theta_dot,
    RobotDynamics& robot,
    double dt,
    Eigen::Vector3d& integral_error
) const {
    // Extract desired states
    const Eigen::Vector3d& desired_pos = desired_state.x;
    const Eigen::Vector3d& desired_vel = desired_state.x_dot;
    const Eigen::Vector3d& desired_accel = desired_state.x_ddot;

    // Compute errors
    Eigen::Vector3d error_pos = desired_pos - current_pos;
    Eigen::Vector3d error_vel = desired_vel - current_vel;

    // Update integral error with anti-windup
    integral_error += error_pos * dt;
    integral_error = integral_error.cwiseMax(-integral_max).cwiseMin(integral_max);

    // Combine accelerations
    Eigen::Vector3d x_ddot_desired = desired_accel +
                                Kp.cwiseProduct(error_pos) + 
                                Kd.cwiseProduct(error_vel) + 
                                Ki.cwiseProduct(integral_error);
                                
    // Get Jacobian and dynamics
    Eigen::Matrix3d J = robot.computeJ(current_pos, theta);
    Eigen::Matrix3d J_dot = robot.computeJDot(current_vel, theta, theta_dot);
    Eigen::Matrix3d K = robot.computeK(current_pos, theta);
    Eigen::Matrix3d K_dot = robot.computeKDot(current_pos, current_vel, theta, theta_dot);

    // Convert task-space acceleration to joint-space
    Eigen::Matrix3d K_inv = robot.dampedPseudoInverse(K);
    Eigen::Vector3d theta_ddot_desired = K_inv * (- K_dot * theta_dot + (J * x_ddot_desired + J_dot * current_vel));

    // Compute Model
    Eigen::Matrix3d M = robot.computeMassMatrix(theta, current_pos);
    Eigen::Vector3d G = robot.computeGravity(theta, current_pos);

    Eigen::Vector3d tau_desired = M * theta_ddot_desired + G;

    const std::array<double,3> Gr = {1.0, 1.0, 20.0}; // gear ratios

    for (int i=0;i<3;++i) {
        double tau_max = Gr[i]*eta_fw * K_t * Imax;
        double tau_safe = 0.8 * tau_max;
        // Real torque limit
        // tau_desired[i] = std::clamp(tau_desired[i], -tau_safe, +tau_safe); 
        tau_desired[i] = std::clamp(tau_desired[i], -30.0, +30.0);   
    }


    return tau_desired;
}

Eigen::Vector3d Controller::computeMPCStepRateTorque(
    const TrajectoryPoint& desired_state,
    const Eigen::Vector3d& current_pos,
    const Eigen::Vector3d& current_vel,
    const Eigen::Vector3d& current_accel,
    const Eigen::Vector3d& theta,
    const Eigen::Vector3d& theta_dot,
    RobotDynamics& robot,
    double dt,
    Eigen::Vector3d& integral_error,
    Eigen::Vector3d& delta_prev
) const {
    // Extract desired states
    const Eigen::Vector3d& desired_pos = desired_state.x;
    const Eigen::Vector3d& desired_vel = desired_state.x_dot;
    const Eigen::Vector3d& desired_accel = desired_state.x_ddot;

    // Compute errors
    Eigen::Vector3d error_pos = desired_pos - current_pos;
    Eigen::Vector3d error_vel = desired_vel - current_vel;

    // Update integral error with anti-windup
    integral_error += error_pos * dt;
    integral_error = integral_error.cwiseMax(-integral_max).cwiseMin(integral_max);

    // Combine accelerations
    Eigen::Vector3d x_ddot_desired = desired_accel +
                                Kp.cwiseProduct(error_pos) + 
                                Kd.cwiseProduct(error_vel) + 
                                Ki.cwiseProduct(integral_error);
                                
    // Get Jacobian and dynamics
    Eigen::Matrix3d J = robot.computeJ(current_pos, theta);
    Eigen::Matrix3d J_dot = robot.computeJDot(current_vel, theta, theta_dot);
    Eigen::Matrix3d K = robot.computeK(current_pos, theta);
    Eigen::Matrix3d K_dot = robot.computeKDot(current_pos, current_vel, theta, theta_dot);

    // Convert task-space acceleration to joint-space
    Eigen::Matrix3d K_inv = robot.dampedPseudoInverse(K);
    Eigen::Vector3d theta_ddot_desired = K_inv * (- K_dot * theta_dot + (J * x_ddot_desired + J_dot * current_vel));

    // Compute Model
    Eigen::Matrix3d M = robot.computeMassMatrix(theta, current_pos);
    Eigen::Vector3d G = robot.computeGravity(theta, current_pos);

    Eigen::Vector3d tau_desired = M * theta_ddot_desired + G;

    const std::array<double,3> Gr = {1.0, 1.0, 20.0}; // gear ratios
    Eigen::Vector3d delta_des;
    for (int i=0;i<3;++i) {
        double tau_max = Gr[i]*eta_fw * K_t * Imax;
        double tau_safe = 0.8 * tau_max;  
        delta_des(i) = std::asin( std::clamp( tau_desired(i)/tau_safe, -1.0, 1.0) );
    }


    //  Open-loop phase-lag tracking
    static Eigen::Vector3d delta_int = Eigen::Vector3d::Zero();// Integral lag error

    if (integral_error.isZero(1e-12)) {
        delta_int.setZero();
    }
    
    constexpr double K_delta  = 8.0e3;   // pulses · s⁻¹ / rad
    constexpr double Ki_delta = 2.0*K_delta;                          

    Eigen::Vector3d rotor_vel_est;
    Eigen::Vector3d f_pulse;
    for (int i = 0; i < 3; ++i)
    {
        rotor_vel_est(i) = Gr[i] * theta_dot(i);

        // lag error in (-π, π]
        double err = std::atan2(std::sin(delta_des(i) - delta_prev(i)),
                                std::cos(delta_des(i) - delta_prev(i)));

        // PI on lag error (works without encoder)
        delta_int(i) = delta_int(i) + err * dt;
        double delta_rate = K_delta*err + Ki_delta*delta_int(i);   // rad s⁻¹

        double theta_stator_rate = rotor_vel_est(i) + delta_rate;
        f_pulse(i) = std::clamp(theta_stator_rate / (kMicroStepRad / Gr[i]),
                                -pulses_max, +pulses_max);

        delta_prev(i) += (theta_stator_rate - rotor_vel_est(i)) * dt;
        delta_prev(i) = std::atan2(std::sin(delta_prev(i)), std::cos(delta_prev(i)));
    }

    return f_pulse;
}