#include "Controller.hpp"
#include <algorithm>
#include <stdexcept>

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

    // Compute torque
    Eigen::Matrix3d M = robot.computeMassMatrix(theta, current_pos);
    Eigen::Vector3d G = robot.computeGravity(theta, current_pos);

    Eigen::Vector3d tau = M * theta_ddot_desired + G;

    // Apply torque limits
    return tau.cwiseMax(-torque_limit).cwiseMin(torque_limit);
}