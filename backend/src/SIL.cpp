#include "RobotModel.hpp"
#include "TorqueQP.hpp"
#include "SIL.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <algorithm>

Trajectory parseTrajectoryFromJson(const nlohmann::json& j) {
    Trajectory traj;
    for (const auto& point : j) {
        TrajectoryPoint tp;
        tp.t = point.at("t").get<double>();
        std::vector<double> x_vec = point.at("x").get<std::vector<double>>();
        if (x_vec.size() != 3)
            throw std::runtime_error("Invalid x size in trajectory point");
        tp.x = Eigen::Vector3d(x_vec[0], x_vec[1], x_vec[2]);
        traj.push_back(tp);
    }
    return traj;
}

TrajectoryPoint interpolateTrajectory(const Trajectory& traj, double t_query) {
    if (traj.empty()) {
        throw std::runtime_error("Trajectory is empty.");
    }

    // Clamp if outside bounds
    if (t_query <= traj.front().t) return traj.front();
    if (t_query >= traj.back().t) return traj.back();

    // Find interval: t_i <= t_query < t_{i+1}
    for (size_t i = 0; i < traj.size() - 1; ++i) {
        const auto& p0 = traj[i];
        const auto& p1 = traj[i + 1];

        if (t_query >= p0.t && t_query < p1.t) {
            double alpha = (t_query - p0.t) / (p1.t - p0.t);
            Eigen::Vector3d interp_x = (1.0 - alpha) * p0.x + alpha * p1.x;

            // Optional: velocity estimation
            Eigen::Vector3d vel = (p1.x - p0.x) / (p1.t - p0.t);

            return { t_query, interp_x };  // add vel if needed
        }
    }

    throw std::runtime_error("Interpolation failed — time not bracketed.");
}


Eigen::Vector3d computeMPCTorque(
    const Eigen::Vector3d& desired_pos,
    const Eigen::Vector3d& current_pos,
    const Eigen::Vector3d& current_vel,
    const Eigen::Vector3d& theta,
    const Eigen::Vector3d& theta_dot,
    RobotDynamics& robot,
    double dt
) {
    //static TorqueQP qp_solver(robot);
    //return qp_solver.solveMPC(current_pos, current_vel, desired_pos, theta, theta_dot, dt);

    // PID Gains (tune these values)
    const double Kp = 150.0;  // Proportional gain
    const double Kd = 30.0;   // Derivative gain
    const double Ki = 5.0;   // Integral gain
    const double torque_limit = 100.0;  // Torque saturation

    // Static variable to accumulate the integral error
    static Eigen::Vector3d error_integral = Eigen::Vector3d::Zero();

    // Compute task-space error and derivative
    Eigen::Vector3d error = desired_pos - current_pos;
    Eigen::Vector3d error_derivative = -current_vel;  // Assuming desired velocity = 0

    // Integrate error (anti-windup: clamp accumulation)
    error_integral += error * dt;
    const double max_integral = 1.0; 
    for (int i = 0; i < 3; ++i) {
        if (std::abs(error_integral[i]) > max_integral) {
            error_integral[i] = std::copysign(max_integral, error_integral[i]);
        }
    }

    // Task-space PID force
    Eigen::Vector3d F = Kp * error + Kd * error_derivative + Ki * error_integral;

    // Compute Jacobian and dynamics terms
    Eigen::Matrix3d J = robot.computeJ(current_pos, theta);
    Eigen::Vector3d C = robot.computeCoriolis(current_pos, current_vel, theta, theta_dot);
    Eigen::Vector3d G = robot.computeGravity(theta);

    // Platform forces
    Eigen::Vector3d f_e = robot.computePlatformForces(theta, theta_dot);

    // Map task-space force to joint torques
    Eigen::Vector3d tau = J.transpose() * (F + f_e) + C + G;

    // Apply torque limits
    tau = tau.cwiseMax(-torque_limit).cwiseMin(torque_limit);

    return tau;
}

void run_sil_simulation(
    const Trajectory& traj,
    std::vector<nlohmann::json>& results_out)
    {

    RobotDynamics robot;
    if (!robot.loadFromConfig("./configs/temp.json")) {
        std::cerr << "Failed to initialize robot model\n";
        return;
    }

    results_out.clear();

    // Initial states
    Eigen::Vector3d x_0 = robot.getInitialPosition();
    Eigen::Vector3d theta_0(-2.75719588, -2.75719588 + 1e-3, -1.92068673);
    Eigen::Vector3d theta = theta_0;
    Eigen::Vector3d theta_dot = Eigen::Vector3d::Zero();
    Eigen::Vector3d theta_ddot = Eigen::Vector3d::Zero();
    Eigen::Vector3d x = x_0;
    Eigen::Vector3d x_dot = Eigen::Vector3d::Zero();
    Eigen::Vector3d x_ddot = Eigen::Vector3d::Zero();

    // Adaptive stepping parameters
    double t = 0.0;
    double t_final = traj.back().t;
    double dt = 0.001;
    double dt_min = 1e-8;
    double dt_max = 0.01;
    bool first_step = true;
    double error_tol = 1e-3;
    int max_attempts = 50;
    int total_rejects = 0;
    int max_total_rejects = 10000;
    double prev_energy = 0.0;

    try {
        while (t < t_final) {

            if (first_step) {
                dt = 1e-6;  // Extremely small initial step
                first_step = false;
            }

            int attempts = 0;
            bool step_accepted = false;
            
            // Store previous state
            Eigen::Vector3d theta_prev = theta;
            Eigen::Vector3d theta_dot_prev = theta_dot;
            Eigen::Vector3d x_prev = x;
            Eigen::Vector3d x_dot_prev = x_dot;

            // Get desired position
            //Eigen::Vector3d desired_pos = x_0 - Eigen::Vector3d(0, 0, 5e-2); // Constant position for testing

            TrajectoryPoint target = interpolateTrajectory(traj, t);
            Eigen::Vector3d desired_pos = target.x;

            // Compute torque
            Eigen::Vector3d torque = computeMPCTorque(
                desired_pos, x, x_dot, theta, theta_dot, robot, dt
            );

            // RK4 integration with proper state propagation
            auto computeDerivatives = [&](const Eigen::Vector3d& th, 
                                         const Eigen::Vector3d& th_dot,
                                         const Eigen::Vector3d& x_current,
                                         const Eigen::Vector3d& x_dot_current) {
                Eigen::Matrix3d J = robot.computeJ(x_current, th);
                Eigen::Matrix3d J_dot = robot.computeJDot(x_dot_current, th, th_dot);
                return robot.computeForwardDynamics(th, th_dot, torque, x_current, 
                                                   x_dot);
            };

            if (t == 0.0) {
                Eigen::Matrix3d J = robot.computeJ(x, theta);
                std::cout << "Initial J:\n" << J << "\nDeterminant: " << J.determinant() << std::endl;
                std::cout << "Initial torque: " << torque.transpose() << std::endl;
                theta_ddot = computeDerivatives(theta, theta_dot, x, x_dot);
                std::cout << "Initial theta_ddot: " << theta_ddot.transpose() << std::endl;
            }

            while (!step_accepted && attempts < max_attempts) {
                // RK4 stages with intermediate state tracking
                Eigen::Vector3d k1_th_dot = theta_dot;
                Eigen::Vector3d k1_th_ddot = computeDerivatives(theta, theta_dot, x, x_dot);

                // Stage k2
                Eigen::Vector3d k2_th = theta + 0.5 * dt * k1_th_dot;
                Eigen::Vector3d k2_th_dot = theta_dot + 0.5 * dt * k1_th_ddot;
                Eigen::Vector3d k2_x = x + 0.5 * dt * x_dot;
                // Compute x_dot for k2 using current k2_th and k2_th_dot
                Eigen::Matrix3d J_k2 = robot.computeJ(k2_x, k2_th);
                Eigen::Vector3d k2_x_dot = J_k2 * k2_th_dot;
                Eigen::Vector3d k2_th_ddot = computeDerivatives(k2_th, k2_th_dot, k2_x, k2_x_dot);

                // Stage k3
                Eigen::Vector3d k3_th = theta + 0.5 * dt * k2_th_dot;
                Eigen::Vector3d k3_th_dot = theta_dot + 0.5 * dt * k2_th_ddot;
                Eigen::Vector3d k3_x = x + 0.5 * dt * x_dot;
                // Compute x_dot for k3
                Eigen::Matrix3d J_k3 = robot.computeJ(k3_x, k3_th);
                Eigen::Vector3d k3_x_dot = J_k3 * k3_th_dot;
                Eigen::Vector3d k3_th_ddot = computeDerivatives(k3_th, k3_th_dot, k3_x, k3_x_dot);

                // Stage k4
                Eigen::Vector3d k4_th = theta + dt * k3_th_dot;
                Eigen::Vector3d k4_th_dot = theta_dot + dt * k3_th_ddot;
                Eigen::Vector3d k4_x = x + dt * x_dot;
                // Compute x_dot for k4
                Eigen::Matrix3d J_k4 = robot.computeJ(k4_x, k4_th);
                Eigen::Vector3d k4_x_dot = J_k4 * k4_th_dot;
                Eigen::Vector3d k4_th_ddot = computeDerivatives(k4_th, k4_th_dot, k4_x, k4_x_dot);

                // Compute RK4 and RK3 estimates for θ and θ_dot
                Eigen::Vector3d theta_rk4 = theta + (dt/6.0) * (k1_th_dot + 2*k2_th_dot + 2*k3_th_dot + k4_th_dot);
                Eigen::Vector3d theta_dot_rk4 = theta_dot + (dt/6.0) * (k1_th_ddot + 2*k2_th_ddot + 2*k3_th_ddot + k4_th_ddot);

                // RK3 estimate (Bogacki-Shampine method)
                Eigen::Vector3d theta_rk3 = theta + dt * (2.0/9.0 * k1_th_dot + 1.0/3.0 * k2_th_dot + 4.0/9.0 * k3_th_dot);
                Eigen::Vector3d theta_dot_rk3 = theta_dot + dt * (2.0/9.0 * k1_th_ddot + 1.0/3.0 * k2_th_ddot + 4.0/9.0 * k3_th_ddot);

                // Compute error using state differences
                Eigen::Vector3d error_theta = theta_rk4 - theta_rk3;
                Eigen::Vector3d error_theta_dot = theta_dot_rk4 - theta_dot_rk3;

                // Combine errors with scaling
                double error = std::max(
                    error_theta.cwiseAbs().maxCoeff(),
                    error_theta_dot.cwiseAbs().maxCoeff()
                    );

                if (error > error_tol) {
                    double safety = 0.9;
                    double max_scale = 5.0;
                    double min_scale = 0.3;
                    if (error > 0) {
                        double scale = safety * std::pow(error_tol / error, 0.25);
                        scale = std::clamp(scale, min_scale, max_scale);
                        dt = std::max(scale * dt, dt_min);
                    }

                    attempts++;
                    total_rejects++;
                    
                    // Rollback state
                    theta = theta_prev;
                    theta_dot = theta_dot_prev;
                    x = x_prev;
                    x_dot = x_dot_prev;

                    if (total_rejects > max_total_rejects) {
                        throw std::runtime_error("Simulation diverged: Too many rejected steps");
                    }
                } else {
                    // Accept step and update state
                    theta += (dt/6.0) * (k1_th_dot + 2*k2_th_dot + 2*k3_th_dot + k4_th_dot);
                    theta_dot += (dt/6.0) * (k1_th_ddot + 2*k2_th_ddot + 2*k3_th_ddot + k4_th_ddot);
                    
                    // Update task-space variables
                    Eigen::Matrix3d J = robot.computeJ(x, theta);
                    x_dot = J * theta_dot;
                    x += x_dot * dt;

                    RobotDynamics::IKSolution theta_kine = robot.invKineSinglePoint(x, theta);  // Project joint angles back to match Cartesian state
                    theta = theta_kine.theta;
                    theta_dot = robot.dampedPseudoInverse(J) * x_dot;

                    // Energy check
                    double kinetic = 0.5 * theta_dot.dot(robot.computeMassMatrix(theta, x) * theta_dot);
                    double potential = robot.computeGravity(theta).dot(theta);
                    if (t > dt && std::abs(kinetic + potential - prev_energy) > 0.1*prev_energy) {
                        throw std::runtime_error("Energy violation detected");
                    }
                    prev_energy = kinetic + potential;

                    // Build JSON frame
                    nlohmann::json frame;
                    frame["t"] = t;
                    frame["x"] = {x[0], x[1], x[2]};
                    frame["x_dot"] = {x_dot[0], x_dot[1], x_dot[2]};
                    frame["theta"] = {theta[0], theta[1], theta[2]};
                    frame["theta_dot"] = {theta_dot[0], theta_dot[1], theta_dot[2]};
                    frame["tau"] = {torque[0], torque[1], torque[2]};

                    results_out.push_back(frame);

                    t += dt;
                    dt = std::min(dt * 1.2, dt_max);
                    step_accepted = true;
                }
            }

            if (!step_accepted) {
                if (dt <= dt_min) {
                    std::cerr << "Critical error: Minimum step size reached at t=" << t << std::endl;
                    break;
                } else {
                    // Force Euler step
                    Eigen::Vector3d th_ddot = computeDerivatives(theta, theta_dot, x, x_dot);
                    theta_dot += dt * th_ddot;
                    theta += theta_dot * dt;
                    
                    Eigen::Matrix3d J = robot.computeJ(x, theta);
                    x_dot = J * theta_dot;
                    x += x_dot * dt;

                    RobotDynamics::IKSolution theta_kine = robot.invKineSinglePoint(x, theta);  // Project joint angles back to match Cartesian state
                    theta = theta_kine.theta;
                    theta_dot = robot.dampedPseudoInverse(J) * x_dot;
                    
                    t += dt;

                    // Build JSON frame
                    nlohmann::json frame;
                    frame["t"] = t;
                    frame["x"] = {x[0], x[1], x[2]};
                    frame["x_dot"] = {x_dot[0], x_dot[1], x_dot[2]};
                    frame["theta"] = {theta[0], theta[1], theta[2]};
                    frame["theta_dot"] = {theta_dot[0], theta_dot[1], theta_dot[2]};
                    frame["tau"] = {torque[0], torque[1], torque[2]};

                    results_out.push_back(frame);
                }
            }

            // Final sanity check
            if (!theta.allFinite() || !x.allFinite()) {
                throw std::runtime_error("Non-finite values detected");
                return;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Simulation error: " << e.what() << "\n";
        return;
    }
    return;
}