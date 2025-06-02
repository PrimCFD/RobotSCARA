#include "RobotModel.hpp"
#include "SIL.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <algorithm>

void run_sil_simulation(
    const std::vector<Waypoint>& binary_traj,
    std::vector<Frame>& results_out)
    {

    // Pre-allocate memory for results
    size_t estimated_frames = static_cast<size_t>((binary_traj.back().t - binary_traj.front().t) * 1000);
    results_out.reserve(std::min(estimated_frames, static_cast<size_t>(MAX_FRAME_POINTS)));
    
    // Convert trajectory in chunks
    std::vector<TrajectoryPoint> traj;
    traj.reserve(binary_traj.size());
    
    for (size_t i = 0; i < binary_traj.size(); ++i) {
        traj.push_back(WaypointToTrajectoryPoint(binary_traj[i]));
        
        // Periodically report progress
        if (i % 1000 == 0) {
            std::cout << "Converting waypoints: " << i << "/" 
                      << binary_traj.size() << std::endl;
        }
    }

    RobotDynamics robot;
    robot.loadHardcodedParams();

    // Ensure trajectory is valid
    if (traj.empty()) {
        std::cerr << "Error: Empty trajectory provided\n";
        return;
    }

    // Create and configure controller
    Controller controller;

    // Use trajectory's starting point as initial state
    Eigen::Vector3d x_0 = traj.front().x;
    Eigen::Vector3d x_dot_0 = traj.front().x_dot;
    Eigen::Vector3d x_ddot_0 = traj.front().x_ddot;  // Capture initial acceleration

    TrajectoryPoint target_ini;
    target_ini.t = 0.0, target_ini.x = x_0, target_ini.x_dot = x_dot_0, target_ini.x_ddot = x_ddot_0;

    // Solve inverse kinematics for initial position
    Eigen::Vector3d initial_guess(-2.72, -0.38, -1.92);
    auto init_sol = robot.invKineSinglePoint(x_0, initial_guess);

    if (!init_sol.valid) {
        throw std::runtime_error("No valid IK solution at t=0");
    }

    Eigen::Vector3d theta = init_sol.theta;
    Eigen::Vector3d x_ref;

    // FK validation
    Eigen::Vector3d x0_fk = robot.forwardKinematics(init_sol.theta, x_0);

    if ((x0_fk - x_0).norm() > 1e-3) {
        std::cerr << "Warning: Initial FK error: " 
                << (x0_fk - x_0).norm() << std::endl;
        // Use the IK solution anyway to proceed
        x_ref = x0_fk;
    } else {
        x_ref = x0_fk;
    }


    // Compute initial joint velocity from Cartesian velocity
    Eigen::Matrix3d J_init = robot.computeJ(x_ref, theta);
    Eigen::Matrix3d K_init = robot.computeK(x_ref, theta);
    Eigen::Matrix3d K_inv_init = robot.dampedPseudoInverse(K_init);
    Eigen::Vector3d theta_dot = K_inv_init * J_init * x_dot_0;

    Eigen::Vector3d integral_error = Eigen::Vector3d::Zero();

    Eigen::Vector3d initial_torque = controller.computeMPCTorque(
        target_ini,
            x_0,        // current_pos
            x_dot_0,    // current_vel
            x_ddot_0,   // current_accel
            theta,
            theta_dot,
            robot,
            0.0,
            integral_error);

    Eigen::Vector3d theta_ddot = robot.computeForwardDynamics(
        theta, theta_dot, initial_torque, x_ref, x_dot_0);

    // Initialize state variables
    Eigen::Vector3d x = x_ref;
    Eigen::Vector3d x_dot = x_dot_0;
    Eigen::Vector3d x_ddot = x_ddot_0;

    // Adaptive stepping parameters
    double t = 0.0;
    double t_final = traj.back().t;
    double dt = 0.001;
    double dt_min = 1e-6;
    double dt_max = 1e-2;
    bool first_step = true;
    double error_tol = 1e-3;
    int max_attempts = 50;
    int total_rejects = 0;
    int max_total_rejects = 100000;

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
            Eigen::Vector3d integral_error_prev = integral_error;

            Eigen::Vector3d x_ref_prev = x_ref;  // Save previous valid reference

            // Get desired position
            TrajectoryPoint target = controller.interpolateTrajectory(traj, t);

            // Compute torque
            Eigen::Vector3d torque = controller.computeMPCTorque(
                target,   // Combined desired state
                x,        // current_pos
                x_dot,    // current_vel
                x_ddot,   // current_accel
                theta,
                theta_dot,
                robot,
                dt,
                integral_error
            );

            if (std::isnan(torque.sum())) {
                // Diagnostic output
                std::cout << "NaN torque at t=" << t 
                        << " pos=" << x.transpose()
                        << " theta=" << theta.transpose() << std::endl;
                // Reset to zero to prevent propagation
                torque = Eigen::Vector3d::Zero();
            }

            // RK4 integration with proper state propagation
            auto computeDerivatives = [&](const Eigen::Vector3d& th, 
                                         const Eigen::Vector3d& th_dot) {

                Eigen::Vector3d fk_ref = x_ref_prev + x_dot * dt;
                Eigen::Vector3d x_current = robot.forwardKinematics(th, fk_ref);
                Eigen::Matrix3d J = robot.computeJ(x_current, th);
                Eigen::Matrix3d K = robot.computeK(x_current, th);
                Eigen::Matrix3d J_inv = robot.dampedPseudoInverse(J);
                Eigen::Vector3d x_dot_current = J_inv * K * th_dot;

                return robot.computeForwardDynamics(th, th_dot, torque, x_current, 
                                                   x_dot_current);
            };

            while (!step_accepted && attempts < max_attempts) {
                // RK4 stages with intermediate state tracking
                // Stage k1 
                Eigen::Vector3d k1_th_dot = theta_dot;
                Eigen::Vector3d k1_th_ddot = computeDerivatives(theta, theta_dot);

                // Stage k2
                Eigen::Vector3d k2_th = theta + 0.5 * dt * k1_th_dot;
                Eigen::Vector3d k2_th_dot = theta_dot + 0.5 * dt * k1_th_ddot;
                Eigen::Vector3d k2_th_ddot = computeDerivatives(k2_th, k2_th_dot);

                // Stage k3
                Eigen::Vector3d k3_th = theta + 0.5 * dt * k2_th_dot;
                Eigen::Vector3d k3_th_dot = theta_dot + 0.5 * dt * k2_th_ddot;
                Eigen::Vector3d k3_th_ddot = computeDerivatives(k3_th, k3_th_dot);

                // Stage k4
                Eigen::Vector3d k4_th = theta + dt * k3_th_dot;
                Eigen::Vector3d k4_th_dot = theta_dot + dt * k3_th_ddot;
                Eigen::Vector3d k4_th_ddot = computeDerivatives(k4_th, k4_th_dot);

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
                    integral_error = integral_error_prev;
                    x_ref = x_ref_prev;

                    if (total_rejects > max_total_rejects) {
                        throw std::runtime_error("Simulation diverged: Too many rejected steps");
                    }
                } else {
                    // Accept step and update state
                    theta += (dt/6.0) * (k1_th_dot + 2*k2_th_dot + 2*k3_th_dot + k4_th_dot);
                    theta_dot += (dt/6.0) * (k1_th_ddot + 2*k2_th_ddot + 2*k3_th_ddot + k4_th_ddot);

                    Eigen::Vector3d fk_ref = x_ref_prev + x_dot * dt;
                    x = robot.forwardKinematics(theta, fk_ref);
                    
                    // Update task-space variables
                    Eigen::Matrix3d J = robot.computeJ(x, theta);
                    Eigen::Matrix3d K = robot.computeK(x, theta);
                    Eigen::Matrix3d J_inv = robot.dampedPseudoInverse(J);

                    x_dot = J_inv * K * theta_dot;

                    // Compute new task-space acceleration
                    theta_ddot = computeDerivatives(theta, theta_dot);
                    Eigen::Matrix3d J_dot = robot.computeJDot(x_dot, theta, theta_dot);
                    Eigen::Matrix3d K_dot = robot.computeKDot(x, x_dot, theta, theta_dot);

                    x_ddot = J_inv * (- J_dot * x_dot + (K * theta_ddot + K_dot * theta_dot));

                    // Build binary frame
                    Frame frame = CreateFrame(t, x, x_dot, theta, theta_dot, torque);
                    results_out.push_back(frame);

                    x_ref = x;

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
                    Eigen::Vector3d th_ddot = computeDerivatives(theta, theta_dot);
                    theta_dot += dt * th_ddot;
                    theta += theta_dot * dt;

                    Eigen::Vector3d fk_ref = x_ref_prev + x_dot * dt;
                    x = robot.forwardKinematics(theta, fk_ref);

                    // Update task-space variables
                    Eigen::Matrix3d J = robot.computeJ(x, theta);
                    Eigen::Matrix3d K = robot.computeK(x, theta);
                    Eigen::Matrix3d J_inv = robot.dampedPseudoInverse(J);

                    x_dot = J_inv * K * theta_dot;

                    // Compute new task-space acceleration
                    theta_ddot = computeDerivatives(theta, theta_dot);
                    Eigen::Matrix3d J_dot = robot.computeJDot(x_dot, theta, theta_dot);
                    Eigen::Matrix3d K_dot = robot.computeKDot(x, x_dot, theta, theta_dot);

                    x_ddot = J_inv * (- J_dot * x_dot + (K * theta_ddot + K_dot * theta_dot));
                    
                    t += dt;
                    x_ref = x;

                    // Build binary frame
                    Frame frame = CreateFrame(t, x, x_dot, theta, theta_dot, torque);
                    results_out.push_back(frame);
                }
            }

            // Final sanity check
            if (!theta.allFinite() || !x.allFinite()) {
                throw std::runtime_error("Non-finite values detected");
                return;
            }

            // Periodically check memory usage
            if (results_out.size() % 10000 == 0) {
                std::cout << "Simulation progress: t=" << t << "/" << t_final
                        << " (" << (100 * t / t_final) << "%), "
                        << results_out.size() << " frames" << std::endl;
            }
            
            // Break if exceeding max frames
            if (results_out.size() >= MAX_FRAME_POINTS) {
                std::cerr << "Warning: Exceeded maximum frame count, truncating results" << std::endl;
                break;
            }
        
        }

     } catch (const std::exception& e) {
        std::cerr << "Simulation error: " << e.what() << "\n";
        return;
    }

    return;
}
