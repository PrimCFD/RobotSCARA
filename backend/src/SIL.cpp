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
    std::vector<Frame>& results_out,
    std::vector<IdealTorquePoint>& ideal_torques_out,
    const Eigen::Vector3d& elbow_pos,
    double l_arm_proth)
    {

    RobotDynamics robot_ideal;
    robot_ideal.loadHardcodedParams();
    robot_ideal.setElbowArm(elbow_pos, l_arm_proth);

    // Pre-compute ideal torques
    ideal_torques_out.clear();
    ideal_torques_out.reserve(binary_traj.size());

    Eigen::Vector3d theta_prev_py(-2.76, -0.38, 2.84);
    Eigen::Vector3d theta_prev = robot_ideal.toCppAngles(theta_prev_py);

    for (size_t i = 0; i < binary_traj.size(); ++i) {
        const Waypoint& wp = binary_traj[i];
        Eigen::Vector3d x(wp.x[0], wp.x[1], wp.x[2]);
        Eigen::Vector3d x_dot(wp.x_dot[0], wp.x_dot[1], wp.x_dot[2]);
        Eigen::Vector3d x_ddot(wp.x_ddot[0], wp.x_ddot[1], wp.x_ddot[2]);

        // Solve IK
        RobotDynamics::IKSolution ik_sol = robot_ideal.invKineSinglePoint(x, theta_prev);
        if (!ik_sol.valid) {
            ik_sol.theta = theta_prev;  // Use previous if invalid
        }
        theta_prev = ik_sol.theta;

        // Compute inverse dynamics
        RobotDynamics::DynamicsResult res = robot_ideal.computeInverseDynamics(x, x_dot, x_ddot, ik_sol.theta);

        Eigen::Vector3d theta_py = robot_ideal.toPyAngles(ik_sol.theta);
        Eigen::Vector3d theta_dot_py = robot_ideal.toPyDq(res.theta_dot);
        Eigen::Vector3d tau_py = robot_ideal.toPyTorque(res.torque);


        IdealTorquePoint itp;
        itp.t = wp.t;
        // Store joint positions/velocities
        for (int j = 0; j < 3; j++) {
            itp.theta[j] = theta_py(j);
            itp.theta_dot[j] = theta_dot_py(j);
        }
        if (res.success) {
            std::copy(tau_py.data(), tau_py.data()+3, itp.tau_ideal);
        } else {
            std::fill(itp.tau_ideal, itp.tau_ideal+3, 0.0);
        }
        ideal_torques_out.push_back(itp);
    }

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

    // Ensure trajectory is valid
    if (traj.empty()) {
        std::cerr << "Error: Empty trajectory provided\n";
        return;
    }

    RobotDynamics robot;
    robot.loadHardcodedParams();
    robot.setElbowArm(elbow_pos, l_arm_proth);

    // Create and configure controller
    Controller controller;

    // Use trajectory's starting point as initial state
    Eigen::Vector3d x_0 = traj.front().x;
    Eigen::Vector3d x_dot_0 = traj.front().x_dot;
    Eigen::Vector3d x_ddot_0 = traj.front().x_ddot;  // Capture initial acceleration

    TrajectoryPoint target_ini;
    target_ini.t = 0.0, target_ini.x = x_0, target_ini.x_dot = x_dot_0, target_ini.x_ddot = x_ddot_0;

    // Solve inverse kinematics for initial position
    Eigen::Vector3d initial_guess_py(-2.76, -0.38,  2.84);
    Eigen::Vector3d initial_guess = robot.toCppAngles(initial_guess_py);
    RobotDynamics::IKSolution init_sol = robot.invKineSinglePoint(x_0, initial_guess);

    std::cout<<robot.toPyAngles(init_sol.theta).transpose()<<std::endl;


    if (!init_sol.valid) {
        throw std::runtime_error("No valid IK solution at t=0");
    }

    Eigen::Vector3d theta = init_sol.theta;
    Eigen::Vector3d x_ref;

    // FK validation
    std::cout<<x_0.transpose()<<std::endl;
    Eigen::Vector3d x0_fk = robot.forwardKinematics(init_sol.theta, x_0);

    std::cout<<x0_fk.transpose()<<std::endl;


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

    Eigen::Vector3d theta_ddot = robot.computeForwardDynamics(theta, initial_torque, x_ref);

    // Initialize state variables
    Eigen::Vector3d x = x_ref;
    Eigen::Vector3d x_dot = x_dot_0;
    Eigen::Vector3d x_ddot = x_ddot_0;

    // Adaptive stepping parameters
    double t = 0.0;
    double t_final = traj.back().t;
    double dt = 1e-6;
    double dt_min = 1e-8;
    double dt_max = 1e-4;
    bool first_step = true;
    int max_attempts = 50;
    int total_rejects = 0;
    int max_total_rejects = 100000;

    // Error                

    Eigen::VectorXd absTol(9), relTol(9);
    absTol << 1e-4,1e-4,1e-4,     //  x,y,z
            1e-4,1e-4,1e-4,     //  θ₁,θ₂,θ₃
            1e-3,1e-3,1e-3;     //  θ̇
    relTol.setConstant(1e-3);

    auto scaledError = [&](const Eigen::VectorXd& y_old,
                        const Eigen::VectorXd& y_new,
                        const Eigen::VectorXd& y_err)
    {
        double rho = 0.0;
        for(int i = 0; i < y_err.size(); ++i)
        {
            double sc = absTol(i) + relTol(i) *
                        std::max(fabs(y_old(i)), fabs(y_new(i)));
            rho = std::max(rho, fabs(y_err(i)) / sc);
        }
        return rho;
    };

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

            Eigen::Vector3d x_prev = x;  // Save previous valid reference
            Eigen::Vector3d integral_error_prev = integral_error;

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

            auto torqueStage = [&](double t_local,
                            const Eigen::Vector3d& th,
                            const Eigen::Vector3d& thd,
                            const Eigen::Vector3d& x_current,
                            Eigen::Vector3d int_err) {

            TrajectoryPoint target = controller.interpolateTrajectory(traj, t_local);
            Eigen::Matrix3d J = robot.computeJ(x_current, th);
            Eigen::Matrix3d K = robot.computeK(x_current, th);
            Eigen::Matrix3d J_inv = robot.dampedPseudoInverse(J);
            return controller.computeMPCTorque(target,
                                            x_current,
                                            /*current vel*/ J_inv*K*thd,
                                            /*current acc*/ Eigen::Vector3d::Zero(),
                                            th, thd, robot, dt, int_err);
                                            
                            };

            // RK4 integration with proper state propagation


            while (!step_accepted && attempts < max_attempts) {
                // Stage k1 - Use current x
                Eigen::Vector3d k1_th_dot = theta_dot;
                Eigen::Vector3d torque_1 = torqueStage(t, theta, k1_th_dot, x_prev, integral_error_prev);
                Eigen::Vector3d k1_th_ddot = robot.computeForwardDynamics(theta, torque_1, x_prev);

                // Stage k2 - Update x_k2 and use it for dynamics
                Eigen::Vector3d k2_th = theta + 0.5 * dt * k1_th_dot;
                Eigen::Vector3d x_k2 = robot.forwardKinematics(k2_th, x_prev);
                Eigen::Vector3d k2_th_dot = theta_dot + 0.5 * dt * k1_th_ddot;
                // Recompute dynamics matrices for x_k2
                Eigen::Vector3d torque_2 = torqueStage(t + 0.5 * dt, k2_th, k2_th_dot, x_k2, integral_error_prev + 0.5 * dt * (target.x - x_k2));
                Eigen::Vector3d k2_th_ddot = robot.computeForwardDynamics(k2_th, torque_2, x_k2);  // Use x_k2

                // Stage k3 - Update x_k3 and use it for dynamics
                Eigen::Vector3d k3_th = theta + 0.5 * dt * k2_th_dot;
                Eigen::Vector3d x_k3 = robot.forwardKinematics(k3_th, x_k2);  // Use x_k2 as reference
                Eigen::Vector3d k3_th_dot = theta_dot + 0.5 * dt * k2_th_ddot;
                // Recompute dynamics matrices for x_k3
                Eigen::Vector3d torque_3 = torqueStage(t + 0.5 * dt, k3_th, k3_th_dot, x_k3, integral_error_prev + 0.5 * dt * (target.x - x_k3));
                Eigen::Vector3d k3_th_ddot = robot.computeForwardDynamics(k3_th, torque_3, x_k3);  // Use x_k3

                // Stage k4 - Update x_k4 and use it for dynamics
                Eigen::Vector3d k4_th = theta + dt * k3_th_dot;
                Eigen::Vector3d x_k4 = robot.forwardKinematics(k4_th, x_k3);  // Use x_k3 as reference
                Eigen::Vector3d k4_th_dot = theta_dot + dt * k3_th_ddot;
                // Recompute dynamics matrices for x_k4
                Eigen::Vector3d torque_4 = torqueStage(t + dt, k4_th, k4_th_dot, x_k4, integral_error_prev + 0.5 * dt * (target.x - x_k4));
                Eigen::Vector3d k4_th_ddot = robot.computeForwardDynamics(k4_th, torque_4, x_k4);  // Use x_k4

                // Compute RK4 and RK3 estimates for θ and θ_dot
                Eigen::Vector3d theta_rk4 = theta + (dt/6.0) * (k1_th_dot + 2*k2_th_dot + 2*k3_th_dot + k4_th_dot);
                Eigen::Vector3d theta_dot_rk4 = theta_dot + (dt/6.0) * (k1_th_ddot + 2*k2_th_ddot + 2*k3_th_ddot + k4_th_ddot);

                // RK3 estimate (Bogacki-Shampine method)
                Eigen::Vector3d theta_rk3 = theta + dt * (2.0/9.0 * k1_th_dot + 1.0/3.0 * k2_th_dot + 4.0/9.0 * k3_th_dot);
                Eigen::Vector3d theta_dot_rk3 = theta_dot + dt * (2.0/9.0 * k1_th_ddot + 1.0/3.0 * k2_th_ddot + 4.0/9.0 * k3_th_ddot);

                // Compute error using state differences
                Eigen::Vector3d error_theta = theta_rk4 - theta_rk3;
                Eigen::Vector3d error_x = x_k4 - x_k3;
                Eigen::Vector3d error_pos = error_theta.cwiseAbs() + error_x.cwiseAbs();
                Eigen::Vector3d error_theta_dot = theta_dot_rk4 - theta_dot_rk3;

                // Combine errors with scaling
                Eigen::VectorXd  y_old(9), y_new(9), y_err(9);
                y_old << x_prev, theta_prev, theta_dot_prev;
                y_new << x_k4,   theta_rk4,  theta_dot_rk4;
                y_err << (x_k4   - x_k3),
                        (theta_rk4 - theta_rk3),
                        (theta_dot_rk4 - theta_dot_rk3);

                double rho = scaledError(y_old, y_new, y_err);

                if (rho > 1.0) {
                    double safety = 0.9;
                    double max_scale = 5.0;
                    double min_scale = 0.3;
                    if (rho > 0) {
                        double scale = safety * std::pow(rho, -0.25);
                        scale = std::clamp(scale, min_scale, max_scale);
                        dt = std::clamp(scale * dt, dt_min, dt_max);
                    }

                    attempts++;
                    total_rejects++;
                    
                    // Rollback state
                    theta = theta_prev;
                    theta_dot = theta_dot_prev;
                    integral_error = integral_error_prev;
                    x = x_prev;

                    if (total_rejects > max_total_rejects) {
                        throw std::runtime_error("Simulation diverged: Too many rejected steps");
                    }
                } else {
                    // Accept step and update state
                    theta += (dt/6.0) * (k1_th_dot + 2*k2_th_dot + 2*k3_th_dot + k4_th_dot);
                    theta_dot += (dt/6.0) * (k1_th_ddot + 2*k2_th_ddot + 2*k3_th_ddot + k4_th_ddot);

                    x = robot.forwardKinematics(theta, x_prev);
                    
                    // Update task-space variables
                    Eigen::Matrix3d J = robot.computeJ(x, theta);
                    Eigen::Matrix3d K = robot.computeK(x, theta);
                    Eigen::Matrix3d J_inv = robot.dampedPseudoInverse(J);

                    x_dot = J_inv * K * theta_dot;

                    // Compute new task-space acceleration
                    theta_ddot = robot.computeForwardDynamics(theta, torque, x);
                    Eigen::Matrix3d J_dot = robot.computeJDot(x_dot, theta, theta_dot);
                    Eigen::Matrix3d K_dot = robot.computeKDot(x, x_dot, theta, theta_dot);

                    x_ddot = J_inv * (- J_dot * x_dot + (K * theta_ddot + K_dot * theta_dot));

                    // Build binary frame
                    Eigen::Vector3d theta_py = robot.toPyAngles(theta);
                    Eigen::Vector3d theta_dot_py =  robot.toPyDq(theta_dot);
                    Eigen::Vector3d torque_py =  robot.toPyTorque(torque);

                    Frame frame = CreateFrame(t, x, x_dot, theta_py, theta_dot_py, torque_py);
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
                    Eigen::Vector3d th_ddot = robot.computeForwardDynamics(theta, torque, x + dt * x_dot);
                    theta_dot += dt * th_ddot;
                    theta += theta_dot * dt;

                    x = robot.forwardKinematics(theta, x_prev);

                    // Update task-space variables
                    Eigen::Matrix3d J = robot.computeJ(x, theta);
                    Eigen::Matrix3d K = robot.computeK(x, theta);
                    Eigen::Matrix3d J_inv = robot.dampedPseudoInverse(J);

                    x_dot = J_inv * K * theta_dot;

                    // Compute new task-space acceleration
                    Eigen::Matrix3d J_dot = robot.computeJDot(x_dot, theta, theta_dot);
                    Eigen::Matrix3d K_dot = robot.computeKDot(x, x_dot, theta, theta_dot);

                    x_ddot = J_inv * (- J_dot * x_dot + (K * theta_ddot + K_dot * theta_dot));
                    
                    t += dt;

                    // Build binary frame
                    Eigen::Vector3d theta_py = robot.toPyAngles(theta);
                    Eigen::Vector3d theta_dot_py =  robot.toPyDq(theta_dot);
                    Eigen::Vector3d torque_py =  robot.toPyTorque(torque);

                    Frame frame = CreateFrame(t, x, x_dot, theta_py, theta_dot_py, torque_py);
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
