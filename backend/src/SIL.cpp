#include "RobotModel.hpp"
#include "SIL.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <random>
#include <chrono>

// Sensor simulation
const double ANGLE_RESOLUTION = 0.0052; // 0.3 degrees in radians

// Helper function to quantize values
double quantize(double value, double resolution, double noise_std = 0.0) {
    static std::default_random_engine generator(
        static_cast<unsigned>(std::chrono::system_clock::now().time_since_epoch().count()));
    if (noise_std > 0.0) {
        std::normal_distribution<double> distribution(0.0, noise_std);
        double noise = distribution(generator);
        value += noise;
    }
    return std::round(value / resolution) * resolution;
}

 void run_sil_streaming(const std::vector<Waypoint>& binary_traj,
                        socket_t sock,
                        const Eigen::Vector3d& elbow_pos,
                        double l_arm_proth) {

    RobotDynamics robot_ideal;
    robot_ideal.loadHardcodedParams();
    robot_ideal.setElbowArm(elbow_pos, l_arm_proth);
    
    // Trajcetory
    std::vector<TrajectoryPoint> traj;
    traj.reserve(binary_traj.size());
    
    for (size_t i = 0; i < binary_traj.size(); ++i) {
        traj.push_back(WaypointToTrajectoryPoint(binary_traj[i]));
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

    // Radial bias
    double r_bias  = 0.0;      // will converge to the true error (m)

    // Use trajectory's starting point as initial state
    Eigen::Vector3d x_0 = traj.front().x;
    Eigen::Vector3d x_dot_0 = traj.front().x_dot;
    Eigen::Vector3d x_ddot_0 = traj.front().x_ddot;  // Capture initial acceleration

    r_bias = (x_0 - elbow_pos).norm() - l_arm_proth;

    TrajectoryPoint target_ini;
    target_ini.t = 0.0, target_ini.x = x_0, target_ini.x_dot = x_dot_0, target_ini.x_ddot = x_ddot_0;

    // Solve inverse kinematics for initial position
    Eigen::Vector3d initial_guess_py(-2.76, -0.38,  2.84);
    Eigen::Vector3d initial_guess = robot.toCppAngles(initial_guess_py);
    RobotDynamics::IKSolution init_sol = robot.invKineSinglePoint(x_0, initial_guess);


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
    Eigen::Vector3d theta_ddot(0.0, 0.0, 0.0);

    // Initialize state variables
    Eigen::Vector3d x = x_ref;
    Eigen::Vector3d x_dot = x_dot_0;
    Eigen::Vector3d x_ddot = x_ddot_0;

    // Initialize torque to zero
    Eigen::Vector3d torque = Eigen::Vector3d::Zero();

        // Add variables for measured states
    Eigen::Vector3d x_measured = x;
    Eigen::Vector3d x_dot_measured = x_dot_0;
    Eigen::Vector3d x_ddot_measured = x_ddot_0;
    Eigen::Vector3d theta_measured = theta;
    Eigen::Vector3d theta_dot_measured = theta_dot;

    
    // Add history for velocity estimation
    Eigen::Vector3d x_prev_measured = x;
    Eigen::Vector3d theta_prev_measured = theta;
    double t_prev_control = 0.0;

    // Controller rate setup (1ms period)
    const double control_dt = 0.001; // 1ms controller period
    double t_control_next = 0.0;     // Next controller update time

    // Kalman Filter for velocity estimation
    struct KalmanFilter3D {
        Eigen::VectorXd state;   // [x, y, z, vx, vy, vz]^T
        Eigen::MatrixXd P;       // Covariance matrix
        double t_last;           // Time of last update
        double sigma_a;          // Process noise (acceleration std dev)
        double sigma_m;          // Measurement noise (position std dev)

        KalmanFilter3D(double t0, const Eigen::Vector3d& pos0, 
                      const Eigen::Vector3d& vel0, 
                      double sigma_a_init, double sigma_m_init)
            : sigma_a(sigma_a_init), sigma_m(sigma_m_init), t_last(t0)
        {
            state = Eigen::VectorXd(6);
            state << pos0, vel0;
            P = Eigen::MatrixXd::Zero(6,6);
            // Initial covariance: position uncertainty from measurement noise,
            // velocity uncertainty set high
            P.topLeftCorner(3,3) = (sigma_m * sigma_m) * Eigen::Matrix3d::Identity();
            P.bottomRightCorner(3,3) = 1000.0 * Eigen::Matrix3d::Identity();
        }

        void predict(double t_current) {
            double dt = t_current - t_last;
            if (dt <= 0) return; // No prediction needed

            // State transition matrix (constant velocity model)
            Eigen::MatrixXd F(6,6);
            F.setIdentity();
            F.topRightCorner(3,3) = dt * Eigen::Matrix3d::Identity();

            // Process noise covariance
            double dt2 = dt * dt;
            double dt3 = dt2 * dt;
            double dt4 = dt2 * dt2;
            double sig2 = sigma_a * sigma_a;
            Eigen::Matrix3d Q_pos = (dt4/4.0) * sig2 * Eigen::Matrix3d::Identity();
            Eigen::Matrix3d Q_vel = dt2 * sig2 * Eigen::Matrix3d::Identity();
            Eigen::Matrix3d Q_cross = (dt3/2.0) * sig2 * Eigen::Matrix3d::Identity();

            Eigen::MatrixXd Q(6,6);
            Q << Q_pos, Q_cross,
                 Q_cross, Q_vel;

            // Prediction step
            state = F * state;
            P = F * P * F.transpose() + Q;
            t_last = t_current;
        }

        void update(const Eigen::Vector3d& z, double t) {
            predict(t); // Predict to current time

            // Measurement matrix (only position measured)
            Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3,6);
            H.topLeftCorner(3,3) = Eigen::Matrix3d::Identity();
            
            // Measurement noise
            Eigen::Matrix3d R = (sigma_m * sigma_m) * Eigen::Matrix3d::Identity();
            
            // Innovation
            Eigen::Vector3d y = z - H * state;
            Eigen::MatrixXd S = H * P * H.transpose() + R;
            Eigen::MatrixXd K = P * H.transpose() * S.inverse();

            // Update state and covariance
            state += K * y;
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6,6);
            P = (I - K * H) * P;
        }

        Eigen::Vector3d getPosition() const { return state.head<3>(); }
        Eigen::Vector3d getVelocity() const { return state.tail<3>(); }
    };

    std::unique_ptr<KalmanFilter3D> kalman_filter;

    const double delta_r_allowed = 0.0007;   // 0.7 mm allowable stretch

    // Adaptive stepping parameters
    double t = 0.0;
    double t_final = traj.back().t;
    double dt = 1e-6;
    double dt_min = 1e-8;
    double dt_max = 5e-5;
    bool first_step = true;
    int max_attempts = 50;
    int total_rejects = 0;
    int max_total_rejects = 500000;

    // Error                

    Eigen::VectorXd absTol(6), relTol(6);

    // 0–2: joint angles (radians) – 1/2 µstep each axis
    absTol.segment<3>(0).setConstant(0.5 * kMicroStepRad);

    // 3–5: joint velocities (rad/s) – <1 ‰ of max_vel
    absTol.segment<3>(3).setConstant(0.003);

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

            double rho_prev = 1.0; 

            // --- CONTROLLER UPDATE AT FIXED RATE ---
            if (t >= t_control_next) {
                // Convert true position to spherical coordinates
                Eigen::Vector3d pos_rel = x - robot.params_.vec_elbow;
                const double r0 = robot.params_.l_arm_proth;

                auto sgn = [](double v)
                {
                    return v == 0.0 ? 0 : static_cast<int>(std::copysign(1.0, v));
                };

                double theta_sp = std::atan2(pos_rel.head<2>().norm(), pos_rel.z());
                double phi_sp = std::atan2(pos_rel.y(), pos_rel.x());

                theta_sp = quantize(theta_sp, ANGLE_RESOLUTION, 0.1*ANGLE_RESOLUTION);
                phi_sp = quantize(phi_sp, ANGLE_RESOLUTION, 0.1*ANGLE_RESOLUTION);

                const double r_true = pos_rel.norm();

                // Enforce rigid-rod length: ρ = L + r_bias
                const double r_est = r0 + r_bias; // L is l_arm_proth

                x_measured = robot.params_.vec_elbow + (r_est) * Eigen::Vector3d(
                                std::sin(theta_sp)*std::cos(phi_sp),
                                std::sin(theta_sp)*std::sin(phi_sp),
                                std::cos(theta_sp));
                                
                RobotDynamics::IKSolution sol_measured = robot.invKineSinglePoint(x_measured, theta_prev_measured);
                theta_measured = sol_measured.theta;

                constexpr double sigma_p = 3e-4;   // 0.3 mm ≈ pulse radius noise
                constexpr double sigma_r = 1e-4;   // random-walk std per √s
                double y = r_true - r0;       // measurement ρ – L
                double err = y - r_bias;      // innovation

                // RLS filter
                static double P = 1e-3;               // initial variance
                double Kgain = P / (P + sigma_p*sigma_p); // Kalman gain
                r_bias += Kgain * err;
                P = (1 - Kgain) * P + sigma_r*sigma_r;    // sigma_r ≈ 1e-5 m·√s

                // Initialize or update Kalman filter
                if (!kalman_filter) {
                    double sigma_m = l_arm_proth * ANGLE_RESOLUTION; // Measure noise
                    double sigma_a = 1e-2; // Process noise m.s^-2
                    kalman_filter = std::make_unique<KalmanFilter3D>(
                        t, x_measured, x_dot_0, sigma_a, sigma_m);
                } else {
                    kalman_filter->update(x_measured, t);
                }

                // Velocity estimation with low-pass filtering
                x_dot_measured = kalman_filter->getVelocity();

                // Joint velocity calculation
                Eigen::Matrix3d J_meas = robot.computeJ(x_measured, theta_measured);
                Eigen::Matrix3d K_meas = robot.computeK(x_measured, theta_measured);
                Eigen::Matrix3d K_inv_meas = robot.dampedPseudoInverse(K_meas);
                theta_dot_measured = K_inv_meas * J_meas * x_dot_measured;

                // Update history
                x_prev_measured = x_measured;
                theta_prev_measured = theta_measured;
                t_prev_control = t;

                // Get desired state at current time
                TrajectoryPoint target = controller.interpolateTrajectory(traj, t);
                
                // Update pulse using MEASURED states
                static Eigen::Vector3d delta_prev = Eigen::Vector3d::Zero();
                torque = controller.computeMPCTorque(
                    target,
                    x_measured,      // measured position
                    x_dot_measured,  // measured velocity
                    x_ddot_measured, // measured acceleration => not used
                    theta_measured,
                    theta_dot_measured,
                    robot,
                    control_dt,
                    integral_error
                );
                t_control_next += control_dt; // Schedule next update
            }

            if (std::isnan(torque.sum())) {
                // Diagnostic output
                std::cout << "NaN torque at t=" << t 
                        << " pos=" << x.transpose()
                        << " theta=" << theta.transpose() << std::endl;
                // Reset to zero to prevent propagation
                torque = Eigen::Vector3d::Zero();
            }

            // RK4 integration with proper state propagation

            while (!step_accepted && attempts < max_attempts) {
                // Stage k1 - Use current x
                Eigen::Vector3d k1_th_dot = theta_dot;
                Eigen::Vector3d k1_th_ddot = robot.computeForwardDynamics(theta, k1_th_dot, torque, x_prev);

                // Stage k2 - Update x_k2 and use it for dynamics
                Eigen::Vector3d k2_th = theta + 0.5 * dt * k1_th_dot;
                Eigen::Vector3d x_k2 = robot.forwardKinematics(k2_th, x_prev);
                Eigen::Vector3d k2_th_dot = theta_dot + 0.5 * dt * k1_th_ddot;
                Eigen::Vector3d k2_th_ddot = robot.computeForwardDynamics(k2_th, k2_th_dot, torque, x_k2);  // Use x_k2

                // Stage k3 - Update x_k3 and use it for dynamics
                Eigen::Vector3d k3_th = theta + 0.5 * dt * k2_th_dot;
                Eigen::Vector3d x_k3 = robot.forwardKinematics(k3_th, x_k2);  // Use x_k2 as reference
                Eigen::Vector3d k3_th_dot = theta_dot + 0.5 * dt * k2_th_ddot;
                Eigen::Vector3d k3_th_ddot = robot.computeForwardDynamics(k3_th, k3_th_dot, torque, x_k3);  // Use x_k3

                // Stage k4 - Update x_k4 and use it for dynamics
                Eigen::Vector3d k4_th = theta + dt * k3_th_dot;
                Eigen::Vector3d x_k4 = robot.forwardKinematics(k4_th, x_k3);  // Use x_k3 as reference
                Eigen::Vector3d k4_th_dot = theta_dot + dt * k3_th_ddot;
                Eigen::Vector3d k4_th_ddot = robot.computeForwardDynamics(k4_th, k4_th_dot, torque, x_k4);  // Use x_k4

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
                Eigen::VectorXd  y_old(6), y_new(6), y_err(6);
                y_old << theta_prev, theta_dot_prev;
                y_new <<theta_rk4,  theta_dot_rk4;
                y_err << error_theta,
                        error_theta_dot;

                double rho = scaledError(y_old, y_new, y_err);

                double safety = 0.9;
                double max_scale = 5.0;
                double min_scale = 0.3;

                if (rho > 1.0) {

                    if (rho > 0) {
                        double scale = safety * std::pow(rho, -0.2) * std::pow(rho_prev, 0.04);
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
                    theta_ddot = robot.computeForwardDynamics(theta, theta_dot, torque, x);
                    Eigen::Matrix3d J_dot = robot.computeJDot(x_dot, theta, theta_dot);
                    Eigen::Matrix3d K_dot = robot.computeKDot(x, x_dot, theta, theta_dot);

                    x_ddot = J_inv * (- J_dot * x_dot + (K * theta_ddot + K_dot * theta_dot));

                    // Build binary frame
                    Eigen::Vector3d theta_py = robot.toPyAngles(theta);
                    Eigen::Vector3d theta_dot_py =  robot.toPyDq(theta_dot);
                    Eigen::Vector3d torque_py =  robot.toPyTorque(torque);

                    Frame frame = CreateFrame(t, x, x_dot, theta_py, theta_dot_py, torque_py);

                    if (!send_all(sock, reinterpret_cast<const char*>(&frame), sizeof(Frame))) {
                                throw std::runtime_error("Failed to send frame data");
                            }

                    t += dt;
                    double scale = safety * std::pow(rho, -0.2) * std::pow(rho_prev, 0.04);
                    scale = std::clamp(scale, min_scale, max_scale);
                    dt = std::clamp(scale * dt, dt_min, dt_max);
                    rho_prev = rho;
                    step_accepted = true;
                }
            }

            if (!step_accepted) {
                if (dt <= dt_min) {
                    std::cerr << "Critical error: Minimum step size reached at t=" << t << std::endl;
                    break;
                } else {
                    std::cerr << "Critical error: Couldn't make step converge or reach dt_min at t=" << t << std::endl;
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

    // --- send 16-byte padding ---
    Eigen::Vector3d fake_torque(0.0, 0.0, 0.0);
    Frame frame = CreateFrame(t_final, x, x_dot, theta, theta_dot, fake_torque);
    if (!send_all(sock, reinterpret_cast<const char*>(&frame), sizeof(Frame))) {
        throw std::runtime_error("Failed to send phase marker");
    }

    // Pre-compute ideal torques

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

        if (!send_all(sock, reinterpret_cast<const char*>(&itp), sizeof(IdealTorquePoint))) {
               throw std::runtime_error("Failed to send ideal torque data");
        }

    }

    return;                     
}