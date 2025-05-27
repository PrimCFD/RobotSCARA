#include "TorqueQP.hpp"
#include <qpOASES.hpp>
#include <iostream>

TorqueQP::TorqueQP(const RobotDynamics& robot) 
    : robot_ref(robot), qp(NUM_VARS, NUM_CONS), prev_time(0.0) 
{
    theta_current.setZero();
    theta_dot_current.setZero();
    pos_current.setZero();
    vel_current.setZero();
    desired_pos.setZero();
    prev_error.setZero();
    integral_error.setZero();

    H.resize(NUM_VARS, NUM_VARS);
    g.resize(NUM_VARS);
    A.resize(NUM_CONS, NUM_VARS);
    lb.resize(NUM_VARS);
    ub.resize(NUM_VARS);
    lbA.resize(NUM_CONS);
    ubA.resize(NUM_CONS);

    // QP Options
    options.setToMPC();
    options.printLevel = qpOASES::PL_NONE;
    qp.setOptions(options);

    Q_i = Eigen::Matrix3d::Identity() * 0.0;  // Integral gain
    Q = Eigen::Matrix3d::Identity() * 10.0;    // State error weight
    R = Eigen::Matrix3d::Identity() * 1.0;     // Control effort weight

    Eigen::Vector3d tau_prev = Eigen::Vector3d::Zero();
}

void TorqueQP::updateModelParameters() {
    M = robot_ref.computeMassMatrix(theta_current, pos_current);
    C = robot_ref.computeCoriolis(pos_current, vel_current, theta_current, theta_dot_current);
    G = robot_ref.computeGravity(theta_current);
}

Eigen::Vector3d TorqueQP::solveMPC(const Eigen::Vector3d& current_pos,
                                 const Eigen::Vector3d& current_vel,
                                 const Eigen::Vector3d& desired_pos,
                                 const Eigen::Vector3d& theta,
                                 const Eigen::Vector3d& theta_dot,
                                 double dt) 
{
    // Store current state
    pos_current = current_pos;
    vel_current = current_vel;
    theta_current = theta;
    theta_dot_current = theta_dot;
    this->desired_pos = desired_pos;

    // Update integral term with trapezoidal integration
    if(prev_time > 1e-6) {  // Skip first iteration
        Eigen::Vector3d error = desired_pos - current_pos;
        integral_error += 0.5 * dt * (error + prev_error);
    }
    prev_error = desired_pos - current_pos;
    prev_time += dt;

    // Anti-windup clamping
    integral_error = integral_error.cwiseMax(-10.0).cwiseMin(10.0);

    // Update dynamics matrices
    updateModelParameters();

    // Build QP problem with prediction horizon
    setupQP();

    // Solve QP
    qpOASES::int_t nWSR = 1000;
    qpOASES::returnValue ret = qpOASES::SUCCESSFUL_RETURN;

    if(first_solve) {
        ret = qp.init(H.data(), g.data(), 
                     A.data(), lb.data(), ub.data(),
                     lbA.data(), ubA.data(), nWSR);
        first_solve = false;
    } else {
        ret = qp.hotstart(H.data(), g.data(),
                         A.data(), lb.data(), ub.data(),
                         lbA.data(), ubA.data(), nWSR);
    }

    if (ret != qpOASES::SUCCESSFUL_RETURN) {
        std::cerr << "[QP FAILURE] code = " << static_cast<int>(ret) << std::endl;
    }

    // Extract and return solution
    if(ret == qpOASES::SUCCESSFUL_RETURN) {
        qpOASES::real_t sol[NUM_VARS];
        qp.getPrimalSolution(sol);
        Eigen::Vector3d solution(sol[0], sol[1], sol[2]);
        Eigen::Vector3d tau_opt = M * solution + C + G;
        Eigen::Vector3d tau_pred = tau_prev;
        Eigen::Vector3d tau_error = tau_opt - tau_pred;
        std::cout << "[τ_opt]    " << tau_opt.transpose() << std::endl;
        std::cout << "[τ_prev]   " << tau_pred.transpose() << std::endl;
        std::cout << "[τ_error]  " << tau_error.transpose() << " | norm = " << tau_error.norm() << std::endl;
        tau_prev = tau_opt;
        return solution;
    }

    return Eigen::Vector3d::Zero();
}

void TorqueQP::setupQP() {
    H.setZero();
    g.setZero();
    A.setZero();

    lbA.setConstant(1e9);   // Reset to avoid old bounds
    ubA.setConstant(-1e9); 

    // State prediction variables
    Eigen::Vector3d pos_pred = pos_current;
    Eigen::Vector3d vel_pred = vel_current;
    Eigen::Vector3d theta_pred = theta_current;
    Eigen::Vector3d theta_dot_pred = theta_dot_current;

    Eigen::Vector3d x_err = desired_pos - pos_current + Q_i * integral_error;

    for(int i = 0; i < N; ++i) {
        // Compute angular velocity using RK4

        Eigen::Vector3d new_theta_dot = robot_ref.computeRK4Integration(
            theta_pred, 
            theta_dot_pred,
            tau_prev,
            pos_pred,
            vel_pred,
            dt_pred
        );
        
        // Update positions using kinematics
        Eigen::Matrix3d J_pred = robot_ref.computeJ(pos_pred, theta_pred);
        pos_pred += J_pred * theta_dot_pred * dt_pred;
    
        // Update states
        theta_pred += theta_dot_pred * dt_pred;
        theta_dot_pred = new_theta_dot;

        Eigen::Vector3d theta_ddot_pred = robot_ref.computeForwardDynamics(
            theta_pred,
            theta_dot_pred,
            tau_prev,
            pos_pred,
            vel_pred
        );

        // Recompute dynamics matrices for predicted state
        Eigen::Matrix3d M_pred = robot_ref.computeMassMatrix(theta_pred, pos_pred);
        Eigen::Vector3d C_pred = robot_ref.computeCoriolis(pos_pred, vel_pred, theta_pred, theta_dot_pred);
        Eigen::Vector3d G_pred = robot_ref.computeGravity(theta_pred);

        Eigen::Matrix3d A_dyn = M_pred;
        Eigen::Vector3d b_dyn = C_pred + G_pred;
        Eigen::Vector3d tau_ref = M_pred * theta_ddot_pred + C_pred + G_pred;

        H.block<3,3>(3*i, 3*i) = A_dyn.transpose() * R * A_dyn + Q;
        g.segment<3>(3*i) =  - A_dyn.transpose() * R * tau_ref - Q * x_err;

        Eigen::MatrixXd torque_constr_lower = M_pred;
        Eigen::Vector3d torque_rhs_lower = -torque_limit * Eigen::Vector3d::Ones() - C_pred - G_pred;
        Eigen::MatrixXd torque_constr_upper = M_pred;
        Eigen::Vector3d torque_rhs_upper = torque_limit * Eigen::Vector3d::Ones() - C_pred - G_pred;

        // Add torque constraints to A, lbA, ubA
        int constr_idx = 2 * 3 * i; // 2 constraints per timestep
        A.block<3, 3>(constr_idx, 3*i) = torque_constr_lower;
        lbA.segment<3>(constr_idx) = torque_rhs_lower;
        ubA.segment<3>(constr_idx) = Eigen::Vector3d::Constant(1e9); // Upper bound not active here

        A.block<3, 3>(constr_idx + 3, 3*i) = torque_constr_upper;
        lbA.segment<3>(constr_idx + 3) = Eigen::Vector3d::Constant(-1e9);
        ubA.segment<3>(constr_idx + 3) = torque_rhs_upper;
    }

    // Torque
    for(int i = 0; i < NUM_VARS; ++i) {
        lb[i] = -torque_limit;
        ub[i] = torque_limit;
    }
}