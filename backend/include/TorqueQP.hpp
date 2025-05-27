#pragma once
#include <qpOASES.hpp>
#include <Eigen/Dense>
#include "RobotModel.hpp"

class TorqueQP {
public:
    explicit TorqueQP(const RobotDynamics& robot);
    
    Eigen::Vector3d solveMPC(const Eigen::Vector3d& current_pos,
                            const Eigen::Vector3d& current_vel,
                            const Eigen::Vector3d& desired_pos,
                            const Eigen::Vector3d& theta,
                            const Eigen::Vector3d& theta_dot,
                            double dt);

    void updateModelParameters();
    
private:

    void setupQP();

    static constexpr double dt_pred = 0.01;  // Prediction time step
    Eigen::Vector3d prev_error;
    Eigen::Vector3d integral_error;
    double prev_time;
    Eigen::Matrix3d Q, R;  // Cost matrices
    Eigen::Matrix3d Q_i; // Integral matrix
    
    // Prediction matrices
    Eigen::Matrix3d M_pred;
    Eigen::Vector3d C_pred;
    Eigen::Vector3d G_pred;
                
    // QP Problem dimensions
    static constexpr int N = 5;          // Control horizon
    static constexpr int NUM_VARS = 3*N; // Torques over horizon
    static constexpr int NUM_CONS = 6*N; // Dynamics + constraints

    // QP Objects
    qpOASES::SQProblem qp;
    qpOASES::Options options;
    bool first_solve = true;

    // QP Matrices
    Eigen::MatrixXd H;       // Hessian
    Eigen::VectorXd g;       // Gradient
    Eigen::MatrixXd A;       // Constraint matrix
    Eigen::VectorXd lb, ub;  // Variable bounds
    Eigen::VectorXd lbA, ubA;// Constraint bounds

    // Dynamics parameters
    Eigen::Matrix3d M;       // Mass matrix
    Eigen::Vector3d C;       // Coriolis
    Eigen::Vector3d G;       // Gravity
    double torque_limit = 50.0; // Nm

    Eigen::Vector3d theta_current;
    Eigen::Vector3d theta_dot_current;
    Eigen::Vector3d pos_current;
    Eigen::Vector3d vel_current;
    Eigen::Vector3d desired_pos;
    Eigen::Vector3d tau_prev;
    const RobotDynamics& robot_ref;  // Reference to robot model
};