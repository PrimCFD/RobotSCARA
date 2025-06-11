#pragma once
#include "RobotModel.hpp"
#include <Eigen/Dense>
#include <vector>

struct TrajectoryPoint {
    double t;
    Eigen::Vector3d x;
    Eigen::Vector3d x_dot;
    Eigen::Vector3d x_ddot;
};

class Controller {
public:
    // Configuration parameters (settable)
    Eigen::Vector3d Kp = Eigen::Vector3d(30.0, 30.0, 30.0);
    Eigen::Vector3d Kd = Eigen::Vector3d(20.0, 20.0, 20.0);
    Eigen::Vector3d Ki = Eigen::Vector3d(2.0, 2.0, 2.0);
    double torque_limit = 30.0;
    double integral_max = 1.0;

    // Trajectory interpolation
    TrajectoryPoint interpolateTrajectory(
        const std::vector<TrajectoryPoint>& traj,
        double t_query
    ) const;

    // Torque calculation
    Eigen::Vector3d computeMPCTorque(
        const TrajectoryPoint& desired_state,
        const Eigen::Vector3d& current_pos,
        const Eigen::Vector3d& current_vel,
        const Eigen::Vector3d& current_accel,
        const Eigen::Vector3d& theta,
        const Eigen::Vector3d& theta_dot,
        RobotDynamics& robot,
        double dt,
        Eigen::Vector3d& integral_error
    ) const;
};