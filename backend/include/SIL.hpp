// SIL.hpp
#pragma once
#include <vector>
#include <Eigen/Dense>
#include "Controller.hpp"
#include "Types.hpp"
#include "SocketUtils.hpp"

constexpr size_t MAX_TRAJECTORY_POINTS = 50000;

using Trajectory = std::vector<Waypoint>;

// Helper conversion functions
inline TrajectoryPoint WaypointToTrajectoryPoint(const Waypoint& wp) {
    return {
        wp.t,
        Eigen::Vector3d(wp.x[0], wp.x[1], wp.x[2]),
        Eigen::Vector3d(wp.x_dot[0], wp.x_dot[1], wp.x_dot[2]),
        Eigen::Vector3d(wp.x_ddot[0], wp.x_ddot[1], wp.x_ddot[2])
    };
}

struct IdealTorquePoint {
    double t;
    double theta[3];
    double theta_dot[3];
    double tau_ideal[3];
};

inline Frame CreateFrame(double t, 
                         const Eigen::Vector3d& x,
                         const Eigen::Vector3d& x_dot,
                         const Eigen::Vector3d& theta,
                         const Eigen::Vector3d& theta_dot,
                         const Eigen::Vector3d& tau) {
    return {
        t,
        {x[0], x[1], x[2]},
        {x_dot[0], x_dot[1], x_dot[2]},
        {theta[0], theta[1], theta[2]},
        {theta_dot[0], theta_dot[1], theta_dot[2]},
        {tau[0], tau[1], tau[2]}
    };
}

TrajectoryPoint interpolateTrajectory(const Trajectory& traj, double t_query);

 void run_sil_streaming(const std::vector<Waypoint>& binary_traj,
                        socket_t sock,
                        const Eigen::Vector3d& elbow_pos,
                        double l_arm_proth);