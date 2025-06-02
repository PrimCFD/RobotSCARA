// SIL.hpp
#pragma once
#include <vector>
#include <Eigen/Dense>
#include "Controller.hpp"

constexpr size_t MAX_TRAJECTORY_POINTS = 50000;
constexpr size_t MAX_FRAME_POINTS = 1000000;

#pragma pack(push, 1)
struct Waypoint {
    double t;
    double x[3];
    double x_dot[3];
    double x_ddot[3];
};

struct Frame {
    double t;
    double x[3];
    double x_dot[3];
    double theta[3];
    double theta_dot[3];
    double tau[3];
};
#pragma pack(pop)

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
void run_sil_simulation(const Trajectory& traj, std::vector<Frame>& results_out);