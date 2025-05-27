#pragma once

#include <vector>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>

struct TrajectoryPoint {
    double t;                  // Timestamp
    Eigen::Vector3d x;         // Position
};

using Trajectory = std::vector<TrajectoryPoint>;

Trajectory parseTrajectoryFromJson(const nlohmann::json& j);
TrajectoryPoint interpolateTrajectory(const Trajectory& traj, double t_query);

void run_sil_simulation(const Trajectory& traj, std::vector<nlohmann::json>& results_out);