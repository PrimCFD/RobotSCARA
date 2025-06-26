#pragma once
#include "Types.hpp"
#include "SocketUtils.hpp"
#include <Eigen/Dense>
#include <vector>
#include <string>

void run_hil_streaming(const std::vector<Waypoint>& traj,
                       socket_t                     sock,
                       const Eigen::Vector3d&       elbowPos,
                       double                       l_arm_proth,
                       const std::string&           sensorPort,
                       const std::string&           arduinoPort);