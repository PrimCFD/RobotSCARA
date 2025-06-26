#pragma once
#include "RobotModel.hpp"
#include <Eigen/Dense>
#include <vector>

static constexpr double PI = 3.14159265358979323846;

// Hardware

// Stepper model
constexpr double kFullStepRad = 1.8 * PI / 180.0;
constexpr double kMicroStepRad = kFullStepRad / 8.0;

const double pulses_max = 62.5e3; // pulses s⁻¹

const double J_rotor = 1.4e-4;  // kg·m²  (datasheet 1400 g·cm²)
const double B_viscous = 1.5e-2;  // N·m·s/rad (empirical)
const double K_t = 0.82; // N·m/A  (torque constant)
const double Imax = 5.5; // rated phase current (A)
const double coulomb = 0.14; // N·m  (datasheet detent torque)
const double backlash = 0.4 * PI/180; // rad
const double ctrl_dt = 0.01; // s
const double gear_ratio = 1.0;
const double eta_fw  = 1.0;   // motor → load
const double eta_rev = 1.0;   // load → motor

// Electrical parameters for RL model
const double R = 0.58;      // Ohms (copper resistance)
const double L = 0.0022;    // Henry (coil inductance)
const double V_supply = 48.0; // Volts (supply voltage)
const double K_e = K_t;    // V·s/rad (back EMF constant)

struct TrajectoryPoint {
    double t;
    Eigen::Vector3d x;
    Eigen::Vector3d x_dot;
    Eigen::Vector3d x_ddot;
};


class Controller {
public:
    // Configuration parameters (settable)
    Eigen::Vector3d Kp = {30.0, 30.0, 30.0};
    Eigen::Vector3d Kd = {4.0, 4.0, 4.0};
    Eigen::Vector3d Ki = {4.0, 4.0, 4.0};

    // Per-axis velocity limits (joint side)
    double integral_max = 1.0;

    // Trajectory interpolation
    TrajectoryPoint interpolateTrajectory(
        const std::vector<TrajectoryPoint>& traj,
        double t_query
    ) const;

    // Continuous torque
    Eigen::Vector3d Controller::computeMPCTorque(
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

    // Pulses calculation
    Eigen::Vector3d computeMPCStepRateTorque(
        const TrajectoryPoint& desired_state,
        const Eigen::Vector3d& current_pos,
        const Eigen::Vector3d& current_vel,
        const Eigen::Vector3d& current_accel,
        const Eigen::Vector3d& theta,
        const Eigen::Vector3d& theta_dot,
        RobotDynamics& robot,
        double dt,
        Eigen::Vector3d& integral_error,
        Eigen::Vector3d& delta_prev
    ) const;
};