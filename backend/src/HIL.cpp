#include "RobotModel.hpp"
#include "Controller.hpp"
#include "HardcodedParams.hpp"
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <chrono>
#include <iostream>
#include <memory>

constexpr double PI = 3.14159265358979323846;

// Sensor data structure with timestamp
struct SensorReading {
    double timestamp;
    double r;       // Radius (m)
    double theta;   // Polar angle (rad)
    double phi;     // Azimuthal angle (rad)
};

// Abstract hardware interface
class HardwareInterface {
public:
    virtual ~HardwareInterface() = default;
    virtual SensorReading getSensorData() = 0;
    virtual void sendMotorTorques(const Eigen::Vector3d& torques) = 0;
};

// Simulated hardware for testing
class SimulatedHardware : public HardwareInterface {
public:
    SensorReading getSensorData() override {
        // In real implementation, this would read from actual sensors
        static double t = 0.0;
        t += 0.001;
        
        SensorReading reading;
        reading.timestamp = t;
        reading.r = 0.5;
        reading.theta = 1.2 + 0.1 * sin(2 * PI * 0.5 * t);
        reading.phi = 0.5 + 0.2 * cos(2 * PI * 0.3 * t);
        return reading;
    }
    
    void sendMotorTorques(const Eigen::Vector3d& torques) override {
        std::cout << "Applying torques: " 
                  << torques[0] << ", "
                  << torques[1] << ", "
                  << torques[2] << std::endl;
    }
};

// Coordinate conversion utilities
namespace SphericalUtils {
    Eigen::Vector3d toCartesian(double r, double theta, double phi, 
                               const Eigen::Vector3d& center) {
        return {
            r * sin(theta) * cos(phi) + center.x(),
            r * sin(theta) * sin(phi) + center.y(),
            r * cos(theta) + center.z()
        };
    }
    
    // Simple moving average filter for embedded systems
    class LowPassFilter {
    public:
        LowPassFilter(double alpha = 0.2) : alpha_(alpha), initialized_(false) {}
        
        double filter(double new_value) {
            if (!initialized_) {
                value_ = new_value;
                initialized_ = true;
            } else {
                value_ = alpha_ * new_value + (1 - alpha_) * value_;
            }
            return value_;
        }
        
    private:
        double alpha_;
        double value_;
        bool initialized_;
    };
}

// Main HIL controller
class HILController {
public:
    HILController(std::unique_ptr<HardwareInterface> hardware, 
                 const Eigen::Vector3d& sphere_center)
        : hardware_(std::move(hardware)),
          sphere_center_(sphere_center),
          robot_(),
          controller_(),
          last_position_(Eigen::Vector3d::Zero()),
          last_velocity_(Eigen::Vector3d::Zero()),
          last_theta_(Eigen::Vector3d(-2.72, -0.38, -1.92)),  // Initial guess
          integral_error_(Eigen::Vector3d::Zero()),
          last_time_(0.0),
          r_filter_(0.3), theta_filter_(0.3), phi_filter_(0.3) 
    {
        // Initialize robot parameters
        RobotDynamics::RobotParams params;
        robot_.loadHardcodedParams();
        robot_.setParameters(params);
    }

    void run(double frequency_hz) {
        const double dt = 1.0 / frequency_hz;
        std::cout << "Starting HIL loop at " << frequency_hz << " Hz" << std::endl;
        
        while (true) {
            auto loop_start = std::chrono::steady_clock::now();
            
            // Get sensor data
            SensorReading sensor = hardware_->getSensorData();
            
            // Apply filtering to raw sensor data
            const double filtered_r = r_filter_.filter(sensor.r);
            const double filtered_theta = theta_filter_.filter(sensor.theta);
            const double filtered_phi = phi_filter_.filter(sensor.phi);
            
            // Convert to Cartesian coordinates
            Eigen::Vector3d position = SphericalUtils::toCartesian(
                filtered_r, filtered_theta, filtered_phi, sphere_center_);
            
            // Estimate velocity (finite difference)
            Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
            if (last_time_ > 0) {
                double actual_dt = sensor.timestamp - last_time_;
                if (actual_dt > 1e-6) {
                    velocity = (position - last_position_) / actual_dt;
                }
            }
            
            // Get desired state (trajectory interpolation)
            // In real implementation, this would come from trajectory generator
            TrajectoryPoint desired;
            desired.t = sensor.timestamp;
            desired.x = position;  // For testing, use current position as target
            desired.x_dot = velocity;
            desired.x_ddot = Eigen::Vector3d::Zero();
            
            // 5. Compute inverse kinematics
            auto ik_solution = robot_.invKineSinglePoint(position, last_theta_);
            if (!ik_solution.valid) {
                std::cerr << "IK failed! Using previous angles" << std::endl;
            } else {
                last_theta_ = ik_solution.theta;
            }
            
            // Estimate joint velocity (simplified)
            Eigen::Vector3d theta_dot = (last_theta_ - prev_theta_) / dt;
            prev_theta_ = last_theta_;
            
            // Compute control torque
            Eigen::Vector3d torque = controller_.computeMPCTorque(
                desired,
                position,
                velocity,
                Eigen::Vector3d::Zero(),  // Acceleration not available
                last_theta_,
                theta_dot,
                robot_,
                dt,
                integral_error_
            );
            
            // Send commands to motors
            hardware_->sendMotorTorques(torque);
            
            // Update state history
            last_position_ = position;
            last_velocity_ = velocity;
            last_time_ = sensor.timestamp;
            
            // Maintain real-time execution
            auto loop_end = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start);
            long sleep_us = static_cast<long>(dt * 1e6) - elapsed.count();
            
            if (sleep_us > 0) {
                std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
            } else {
                std::cerr << "Loop overrun by " << -sleep_us << " us" << std::endl;
            }
        }
    }

private:
    std::unique_ptr<HardwareInterface> hardware_;
    Eigen::Vector3d sphere_center_;
    RobotDynamics robot_;
    Controller controller_;
    
    // State variables
    Eigen::Vector3d last_position_;
    Eigen::Vector3d last_velocity_;
    Eigen::Vector3d last_theta_;
    Eigen::Vector3d prev_theta_;
    Eigen::Vector3d integral_error_;
    double last_time_;
    
    // Sensor filters
    SphericalUtils::LowPassFilter r_filter_;
    SphericalUtils::LowPassFilter theta_filter_;
    SphericalUtils::LowPassFilter phi_filter_;
};

int main() {
    // Configuration - should match your physical setup
    const Eigen::Vector3d sphere_center(0.1, -0.05, 0.3); // Calibration point
    const double control_frequency = 1000.0; // Hz
    
    // Create hardware interface (replace with real implementation)
    auto hardware = std::make_unique<SimulatedHardware>();
    
    // Initialize and run controller
    HILController controller(std::move(hardware), sphere_center);
    controller.run(control_frequency);
    
    return 0;
}