#pragma once
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <memory>

class RobotDynamics {
public:
    struct RobotParams {
        // Geometric parameters
        double a1x, a1y;  // Leg 1 base XY
        double a2x, a2y;  // Leg 2 base XY
        double a3x, a3y;  // Leg 3 base XY
        double d, v_1, v_2, v_3;
        double h_1, h_2, h_3;
        double l_11, l_21, l_31;
        double l_12, l_22, l_32;
        
        // Mass parameters
        double m_11, m_21, m_31;  // Proximal link masses
        double m_12, m_22, m_32;  // Distal link masses
        double m_d, m_arm;        // Platform /arm mass
        double l_arm_proth;       // Prosthetic arm length
        
        // Elbow and shoulder vectors
        Eigen::Vector3d vec_elbow;
        Eigen::Vector3d vec_shoulder;
        Eigen::Vector3d initial_pos; 
    };

    RobotParams params_;

    // Constructor
    RobotDynamics();
    explicit RobotDynamics(const RobotParams& params);
    
    // Configuration loading
    void RobotDynamics::loadHardcodedParams();
    void setParameters(const RobotParams& params);

    Eigen::Vector3d getInitialPosition() const {
    return params_.initial_pos;
    }
    
    // Core matrix computations (based on analytic.py)
    Eigen::Matrix3d computeK(const Eigen::Vector3d& pos, 
                            const Eigen::Vector3d& theta) const;
    
    Eigen::Matrix3d computeJ(const Eigen::Vector3d& pos, 
                            const Eigen::Vector3d& theta) const;
    
    Eigen::Matrix3d computeKDot(const Eigen::Vector3d& pos,
                               const Eigen::Vector3d& vel,
                               const Eigen::Vector3d& theta,
                               const Eigen::Vector3d& theta_dot) const;
    
    Eigen::Matrix3d computeJDot(const Eigen::Vector3d& vel,
                               const Eigen::Vector3d& theta,
                               const Eigen::Vector3d& theta_dot) const;
    
    // Motor inverse kinematics
    Eigen::Vector3d computeMotorSpeed(const Eigen::Vector3d& pos,
                                     const Eigen::Vector3d& vel,
                                     const Eigen::Vector3d& theta) const;
    
    Eigen::Vector3d computeMotorAccel(const Eigen::Vector3d& pos,
                                     const Eigen::Vector3d& vel,
                                     const Eigen::Vector3d& accel,
                                     const Eigen::Vector3d& theta,
                                     const Eigen::Vector3d& theta_dot) const;

    Eigen::Matrix3d computeMassMatrix(const Eigen::Vector3d& theta, const Eigen::Vector3d& pos) const;

    Eigen::Vector3d computeGravity(const Eigen::Vector3d& theta, const Eigen::Vector3d& pos) const;

    Eigen::Vector3d computeForwardDynamics(
        const Eigen::Vector3d& theta,
        const Eigen::Vector3d& theta_dot,
        const Eigen::Vector3d& torque,
        const Eigen::Vector3d& pos,            // Task-space position
        const Eigen::Vector3d& vel          // Task-space velocity
    ) const;

    // Torque computation (inverse dynamics)
    Eigen::Vector3d computeTorque(const Eigen::Vector3d& pos,
                                 const Eigen::Vector3d& vel,
                                 const Eigen::Vector3d& accel,
                                 const Eigen::Vector3d& theta,
                                 const Eigen::Vector3d& theta_dot,
                                 const Eigen::Vector3d& theta_ddot) const;
    
    // Complete inverse dynamics computation
    struct DynamicsResult {
        Eigen::Vector3d theta_dot;
        Eigen::Vector3d theta_ddot;
        Eigen::Vector3d torque;
        bool success;
        std::string error_msg;
    };
    
    DynamicsResult computeInverseDynamics(const Eigen::Vector3d& pos,
                                         const Eigen::Vector3d& vel,
                                         const Eigen::Vector3d& accel,
                                         const Eigen::Vector3d& theta) const;
    
    // Utility functions
    const RobotParams& getParameters() const { return params_; }
    bool isInitialized() const { return initialized_; }

    struct IKSolution {
        Eigen::Vector3d theta;
        bool valid = false;
    };

        // Helper functions for matrix computations
    Eigen::Matrix3d dampedPseudoInverse(
        const Eigen::Matrix3d& matrix, 
        double lambda_tikhonov = 1e-3, 
        double epsilon_diag = 1e-8
    ) const;

    IKSolution invKineSinglePoint(const Eigen::Vector3d& p, const Eigen::Vector3d& theta_prev) const;


    Eigen::Vector3d forwardKinematics(const Eigen::Vector3d& theta,
                                                 const Eigen::Vector3d& ref_pos) const;

private:
    bool initialized_;
    
    // Constants
    static constexpr double GRAVITY = 9.81;

    // FK
    mutable Eigen::Vector3d last_theta_ = Eigen::Vector3d::Zero(); 

};