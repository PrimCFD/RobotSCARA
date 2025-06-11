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
    void loadHardcodedParams();
    void setParameters(const RobotParams& params);

    void setElbowArm(const Eigen::Vector3d& elbow, 
                                      double l_arm_proth);
                                      
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

    Eigen::Vector3d RobotDynamics::computeForwardDynamics(
        const Eigen::Vector3d& theta,
        const Eigen::Vector3d& theta_dot,
        const Eigen::Vector3d& torque,
        const Eigen::Vector3d& pos
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
        bool valid;
        double position_error;
        int iterations;  // Track convergence
    };

    struct FKSolution {
        Eigen::Vector3d position;
        bool valid;
        double constraint_error;
    };

    // Helper functions for matrix computations
    Eigen::Matrix3d dampedPseudoInverse(
        const Eigen::Matrix3d& matrix, 
        double lambda_tikhonov = 1e-3
    ) const;

    IKSolution invKineSinglePoint(const Eigen::Vector3d& p, const Eigen::Vector3d& theta_prev) const;

    RobotDynamics::IKSolution closedFormIK(
    const Eigen::Vector3d& p, 
    const Eigen::Vector3d& theta_prev) const;

    Eigen::Matrix3d RobotDynamics::computeIKJacobian(
    const Eigen::Vector3d& p, 
    const Eigen::Vector3d& theta) const;

    RobotDynamics::IKSolution iterativeIK(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& theta0,
    double tolerance,
    int max_iter) const;


    Eigen::Vector3d forwardKinematics(const Eigen::Vector3d& theta,
                                                 const Eigen::Vector3d& ref_pos) const;

    Eigen::Vector3d forwardKinematicsAnalytic(
        const Eigen::Vector3d& theta,
        const Eigen::Vector3d& x_prev,
        bool& ok) const;

    RobotDynamics::FKSolution iterativeFK(
    const Eigen::Vector3d& theta,
    const Eigen::Vector3d& ref_pos,
    double tolerance,
    int max_iter) const;

    Eigen::Vector3d constraintEquations(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& theta) const;

    Eigen::Matrix3d RobotDynamics::computeConstraintJacobian(
    const Eigen::Vector3d& p, 
    const Eigen::Vector3d& theta) const;

    // Helper functions for angle conversions
    //θcpp₁ = θpy₁ + π/2
    //θcpp₂ = θpy₂ + π/2
    //θcpp₃ = θpy₃ − π/2

    static constexpr double PI = 3.14159265358979323846;

    static inline Eigen::Vector3d toCppAngles (const Eigen::Vector3d& th_py) {
    return { th_py(0) + PI/2.0,
             th_py(1) + PI/2.0,
            th_py(2) - PI/2.0 };
    }

    static inline Eigen::Vector3d toPyAngles  (const Eigen::Vector3d& th_cpp) {
        return { th_cpp(0) - PI/2.0,
                th_cpp(1) - PI/2.0,
                th_cpp(2) + PI/2.0 };
    }

    static inline Eigen::Vector3d toPyDq     (const Eigen::Vector3d& d_th_cpp) {
        return { d_th_cpp(0), d_th_cpp(1), d_th_cpp(2) };
    }

    static inline Eigen::Vector3d toPyTorque (const Eigen::Vector3d& tau_cpp) {
        return { tau_cpp(0),  tau_cpp(1),  tau_cpp(2) };
    }


private:
    bool initialized_;
    
    // Constants
    static constexpr double GRAVITY = 9.81;

    // FK
    mutable Eigen::Vector3d last_theta_ = Eigen::Vector3d::Zero(); 

};