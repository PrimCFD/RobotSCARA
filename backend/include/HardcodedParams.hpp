// Auto-generated from C:/Users/antho/Desktop/COPR02/backend/../configs/temp.json
#pragma once

inline void loadHardcodedParams(RobotDynamics::RobotParams& params) {
    // Geometry parameters
    params.a1x = 0.0;
    params.a1y = 0.0;
    params.a2x = 0.0;
    params.a2y = 0.0;
    params.a3x = 0.0;
    params.a3y = 0.0;
    params.d = 0.5;
    params.v_1 = 0.3;
    params.v_2 = 0.25;
    params.v_3 = 0.0;
    params.h_1 = 0.4;
    params.h_2 = 0.45;
    params.h_3 = 1.0;
    params.l_11 = 0.2;
    params.l_12 = 0.2;
    params.l_21 = 0.2;
    params.l_22 = 0.2;
    params.l_31 = 0.2;
    params.l_32 = 0.5;

    // Mass parameters
    params.m_11 = 0.3;
    params.m_12 = 0.3;
    params.m_21 = 0.3;
    params.m_22 = 0.3;
    params.m_31 = 0.3;
    params.m_32 = 0.75;
    params.m_d = 0.75;
    params.m_arm = 2.5;

    // Position vectors
    params.vec_shoulder = Eigen::Vector3d(0.5707940216581961, 0.5707940216581961, 0.25);

}