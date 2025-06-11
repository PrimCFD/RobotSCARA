#pragma once
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

static_assert(sizeof(Frame) == 16 * sizeof(double),
              "Frame must be exactly 16 doubles");