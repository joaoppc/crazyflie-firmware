#include "mbed.h"
#include "vertical_controller.h"

// Class constructor
VerticalController::VerticalController()
{
    ft = 0.0;
}

//Control thrust force (n) given vertical position (m) and velocity (m/s)
void VerticalController::control(float z_r, float z, float w)
{
    ft = m*(g+VerticalController::control_siso(z_r, z, w, kp_cv, kd_cv));
}

// Control aceleration given reference position(m) and current position (m) and velocity (m/s)
// with given gains
float VerticalController::control_siso(float pos_r, float pos, float vel, float kp, float kd)
{
    return kp*(pos_r-pos)+kd*(0.0-vel);
}