#include "vertical_estimator.h"
#include <math.h>

// Class constructor
VerticalEstimator::VerticalEstimator() : range(E_SDA, E_SCL )
    {
        float z, w, z_m_prev = 0;
        VL53L1X range(E_SDA, E_SCL);
    }

// Initialize class
void VerticalEstimator::init()
    {
        range.init();
    }

// Predict vertical position and velocity from model
void VerticalEstimator::predict(float f_t)
    {
        z += w*dt;
        if (z > 0.05) {
            w += (f_t/m)*dt;
        }
    }

// Correct vertical position and velocity with measurement
void VerticalEstimator::correct(float phi, float theta )
    {
        range.read () ;
        if (range.d < 2.0)
        {
            float z_m = range.d*cos(phi)*cos(theta);
            float w_m =(z_m-z_m_prev)/dt_range;
            z += alpha1*(z_m - z);
            w += alpha2*(w_m - w);
            z_m_prev = z_m;
        }
    }