#include "attitude_estimator.h"
#include <math.h>

// Class constructor
AttitudeEstimator::AttitudeEstimator():imu(IMU_SDA, IMU_SCL)
    {
        phi, theta, psi = 0.0;
        p, q, r = 0.0;
        float p_bias, q_bias, r_bias = 0.0;
        BMI088 imu(IMU_SDA,IMU_SCL);
    }

// Initialize class
void AttitudeEstimator::init()
    {
        imu.init();
        for ( int i =0; i <500; i ++)
        {
            imu . read () ;
            p_bias = imu.gx/500.0;
            q_bias = imu.gy/500.0;
            r_bias = imu.gz/500.0;
            wait(dt);
        }
    }

// Estimate Euler angles (rad ) and angular velocities ( rad /s)
void AttitudeEstimator::estimate()
    {
        imu.read();
        p = imu.gx - p_bias;
        q = imu.gy - q_bias;
        r = imu.gz - r_bias;

//        float phi_g = phi + p*dt;
        float phi_g = phi+(p+sin(phi)*tan(theta)*q+cos(phi)*tan(theta)*r)*dt;
        float phi_a = atan2(-imu.ay,-imu.az);

//        float theta_g = theta + q*dt;
        float theta_g = theta+(cos(phi)*q-sin(phi)*r)*dt;
        float theta_a = atan2(imu.ax, (sqrt(pow(imu.ay,2)+pow(imu.az,2))));

//        float psi_g = psi + r*dt;
        float psi_g = psi+(sin(phi)*(1.0/cos(theta))*q+cos(phi)*(1.0/cos(theta))*r)*dt; 

        
        theta = ((1.0-alpha)*theta_g)+alpha*theta_a;
        phi=((1.0-alpha)*phi_g)+alpha*phi_a;
        psi = psi_g;
    }