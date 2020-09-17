#include "mbed.h"
#include "crazyflie.h"
#include "parameters.h"
#include <math.h>

// Define all motors as PWM objects
PwmOut motor_1 ( MOTOR1 );
PwmOut motor_2 ( MOTOR2 );
PwmOut motor_3 ( MOTOR3 );
PwmOut motor_4 ( MOTOR4 );

// Define angular velocities (rad/s)
float omega_r_1 ;
float omega_r_2 ;
float omega_r_3 ;
float omega_r_4 ;

// Converts desired angular velocity (rad/s) to PWM signal (%)
float control_motor ( float omega_r )
{
    float pwm = a2*pow(omega_r, 2) + a1*omega_r;
    return pwm;
}
// Converts total trust force (N) and torques (N.m) to angular velocities (rad/s)
void mixer ( float f_t , float tau_phi , float tau_theta , float tau_psi )
{
    float omega_r_1_q = f_t*(1/(4*kl)) + tau_phi*(-1/(4*kl*l)) + tau_theta*(-1/(4*kl*l)) + tau_psi*(-1/(4*kd));
    float omega_r_2_q = f_t*(1/(4*kl)) + tau_phi*(-1/(4*kl*l)) + tau_theta*(1/(4*kl*l)) + tau_psi*(1/(4*kd));
    float omega_r_3_q = f_t*(1/(4*kl)) + tau_phi*(1/(4*kl*l)) + tau_theta*(1/(4*kl*l)) + tau_psi*(-1/(4*kd));
    float omega_r_4_q = f_t*(1/(4*kl)) + tau_phi*(1/(4*kl*l)) + tau_theta*(-1/(4*kl*l)) + tau_psi*(1/(4*kd));
    if (omega_r_1_q < 0) {
        omega_r_1 = 0;
    } else {
        omega_r_1 = sqrt(omega_r_1_q);
    }
    if (omega_r_2_q < 0) {
        omega_r_2 = 0;
    } else {
        omega_r_2 = sqrt(omega_r_2_q);
    }
    if (omega_r_3_q < 0) {
        omega_r_3 = 0;
    } else {
        omega_r_3 = sqrt(omega_r_3_q);
    }
    if (omega_r_4_q < 0) {
        omega_r_4 = 0;
    } else {
        omega_r_4 = sqrt(omega_r_4_q);
    }
}
// Actuate motors with desired total trust force (N) and torques (N.m)
void actuate ( float f_t , float tau_phi , float tau_theta , float tau_psi )
{
    mixer (f_t , tau_phi , tau_theta , tau_psi );
    motor_1 = control_motor ( omega_r_1 );
    motor_2 = control_motor ( omega_r_2 );
    motor_3 = control_motor ( omega_r_3 );
    motor_4 = control_motor ( omega_r_4 );
}
// Main program
int main ()
{
    // Actuate motors with 70% mg total thrust force (N) and zero torques (N.m)
    actuate (0* m*g, 0, 0, 0.001) ;
    wait (5) ;
    // Turn off all motors
    actuate (0 ,0 ,0 ,0) ;
    // End of program
    while ( true )
    {

    }
}