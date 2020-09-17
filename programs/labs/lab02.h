#include "mbed.h"
#include "crazyflie.h"
#include "parameters.h"
#include <math.h>

// Define motor 1 as PWM output object
PwmOut motor(MOTOR1);

float control_motor( float omega_r )
 {
     float pwm = a2*pow(omega_r, 2) + a1*omega_r;
     return pwm;
 }
// Main program
int main()
{
    motor.period(1.0/500.0);
    motor = control_motor(1000.0);
    wait (1.0);
    motor = 0.0;

    while(true) 
    {
    }
}