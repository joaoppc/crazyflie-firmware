#include "mbed.h"
#include "crazyflie.h"
// Declare mixer
Mixer mixer;
// Main program
int main ()
    {
    mixer.arm();
    // Actuate motor with 70% mg total thrust force (N) and zero torques
    mixer.actuate (0.7* m*g ,0 ,0 ,0) ;
    wait (5) ;
    // Turn off all motors
    mixer.disarm();
    // End of program
    while (1)
        {
        }
    }