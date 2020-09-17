#include "mbed.h"
#include "crazyflie.h"

// Define all LEDs as digital output objects
DigitalOut ledRedR(LED_RED_R,!false);
DigitalOut ledRedL(LED_RED_L,!false);
DigitalOut ledBlueL(LED_BLUE_L,!false);
DigitalOut ledGreenR(LED_GREEN_R,!false);
DigitalOut ledGreenL(LED_GREEN_L,!false);

// Define all motors as PWM objects
PwmOut motor1(MOTOR1);
PwmOut motor2(MOTOR2);
PwmOut motor3(MOTOR3);
PwmOut motor4(MOTOR4);
int turns = 0;

// Main program
int main()
{
    // Blink blue LED indicating inicialization (5 seconds)
    
    // Turn on red LEDs indicating motors are armed
    
    // Test all motors with different frequencies (to make different noises)
    
    // Turn off red LEDs indicating motors are disarmed
    
    // Blink green LEDs indicating end of program
    while(true)
    {
        while(turns < 10){
            ledBlueL = !ledBlueL;
            wait(0.5);
            turns++;
        }
        ledRedL = 0;
        wait(1.0);
        motor1 = 0.1;
        motor1.period(1.0/400.0);
        wait(3.0);
        motor1 = 0.0;
        motor2 = 0.3;
        motor2.period(1.0/650.0);
        wait(3.0);
        motor2 = 0.0;
        motor3 = 0.5;
        motor3.period(1.0/850.0);
        wait(3.0);
        motor3 = 0.0;
        motor4 = 0.7;
        motor4.period(1.0/1000.0);
        wait(3.0);
        motor4 = 0.0;
        ledRedL = 1;
        turns = 0;
        while(turns < 10){
            ledGreenR = !ledGreenR;
            wait(0.5);
            turns++;
        }
    }
}
