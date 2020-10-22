#ifndef parameters_h
#define parameters_h

#include <cmath>

// Physical constants
const float pi = 3.1416;
const float g = 9.81;       // m/s^2

// Quadcopter dimensions
const float m = 30.0e-3;    // kg
const float I_xx = 16.0e-6; // kg.m^2
const float I_yy = 16.0e-6; // kg.m^2
const float I_zz = 1.16e-07; // kg.m^2
const float l = 33.0e-3;    // m
// parâmetros do motor
const float a2 = 1.16e-07;  //
const float a1 = 4.488e-12; //

const float kl =  1.726e-08;
const float kd = 1.567e-10;

const float wc = 1.0;  //0.1
const float dt = 0.002;
const float alpha = wc*dt/(1.0+wc*dt);

#endif