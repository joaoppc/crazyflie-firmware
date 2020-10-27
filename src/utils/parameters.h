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

// Attitude Estimator
const float wc = 100.0;  //0.1
const float dt = 0.002;
const float alpha = wc*dt/(1.0+wc*dt);

// Attitude Controller
const float OS = 0.05f;
const float TS = 0.2f;
const float zeta = abs(log(OS))/sqrt(pow(log(OS),2)+pow(pi,2));
const float wn = 4.0/(zeta*TS);
const float kp_phi = pow(wn,2);
const float kd_phi = 2.0*zeta*wn;
const float kp_theta = kp_phi; 
const float kd_theta = kd_phi;

const float OS_psi = 0.05f;
const float TS_psi = 0.6f;
const float zeta_psi = abs(log(OS_psi))/sqrt(pow(log(OS_psi),2)+pow(pi,2));
const float wn_psi = 4.0/(zeta_psi*TS_psi);
const float kp_psi = pow(wn_psi,2);
const float kd_psi = 2.0*zeta_psi*wn_psi;

#endif