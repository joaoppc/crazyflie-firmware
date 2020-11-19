#ifndef parameters_h
#define parameters_h

#include <cmath>

// Physical constants
const float pi = 3.1416;
const float g = 9.81;       // m/s^2
const float dt_range = 0.05;
const float dt = 0.002;

// Quadcopter dimensions
const float m = 30.0e-3;    // kg
const float I_xx = 16.0e-6; // kg.m^2
const float I_yy = 16.0e-6; // kg.m^2
const float I_zz = 29.0e-6; // kg.m^2
const float l = 33.0e-3;    // m
// parâmetros do motor
const float a2 = 1.16e-07;  //
const float a1 = 4.488e-12; //

const float kl =  1.726e-08;
const float kd = 1.567e-10;

// Attitude Estimator
const float wc = 1.0;  //0.1
const float alpha = wc*dt/(1.0+wc*dt);

// Attitude Controller
const float OS = 0.005f;
const float TS = 0.3f;
const float zeta = abs(log(OS))/sqrt(pow(log(OS),2)+pow(pi,2));
const float wn = 4.0/(zeta*TS);
const float kp_phi = pow(wn,2);
const float kd_phi = 2.0*zeta*wn;
const float kp_theta = kp_phi; 
const float kd_theta = kd_phi;
const float OS_psi = 0.005f;
const float TS_psi = 0.6f;
const float zeta_psi = abs(log(OS_psi))/sqrt(pow(log(OS_psi),2)+pow(pi,2));
const float wn_psi = 2.0/(zeta_psi*TS_psi);
const float kp_psi = pow(wn_psi,2);
const float kd_psi = 2.0*zeta_psi*wn_psi;



// Controlador vertical
const float OS_cv = 0.005f; //0.005
const float TS_cv = 2.0f; //2.0
const float zeta_cv = abs(log(OS_cv))/sqrt(pow(log(OS_cv),2)+pow(pi,2));
const float wn_cv = 4.0/(zeta_cv*TS_cv);
const float kp_cv = pow(wn_cv,2);
const float kd_cv = 2.0*zeta_cv*wn_cv;
const float freq_c_v = 10.0;


// Estimador Vertical
const float zeta_ev = sqrt(2.0)/2.0;
const float L_o = freq_c_v*freq_c_v;
const float L_o2 = 2*zeta_ev*freq_c_v;
const float alpha1 =  L_o*dt_range; //(L_o*dt_range)/(1.0+L_o*dt_range);// 0.3
const float alpha2 = L_o2*dt_range; // (L_o2*dt_range)/(1.0+L_o2*dt_range); // 0.3

// Estimador horizontal
const float gamma = 42.0*pi/180.0;
const float W = 420.0;
const float sigma = (2.0*tan(gamma/2.0))/(W*dt);
const float l_ov = 50.0;

// Controlador horizontal
const float OS_ch = 0.005f;
const float TS_ch = 2.0f;
const float zeta_ch = abs(log(OS_ch))/sqrt(pow(log(OS_ch),2)+pow(pi,2));
const float wn_ch = 4.0/(zeta_ch*TS_ch);
const float kp_ch = pow(wn_ch,2);
const float kd_ch = 2.0*zeta_ch*wn_ch;

#endif