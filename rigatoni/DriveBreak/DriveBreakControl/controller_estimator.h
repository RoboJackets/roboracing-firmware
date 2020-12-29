#pragma once

#include <Arduino.h>

static const float SQRT_2 = 1.4142135623730951;

// Control Limits and parameters
#define USE_FEEDFORWARD_CONTROL true        //If false, will not use feedforward control

static const float maxVelocity = 10.0;             //maximum velocity in m/s
static const float controller_decel_limit = 6;     //maximum deceleration in m/s^2
static const float controller_accel_limit = 4.5;    //max accel in m/s^2
static const float velocity_filter_bandwidth = 5;

static const float batteryVoltage = 48;
static const byte maxSpeedPwm = 255;
static const byte zeroSpeedPwm = 0; // 100% PWM, 0% Voltage

//Motor feedforward and PI parameters
static const float k_m_inv_r_to_u = 2.8451506121016807;
static const float k_m_inv_r_dot_to_u = 1.93359375;
static const float k_m_inv_r_to_x = 1.0;

static const float k_1m = 4.908560325397578;   //P gain
static const float k_2m = 7.773046874998515;   //I gain

//Brake feedforward and PI parameters
static const float k_b_inv_r_to_u = -10.0;
static const float k_b_inv_r_dot_to_u = -150.0;
static const float k_b_inv_r_to_x = 1.0;

static const float k_1b = -591.5;   //P gain
static const float k_2b = -603;   //I gain

static const float maxBrakingForce = 600.0;    //In Newtons

/*Estimator*/
//Gain matricies
static const float L_pos = 39.94333333;
static const float L_vel = 397.53711112;

//Car parameters. THIS ONLY AFFECTS PART OF THE ESTIMATOR. Go back to the ipython notebook and recalculate ALL the gains in this file if you change these.
static const float d = 10.0;        //Drag in N/(m/s)
static const float Gr = 64.0/22.0;  //Gear ratio
static const float m = 150;         //Car mass in kg
static const float rw = 0.27/2;     //Tire radius in m
static const float Kt = 0.1260;      //Nm/Amp


//Replacement for std::pair only good for floats
struct FloatPair{   
    float first;
    float second;
};

/*Function headers. There are many helper functions not listed here - you don't need to call them!*/
//Encoder
void HallEncoderInterrupt();                        //Encoder callback
float estimate_vel(float, float, float, float);     //Estimates velocity. Call this once per loop. Can handle a timestep of 0.0
float get_speed();                                  //Getter function that returns the current velocity

//Controller
FloatPair gen_control_voltage_brake_force(float, float, float);     //Returns (motor voltage, braking force). All arguments are in SI units (seconds, m/s)
float get_curr_target_speed();                                      //Does not calculate anything. Returns target speed from last call of controller function
