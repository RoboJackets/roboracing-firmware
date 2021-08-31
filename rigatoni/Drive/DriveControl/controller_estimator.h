#pragma once
/*
This file contains the constants and definitions for the speed controller
The controller first acceleration-limits Software's velocity, then Butterworth lowpass filter it
Then we do state-space PI control with feedforward control to almost exactly follow our filtered target velocity
See motor_and_brake_independent_controllers.ipynb

Changing the PI gains for the motor and brake DOES NOT AFFECT THE CAR'S RESPONSE. Almost all the delay in our response
is caused by the Butterworth lowpass filter. For a more aggressive response increase velocity_filter_bandwidth
(this can be done without recalculating other controller gains).
*/

static const float pi = 3.141592653589793;

static const float SQRT_2 = 1.4142135623730951;

// Control Limits and parameters
#define USE_FEEDFORWARD_CONTROL true        //If false, will not use feedforward control

static const float switch_direction_max_speed = 0.1;    //Will switch directions only when estimated speed is less than this

static const float controller_decel_limit = 6;      //maximum deceleration in m/s^2
static const float controller_accel_limit = 4.5;    //max accel in m/s^2
static const float velocity_filter_bandwidth = 5;   //Lowpass filter on command velocity. This controls how aggressive the car's response is

//Motor feedforward and PI parameters
static const float k_m_inv_r_to_u = 2.845150612101681;
static const float k_m_inv_r_dot_to_u = 1.2890625;
static const float k_m_inv_r_to_x = 1.0;

static const float k_1m = 2.323990012897826;   //P gain
static const float k_2m = 5.182031249999011;   //I gain

//Brake feedforward and PI parameters
static const float k_b_inv_r_to_u = -10.0;
static const float k_b_inv_r_dot_to_u = -100.0;
static const float k_b_inv_r_to_x = 1.0;

static const float k_1b = -391;   //P gain
static const float k_2b = -402;   //I gain

static const float maxBrakingForce = 900.0;    //In Newtons

/*Estimator*/
//Gain matricies
static const float L_pos = 39.91;
static const float L_vel = 396.209;

//Car physics parameters. THIS ONLY AFFECTS PART OF THE ESTIMATOR. Go back to the ipython notebook and recalculate ALL the gains in this file if you change these.
static const float d = 10.0;        //Drag in N/(m/s)
static const float Gr = 64.0/22.0;  //Gear ratio
static const float m = 100.0;         //Car mass in kg
static const float rw = 0.27/2;     //Tire radius in m
static const float Kt = 0.1260;      //Nm/Amp

static const int num_magnets_on_shaft = 16; //Number of encoder ticks per revolution

static const float meters_per_encoder_tick = 2*pi*rw/num_magnets_on_shaft;

//Replacement for std::pair only good for floats
extern "C" struct FloatPair{   
    float first;
    float second;
};

/*Function headers. There are many helper functions not listed here - you don't need to call them!*/
//Encoder
extern "C" float estimate_vel(float, float, float, long);     //Estimates velocity. Call this once per loop. Can handle a timestep of 0.0
extern "C" float get_speed();                                  //Getter function that returns the current velocity

//Controller
extern "C" FloatPair gen_control_voltage_brake_force(float, float, float);     //Returns (motor voltage, braking force). All arguments are in SI units (seconds, m/s)
extern "C" float get_curr_target_speed();                                      //Does not calculate anything. Returns target speed from last call of controller function
extern "C" void reset_controller(float);
extern "C" float get_error_integral();                                         //Returns the current accumulated error integral. Useful for debugging.
