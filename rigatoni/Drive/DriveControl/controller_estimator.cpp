#include "controller_estimator.h"

#ifndef max
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#endif


//Pair-making function
FloatPair make_float_pair(float first, float second){
    struct FloatPair retVal = {first, second};
    return retVal;
}

////
// ENCODER
// See motor_and_brake_independent_controllers.ipynb
// Encoder measures position and we want speed, so we created an estimator
////

float est_vel = 0;
float est_pos = 0;

////
// MOTOR CONTROL
// See motor_and_brake_independent_controllers.ipynb
// We construct two independent feedforward controllers - one for the brakes and one for the motor - and run them simultaneously
// Motor controller is in volts, brake controller is in Newtons
////

//Global variables
float error_integral = 0;
float trapezoidal_target_velocity = 0;  //trapezoidal interpolation of Software's commands
float filtered_target_vel = 0;          //Lowpass filtered version of trapezoidal interpolated velocities
float filtered_target_accel = 0;

////
// SPEED ESTIMATOR FUNCTIONS
// See ipython notebook
////

float estimate_vel(float delta_t, float motor_current, float brake_force, long currEncoderCount){
    //All float arguments are in SI units
    //currEncoderCount is in encoder ticks
    //This updates the global velocity estimate, currentSpeed
    //Use the Forward Euler method
    //Call this ONCE and ONLY ONCE per loop. Use get_speed() if you want to get the speed again.
    //Beware of max() and min(), which expand to have multiple function calls!
    //If you are going in reverse, you have to negate brake_force. We assume braking force makes velocity
    //more negative and motor current makes the velocity more positive
    /*
    You MUST zero your encoder count externally when calling this function, ex:
    long currEncoderCount = myEnc.read();
    myEnc.write(0);
    est_speed = estimate_vel(float, float, float, currEncoderCount);
    */
    float SI_encoder_pos = currEncoderCount * meters_per_encoder_tick;
    float change_in_position = SI_encoder_pos - est_pos;
    
    //To prevent floating-point overflow, subtract encoder count from both encoder count AND current position (i.e. shift start point forward by currEncoderCount
    est_pos -= SI_encoder_pos;
    
    if(est_vel < 0){
        //Going backwards, so brakes accelerate not decelerate
        brake_force = -brake_force;
    }

    float new_est_pos = est_pos + delta_t * (est_vel + L_pos*change_in_position);
    float new_est_vel = est_vel + delta_t * (-d/m*est_vel + Gr*Kt/(m*rw)*motor_current - 1.0/m*brake_force + L_vel*change_in_position);
    est_pos = new_est_pos;
    est_vel = new_est_vel;
    return est_vel;
}

float get_speed(){
    return est_vel;
}

////
// MOTOR CONTROLLER FUNCTIONS
// All these assume you are going forwards: brakes decelerate and motors accelerate!
////

//Get current target velocity
float get_curr_target_speed(){
    return filtered_target_vel;
}

//Generate the target velocity and acceleration

float gen_trapezoidal_vel(float timestep, float last_trapezoidal_vel, float software_cmd_vel){
    //Limit our maximum acceleration.
    float trapezoidal_target_vel = min(max(software_cmd_vel, last_trapezoidal_vel - timestep * controller_decel_limit), last_trapezoidal_vel + timestep * controller_accel_limit);
    
    //Cap our speed
    trapezoidal_target_vel = max(trapezoidal_target_vel, 0.0);
    return trapezoidal_target_vel;
}

const float butterworth_coeff_accel = -SQRT_2*velocity_filter_bandwidth;
const float butterworth_coeff_vel = velocity_filter_bandwidth*velocity_filter_bandwidth;

void filter_target_vel_accel(float timestep, float trap_target_vel){
    //Applies a butterworth lowpass filter to the trapezoidal velocities, using the forward Euler approximation
    //Updates filtered_target_vel, filtered_target_accel in place
    
    float new_filtered_target_vel = filtered_target_vel + timestep*filtered_target_accel;
    float new_filtered_target_accel = filtered_target_accel + timestep*(butterworth_coeff_accel*filtered_target_accel + butterworth_coeff_vel*(trap_target_vel - filtered_target_vel));
   
    filtered_target_vel = new_filtered_target_vel;
    filtered_target_accel = new_filtered_target_accel;
}

void reset_controller(float new_speed){
    //A way of resetting the trapezoidal interpolator and the speed filter to a new tartget speed
    //Also resets the error integral
    trapezoidal_target_velocity = new_speed;
    filtered_target_vel = new_speed;
    filtered_target_accel = 0;
    error_integral = 0;
}

//Feedforward reference commands for motor + brake

FloatPair gen_motor_feedforward_reference(float target_vel, float target_accel){
    //Returns (voltage reference in volts, velocity reference in m/s) for motor controller
    if(!USE_FEEDFORWARD_CONTROL){
        return make_float_pair(0.0, 0.0);
    }
    //Generate the feedforward reference commands
    float voltage_ref = k_m_inv_r_to_u * target_vel + k_m_inv_r_dot_to_u * target_accel;
    float vel_ref_m = k_m_inv_r_to_x * target_vel;
    
    return make_float_pair(voltage_ref, vel_ref_m);
}

FloatPair gen_brake_feedforward_reference(float target_vel, float target_accel){
    //Returns (force reference in N, velocity reference in m/s) for brake controller
    if(!USE_FEEDFORWARD_CONTROL){
        return make_float_pair(0.0, 0.0);
    }
    //Generate the feedforward reference commands
    float brake_force_ref = k_b_inv_r_to_u * target_vel + k_b_inv_r_dot_to_u * target_accel;
    float vel_ref_b = k_b_inv_r_to_x * target_vel;
    
    return make_float_pair(brake_force_ref, vel_ref_b);
}

//PI control functions for motor and brake
float gen_motor_PI_control_voltage(float voltage_ref, float vel_ref_m, float current_vel, float err_integral){
    return voltage_ref - k_1m * (current_vel - vel_ref_m) - k_2m * err_integral;
}

float gen_brake_PI_control_voltage(float brake_force_ref, float vel_ref_b, float current_vel, float err_integral){
    return brake_force_ref - k_1b * (current_vel - vel_ref_b) - k_2b * err_integral;
}

FloatPair gen_control_voltage_brake_force(float delta_t, float est_speed, float software_cmd_vel){
    //The main controller function.
    //Returns (motor voltage, braking force). All arguments are in SI units (seconds, m/s)
    //Beware that this assumes you are going forward. If (software_cmd_vel > est_speed) we use the brakes. If
    //If you are in reverse, make est_speed and software_cmd_vel negative
    
    //Get target velocity + accel
    trapezoidal_target_velocity = gen_trapezoidal_vel(delta_t, trapezoidal_target_velocity, software_cmd_vel);
    
    //Now filter it
    filter_target_vel_accel(delta_t, trapezoidal_target_velocity);
    
    //For the motor
    //Get feedforward reference values
    FloatPair motor_voltage_velocity_ref = gen_motor_feedforward_reference(filtered_target_vel, filtered_target_accel);
    float voltage_ref = motor_voltage_velocity_ref.first;
    float vel_ref_m = motor_voltage_velocity_ref.second;
    
    //Get voltage command
    float voltage_command = gen_motor_PI_control_voltage(voltage_ref, vel_ref_m, est_speed, error_integral);
    
    //For the brakes
    //Get feedforward reference values
    FloatPair brake_force_velocity_ref = gen_brake_feedforward_reference(filtered_target_vel, filtered_target_accel);
    float brake_force_ref = brake_force_velocity_ref.first;
    float vel_ref_b = brake_force_velocity_ref.second;
    
    //Get brake force command
    float brake_force_command = gen_brake_PI_control_voltage(brake_force_ref, vel_ref_b, est_speed, error_integral);
    brake_force_command = max(brake_force_command, 0);
    
    //Update error integral
    error_integral += delta_t * (est_speed - filtered_target_vel);
    
    return make_float_pair(voltage_command, brake_force_command);
}

float get_error_integral(){
    return error_integral;
}
