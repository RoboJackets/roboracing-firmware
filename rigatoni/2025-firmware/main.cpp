/**
 * This firmware is targeted for the Nucleo-H743ZI2: https://www.st.com/en/evaluation-tools/nucleo-h743zi.html
 * Note that it is slightly different from the Nucleo-H743ZI which RoboRacing does not have:
 * https://community.st.com/t5/stm32-mcus-boards-and-hardware/what-is-the-difference-between-nucleo-h743zi-and-nucleo-h743zi2/td-p/275380
 *
 * VESC CAN Documentation: https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md
 * Archived note: CAN communication did not work with the old drive motor controller on Rigatoni (FSESC 75/300).
 * A possible reason is that the FSESC's firmware did not recognize duty cycle inputs over CAN.
 * Another reason why this may have been the case is that duty cycle over UART did not work with the FSESC either,
 * so the code used ERPM control for drive.
 *
 * 5 m/s for drive correponds to about 5000 ERPM or 1000 RPM
 *
 * Useful reference for Velocity PID: https://deltamotion.com/support/webhelp/rmctools/Controller_Features/Control_Modes/Velocity_PID.htm
 *
 * Min ERPM is 900 RPM in VESC Tool to spin the drive motor.
 * VESC Tool can be downloaded here: https://vesc-project.com/vesc_tool
 */
#include "mbed.h"
#include "PwmIn.h"
#include "rjnet_mbed_udp.h"
#include <atomic>
#include <arm_acle.h>
#include <VescUart.h>

// Constants for IDs
#define ETHERNET_ID 3
#define DRIVE_VESC_ID 90
#define STEERING_VESC_ID 10
#define MOTOR_STATUS_MSG_ID 9

// Constant expressions
// BAUD_RATE for UART, equivalent to 115200 bytes/second assuming 8N1 is used (8 data bits, no parity, 1 start bit, 1 stop bit)
constexpr int BAUD_RATE = 115200;
constexpr PinName TX = D14;
constexpr PinName RX = D15;

constexpr bool ON_GROUND = true;
// constexpr bool MANUAL_CONTROL = false;

// Object handle for communicating to VESC using UART
// Unused for current CAN bus setup
VescUart vesc;
BufferedSerial vesc_serial(TX, RX, BAUD_RATE);

// Using current control for steering
// constexpr float K_u = 0.1007125;
// constexpr float T_u = 1.47;
// constexpr float K_p = 0.6 * K_u;
// constexpr float K_i = 0.4 * K_u / T_u;
// constexpr float K_d = 0.4 * K_u * T_u;

// Using duty cycle control for steering off the ground
// Using slightly modified values of classic PID row in Ziegler-Nichols method:
// Only change is the Kp value which is listed as 0.6 in the table for the constant factor
// as opposed to 0.5 which is used here.
// https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
constexpr float K_u_duty = 0.7957013448 * 1.8333333;
constexpr float T_u_duty = 0.46;
constexpr float K_p_duty = 0.5 * K_u_duty;
constexpr float K_i_duty = 1.2 * K_u_duty / T_u_duty;
constexpr float K_d_duty = 0.075 * K_u_duty * T_u_duty;

// Duty cycle control for on the ground
// ong means on the ground, nothing means off the ground
// 2.39 is the previous known working constant when the steering motor wasn't loose
// constexpr float K_u_duty_ong = 2.39;
// constexpr float K_u_duty_ong = 1.195;
//
// Kf corresponds to a feedforward term which adds a proportional amount of control input
// to the steering angle. The reason for doing this is to counteract the effect of gravity
// on the steering wheels because the Ackermann steering system actually lifts the wheels
// off the ground, so there's an additional force from gravity that needs to be accounted for.
// ** These values are still not finalized. **
constexpr float K_u_duty_ong = 1;
constexpr float T_u_duty_ong = 0.34;
constexpr float K_p_duty_ong = 0.6 * K_u_duty_ong;
constexpr float K_i_duty_ong = 1.2 * K_u_duty_ong / T_u_duty_ong;
constexpr float K_d_duty_ong = 0.075 * K_u_duty_ong * T_u_duty_ong;
constexpr float K_f_duty_ong = 0.04;

// Bounds for PID control to account for any non-idealities in the system
// For MIN_DUTY, this is used to indicate what the minimum control input is
// that allows the steering angle to actually change. If the control input
// is below this value, the wheels cannot turn and the error cannot be reduced.
constexpr float MIN_DUTY_CMD = 0.015;
constexpr float MIN_DUTY_CMD_ONG = 0.02;
// To prevent integral windup: https://en.wikipedia.org/wiki/Integral_windup
constexpr float MAX_INTEGRAL_TERM_ONG = 0.01;
constexpr float MAX_INTEGRAL_TERM = 0.001;

// constexpr float kP = 1.85;
// constexpr float kI = 0.04;
// constexpr float kD = 12;
// constexpr float kF = 0.3;

// Max PID command value
// Used to cap the maximum value that the PID loop can reach
// A little bit redundant since the actual control input (duty cycle for steering) is also capped.
constexpr unsigned int MAX_COMMAND = 75;

// This is a conversion factor from the current value of the potentiometer to the current steering angle
// of the wheels. This is necessary because even though the potentiometer turns all the way from 0-1,
// the amount of rotation for wiper in a potentiometer is usually around 270 degrees, but the wheels
// of the robot can only turn a maximum of 30 degrees in either direction (maximum range of 60 degrees).
// Thus, the actual range of ADC values that the potentiometer will read is less than 0-1. With
// the current value of 147.06, the range of potentiometer values is 0.345-0.753
// A sample calculation is as follows:
//   If the potentiometer is read as 0.6 by the ADC, the current angle is:
//   (0.6 - 0.549) * 147.06 = 7.5 degrees
constexpr float POT_ELECTRICAL_RANGE = 147.06;

// Potentiometer reading when wheels are straight
// This is on a scale of 0-1
// If the potentiometer is turned all the way one way, it will have a value of 0
// If it's turned all the way the other way, it will have a value of 1
// This is because the voltage read by the microcontroller's ADC will range from 0-3.3V which
// is automatically converted to a scale of 0-1.
// If the potentiometer is ever removed from the steering shaft,
// this offset will need to modified to the value of the ADC input when the
// potentiometer is remounted and the wheels are pointed straight.
constexpr float POT_OFFSET = 0.549;

// Max physical steering angle of the robot
// If the wheels turn any further, they will hit the electrical board which
// is not desired
constexpr float MAX_ANGLE = 30;

// Used to cap the max driving speed of the robot
// Can be increased if the software stack is more reliable and
// the robot can reasonably drive faster without going off course
// or hitting an obstacle
//
// Currently unused
constexpr float MAX_VELOCITY = 5.0;

// Maximum duty cycle used for controlling the steering motor
// Caps the maximum rate of change for the angle to try to prevent
// anything mechanical from brekaing
constexpr float MAX_STEERING_DUTY_CYCLE = 0.35;

// Units of microseconds
// Duration of the high portion of the pulse
// Corresponds to when the steering wheel on the RC car remote is not turned/robot's wheels should be going straight
constexpr int STRAIGHT_PULSEWIDTH = 1540;

// The max angle divided by the range of the pwm pulse
// The range of values for the PWM pulse width is 1080-2000 us, so the
// range from the center value is 460 us
constexpr float PULSEWIDTH_TO_DEGREES = MAX_ANGLE/460.0;

// Corresponds to when the trigger on the RC car remote is not moved/robot is stopped
constexpr int ZERO_SPEED_PULSEWIDTH = 1540;

// Units of m/s
// The max speed of the robot when being controlled by the RC car remote in manual mode
constexpr float MAX_MANUAL_SPEED = 3.65;

// The max speed divided by the range of the pwm pulse
// The range of values for the PWM pulse width is 1080-2000 us, so the
// range from the center value is 460 us
constexpr float PULSEWIDTH_TO_MPS = MAX_MANUAL_SPEED/460.0;

// RPM is revolutions per minute, and it needs to be converted to
// meters per second. There are 60 seconds in a minute and the
// circumference of the rear wheels are 83.4 cm. The circumference
// was calculated using pi*diameter, and the diameter of the rear wheel
// was measured to be 26.5 cm or 10.4 in
constexpr float RPM_TO_VEL = 0.834f/60.f;

// Corresponds to the gear ratio of the drive motor shaft's gear to the robot's rear axle's gear
// since a pulley connects the two gears
constexpr float GEAR_RATIO = 62.f/22.f;

// There are 5 pole pairs for the drive motor (NOTE: this is not the same for the steering motor)
// so the electrical RPM of the drive motor is 5 times higher than the actual RPM of the drive motor's
// shaft
constexpr float ERPM_TO_RPM = 0.2f;

// Unused
// Don't remember clearly anymore, but this may correspond to how much increase in duty cycle
// is needed to add one meter of second to the drive speed. That almost may not be true
constexpr float DUTY_CYCLE_PER_MPS = 0.05476f;

// This is the refresh rate for the steering and drive control loops
// 10ms is equivalent to 100 Hz
constexpr Kernel::Clock::duration_u32 REFRESH_RATE = 10ms;

// Instantiate all I/O interface objects
// Bidirectional
CAN can(PD_0, PD_1);

// Inputs
AnalogIn pot(A0);
PwmIn steering_RC(D7);
PwmIn drive_RC(D6);
DigitalIn standby(D9);
DigitalIn stopped(D10);
DigitalIn manual_mode(D11);
DigitalIn software_mode(D12);

// Outputs
DigitalOut green_led(LED1);
DigitalOut yellow_led(LED2);
DigitalOut red_led(LED3);
DigitalOut HV_enable(D5);

// Counters for counting how many CAN messages are sent successfully by the microcontroller
// to the motor controllers
uint64_t steering_counter = 0;
uint64_t drive_counter = 0;
bool writing_steering_messages = false;
bool writing_drive_messages = false;

// Atomic variables for storing values used across threads

// The target angle that the robot's steering shaft should be turned to
std::atomic<float> desired_angle;
std::atomic<float> manual_steering_angle;
std::atomic<float> manual_drive_speed;

// Units of degrees, current steering angle
std::atomic<float> current_steering_angle;

// Units of m/s, current drive speed of the robot
std::atomic<float> current_velocity;

// Units of m/s
// Stores the desired speed sent by the software stack on the Jetson to the microcontroller
// via UDP over Ethernet
std::atomic<float> autonomous_drive_speed;

enum State
{
    STOPPED,
    STANDBY, 
    MANUAL,
    SOFTWARE
};

// Store current and last state to keep track of when the state has changed
State current_state;
State last_state;

// Currently 6 threads are used in total
// These are the 5 additional threads that need to be declared
// There is also the main thread which is the looping main function
Thread send_steering_thread;
Thread manual_steering_thread;
Thread manual_drive_thread;
Thread send_drive_thread;
Thread network_thread;

// Networking stuff
// Function header needed to instantiate UDP object for reading and sending UDP packets
void process_single_message(const SocketAddress & senders_address, const char incoming_udp_message[], unsigned int num_bytes_in_message);

RJNetMbed rjnet_udp(nucleoIP, &process_single_message);

// Inlined functions for optimization

// Returns 1 or -1 for the sign of a numeric input value
template<typename T>
inline int sgn(T n) {
    return (n > 0) - (n < 0);
}

// Returns the value of range or the max bound value if the range value is larger than the max bound in magnitude (irregardless of sign)
template<typename T>
inline T abs_max_bound(T val, T range) {
    return sgn(val) * min(abs(val), abs(range));
}

// Returns the maximum of val and range in magnitude (ignores sign)
// which is equivalent to setting a minimum bound for the range input value
template<typename T>
inline T abs_min_bound(T val, T range) {
    return sgn(val) * max(abs(val), abs(range));
}

// Converts ERPM to velocity in m/s for the drive motor
template<typename T>
inline T erpm_to_vel(T val) {
    return val * RPM_TO_VEL / GEAR_RATIO * ERPM_TO_RPM;
}

// Converts velocity to ERPM for the drive motor
template<typename T>
inline T vel_to_erpm(T val) {
    return val / RPM_TO_VEL * GEAR_RATIO / ERPM_TO_RPM;
}

// Functions

// Used to write CAN message to drive or steering motor controller
// The ID corresponds to the motor controller's unique ID
// The data is the payload for the CAN message
// The length is for the number of bytes in the data payload
// API Reference for CANMessage class: https://os.mbed.com/docs/mbed-os/v6.16/mbed-os-api-doxy/classmbed_1_1_c_a_n_message.html
// API Reference for CAN class: https://os.mbed.com/docs/mbed-os/v6.16/apis/other-driver-apis.html
// Source code for CAN header: https://github.com/NordicPlayground/mbed/blob/master/libraries/mbed/api/CAN.h
void can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
    // printf("ID: %d\n", id);
    // Mask lower 8 bits to check for VESC_ID
    if ((id & 0xff) == STEERING_VESC_ID) {
        // Increment counter if message was able to send successfully
        // and keep track of whether a CAN message was able to send or not
        // If the CAN messages aren't able to send, the microcontroller can
        // reset the CAN interface to try to get it to start sending again
        if (can.write(CANMessage(id, data, len, CANData, CANExtended))) {
            // printf("Steering message sent: %llu\n", steering_counter);
            steering_counter++;
            writing_steering_messages = true;
        } else {
            // printf("Err\n");
            writing_steering_messages = false;
        }
    } else if ((id & 0xff) == DRIVE_VESC_ID) {
        if (can.write(CANMessage(id, data, len, CANData, CANExtended))) {
            drive_counter++;
            writing_drive_messages = true;
        } else {
            writing_drive_messages = false;
        }
    }
}

// Following functions taken from here:
// https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md
// For duty cycle control over CAN
void comm_can_set_duty(uint8_t controller_id, float duty) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
	can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

// Current control over CAN
void comm_can_set_current(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
	can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

// RPM control over CAN
void comm_can_set_rpm(uint8_t controller_id, float rpm) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)rpm, &send_index);
	can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

// Used to read the potentiometer's value
// Range is from 0-1
// Samples the ADC value 100 times and averages it to try to reduce
// the effect of noise on the measurement
inline float sample_pot() {
    float pot_read = 0;
    for (int i = 0; i < 100; ++i) {
        pot_read += pot.read();
    }
    pot_read /= 100;
    return pot_read;
}

// Read the steering angle input value from the wheel on the RC car remote
void read_manual_steering()
{
    printf("Manual steering thread started\n");
    while (true) {
        auto time = Kernel::Clock::now();
        if (current_state == MANUAL) {
            float pulse = steering_RC.pulsewidth();
            // If wireless receiver or something else glitches and returns a 0 for the pulse width,
            // use the last known target angle to prevent the actual target angle from being set to
            // all the way to the left (-30 degrees)
            if (pulse == 0)
            {
                desired_angle.store(manual_steering_angle.load());
            }
            // Valid pulse width range with 80 us of slack if the pulse width is slightly outside the expected range
            // due to non-idealities
            // If pulse width is > 2080 or 0 < width < 1000, ideally we should be logging it to see if the wireless
            // receiver ever glitches and returns a pulse width that is too large or too small
            else if (pulse >= 1000.0f && pulse <= 2080.0f)
            {
                float target_angle = abs_max_bound((pulse - STRAIGHT_PULSEWIDTH) * PULSEWIDTH_TO_DEGREES, MAX_ANGLE);
                desired_angle.store(target_angle);
                manual_steering_angle.store(target_angle);
                printf("Steering pulse width: %f\n", target_angle);
            }
        }
        ThisThread::sleep_until(time + 50ms);
    }
}

// Read the drive speed input value from the trigger of the RC car remote
void read_manual_drive()
{
    printf("Manual drive thread started\n");
    while (true) {
        auto time = Kernel::Clock::now();
        if (current_state == MANUAL) {
            float pulse = drive_RC.pulsewidth();
            // Valid range for pulse width
            if (pulse >= 1000.0f && pulse <= 2080.0f) {
                float target_speed = abs_max_bound((pulse - ZERO_SPEED_PULSEWIDTH) * PULSEWIDTH_TO_MPS, MAX_MANUAL_SPEED);
                manual_drive_speed.store(target_speed);
                printf("Drive pulse width: %f\n", target_speed);
            }
        }
        ThisThread::sleep_until(time + 50ms);
    }
}

// Send a new steering command value to the steering motor controller over CAN
// Uses a PID control loop to move the current steering angle to the target angle
// while minimizing error
void send_steering_command() {
    float pos = (sample_pot() - POT_OFFSET) * POT_ELECTRICAL_RANGE;
    float prevPos = pos;
    float I = 0;
    CANMessage msg;
    msg.format = CANExtended;
    bool read_message = false;
    bool receiving_messages = false;
    while (true) {
        auto time = Kernel::Clock::now();
        auto timeMs = Kernel::get_ms_count();
        pos = (sample_pot() - POT_OFFSET) * POT_ELECTRICAL_RANGE;

        float err = (desired_angle - pos);
        // I is the accumulation of error over time
        I += err;
        // Bound I to prevent integral windup: https://en.wikipedia.org/wiki/Integral_windup
        if (ON_GROUND) {
            I = abs_max_bound(I, MAX_INTEGRAL_TERM_ONG * MAX_COMMAND / K_i_duty_ong);
        } else {
            I = abs_max_bound(I, MAX_INTEGRAL_TERM * MAX_COMMAND / K_i_duty);
        }
        float command;
        // This is the actual PID calculation
        // P is proportional - the Kp constant is multiplied by the error (difference between current angle and target angle)
        // I is integral - The Ki constant is multiplied by the accumulated error over time
        //   This allows for things like environmental disturbances to be accounted for because there may be a consistent
        //   force such as an obstacle touching the wheel that prevents the wheels from turning to the desired angle with
        //   just the P term because the error at a single point in time is too small for the command to be large enough
        //   to turn the steering shaft with the motor. With accumulation of error over time, it allows the command to keep
        //   growing until the force is large enough to overcome the environmental disturbance and decrease the error. Keep in
        //   mind though that the accumulated error will only increase once error is added from the opposite direction, so
        //   oscillation is inevitable, but ideally the oscillation's magnitude decreases until the command is stable.
        // D is derivative - The Kd constant is multiplied by the change in steering angle. This accounts for overshoot where
        //    the command is changing the steering angle too rapidly and would cause the wheels to oscillate for longer before
        //    stabilizing. With the D term, the steering angle can settle to the target angle more quickly which is ideal. In
        //    other words, this term helps with rapid changes when the change in steering angle is very large (such as switching
        //    from turning all the way to the left to turning all the way to the right, a swing of 60 degrees which is the max)
        if (ON_GROUND) {
            command = K_p_duty_ong * err + K_d_duty_ong * (pos - prevPos) + K_i_duty_ong * I + K_f_duty_ong * desired_angle;
        } else {
            command = K_p_duty * err + K_d_duty * (pos - prevPos) + K_i_duty * I;
        }

        // Used for determining K_u, the ultimate gain where the wheels oscillate at consistent angles
        // This value should be the smallest possible gain that causes the wheels to never reach the desired angle
        // float command = K_u_duty_ong * err;
        command = abs_max_bound<float>(command, MAX_COMMAND);
        command = command / MAX_COMMAND * MAX_STEERING_DUTY_CYCLE;

        // On the ground code:
        if (ON_GROUND) {
            if (abs(desired_angle) > 25)
            {
                command = abs_min_bound<float>(command, MIN_DUTY_CMD_ONG);
            }
            else if (abs(desired_angle) > 14)
            {
                // Old error threshold when steering motor wasn't loose
                // if (abs(err) < 0.125)
                if (abs(err) < 0.5)
                {
                    command = 0;
                }
                else {
                    command = abs_min_bound<float>(command, MIN_DUTY_CMD_ONG);
                }
            }
            else
            {
                if (abs(err) < 5)
                {
                    command = 0;
                }
                else {
                    command = abs_min_bound<float>(command, MIN_DUTY_CMD_ONG);
                }
            }
        } else {
            // 1 degree of error seems to work for a desired angle of 30 or -30 degrees (only for off the ground)
            if (abs(err) < 1)
            {
                command = 0;
            }
            else {
                command = abs_min_bound<float>(command, MIN_DUTY_CMD);
            }
        }
        
        printf("Time: %llu:, Target: %f, Position: %f, Duty Cycle: %f\n", timeMs, desired_angle.load(), pos, -command);
        comm_can_set_duty(STEERING_VESC_ID, -command);
        current_steering_angle.store(pos);

        // char outgoing_message [64];
        // sprintf(outgoing_message, "Current angle = %f", pos);
        // rjnet_udp.send_single_message(outgoing_message, jetsonIP);

        prevPos = pos;
        // if (can.read(msg)) {
        //     read_message = true;
        //     receiving_messages = true;
        //     // ID for EPRM, Current, and duty cycle is 9
        //     // if (msg.id >> 8 == MOTOR_STATUS_MSG_ID) {
        //     //     uint32_t* rpm = (uint32_t*)msg.data;
        //     //     uint16_t* current = (uint16_t* )(msg.data + 4);

        //     //     char receivedByte[4];
        //     //     sprintf(receivedByte, "%02X%02X", msg.data[6], msg.data[7]);
        //     //     float duty_cycle = (short) strtol(receivedByte, NULL, 16);

        //     //     char currentBytes[4];
        //     //     sprintf(currentBytes, "%02X%02X", msg.data[4], msg.data[5]);
        //     //     float current_f = (short) strtol(currentBytes, NULL, 16);

        //     //     unsigned char erpm_vals[4];
        //     //     erpm_vals[0] = msg.data[3];
        //     //     erpm_vals[1] = msg.data[2];
        //     //     erpm_vals[2] = msg.data[1];
        //     //     erpm_vals[3] = msg.data[0];
        //     //     int erpm = *(int *)erpm_vals;
        //     //     // printf("Steering - ID: %u, ERPM: %d, Current: %f, Duty Cycle %f\n", msg.id, erpm, current_f * 0.1, duty_cycle * 0.001);
        //     // } else {
        //     //     // printf("Steering - ID: %u\n", msg.id);
        //     // }
        // } else {
        //     receiving_messages = false;
        // }
        if (steering_counter > 0 && !writing_steering_messages) {
            can.reset();
        }
        ThisThread::sleep_until(time + REFRESH_RATE);
    }
}

void send_drive_command() {
    CANMessage msg;
    msg.format = CANExtended;
    while (true) {
        auto time = Kernel::Clock::now();
        float target_speed;
        switch (current_state) {
            case STANDBY:
                target_speed = 0;
                break;
            case MANUAL:
                target_speed = manual_drive_speed.load();
                break;
            case SOFTWARE:
                target_speed = autonomous_drive_speed.load();
                break;
            default:
                target_speed = 0;
        }
        float command = abs_max_bound(target_speed, MAX_MANUAL_SPEED);
        command = vel_to_erpm(target_speed);
        // printf("Target: %f, Command: %f\n", target_speed, command);
        comm_can_set_rpm(DRIVE_VESC_ID, command);

        if (can.read(msg)) {
            // read_message = true;
            // receiving_messages = true;
            // ID for EPRM, Current, and duty cycle is 9
            if ((msg.id & 0xff) == DRIVE_VESC_ID && (msg.id >> 8) == MOTOR_STATUS_MSG_ID) {
                // uint32_t* rpm = (uint32_t*)msg.data;
                // uint16_t* current = (uint16_t* )(msg.data + 4);

                char receivedByte[4];
                sprintf(receivedByte, "%02X%02X", msg.data[6], msg.data[7]);
                float duty_cycle = (short) strtol(receivedByte, NULL, 16);

                char currentBytes[4];
                sprintf(currentBytes, "%02X%02X", msg.data[4], msg.data[5]);
                float current_f = (short) strtol(currentBytes, NULL, 16);

                unsigned char erpm_vals[4];
                erpm_vals[0] = msg.data[3];
                erpm_vals[1] = msg.data[2];
                erpm_vals[2] = msg.data[1];
                erpm_vals[3] = msg.data[0];
                int erpm = *(int *)erpm_vals;
                float velocity = erpm_to_vel(static_cast<float>(erpm));

                // char outgoing_message [64];
                // sprintf(outgoing_message, "Current speed = %f", velocity);
                // rjnet_udp.send_single_message(outgoing_message, jetsonIP);

                current_velocity.store(velocity);

                auto timeMs = Kernel::get_ms_count();
                printf("Time: %llu, Drive - ID: %u, ERPM: %d, Current: %f, Duty Cycle %f\n", timeMs, msg.id, erpm, current_f * 0.1, duty_cycle * 0.001);
            } else {
                printf("Drive - Message ID: %u\n", msg.id);
            }
        }
        ThisThread::sleep_until(time + REFRESH_RATE);
    }
}

void process_single_message(const SocketAddress & senders_address, const char incoming_udp_message[], unsigned int num_bytes_in_message) {
    //Parses a UDP message we just recieved. Sends any received data through the CAN network.
    if(rjnet_udp.are_ip_addrs_equal(jetsonIP, senders_address)) {
        //Parse angle from message. Doing incoming_message + 2 ignores the first two characters
        if (incoming_udp_message[0] == 'V') {
            // Message format for velocity is "V=%f"
            float velocity;
            sscanf(incoming_udp_message + 2, "%f", &velocity);

            //Reply to Jetson at once
            char outgoing_message [64];
            sprintf(outgoing_message, "Got speed = %f", velocity);
            printf("Got speed = %f\n", velocity);
            rjnet_udp.send_single_message(outgoing_message, jetsonIP);
            if (current_state == SOFTWARE) {
                autonomous_drive_speed.store(abs_max_bound(velocity, MAX_MANUAL_SPEED));
            }
        } else if (incoming_udp_message[0] == 'A') {
            // Message format for steering angle is "A=%f"
            float angle;
            sscanf(incoming_udp_message + 2, "%f", &angle);

            //Reply to Jetson at once
            char outgoing_message [64];
            sprintf(outgoing_message, "Got angle = %f", angle);
            printf("Got angle = %f\n", angle);
            rjnet_udp.send_single_message(outgoing_message, jetsonIP);
            if (current_state == SOFTWARE) {
                desired_angle.store(abs_max_bound(angle, MAX_ANGLE));
            }
        } else if (incoming_udp_message[0] == 'R') {
            NVIC_SystemReset();
        }
    }
}

void evaluateState()
{
    if (stopped.read()) {
        current_state = STOPPED;
        printf("Currently in STOPPED state\n");
    } else if (standby.read()) {
        current_state = STANDBY;
        printf("Currently in STANDBY state\n");
    } else if (manual_mode.read()) {
        current_state = MANUAL;
        printf("Currently in MANUAL state\n");
    } else if (software_mode.read()) {
        current_state = SOFTWARE;
        printf("Currently in SOFTWARE state\n");
    } else {
        // something wrong or unexpected happened, best to stop
        current_state = STANDBY;
        printf("Unexpected state!\n");
    }
}

int main()
{
    printf("Reset\n");
    can.frequency(500000);
    

    // Testing code for measuring potentiometer accuracy
    // while (true) {
    //     // float position = (sample_pot() - POT_OFFSET) * POT_ELECTRICAL_RANGE;
    //     float position = sample_pot();
    //     printf("Position - %f\n", position);
    //     ThisThread::sleep_for(100ms);
    // }

    // Testing code for sending and reading drive VESC
    // while (true) {
    //     comm_can_set_rpm(DRIVE_VESC_ID, 1000);
    //     if (can.read(msg)) {
    //         // read_message = true;
    //         // receiving_messages = true;
    //         // ID for EPRM, Current, and duty cycle is 9
    //         if ((msg.id & 0xff) == DRIVE_VESC_ID && msg.id >> 8 == 0x9) {
    //             uint32_t* rpm = (uint32_t*)msg.data;
    //             uint16_t* current = (uint16_t* )(msg.data + 4);

    //             char receivedByte[4];
    //             sprintf(receivedByte, "%02X%02X", msg.data[6], msg.data[7]);
    //             float duty_cycle = (short) strtol(receivedByte, NULL, 16);

    //             char currentBytes[4];
    //             sprintf(currentBytes, "%02X%02X", msg.data[4], msg.data[5]);
    //             float current_f = (short) strtol(currentBytes, NULL, 16);

    //             unsigned char erpm_vals[4];
    //             erpm_vals[0] = msg.data[3];
    //             erpm_vals[1] = msg.data[2];
    //             erpm_vals[2] = msg.data[1];
    //             erpm_vals[3] = msg.data[0];
    //             int erpm = *(int *)erpm_vals;
    //             auto timeMs = Kernel::get_ms_count();
    //             printf("Time: %llu, Drive - ID: %u, ERPM: %d, Current: %f, Duty Cycle %f\n", timeMs, msg.id, erpm, current_f * 0.1, duty_cycle * 0.001);
    //         } else {
    //             printf("Drive - Message ID: %u\n", msg.id);
    //         }
    //     }
    //     ThisThread::sleep_for(50ms);
    // }
    
    desired_angle.store((sample_pot() - POT_OFFSET) * POT_ELECTRICAL_RANGE);

    rjnet_udp.start_network_and_listening_threads();

    send_steering_thread.start(send_steering_command);
    send_drive_thread.start(send_drive_command);

    manual_drive_thread.start(read_manual_drive);
    manual_steering_thread.start(read_manual_steering);

    evaluateState();
    last_state = current_state;
    string state_string;

    while (true) {
        auto time = Kernel::Clock::now();
        evaluateState();
        switch (current_state) {
            case STOPPED:
                // displayStackLights(1, 0, 0);
                red_led.write(1);
                yellow_led.write(0);
                green_led.write(0);
                state_string = "OFF";
                break;
            case STANDBY:
                // displayStackLights(0, 1, 0);
                red_led.write(0);
                yellow_led.write(1);
                green_led.write(0);
                state_string = "STOPPED";
                break;
            case MANUAL:
                // displayStackLights(0, 1, 1);
                red_led.write(0);
                yellow_led.write(1);
                green_led.write(1);
                state_string = "MANUAL";
                break;
            case SOFTWARE:
                // displayStackLights(0, 0, 1);
                red_led.write(0);
                yellow_led.write(0);
                green_led.write(1);
                state_string = "AUTONOMOUS";
                break;
            default:
                // displayStackLights(1, 1, 1);
                red_led.write(1);
                yellow_led.write(1);
                green_led.write(1);
                state_string = "UNKNOWN";
                break;
        }

        if (current_state == STOPPED) {
            // Using NOR gate for logic level shifting from 3.3V to 5V, so need to use inverted value for on/off
            HV_enable.write(1);
        } else {
            HV_enable.write(0);
        }

        if (current_state == STANDBY) {
            desired_angle.store(0);
            manual_drive_speed.store(0);
            autonomous_drive_speed.store(0);
        }

        char outgoing_message[128];
        sprintf(outgoing_message, "Current angle = %f, current speed = %f, current state = %s", current_steering_angle.load(), current_velocity.load(), state_string.c_str());
        rjnet_udp.send_single_message(outgoing_message, jetsonIP);

        last_state = current_state;
        ThisThread::sleep_until(time + 50ms);
    }
}

