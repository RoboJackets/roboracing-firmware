/**
 * VESC CAN Documentation: https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md
 * Note: CAN communication with the current drive motor controller on Rigatoni (FSESC 75/300) has been unable to
 * work, with a possible reason being that the FSESC's firmware may not recognize duty cycle inputs over CAN.
 * Another reason why this may be the case is that duty cycle over UART does not work with the FSESC either,
 * so the code below uses ERPM control for drive.
 *
 * 5 m/s for drive correponds to about 5000 ERPM or 1000 RPM
 *
 * Useful reference for Velocity PID: https://deltamotion.com/support/webhelp/rmctools/Controller_Features/Control_Modes/Velocity_PID.htm
 */
#include "mbed.h"
#include "PwmIn.h"
#include <atomic>
#include <arm_acle.h>
#include <VescUart.h>

#define ETHERNET_ID 3
#define DRIVE_VESC_ID 9
#define STEERING_VESC_ID 10

constexpr int BAUD_RATE = 115200;
constexpr PinName TX = D14;
constexpr PinName RX = D15;

constexpr bool ON_GROUND = true;

VescUart vesc;
BufferedSerial vesc_serial(TX, RX, BAUD_RATE);

// Using current control for steering
// constexpr float K_u = 0.1007125;
// constexpr float T_u = 1.47;
// constexpr float K_p = 0.6 * K_u;
// constexpr float K_i = 0.4 * K_u / T_u;
// constexpr float K_d = 0.4 * K_u * T_u;

// Using duty cycle control for steering off the ground
constexpr float K_u_duty = 0.7957013448;
constexpr float T_u_duty = 0.46;
constexpr float K_p_duty = 0.5 * K_u_duty;
constexpr float K_i_duty = 1.2 * K_u_duty / T_u_duty;
constexpr float K_d_duty = 0.075 * K_u_duty * T_u_duty;

// Duty cycle control for on the ground
// ong means on the ground, nothing means off the ground
// 2.39 is the previous known working constant when the steering motor wasn't loose
// constexpr float K_u_duty_ong = 2.39;
constexpr float K_u_duty_ong = 1.195;
constexpr float T_u_duty_ong = 0.34;
constexpr float K_p_duty_ong = 0.6 * K_u_duty_ong;
constexpr float K_i_duty_ong = 1.2 * K_u_duty_ong / T_u_duty_ong;
constexpr float K_d_duty_ong = 0.075 * K_u_duty_ong * T_u_duty_ong;
constexpr float K_f_duty_ong = 0.04;

constexpr float MIN_DUTY_CMD = 0.015;
constexpr float MIN_DUTY_CMD_ONG = 0.02;
// To prevent integral windup
constexpr float MAX_INTEGRAL_TERM_ONG = 0.01;
constexpr float MAX_INTEGRAL_TERM = 0.001;

constexpr float kP = 1.85;
constexpr float kI = 0.04;
constexpr float kD = 12;
constexpr float kF = 0.3;
constexpr unsigned int MAX_COMMAND = 50;
constexpr float POT_ELECTRICAL_RANGE = 260;
constexpr float POT_OFFSET = 0.50;
constexpr float MAX_ANGLE = 55;
constexpr float MAX_VELOCITY = 5.0;

constexpr float MAX_STEERING_DUTY_CYCLE = 0.5;

constexpr int STRAIGHT_PULSEWIDTH = 1540;
constexpr float PULSEWIDTH_TO_DEGREES = MAX_ANGLE/460.0;

constexpr int ZERO_SPEED_PULSEWIDTH = 1540;
constexpr float MAX_MANUAL_SPEED = 3.65; // units of m/s
constexpr float PULSEWIDTH_TO_MPS = MAX_MANUAL_SPEED/460.0;

constexpr float RPM_TO_VEL = 0.834f/60.f;
constexpr float GEAR_RATIO = 62.f/22.f;
constexpr float ERPM_TO_RPM = 0.2f;
constexpr float DUTY_CYCLE_PER_MPS = 0.05476f;


constexpr Kernel::Clock::duration_u32 REFRESH_RATE = 10ms;

CAN can(PD_0, PD_1);
AnalogIn pot(A0);
// DigitalOut HV_enable(D7);
PwmIn steering_RC(D7);
PwmIn drive_RC(D6);

uint64_t steering_counter = 0;
uint64_t drive_counter = 0;
bool writing_steering_messages = false;
bool writing_drive_messages = false;

std::atomic<float> desired_angle;
std::atomic<float> manual_steering;

std::atomic<float> manual_drive;

Thread send_steering_thread;
Thread manual_steering_thread;
Thread manual_drive_thread;


template<typename T>
inline int sgn(T n) {
    return (n > 0) - (n < 0);
}

template<typename T>
inline T abs_max_bound(T val, T range) {
    return sgn(val) * min(abs(val), abs(range));
}

template<typename T>
inline T abs_min_bound(T val, T range) {
    return sgn(val) * max(abs(val), abs(range));
}

template<typename T>
inline T erpm_to_vel(T val) {
    return val * RPM_TO_VEL / GEAR_RATIO * ERPM_TO_RPM;
}

template<typename T>
inline T vel_to_erpm(T val) {
    return val / RPM_TO_VEL * GEAR_RATIO / ERPM_TO_RPM;
}


void can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
    printf("ID: %d\n", id);
    if ((id & 0xff) == STEERING_VESC_ID) {
        if (can.write(CANMessage(id, data, len, CANData, CANExtended))) {
            printf("Steering message sent: %llu\n", steering_counter);
            steering_counter++;
            writing_steering_messages = true;
        } else {
            printf("Err\n");
            writing_steering_messages = false;
        }
    }
}

void comm_can_set_duty(uint8_t controller_id, float duty) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
	can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

void comm_can_set_current(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
	can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

void comm_can_set_rpm(uint8_t controller_id, float rpm) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)rpm, &send_index);
	can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

inline float sample_pot() {
    float pot_read = 0;
    for (int i = 0; i < 100; ++i) {
        pot_read += pot.read();
    }
    pot_read /= 100;
    return pot_read;
}

void read_manual_steering()
{
    while (true) {
        float pulse = steering_RC.pulsewidth();
        if (pulse == 0)
        {
            desired_angle.store(manual_steering.load());
        }
        else if (pulse >= 1000.0f && pulse <= 2080.0f)
        {
            float target_angle = abs_max_bound((pulse - STRAIGHT_PULSEWIDTH) * PULSEWIDTH_TO_DEGREES, MAX_ANGLE);
            desired_angle.store(target_angle);
            manual_steering.store(target_angle);
        }
        printf("Steering pulse width: %f\n", pulse);
        ThisThread::sleep_for(50ms);
    }
}

void read_manual_drive()
{
    while (true) {
        float pulse = drive_RC.pulsewidth();
        if (pulse >= 1000.0f && pulse <= 2080.0f) {
            float target_speed = abs_max_bound((pulse - ZERO_SPEED_PULSEWIDTH) * PULSEWIDTH_TO_MPS, MAX_MANUAL_SPEED);
            manual_drive.store(target_speed);
        }
        printf("Drive pulse width: %f\n", pulse);
        ThisThread::sleep_for(50ms);
    }
}

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
        I += err;
        I = abs_max_bound(I, MAX_INTEGRAL_TERM_ONG * MAX_COMMAND / K_i_duty);
        // I = abs_max_bound(I, MAX_INTEGRAL_TERM * MAX_COMMAND / K_i_duty_ong);
        float command;
        if (ON_GROUND) {
            command = K_p_duty_ong * err + K_d_duty_ong * (prevPos - pos) + K_i_duty_ong * I + K_f_duty_ong * desired_angle;
        } else {
            command = K_p_duty * err + K_d_duty * (prevPos - pos) + K_i_duty * I;
        }

        // Used for determining K_u, the ultimate gain where the wheels oscillate at consistent angles
        // This value should be the smallest possible gain that causes the wheels to never reach the desired angle
        // float command = K_u_duty_ong * err;
        command = abs_max_bound<float>(command, MAX_COMMAND);
        command = command / MAX_COMMAND * MAX_STEERING_DUTY_CYCLE;

        // On the ground code:
        if (ON_GROUND) {
            if (abs(desired_angle) > 45)
            {
                command = abs_min_bound<float>(command, MIN_DUTY_CMD_ONG);
            }
            else if (abs(desired_angle) > 25)
            {
                // Old error threshold when steering motor wasn't loose
                // if (abs(err) < 0.125)
                if (abs(err) < 5)
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
            // 1 degree of error seems to work for a desired angle of 55 or -55 degrees (only for off the ground)
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
        prevPos = pos;
        if (can.read(msg)) {
            read_message = true;
            receiving_messages = true;
            // ID for EPRM, Current, and duty cycle is 9
            if (msg.id >> 8 == 0x9) {
                uint32_t* rpm = (uint32_t*)msg.data;
                uint16_t* current = (uint16_t* )(msg.data + 4);

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
                printf("Steering - ID: %u, ERPM: %d, Current: %f, Duty Cycle %f\n", msg.id, erpm, current_f * 0.1, duty_cycle * 0.001);
            } else {
                printf("Steering - ID: %u\n", msg.id);
            }
        } else {
            receiving_messages = false;
        }
        if (steering_counter > 0 && !writing_steering_messages) {
            can.reset();
        }
        ThisThread::sleep_until(time + REFRESH_RATE);
    }
}

void send_drive_command() {
    while (true) {
        auto time = Kernel::Clock::now();
        float target_speed = manual_drive.load();
        float command = abs_max_bound(target_speed, MAX_MANUAL_SPEED);
        command = vel_to_erpm(target_speed);
        printf("Target: %f, Command: %f\n", target_speed, command);
        vesc.setRPM(command);
        ThisThread::sleep_until(time + REFRESH_RATE);
    }
}

int main()
{
    printf("Reset\n");
    // HV_enable.write(1);
    can.frequency(500000);

    vesc.setDebugEnable(false);
    vesc.setSerialPort(&vesc_serial);
    

    desired_angle.store((sample_pot() - POT_OFFSET) * POT_ELECTRICAL_RANGE);

    manual_steering_thread.start(read_manual_steering);
    manual_drive_thread.start(read_manual_drive);

    send_steering_thread.start(send_steering_command);
    send_drive_command();
}

