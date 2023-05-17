#include "mbed.h"
#include "rjnet_mbed_udp.h"
#include <VescUart.h>
#include <atomic>
#include <cstdint>

/*
Setup for the Vesc
You cannot use D0, D1 for the serial port. Printf messes with those.
*/
constexpr int BAUD_RATE = 115200;
constexpr PinName TX = D14;
constexpr PinName RX = D15;

constexpr float MAX_RPM = 7000;
constexpr float MIN_RPM = 1500;
constexpr float RPM_TO_SI = 0.85f/60.f;
constexpr float GEAR_RATIO = 62.f/22.f;
constexpr float ERPM_TO_RPM = 0.2f;

constexpr unsigned int MAX_AMPS = 300;
constexpr float kP = 0.02;
constexpr float kI = 0.001;
constexpr float kD = 0.2;
constexpr float kF = 2.5;

constexpr Kernel::Clock::duration_u32 REFRESH_RATE = 10ms;
constexpr int WATCHDOG_TIMEOUT = 50;

void process_single_message(const SocketAddress & senders_address, const char incoming_udp_message[], unsigned int num_bytes_in_message);

VescUart vesc;
BufferedSerial vesc_serial(TX, RX, BAUD_RATE);
RJNetMbed rjnet_udp(driveIP, &process_single_message);

std::atomic<float> desired_rpm;

template<typename T>
inline int sgn(T n) {
    return (n > 0) - (n < 0);
}

template<typename T>
inline T abs_max_bound(T val, T range) {
    return sgn(val) * min(abs(val), abs(range));
}

template<typename T>
inline T si_to_erpm(T val) {
    return val / RPM_TO_SI * GEAR_RATIO / ERPM_TO_RPM;
}

template<typename T>
inline T erpm_to_si(T val) {
    return val * RPM_TO_SI / GEAR_RATIO * ERPM_TO_RPM;
}

void process_single_message(const SocketAddress & senders_address, const char incoming_udp_message[], unsigned int num_bytes_in_message) {
    //Parses a UDP message we just recieved. Places any received data in global variables.
    if(rjnet_udp.are_ip_addrs_equal(nucIP, senders_address)) {
        //Parse angle from message. Doing incoming_message + 2 ignores the first two characters
        if (incoming_udp_message[0] == 'V') {
            float temp;
            sscanf(incoming_udp_message + 2, "%f", &temp);
            //Reply to NUC at once
            char outgoing_message [64];
            sprintf(outgoing_message, "Got speed = %f", temp);
            rjnet_udp.send_single_message(outgoing_message, nucIP);
            temp = si_to_erpm(temp);
            if (abs(temp) > 100 && abs(temp) <= MIN_RPM) {
                temp = MIN_RPM * sgn(temp);
            }
            desired_rpm.store(abs_max_bound(temp, MAX_RPM));
        } else if (incoming_udp_message[0] == 'R') {
            NVIC_SystemReset();
        }
    }
}

void watchdog_kick_thread() {
    while (true) {
        Watchdog::get_instance().kick();
        ThisThread::sleep_for(REFRESH_RATE);
    }
}


/* 
 * Useful reference for Velocity PID: https://deltamotion.com/support/webhelp/rmctools/Controller_Features/Control_Modes/Velocity_PID.htm
 */
int main() {
    Watchdog::get_instance().start(WATCHDOG_TIMEOUT);
    Thread watchdog_thread(osPriorityLow);
    watchdog_thread.start(&watchdog_kick_thread);

    DigitalOut led1(LED1);
    DigitalOut led2(LED2);

    if (ResetReason::get() == RESET_REASON_WATCHDOG) {
        led1 = false;
        led2 = true;
    } else {
        led1 = true;
        led2 = false;
    }

    Thread network_start(osPriorityLow);
    network_start.start([&]() { rjnet_udp.start_network_and_listening_threads(); });

    //Set the Vesc's serial port
    vesc.setDebugEnable(false);
    vesc.setSerialPort(&vesc_serial);

    desired_rpm = 0.0;
    double I = 0;
    double prev_err = 0;

    while (true) {
        auto time = Kernel::Clock::now();
        vesc.setRPM(desired_rpm);

        // vesc.getVescValues();
        // auto vel = vesc.data.rpm;

        // if (abs(desired_rpm) > 0 && abs(vel) < MAX_RPM) {
        //     vesc.setCurrent(300 * sgn(desired_rpm.load()));
        // } else {
        //     vesc.setCurrent(300 * -sgn(vel));
        // }

        // vesc.getVescValues();
        // auto vel = vesc.data.rpm;

        // char outgoing_message [64];
        // sprintf(outgoing_message, "vel = %f", erpm_to_si(vel));
        // rjnet_udp.send_single_message(outgoing_message, nucIP);

        // auto err = desired_rpm - vel;
        // I += err;
        // I = abs_max_bound(I, 10.0/kI);
        // auto command = kP * err +  kI * I + kF * sgn(desired_rpm.load()) + kD * (err - prev_err);

        // if (abs(vel) < MIN_RPM && abs(desired_rpm) > 0) {
        //     command = 300 * sgn(desired_rpm.load());
        // }

        // command = abs_max_bound<double>(command, MAX_AMPS);

        // vesc.setCurrent(command);

        // prev_err = err;
        ThisThread::sleep_until(time + REFRESH_RATE);
    }
}
