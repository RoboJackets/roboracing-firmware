/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "rjnet_mbed_udp.h"
#include <VescUart.h>
#include <atomic>

/*
Setup for the Vesc
You cannot use D0, D1 for the serial port. Printf messes with those.
*/
constexpr int BAUD_RATE = 115200;
constexpr PinName TX = D14;
constexpr PinName RX = D15;

constexpr unsigned int TICKS_PER_REV = 42 * 48;
constexpr float DEGREE_RATIO = 360.0 / TICKS_PER_REV;
constexpr unsigned int kP = 150;
constexpr unsigned int MAX_RPM = 30000;
constexpr unsigned int MIN_RPM = 1500;

constexpr Kernel::Clock::duration_u32 REFRESH_RATE = 10ms;
constexpr int WATCHDOG_TIMEOUT = 50;

void process_single_message(const SocketAddress & senders_address, const char incoming_udp_message[], unsigned int num_bytes_in_message);

VescUart vesc;
BufferedSerial vesc_serial(TX, RX, BAUD_RATE);
RJNetMbed rjnet_udp(steeringIP, &process_single_message);

std::atomic<float> desired_angle;

void process_single_message(const SocketAddress & senders_address, const char incoming_udp_message[], unsigned int num_bytes_in_message) {
    //Parses a UDP message we just recieved. Places any received data in global variables.
    if(rjnet_udp.are_ip_addrs_equal(nucIP, senders_address)) {
        //Parse angle from message. Doing incoming_message + 2 ignores the first two characters
        if (incoming_udp_message[0] == 'A') {
            float temp;
            sscanf(incoming_udp_message + 2, "%f", &temp);
            //Reply to NUC at once
            char outgoing_message [64];
            sprintf(outgoing_message, "Got angle = %f", temp);
            rjnet_udp.send_single_message(outgoing_message, nucIP);
            desired_angle.store(temp);
        } else if (incoming_udp_message[0] == 'R') {
            NVIC_SystemReset();
        }
    }
}

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

void watchdog_kick_thread() {
    while (true) {
        Watchdog::get_instance().kick();
        ThisThread::sleep_for(REFRESH_RATE);
    }
}

int main() {
    DigitalOut led1(LED1);
    DigitalOut led2(LED2);
    AnalogIn pot(A0);

    if (ResetReason::get() == RESET_REASON_WATCHDOG) {
        led1 = false;
        led2 = true;
    } else {
        led1 = true;
        led2 = false;
    }

    rjnet_udp.start_network_and_listening_threads();
    
    Watchdog::get_instance().start(WATCHDOG_TIMEOUT);
    Thread watchdog_thread(osPriorityLow);
    watchdog_thread.start(&watchdog_kick_thread);

    //Set the Vesc's serial port
    vesc.setDebugEnable(false);
    vesc.setSerialPort(&vesc_serial);
    vesc.getVescValues();

    const long TACH_OFFSET = vesc.data.tachometer;
    float pos = ((vesc.data.tachometer - TACH_OFFSET) % TICKS_PER_REV + TICKS_PER_REV) % TICKS_PER_REV * DEGREE_RATIO;

    while (true) {
        vesc.getVescValues();
        pos = ((vesc.data.tachometer - TACH_OFFSET) % TICKS_PER_REV + TICKS_PER_REV) % TICKS_PER_REV * DEGREE_RATIO;
        float command = kP * (desired_angle - pos);
        command = abs_max_bound<float>(command, MAX_RPM);
        if (abs(desired_angle - pos) > 1) command = abs_min_bound<float>(command, MIN_RPM);
        vesc.setRPM(command);
        printf("Position: %f\n", pos);
        ThisThread::sleep_for(REFRESH_RATE);
    }
}
