/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

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

constexpr float kP = 2.8;
constexpr float kI = 0.02;
constexpr float kD = 16;
constexpr float kF = 0.25;
constexpr unsigned int MAX_AMPS = 50;
constexpr float POT_ELECTRICAL_RANGE = 260;
constexpr float POT_OFFSET = 0.76;

constexpr float MAX_ANGLE = 60;

constexpr Kernel::Clock::duration_u32 REFRESH_RATE = 10ms;
constexpr int WATCHDOG_TIMEOUT = 50;

void process_single_message(const SocketAddress & senders_address, const char incoming_udp_message[], unsigned int num_bytes_in_message);

VescUart vesc;
BufferedSerial vesc_serial(TX, RX, BAUD_RATE);
AnalogIn pot(A0);
RJNetMbed rjnet_udp(steeringIP, &process_single_message);

std::atomic<float> desired_angle;

template<typename T>
inline int sgn(T n) {
    return (n > 0) - (n < 0);
}

template<typename T>
inline T abs_max_bound(T val, T range) {
    return sgn(val) * min(abs(val), abs(range));
}

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
            desired_angle.store(abs_max_bound(temp, MAX_ANGLE));
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

inline float sample_pot() {
    float pot_read = 0;
    for (int i = 0; i < 100; ++i) {
        pot_read += pot.read();
    }
    pot_read /= 100;
    return pot_read;
}

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

    desired_angle.store((sample_pot() - POT_OFFSET) * POT_ELECTRICAL_RANGE);

    //Set the Vesc's serial port
    vesc.setDebugEnable(false);
    vesc.setSerialPort(&vesc_serial);

    float pos = (sample_pot() - POT_OFFSET) * POT_ELECTRICAL_RANGE;
    float prevPos = pos;
    float I = 0;
    while (true) {
        auto time = Kernel::Clock::now();
        pos = (sample_pot() - POT_OFFSET) * POT_ELECTRICAL_RANGE;

        float err = (desired_angle - pos);
        I += err;
        I = abs_max_bound(I, 20.0f/kI);
        float command = kP * err + kD * (prevPos - pos) + kI * I + kF * desired_angle;
        command = abs_max_bound<float>(command, MAX_AMPS);

        // potentiometer and motor positive are reversed
        vesc.setCurrent(-command);
        printf("Position: %f Command: %f Desired: %f I: %f\n", pos, command, desired_angle.load(), kI * I);
        prevPos = pos;
        ThisThread::sleep_until(time + REFRESH_RATE);
    }
}
