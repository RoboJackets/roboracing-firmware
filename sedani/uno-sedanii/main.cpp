/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cstdint>
#include <mbed.h>

PwmOut pwm_servo(A3);
const uint_fast16_t SERVO_US_MIN = 1000;
const uint_fast16_t SERVO_US_MAX = 2000;
const uint_fast16_t SERVO_PULSE_WIDTH_US = 20e3;

void write_servo(float fraction_full_scale){
    //Initial SERVO_PULSE_WIDTH_US is to deal with inverting it
    uint_fast16_t pulsewidth = SERVO_US_MIN + (SERVO_US_MAX - SERVO_US_MIN)*fraction_full_scale;
    pwm_servo.pulsewidth_us(SERVO_PULSE_WIDTH_US-pulsewidth);
    printf("pwm is: %d\n", pulsewidth);
}

int main() {

    // enable servo and set position to -90 degrees
    pwm_servo.period_us(SERVO_PULSE_WIDTH_US);

    const float max_frac = 0.75;
    const float min_frac = 1-max_frac;
    const float step = 0.05;

    while(1) {
        // Sweep across whole servo range from -90 degrees to +90 degrees
        for (float i = 0.45; i <= 0.55 + 1e-3; i+=step) {
            write_servo(i);
            ThisThread::sleep_for(2s);
        }


        write_servo(0.5);

        ThisThread::sleep_for(5s);

        // Sweep back from 90 degrees to -90 degrees
        /*
        for (float i = max_frac; i >= min_frac - 1e-3; i-=step) {
            write_servo(i);
            ThisThread::sleep_for(500ms);
        }
        ThisThread::sleep_for(1s);
        */
    }
}