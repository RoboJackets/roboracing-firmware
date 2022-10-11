/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cstdint>
#include <mbed.h>

PwmOut pwm_servo(D5);
const uint_fast16_t SERVO_US_MIN = 1000;
const uint_fast16_t SERVO_US_MAX = 2000;

int main() {

    // enable servo and set position to -90 degrees
    pwm_servo.period_ms(20);

    while(1) {
      // Sweep across whole servo range from -90 degrees to +90 degrees
      for (int i = 0; i <= 10; i++) {
        pwm_servo.pulsewidth_us(SERVO_US_MIN + (SERVO_US_MAX - SERVO_US_MIN)*i/10);
        wait_us(10000);
      }

      // Sweep back from 90 degrees to -90 degrees
      for (int i = 10; i >= 0; i--) {
        //servo.write(i);
        pwm_servo.pulsewidth_us(SERVO_US_MIN + (SERVO_US_MAX - SERVO_US_MIN)*i/10);
        wait_us(10000);
      }

      wait_us(1000000);
    }
  }
