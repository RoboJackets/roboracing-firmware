/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include <VescUart.h>

/*
Setup for the Vesc
You cannot use D0, D1 for the serial port. Printf messes with those.
*/
BufferedSerial vesc_serial(D14, D15, 115200);  //tx, rx, baud
VescUart the_vesc;

constexpr unsigned int TICKS_PER_REV = 42 * 48;
constexpr float DEGREE_RATIO = 360.0 / TICKS_PER_REV;
constexpr unsigned int kP = 75;

// Refresh rate in milliseconds
#define REFRESH_RATE     10ms


int sgn(float n) {
    return (n > 0) - (n < 0);
}

int main()
{
    //Set the Vesc's serial port
    the_vesc.setDebugEnable(false);
    the_vesc.setSerialPort(&vesc_serial);
    the_vesc.getVescValues();

    const long TACH_OFFSET = the_vesc.data.tachometer;
    
    float target = 180;
    float pos = ((the_vesc.data.tachometer - TACH_OFFSET) % TICKS_PER_REV + TICKS_PER_REV) % TICKS_PER_REV * DEGREE_RATIO;

    while (true) {
        the_vesc.getVescValues();
        pos = ((the_vesc.data.tachometer - TACH_OFFSET) % TICKS_PER_REV + TICKS_PER_REV) % TICKS_PER_REV * DEGREE_RATIO;
        float command = kP * (target - pos);
        if (fabs(command) > 30000) {
            command = 30000 * sgn(command);
        }
        if (fabs(command) < 1500 && fabs(target - pos) > 1) {
            command = 1500 * sgn(command);
        }
        the_vesc.setRPM(command);
        printf("Position: %f\n", pos);
        ThisThread::sleep_for(REFRESH_RATE);
    }   
}
