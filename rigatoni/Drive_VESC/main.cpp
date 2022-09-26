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


// Blinking rate in milliseconds
#define BLINKING_RATE     500ms


int main()
{
    // Initialise the digital pin LED1 as an output
    DigitalOut led(LED1);

    //Set the Vesc's serial port
    the_vesc.setDebugEnable(true);
    the_vesc.setSerialPort(&vesc_serial);

    while (true) {
        led = !led;
        the_vesc.setRPM(1500);
        the_vesc.getVescValues();
        the_vesc.printVescValues();
        ThisThread::sleep_for(BLINKING_RATE);
    }
}
