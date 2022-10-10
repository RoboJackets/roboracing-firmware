/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#if !DEVICE_FLASH
#error [NOT_SUPPORTED] Flash API not supported for this target.
#endif

#include "mbed.h"
#include "unity/unity.h"
#include "greentea-client/test_env.h"
#include <VescUart.h>
#include <FlashIAP.h>

#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS
#endif

#include <inttypes.h>

/*
Setup for the Vesc
You cannot use D0, D1 for the serial port. Printf messes with those.
*/
BufferedSerial vesc_serial(D14, D15, 115200);  //tx, rx, baud
VescUart the_vesc;
FlashIAP flash;
int numPolePairs;


// Blinking rate in milliseconds
#define BLINKING_RATE     500ms

void mbed_stress_test_erase_flash(void)
{
    int result;

    printf("Initialzie FlashIAP\r\n");

    result = flash.init();
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, result, "failed to initialize FlashIAP");

    uint32_t flash_start = flash.get_flash_start();
    printf("start address: %" PRIX32 "\r\n", flash_start);
    TEST_ASSERT_MESSAGE(MBED_FLASH_INVALID_SIZE != flash_start, "invalid start address");

    uint32_t flash_size = flash.get_flash_size();
    printf("flash size: %" PRIX32 "\r\n", flash_size);
    TEST_ASSERT_MESSAGE(MBED_FLASH_INVALID_SIZE != flash_size, "invalid flash size");

    uint32_t page_size = flash.get_page_size();
    printf("page size: %" PRIu32 "\r\n", page_size);
    TEST_ASSERT_MESSAGE(MBED_FLASH_INVALID_SIZE != page_size, "invalid page size");

    uint32_t last_sector_size = flash.get_sector_size(flash_start + flash_size - 1);
    printf("last sector size: %" PRIu32 "\r\n", last_sector_size);
    TEST_ASSERT_MESSAGE(MBED_FLASH_INVALID_SIZE != last_sector_size, "invalid sector size");
/*
    uint32_t start_address = flash_start + MBED_CONF_APP_ESTIMATED_APPLICATION_SIZE;
    uint32_t erase_size = flash_size - MBED_CONF_APP_ESTIMATED_APPLICATION_SIZE;
    printf("Erase flash: %" PRIX32 " %" PRIX32 "\r\n", start_address, erase_size);

    result = flash.erase(start_address, erase_size);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, result, "failed to erase flash");*/
}

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

/*
Diameter of rear wheel is about 27 cm (not confirmed)
3:1 gear ratio from rear axle to drive motor shaft
speed coming in from software has units of m/s
circumference of rear wheel is PI * 27 cm
m/s * (60 s)/(1 min) * (1 axle revolution)/(PI * .27 m) * (3 motor shaft revolutions)/(1 axle revolution) * (Num of pole pairs)/(1 shaft)
*/
float getDesiredERPMFromSpeed(float speed) {
    return speed * 60 / (3.14159 * .27) * 3 * numPolePairs; // numPolePairs not known yet
}

