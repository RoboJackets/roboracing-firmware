/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#if !DEVICE_FLASH
#error [NOT_SUPPORTED] Flash API not supported for this target.
#endif

#include "mbed.h"
#include "rjnet_mbed_udp.h"
#include "unity/unity.h"
#include "greentea-client/test_env.h"
#include <VescUart.h>
#include <atomic>
#include <FlashIAP.h>

#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS
#endif

#include <inttypes.h>

void process_single_message(const SocketAddress & senders_address, const char incoming_udp_message[], unsigned int num_bytes_in_message);
float getDesiredERPMFromSpeed(float speed);

/*
Setup for the Vesc
You cannot use D0, D1 for the serial port. Printf messes with those.
*/
BufferedSerial vesc_serial(D14, D15, 115200);  //tx, rx, baud
VescUart the_vesc;
RJNetMbed rjnet_udp(driveIP, &process_single_message);
FlashIAP flash;

const static int numPolePairs = 7; // not actually known yet
const static string estopGoMsg = "G";

bool motorEnabled = false;
float nucTargetVelocity = 0;

void process_single_message(const SocketAddress & senders_address, const char incoming_udp_message[], unsigned int num_bytes_in_message) {
    //Parses a UDP message we just recieved. Places any received data in global variables.
    if (rjnet_udp.are_ip_addrs_equal(estopIP, senders_address)) {
        motorEnabled = (estopGoMsg == incoming_udp_message);
    }
    else if(rjnet_udp.are_ip_addrs_equal(nucIP, senders_address)) {
        //Parse angle from message. Doing incoming_message + 2 ignores the first two characters
        if (incoming_udp_message[0] == 'v') {

            printf("Nuc Target Velocity: %f\n", nucTargetVelocity);
            sscanf(incoming_udp_message + 2, "%f", &nucTargetVelocity);
            //Reply to NUC at once
            char outgoing_message [64];
            sprintf(outgoing_message, "Got speed = %f", nucTargetVelocity);
            rjnet_udp.send_single_message(outgoing_message, nucIP);
        } else if (incoming_udp_message[0] == 'R') {
            NVIC_SystemReset();
        } else {
            while (true);
        }
    }
}

// Blinking rate in milliseconds
#define BLINKING_RATE     500ms

void mbed_stress_test_erase_flash(void) {
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

int main() {
    // Initialise the digital pin LED1 as an output
    DigitalOut led(LED1);

    //Set the Vesc's serial port
    the_vesc.setDebugEnable(true);
    the_vesc.setSerialPort(&vesc_serial);

    Thread network_start(osPriorityLow);
    network_start.start([&]() { rjnet_udp.start_network_and_listening_threads(); });

    while (true) {
        led = !led;
        float targetVelocity = getDesiredERPMFromSpeed(nucTargetVelocity);
        the_vesc.setRPM(targetVelocity);
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

