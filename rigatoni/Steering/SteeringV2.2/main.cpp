#include "mbed.h"
#include "ODriveMbed.h"

BufferedSerial odrive_serial(D1, D0, 115200);

ODriveMbed odrive(odrive_serial);

// main() runs in its own thread in the OS
int main()
{
    printf("ODriveMbed RoboRacing:\n");
    printf("Setting Parameters...\n");
    printf("Ready!\n");
    printf("Send the character '0' or '1' to calibrate respective motor (you must do this before you can command movement)\n");
    printf("Send the character 'x' to enter closed loop control mode\n");
    printf("Send the character 's' to exectue test move\n");
    printf("Send the character 'b' to read bus voltage\n");
    while (true) {
        
    }
}

