// This reads the steering and drive data from the Futaba controller
// in two different threads

#include "mbed.h"
#include "PwmIn.h"

PwmIn steering(D6);
PwmIn drive(D7);
Thread thread;

void drive_thread()
{
    while (true) {
        printf("Drive Pulse Width: %f\n", drive.pulsewidth());
    }
}

// main() runs in its own thread in the OS
int main()
{
    thread.start(drive_thread);

    while (true) {
        printf("Steering Pulse Width: %f\n", steering.pulsewidth());
    }
}

