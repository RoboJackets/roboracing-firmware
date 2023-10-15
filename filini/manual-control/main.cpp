#include "mbed.h"
#include "Pulse.h"

PulseInOut steering(D6);
PulseInOut drive(D7);
Thread thread;

void drive_thread()
{
    while (true) {
        printf("Drive Pulse: %d\n", drive.read_us(50000));
        // ThisThread::sleep_for(3ms);
    }
}

// main() runs in its own thread in the OS
int main()
{
    // thread.start(drive_thread);

    while (true) {
        printf("Steering Pulse: %d\n", steering.read_us(50000));
        printf("Drive Pulse: %d\n", drive.read_us(50000));
        // ThisThread::sleep_for(3ms);
    }
}

