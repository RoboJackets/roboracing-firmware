#include "mbed.h"
#include "Pulse.h"

PulseInOut steering(D6);

// main() runs in its own thread in the OS
int main()
{
    while (true) {
        printf("Pulse: %d\n", steering.read_us(500000));
    }
}

