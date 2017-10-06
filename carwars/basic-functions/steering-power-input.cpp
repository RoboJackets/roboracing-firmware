#include "mbed.h"
#include "rtos.h"
#include "algorithm"
#include "stdlib.h"

PwmOut driverPin(p21);
Serial serial(USBTX, USBRX);
DigitalOut led(LED1);
AnalogIn pot(p20);
Timer timer;

const int mainLoopMillis = 20;

const int pwmPeriodMs = 16;
const int pwmPulseMinUs = 1250;
const int pwmPulseMaxUs = 1750;
const int pwmPulseRangeUs = pwmPulseMaxUs - pwmPulseMinUs;

float desiredPower;


inline float clamp(float x, float minX, float maxX) {
    return min(max(x, minX), maxX);
}

unsigned int timeMillis() {
    return (unsigned int)(timer.read() * 1000.0);
}

void steerPower(float x) {
    x = clamp(x, -1.0, 1.0); // -1 <= x <= 1
    x = (x + 1.0) / 2.0; // convert to 0 <= x <= 1
    int width = (int)(pwmPulseMinUs + (pwmPulseRangeUs * x));
    driverPin.pulsewidth_us(width);
}

bool getMessage() {
    bool gotMessage = false;
    if (serial.readable()) {
        char first = serial.getc();
        if (first == '$') {
            serial.scanf("%f", &desiredPower);
            gotMessage = true;
        }
    }
    return gotMessage;
}


int main() {
    timer.start();
    steerPower(0);
    unsigned int lastUpdate = timeMillis();

    Thread::wait(1500);

    while(true) {
        unsigned int thisTime = timeMillis();

        bool received = getMessage();
        if(received) {
            led = !led;
            lastUpdate = thisTime;
        }

        if(thisTime < lastUpdate + 1000) {
            steerPower(desiredPower);
            if (received) {
                serial.printf("$pot=%.2f, drive=%.2f\r\n", pot.read(), desiredPower);
            }
        } else {
            // timeout
            steerPower(0);
            if (received) {
                serial.printf("$[timeout] pot=%.2f\r\n", pot.read());
            }
        }

        int msRemain = thisTime + mainLoopMillis - (int)timeMillis();
        if(msRemain > 0) {
            Thread::wait(msRemain);
        }
        // serial.printf("wait = %d\r\n", msRemain);
    }
}
