#include "mbed.h"
#include "rtos.h"
#include "algorithm"
#include "stdlib.h"

PwmOut driverPinA(p21);
PwmOut driverPinB(p22);
Serial serial(USBTX, USBRX);
DigitalOut led(LED1);
Timer timer;

const int PERIOD = 1000; //us
const int mainLoopMillis = 50;

float desiredPower;

inline float clamp(float x, float minX, float maxX) {
    return min(max(x, minX), maxX);
}

inline unsigned int timeMillis() {
    return (unsigned int)(timer.read() * 1000.0);
}

void drivePower(float x) {
    float dutyCycle = clamp(x, -1.0, 1.0); // -1 <= x <= 1
    if (x >= 0) {
        driverPinA.write(dutyCycle);
        driverPinB.write(0);
    } else {
        driverPinA.write(0);
        driverPinB.write(dutyCycle);
    }
}

bool getMessage() {
    bool gotMessage = false;
    if (serial.readable()) {
        char first = serial.getc();
        if (first == '$') {
            float tmp;
            serial.scanf("%f,%f,%f,%f", &desiredPower, &tmp,&tmp,&tmp);
            gotMessage = true;
        }
    }
    return gotMessage;
}


int main() 
{
    driverPinA.period_us(PERIOD);
    driverPinB.period_us(PERIOD);

    timer.start();
    drivePower(0);
    unsigned int lastUpdate = timeMillis();

    Thread::wait(1500);

    while (true) 
    {
        unsigned int thisTime = timeMillis();

        bool received = getMessage();
        if(received) {
            led = !led;
            lastUpdate = thisTime;
        }

        if(thisTime < lastUpdate + 1000) {
            drivePower(desiredPower);
            // if (received) {
                serial.printf("$drive=%.2f\r\n", desiredPower);
            // }
        } else {
            // timeout
            drivePower(0);
            // if (received) {
                serial.printf("$[timeout]\r\n");
            // }
        }

        int msRemain = thisTime + mainLoopMillis - timeMillis();
        if(msRemain > 0) {
            Thread::wait(msRemain);
        }
        // serial.printf("wait = %d\r\n", msRemain);
    }
}
