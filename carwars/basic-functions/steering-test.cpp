#include "mbed.h"
#include "rtos.h"
#include "algorithm"
#include "stdlib.h"

PwmOut driverPin(p21);
Serial serial(USBTX, USBRX);
DigitalOut led(LED1);
AnalogIn pot(p20);

const int pwmPeriodMs = 16;
const int pwmPulseMinUs = 1250;
const int pwmPulseMaxUs = 1750;
const int pwmPulseRangeUs = pwmPulseMaxUs - pwmPulseMinUs;

const float potMin = 2;
const float potMax = 3;

const float steerPID_P = 0.1;
const float steerPID_I = 0.002;
const float steerPID_D = 0.1;
const int errorHistorySize = 50;
float errorHistory[errorHistorySize] = {0};
int errorHistoryIndex = 0;
float lastError = 0;
float lastPIDTime;

float desiredHeading = (potMin + potMax) / 2.0;

inline float clamp(float x, float minX, float maxX) {
    return min(max(x, minX), maxX);
}

long timeMillis() {
    return (long)(time(NULL) * 1000);
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
            serial.scanf("%f", &desiredHeading);
            gotMessage = true;
        }
    }
    return gotMessage;
}

float getPIDCorrection() {
    float steerReading = pot.read();
    float error = desiredHeading - steerReading;

    errorHistoryIndex = (errorHistoryIndex + 1) % errorHistorySize;
    errorHistory[errorHistoryIndex] = error;
    float errorTotal = 0;
    for(int i = 0; i < errorHistorySize; i++) {
        errorTotal += errorHistory[i];
    }

    float dError = error - lastError;
    lastError = error;
    long thisTime = timeMillis();
    float dt = thisTime - lastPIDTime;
    lastPIDTime = thisTime;

    float correction = steerPID_P * error 
                     + steerPID_I * errorTotal 
                     + steerPID_D * dError/dt;
    return clamp(correction, -1.0, 1.0);
}

int main() {
    long lastUpdate = lastPIDTime = timeMillis();

    while(true) {
        long thisTime = timeMillis();
        if(getMessage()) {
            led = !led;
            lastUpdate = thisTime;
        } 

        if(thisTime < lastUpdate + 2000) {
            float output = getPIDCorrection();
            steerPower(output);
            serial.printf("pot=%.2f, drive=%.2f\r\n", pot.read(), output);
        } else {
            // timeout
            steerPower(0);
            serial.printf("[timeout] pot=%.2f\r\n", pot.read());
        }

        int count = 0;
        while(timeMillis() < thisTime + 20) {
            Thread::wait(1);
            count++;
        }
        serial.printf("wait count = %d\r\n", count);
    }
}
