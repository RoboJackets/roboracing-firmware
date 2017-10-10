// #include "algorithm"
// #include "stdlib.h"
#include <Servo.h>

const int driverPin = 9;
Servo ppmOutput;
const int potPin = A0;

const int mainLoopMillis = 25;
const int timeoutMillis = 500;
unsigned long lastUpdate;

// const int pwmPeriodMs = 16;
const int pwmPulseMinUs = 1250;
const int pwmPulseMaxUs = 1750;
const int pwmPulseRangeUs = pwmPulseMaxUs - pwmPulseMinUs;

const float potMin = 0.18;
const float potMax = 0.91;

float steerPID_P = 0; //0.7;
float steerPID_I = 0;
float steerPID_D = 0; //2;
const int errorHistorySize = 20;
float errorHistory[errorHistorySize] = {0};
int errorHistoryIndex = 0;
float lastError = 0;
unsigned long lastPIDTime;
float lastPIDCorrection = 0;

float desiredHeading = 0;


float clamp(float x, float minX, float maxX) {
    return min(max(x, minX), maxX);
}

float linearRemap(float x, float x1, float x2, float y1, float y2) {
    float proportion = (x - x1) / (x2 - x1);
    return y1 + proportion * (y2 - y1);
}

void steerPower(float x) {
    x = clamp(x, -0.83, 0.83); // constrain speed to safe levels (multiply by 12 for volts)
    x = linearRemap(x, -1, 1, 0, 1); // remap [-1,1] to [0,1] for pwm logic
    int width = (int)(pwmPulseMinUs + (pwmPulseRangeUs * x));
    ppmOutput.writeMicroseconds(width);
}

float getHeading() {
    float a = (float) analogRead(potPin) / 1023.0;
    float h = linearRemap(a, potMin, potMax, -1, 1);
    return clamp(h, -1, 1);
}

bool getMessage() {
    bool gotMessage = false;
    while(Serial.available()) {
        char first = Serial.read();
        if(first == '$') {
            desiredHeading = Serial.parseFloat();
            steerPID_P = Serial.parseFloat();
            steerPID_I = Serial.parseFloat();
            steerPID_D = Serial.parseFloat();
            gotMessage = true;
        }
    }
    return gotMessage;
}

bool sendMessage() {
    bool sentMessage = false;
    if(Serial.availableForWrite()) {
        String message = "$";
        message.concat(getHeading());
        message.concat(", ");
        message.concat(desiredHeading);
        message.concat(", ");
        message.concat(lastPIDCorrection);
        Serial.println(message);
        sentMessage = true;
    }
    return sendMessage;
}

float getPIDCorrection() {
    float error = desiredHeading - getHeading();

    errorHistoryIndex = (errorHistoryIndex + 1) % errorHistorySize;
    errorHistory[errorHistoryIndex] = error;
    float errorTotal = 0;
    for(int i = 0; i < errorHistorySize; i++) {
        errorTotal += errorHistory[i];
    }

    float dError = error - lastError;
    lastError = error;

    long thisTime = millis();
    float dt = thisTime - lastPIDTime;
    lastPIDTime = thisTime;

    float correction = steerPID_P * error 
                     + steerPID_I * errorTotal 
                     + steerPID_D * dError/dt;
    return clamp(correction, -1.0, 1.0);
}

void setup() {
    Serial.begin(9600);

    pinMode(potPin, INPUT);
    pinMode(driverPin, OUTPUT);
    ppmOutput.attach(driverPin);

    lastUpdate = lastPIDTime = millis();
}

void loop() {
    unsigned long thisTime = millis();

    if(getMessage() && sendMessage()) {
        lastUpdate = thisTime;
    }

    if(thisTime < lastUpdate + timeoutMillis) {
        float output = getPIDCorrection();
        steerPower(output);
        lastPIDCorrection = output;
    } else {
        // timeout
        steerPower(0);
    }

    int msRemain = thisTime + mainLoopMillis - millis();
    if(msRemain > 0) {
        delay(msRemain);
    }
    // serial.printf("wait = %d\r\n", msRemain);
}
