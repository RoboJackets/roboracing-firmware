#include <LiquidCrystal.h>
#include <Servo.h>

const int estopPin = A1;
boolean estop = true;

const int escMux = 10;
const int steerMux = 11;

const int encoderA = 3;
const int encoderB = 4;

const int muxStatePin = A0;
boolean muxState = 0;

volatile int tickData = 0;

void setup()
{
    pinMode(estop, INPUT);

    pinMode(escMux, OUTPUT);
    pinMode(steerMux, OUTPUT);

    pinMode(encoderA, INPUT);
    attachInterrupt(encoderA, tick(), CHANGE);
    pinMode(encoderB, INPUT);

    pinMode(muxState, INPUT);

    digitalWrite(escMux, 0);
    digitalWrite(steerMux, 0);

}

void loop()
{
    muxState = digitalRead(muxStatePin);
    estop = digitalRead(estopPin);
    if (estop) {
        digitalWrite(escMux, 0;
        digitalWrite(steerMux, 0;
    } 
}
    
