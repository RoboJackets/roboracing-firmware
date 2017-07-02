#include <LiquidCrystal.h>
#include <Servo.h>

void motor(int val);
void steer(int val);
void update();
int limitDesiredSpeed(int desiredSpeed);
int limitDesiredHeading(int desiredHeading);
void updateHeading();
void updateSpeed();

static int currentSpeed = 0;
static int desiredSpeed=0;
static int currentHeading = 0;
static int desiredHeading = 0;

//control limits
static const int maxSpeed = 30; // maximum velocity 
static const int minSpeed = -15;
static const int minSteer = -25;
static const int maxSteer = 25;

const int estopPin = A1;
boolean estop = true;

Servo esc;
const int escMux = 10;
Servo steering;
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

void update()
{
    if(getMessage()) {
        desiredSpeed = limitDesiredSpeed(desiredSpeed);
        desiredHeading = limitDesiredHeading(desiredHeading); 
    }
    updateHeading();
    updateSpeed();
}

void limitDesiredSpeed(int desiredSpeed) {
    return min(maxSpeed, max(requestedSpeed, minSpeed));
}

void limitDesiredHeading(int desiredHeading) {
    return min(maxHeading, max(requestedHeading, minHeading));    
}

void updateHeading()
{
    if (currentHeading != desiredHeading) {
        currentHeading = desiredHeading;
        steer(currentHeading); 
    }    
}

void updateSpeed(int desiredSpeed)
{
    if (muxState || estop) {
        
    }
}

void steer(int val)
{
    steering.write(val + 90);
}

void motor(int val)
{
    EscMotor.write(val + 90);
}
/*
    - encoder input -> PID -> writing to motor
    - encoder input -> back to Joule
    - read the mux state -> tell PID what do
    - read the E-Stop state -> tell PID what do

*/
