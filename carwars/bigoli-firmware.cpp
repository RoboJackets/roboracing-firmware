#include "mbed.h"
#include "QEI.h"
#include "rtos.h"
#include "algorithm"
#include "string"
#include "stdlib.h"

static volatile int currentTicks = 0;

static float maxSpeed = 430;
static float minSpeed = -90;
static float maxHeading = 0.35;
static float minHeading = -0.35;

static float pid_p = 1;
static float pid_i = 0.0;
static float pid_d = 0.1;

static int trim = 0;

static volatile float currentSpeed = 0;
static volatile float currentHeading = 0;
static float desiredHeading = 0;
static float desiredSpeed = 0;
static float currentMotorPWM = 0;

const int HISTORY_SIZE = 100;
static float         lastError = 0;
static unsigned long lastSpeedUpdateMicros = 0;
static unsigned long lastPIDUpdateMicros = 0;
static int           historyIndex = 0;

//stop conditions for the car
static long lastMessageTime;
static bool timeout = false;

const int estopPin = 10;
bool estop = false;

//autonomous or human control
//const int muxStatePin = A5;
bool muxAuton = false;

bool getMessage();
/*void updateOutputs();
void measureCurrentSpeed();
void stopCar();
void doPid();*/

Serial serial(USBTX, USBRX);
//QEI wheel (p23, p24, NC, 624);
//PwmOut drive(p9);
//PwmOut steering(p7);
DigitalOut led(LED1);

int main() {
    //drive.write(0);
    //steering.write(0);
    lastMessageTime = (static_cast<long> (time(NULL)) * 1000L);
    while (true) {
        if(getMessage()) {
            //measureCurrentSpeed();
            //updateOutputs();
            led = !led;
        }

        Thread::wait(50);
    }
}

bool getMessage() {
    bool gotMessage = false;
    if (serial.readable()) {
        char first = serial.getc();
        if (first == '$') {
            lastMessageTime = (static_cast<long> (time(NULL)) * 1000L);
            timeout = false;
            serial.scanf("%f, %f", &desiredSpeed, &desiredHeading);
            gotMessage = true;
            serial.printf("$%f,%d,%d\r\n", currentSpeed, muxAuton, estop);
        }
        else if (first == '#') {
            // serial.printf("found");
            // #1.0, 2.0, 3.0, 4, 5.0, 6.0, 7.0, 8.0
            serial.scanf("%f, %f, %f, %d, %f, %f, %f, %f", 
                &pid_p, &pid_i, &pid_d, &trim,
                &maxSpeed, &minSpeed, &maxHeading, &minHeading);
            gotMessage = true;
            // serial.printf("applied %f, %f, %f, %d, %f, %f, %f, %f\n\r", 
            //     pid_p, pid_i, pid_d, trim,
            //     maxSpeed, minSpeed, maxHeading, minHeading);
        }
    }
    
    return gotMessage;
}

/*void updateOutputs() {
    desiredHeading = std::min(maxHeading, std::max(desiredHeading, minHeading));
    desiredSpeed = std::min(maxSpeed, std::max(desiredSpeed, minSpeed));
    if (!muxAuton || estop || timeout) {
        stopCar();
    } else {
        doPid();
    }
    if (currentHeading != desiredHeading) {
        currentHeading = desiredHeading;
        int pwm = ((currentHeading - minHeading) * 142.857) - 45;
        steering.write(pwm);   
    }
}

void stopCar() {
    drive.write(0);
    steering.write(0);
    desiredSpeed = 0;
    currentSpeed = 0;
    desiredHeading = 0;
    currentSpeed = 0;   
}

void doPid() {
    float error = desiredSpeed - currentSpeed;
    float dError = error - lastError;
    lastError = error;
    float deltaPWM = (pid_p * error) + (pid_d * dError);
    deltaPWM = std::max(std::min(deltaPWM, 2.f), -2.f);
    currentMotorPWM += deltaPWM;
    drive.write(currentMotorPWM);
}

void measureCurrentSpeed() {
    currentTicks = wheel.getPulses();
    long time = SystemCoreClock;   
    long deltaTime = (long)(time - lastSpeedUpdateMicros) / 50000;
    lastSpeedUpdateMicros = time;
    currentSpeed = currentTicks / deltaTime;
    currentTicks = 0;
}*/
