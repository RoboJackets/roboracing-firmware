#include "mbed.h"
#include "EthernetInterface.h"
#include "rtos.h"  // sleep
#include "QEI.h"
#include "PID.h"

#include <stdio.h>
#include <string>
#include <cstdlib>
#include <cmath>

struct PIDConstants {
  float p;
  float i;
  float d;
};

const unsigned char ECHO_SERVER_PORT = 7;
const char* const MBED_IP = "192.168.2.2";

const int drivePeriod = 1000; //us
const int mainLoopMillis = 50;
const int steerPeriodMs = 16;
const int steerPulseMinUs = 1250;
const int steerPulseMaxUs = 1750;
const int steerPulseRangeUs = steerPulseMaxUs - steerPulseMinUs;

const float potMin = 0.125;
const float potMax = 0.825;

//Encoder Constants
const float wheelCircumference = 0.246 * 3.14159; //meters
const float ticksPerRot = 8637; // TODO update
const float metersPerTick = wheelCircumference / ticksPerRot;

PIDConstants accelDrivePID, decelDrivePID, steerPID;

PID accelDriveController(0.0,0.0,0.0,1.0);
PID decelDriveController(0.0,0.0,0.0,1.0);
PID steerController(0.0,0.0,0.0,1.0);


float desiredSpeed = 0;
float actualSpeed = 0;
float dutyCycle = 0;

float accelLastRunTime = 0;
float decelLastRunTime = 0;
float steerLastRunTime = 0;

uint32_t encoderLastTime = 0;
uint32_t encoderCurrentTime = 0;

// Pinouts
AnalogIn pot(p15);
PwmOut driverPinMotorA(p21);
PwmOut driverPinMotorB(p22);
PwmOut driverPinSteer(p23);
const PinName encoderChannelB = p24;
const PinName encoderChannelA = p25;

QEI driveAxel (encoderChannelA,encoderChannelB, NC, -1);
Serial serial(USBTX, USBRX);
Timer timer;

void steerPower(float);

// Timing

unsigned int timeMillis() {
  return (unsigned int)(timer.read() * 1000.0);
}

// General Compute

inline float clamp(float value, float minX, float maxX) {
  return min(max(value, minX), maxX);
}

float linearRemap(float value, float minX, float maxX, float minY, float maxY) {
    float proportion = (value - minX) / (maxX - minX);
    return minY + proportion * (maxY - minY);
}

// Motor Controls

void bangBangSteer(float targetPosition) {
  float currentPosition = linearRemap(pot.read(), potMin, potMax, -1, 1);
  currentPosition = clamp(currentPosition, -1, 1);
  if (abs(currentPosition - targetPosition) < 0.1) {
    steerPower(0);
  } else {
    steerPower(steerPID.p * (targetPosition - currentPosition));
  }
}

float getPIDCorrection(float setPoint, float processValue, PID& controller, float& lastRun){
  // TODO might need to move to end
  float dt = timeMillis() - lastRun;
  controller.setInterval(dt / 1000.0); // Miliseconds to second runtime
  lastRun = timeMillis();
  //

  controller.setSetPoint(setPoint);
  controller.setProcessValue(processValue);
  return controller.compute();
}

// Motor Power

void steerPower(float x) {
  x = clamp(x, -1.0, 1.0); // -1 <= x <= 1
  x = linearRemap(x, -1, 1, 0, 1); // convert to 0 <= x <= 1
  int width = (int)(steerPulseMinUs + (steerPulseRangeUs * x));
  driverPinSteer.pulsewidth_us(width);
  serial.printf("width: %d\r\n", width);
}

void drivePower(float dutyCycle) {
  float clipDutyCycle = clamp(dutyCycle, -1.0, 1.0); // -1 <= x <= 1
  if (dutyCycle >= 0) {
    driverPinMotorA.write(abs(clipDutyCycle));
    driverPinMotorB.write(0);
  } else {
    driverPinMotorA.write(0);
    driverPinMotorB.write(abs(clipDutyCycle));
  }
}

// find the current speed (m/s)

void measureCurrentSpeed() {
  int encoderTicks;
  encoderTicks = driveAxel.getPulses();
  encoderCurrentTime = timer.read_ms();

  float ticksPerSec = (float)encoderTicks * 1000.0 / (float)(encoderCurrentTime-encoderLastTime);
  encoderLastTime = encoderCurrentTime;
  actualSpeed = ticksPerSec * metersPerTick;
  driveAxel.reset();
}

int main (void) {
  driverPinMotorA.period_us(drivePeriod);
  driverPinMotorB.period_us(drivePeriod);
  driverPinSteer.period_ms(steerPeriodMs);

  EthernetInterface eth;
  eth.init(MBED_IP, 0, 0);
  eth.connect(100000);

  TCPSocketServer server;
  server.bind(ECHO_SERVER_PORT);
  server.listen();

  while (true) {
    timer.start();
    steerPower(0);
    drivePower(0);

    TCPSocketConnection client;
    server.accept(client);
    client.set_blocking(false, 30000); // Timeout after 30s
    printf("Accepted new client\r\n");


    while (true) {

      // Ethernet Input
      char inputBuffer[256];
      int n = client.receive(inputBuffer, sizeof(inputBuffer));
      printf("Receive message: %d\r\n", n);
      if (n <= 0)
        break;

      char firstChar = inputBuffer[0];
      char* p = inputBuffer + 1;

      char outputBuffer[256];
      sprintf(outputBuffer,"");

      if(firstChar == '$') {  // For normal control
        float desiredSpeed = strtod(p, &p);
        float steeringTarget = strtod(p, &p);

        measureCurrentSpeed();

        if(actualSpeed - desiredSpeed < 0){
          dutyCycle += getPIDCorrection(desiredSpeed, actualSpeed, accelDriveController, accelLastRunTime);
        } else {
          dutyCycle += getPIDCorrection(desiredSpeed, actualSpeed, decelDriveController, decelLastRunTime);
          // TODO add brake code here
        }


        drivePower(dutyCycle);
        bangBangSteer(steeringTarget);
        sprintf(outputBuffer, "%.3f %.3f %.3f %.3f", desiredSpeed, pot.read(), steeringTarget, actualSpeed);

      } else if (firstChar == '#') {  // For PID constant init
        accelDrivePID.p = strtod(p,&p);
        accelDrivePID.i = strtod(p,&p);
        accelDrivePID.d = strtod(p,&p);

        accelDriveController = PID(accelDrivePID.p, accelDrivePID.i, accelDrivePID.d, 0.25);
        accelDriveController.setInputLimits(0, 1);
        accelDriveController.setOutputLimits(0, 1);
        accelLastRunTime = timeMillis();

        decelDrivePID.p = strtod(p,&p);
        decelDrivePID.i = strtod(p,&p);
        decelDrivePID.d = strtod(p,&p);

        decelDriveController = PID(decelDrivePID.p, decelDrivePID.i, decelDrivePID.d, 0.25);
        decelDriveController.setInputLimits(-1, 0);
        decelDriveController.setOutputLimits(-1, 0);
        decelLastRunTime = timeMillis();

        steerPID.p = strtod(p,&p);
        steerPID.i = strtod(p,&p);
        steerPID.d = strtod(p,&p);

        steerController = PID(steerPID.p, steerPID.i, steerPID.d, 0.25);
        steerController.setInputLimits(-2, 2);
        steerController.setOutputLimits(-1, 1);
        decelLastRunTime = timeMillis();

        sprintf(outputBuffer, "PID Received");
      }

      // Output Ethernet

      if (strlen(outputBuffer) > 0) {
        printf(outputBuffer);
        n = client.send_all(outputBuffer, sizeof(outputBuffer));
        if (n <= 0)
          break;
      }

      // lastUpdate = curTime;
      wait(0.1);
    }
    timer.stop();
    client.close();
  }
}
