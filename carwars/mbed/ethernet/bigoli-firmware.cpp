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

PID accelDriveController;
PID decelDriveController;
PID steerController;


float desiredSpeed = 0;
float actualSpeed = 0;
float dutyCycle = 0;
uint32_t lastTime = 0;
uint32_t currentTime = 0;

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

inline float clamp(float x, float minX, float maxX) {
  return min(max(x, minX), maxX);
}

float linearRemap(float x, float x1, float x2, float y1, float y2) {
    float proportion = (x - x1) / (x2 - x1);
    return y1 + proportion * (y2 - y1);
}

unsigned int timeMillis() {
  return (unsigned int)(timer.read() * 1000.0);
}

void steerPower(float x) {
  x = clamp(x, -0.83, 0.83); // -1 <= x <= 1
  x = linearRemap(x, -1, 1, 0, 1); // convert to 0 <= x <= 1
  int width = (int)(steerPulseMinUs + (steerPulseRangeUs * x));
  driverPinSteer.pulsewidth_us(width);
  serial.printf("width: %d\r\n", width);
}

void bangBangSteer(float targetPosition) {
  float current = linearRemap(pot.read(), potMin, potMax, -1, 1);
  current = clamp(current, -1, 1);
  if (abs(current - targetPosition) < 0.1) {
    steerPower(0);
  } else {
    steerPower(steerPID.p * (targetPosition - current));
  }
}

float getPIDCorrection(float desiredSpeed, float actualSpeed, PIDConstants constants){
  float dt = millis() - lastDriveMillis;
  lastDriveMillis = millis();

  float error = desiredSpeed - actualSpeed;
  float dError = error - lastError;
  lastError = error;

  errorHistoryIndex = (errorHistoryIndex + 1) % errorHistorySize;
  errorHistory[errorHistoryIndex] = error;
  float errorTotal = 0;
  if(constants.i != 0){
    for(int i = 0; i < errorHistorySize; i++) {
      errorTotal += errorHistory[i];
    }   
  }
  float pidCorrection = (constants.p * error) + (constants.i * errorTotal) + (constants.d * dError/dt);
  return pidCorrection;
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
  currentTime = timer.read_ms();

  float ticksPerSec = (float)encoderTicks * 1000.0 / (float)(currentTime-lastTime);
  lastTime = currentTime;
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

    while (true) {

      char inputBuffer[256];
      char outputBuffer[256];
      int n = client.receive(inputBuffer, sizeof(inputBuffer));
      if (n <= 0) 
        break;

      char firstChar = inputBuffer[0];
      char* p = inputBuffer + 1;
      if(firstChar == '$') {  // For normal control
        float speedTarget = strtod(p, &p);
        float steeringTarget = strtod(p, &p);

        measureCurrentSpeed();
        dutyCycle += getPIDCorrection(speedTarget, actualSpeed);


        drivePower(dutyCycle);
        bangBangSteer(steeringTarget);
        sprintf(outputBuffer, "%.3f %.3f %.3f %.3f", speedTarget, pot.read(), steeringTarget, actualSpeed);
        
      } else if (firstChar == '#') {  // For PID constant init
        accelDrivePID.p = strtod(p,&p);
        accelDrivePID.i = strtod(p,&p);
        accelDrivePID.d = strtod(p,&p);
        
        accelDriveController = PID(accelDrivePID.p, accelDrivePID.i, accelDrivePID.d, 1);

        decelDrivePID.p = strtod(p,&p);
        decelDrivePID.i = strtod(p,&p);
        decelDrivePID.d = strtod(p,&p);

        decelDriveController = PID(decelDrivePID.p, decelDrivePID.i, decelDrivePID.d, 1);

        steerPID.p = strtod(p,&p);
        steerPID.i = strtod(p,&p);
        steerPID.d = strtod(p,&p);

        steerController = PID(steerPID.p, steerPID.i, steerPID.d, 1);
        snprintf(outputBuffer, 256, "PID Receive");
      }


      

      n = client.send_all(outputBuffer, sizeof(outputBuffer));
      if (n <= 0)
        break;

      // lastUpdate = curTime;
      wait(0.1);
    }
    timer.stop();
    client.close();
  }
}
