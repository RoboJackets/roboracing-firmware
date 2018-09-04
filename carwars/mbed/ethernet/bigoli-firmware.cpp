#include "mbed.h"
#include "EthernetInterface.h"
#include "rtos.h"  // sleep
#include "QEI.h"
#include "PID.h"

#include <stdio.h>
#include <string>
#include <cstdlib>
#include <cmath>


/*
  @NOTE
  This firmware is designed to be used with the bigoli_ethernet_drive_relay ROS node
  PID message is prefaced with a # ; EX: #p i d p i d p i d
  Steering and speed message is prefaced with a $ ; EX: $speed steeringANgle

  On receiving a message, this server must respond with an @ and an optional message EX: @PID Received
  This @ symbol denotes messages received properly (OK status)
  The client will wait for the @ symbol before next msg

*/


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

const float potMin = 0.360;
const float potMax = 0.610;

//Encoder Constants
const float wheelCircumference = 0.246 * 3.14159; //meters
const float ticksPerRot = 8637; // TODO update
const float metersPerTick = wheelCircumference / ticksPerRot;

PIDConstants drivePID, steerPID;

PID driveController(0.0,0.0,0.0,1.0);
PID steerController(0.0,0.0,0.0,1.0);


float desiredSpeed = 0;
float actualSpeed = 0;
float dutyCycle = 0;

float driveLastRunTime = 0;
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
  if (abs(currentPosition - targetPosition) < 0.05) {
    steerPower(0);
  } else {
    steerPower(steerPID.p * (targetPosition - currentPosition));
  }
}

float getPIDCorrection(float setPoint, float processValue, PID *controller, float *lastRun){
  // TODO might need to move to end
  float dt = timeMillis() - (unsigned int)lastRun;
  controller->setInterval(dt / 1000.0); // Miliseconds to second runtime
  *lastRun = (float)timeMillis();

  controller->setSetPoint(setPoint);
  controller->setProcessValue(processValue);
  return controller->compute();
}

// Motor Power

void steerPower(float x) {
  x = clamp(x, -1.0, 1.0); // -1 <= x <= 1
  x = linearRemap(x, -1, 1, 0, 1); // convert to 0 <= x <= 1
  int width = (int)(steerPulseMinUs + (steerPulseRangeUs * x));
  driverPinSteer.pulsewidth_us(width);
  printf("width: %d\r\n", width);
}

void drivePower(float dutyCycle) {
  float clipDutyCycle = clamp(dutyCycle, -1.0, 1.0); // -1 <= x <= 1
  printf("ClipDUTYCYLE: %0.2f \r\n", clipDutyCycle);
  clipDutyCycle = -clipDutyCycle;
  if (clipDutyCycle >= 0) {
    driverPinMotorA.write(abs(clipDutyCycle));
    driverPinMotorB.write(0.0f);
  } else {
    driverPinMotorA.write(0.0f);
    driverPinMotorB.write(abs(clipDutyCycle));
  }
}

// find the current speed (m/s)

void measureCurrentSpeed() {
  int encoderTicks;
  encoderTicks = driveAxel.getPulses();
  encoderCurrentTime = timeMillis();

  float ticksPerSec = (float)encoderTicks * 1000.0 / (float)(encoderCurrentTime-encoderLastTime);
  encoderLastTime = encoderCurrentTime;
  actualSpeed = ticksPerSec * metersPerTick;

  driveAxel.reset();
}

int main (void) {
  driverPinMotorA.period_us(drivePeriod);
  driverPinMotorB.period_us(drivePeriod);
  driverPinSteer.period_ms(steerPeriodMs);

  printf("connect EthernetInterface\r\n");
  EthernetInterface eth;
  eth.init(MBED_IP, 0, 0);
  eth.connect(100000);

  printf("connect TCPSocketServer\r\n");
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
      //printf("Received bits: %d \r\n", n);
      //printf("inputBuffer: ");
      //printf(inputBuffer);
      //printf("\r\n");

      if (n <= 0)
        break;

      char firstChar = inputBuffer[0];
      char* p = inputBuffer + 1;

      char outputBuffer[256];
      memset(outputBuffer, 0, sizeof(outputBuffer)); //clear output buffer

      if(firstChar == '$') {  // For normal control
        float desiredSpeed = strtod(p, &p);
        float steeringTarget = strtod(p, &p);

        measureCurrentSpeed();

        printf("DESIREDSPD: %f", desiredSpeed);
        printf("ACTUALSPD: %f", actualSpeed);
        float pidCorrection = getPIDCorrection(desiredSpeed, actualSpeed, &driveController, &driveLastRunTime);
        dutyCycle += pidCorrection;
        printf("PIDCORRECTION: %f", pidCorrection);

        drivePower(dutyCycle);
        bangBangSteer(steeringTarget);
        //Put data in buffer for tcp client
        sprintf(outputBuffer, "@%.3f %.3f %.3f %.3f", desiredSpeed, pot.read(), steeringTarget, actualSpeed); //@NOTE @ symbol denotes ok status

      } else if (firstChar == '#') {  // For PID constant init
        drivePID.p = strtod(p,&p);
        drivePID.i = strtod(p,&p);
        drivePID.d = strtod(p,&p);

        printf("p: %0.2f i: %0.2f d: %0.2f\r\n", drivePID.p, drivePID.i, drivePID.d); //debug

        driveController = PID(drivePID.p, drivePID.i, drivePID.d, 0.25);
        driveController.setInputLimits(0, 1);
        driveController.setOutputLimits(0, 1);
        driveController.reset();
        driveLastRunTime = timeMillis();

        // reading too many PID constants
        strtod(p,&p);
        strtod(p,&p);
        strtod(p,&p);

        steerPID.p = strtod(p,&p);
        steerPID.i = strtod(p,&p);
        steerPID.d = strtod(p,&p);

        steerController = PID(steerPID.p, steerPID.i, steerPID.d, 0.25);
        steerController.setInputLimits(-2, 2);
        steerController.setOutputLimits(-1, 1);
        steerController.reset();
        steerLastRunTime = timeMillis();

        sprintf(outputBuffer, "@PID Received"); //Put data in buffer for tcp client @NOTE @ symbol denotes ok status
      }

      // Output Ethernet

      if (strlen(outputBuffer) > 0) {
        printf("OutputBuffer: ");
        printf(outputBuffer);
        printf("\r\n");
        n = client.send_all(outputBuffer, sizeof(outputBuffer));
        if (n <= 0)
          break;
      }

      // lastUpdate = curTime;
      wait(0.1);
    }
    steerPower(0); //end steering on disconnecting
    drivePower(0);
    timer.stop();
    client.close();
  }
}
