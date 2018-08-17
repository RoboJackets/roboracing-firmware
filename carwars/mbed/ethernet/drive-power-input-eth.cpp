#include "mbed.h"
#include "EthernetInterface.h"
#include "rtos.h"  // sleep
#include "QEI.h"

#include <string>
#include <cstdlib>
#include <cmath>
 
const unsigned char ECHO_SERVER_PORT = 7;
const char* const MBED_IP = "192.168.2.2";

const int drivePeriod = 1000; //us
const int mainLoopMillis = 50;
const int steerPeriodMs = 16;
const int steerPulseMinUs = 1250;
const int steerPulseMaxUs = 1750;
const int steerPulseRangeUs = steerPulseMaxUs - steerPulseMinUs;

const double potMin = 0.125;
const double potMax = 0.825;

const double kP = 0.4;
// const double kI = 0;
// const double kD = 0;

//Encoder Constants

const float wheelCircumference = 0.246 * 3.14159; //meters
const float ticksPerRot = 8637; // TODO update
const float metersPerTick = wheelCircumference / ticksPerRot;

float desiredSpeed = 0;
float actualSpeed = 0;
float lastDutyCycle = 0;
uint32_t lastTime = 0;
uint32_t currentTime = 0;

// Pinouts
AnalogIn pot(p15);
PwmOut driverPinMotorA(p21);
PwmOut driverPinMotorB(p22);
PwmOut driverPinSteer(p23);
const PinName encoderChannelB = p24;
const PinName encoderChannelA = p25;

QEI driveAxel (encoderChannelA,encoderChannelB, NC, -1, X2_ENCODING);
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
  double current = linearRemap(pot.read(), potMin, potMax, -1, 1);
  current = clamp(current, -1, 1);
  if (abs(current - targetPosition) < 0.1) {
    steerPower(0);
  } else {
    steerPower(kP * (targetPosition - current));
  }
}

void drivePower(float x) {
  float dutyCycle = clamp(x, -1.0, 1.0); // -1 <= x <= 1
  if (x >= 0) {
    driverPinMotorA.write(abs(dutyCycle));
    driverPinMotorB.write(0);
  } else {
    driverPinMotorA.write(0);
    driverPinMotorB.write(abs(dutyCycle));
  }
}

// find the current speed (m/s)
void measureCurrentSpeed() {
  encoderTicks = driveAxel.getPulses();
  currentTime = timer.read_ms();

  float ticksPerSec = (float)driveAxel.getPulses() * 1000.0 / (float)(currentTime-lastTime);
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
    // unsigned int lastUpdate = 0;

    TCPSocketConnection client;
    server.accept(client);
    client.set_blocking(false, 30000); // Timeout after 30s

    while (true) {
      // unsigned int curTime = timeMillis();
      // float dt = (curTime - lastUpdate) / 1000.0;

      char buffer[256];
      int n = client.receive(buffer, sizeof(buffer));
      if (n <= 0) 
        break;

      char* p = buffer;
      double speedTarget = strtod(p, &p);
      double steeringTarget = strtod(p, &p);

      measureCurrentSpeed();

      drivePower(speedTarget);
      bangBangSteer(steeringTarget);

      sprintf(buffer, "%.3f %.3f %.3f %.3f", speedTarget, pot.read(), steeringTarget, actualSpeed);

      n = client.send_all(buffer, sizeof(buffer));
      if (n <= 0)
        break;

      // lastUpdate = curTime;
      wait(0.1);
    }
    timer.stop();
    client.close();
  }
}
