#include "QEI.h"


Timer timer;
const PinName encoderChannelA = p25;
const PinName encoderChannelB = p24;

QEI driveAxel (encoderChannelA,encoderChannelB, NC, -1, X2_ENCODING);

const float wheelCircumference = 0.246 * 3.14159; //meters
const float ticksPerRot = 8637; // TODO update
const float metersPerTick = wheelCircumference / ticksPerRot;

float desiredSpeed = 0;
float actualSpeed = 0;
float lastDutyCycle = 0;
uint32_t lastTime = 0;
uint32_t currentTime = 0;

int main() {
	timer.start();
	while(1){
		measureCurrentSpeed();
		wait(0.1);

	}
	timer.stop();
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