#include <Servo.h>

#define MAP(v, minIn, maxIn, minOut, maxOut) ((v - minIn) * (maxOut - minOut) / (maxIn - minIn) + minOut)
#define DEG_TO_PULSE(d) MAP(d, SERVO_MIN_DEGREE, SERVO_MAX_DEGREE, SERVO_MIN_PULSE, SERVO_MAX_PULSE)
#define PULSE_TO_DEG(p) MAP(p, SERVO_MIN_PULSE, SERVO_MAX_PULSE, SERVO_MIN_DEGREE, SERVO_MAX_DEGREE)

Servo::Servo(PinName pin) : pulsePin(pin) {}

void Servo::attach(int startPos) {
    currentPulse = DEG_TO_PULSE(startPos);
    refreshTicker.attach(callback(this, &Servo::startPulse), SERVO_REFRESH_RATE_US/1e6);
}

void Servo::detach() {
    refreshTicker.detach();
    pulseTimer.detach();
}

void Servo::write(int pos) {
    currentPulse = DEG_TO_PULSE(pos);
}

int Servo::read() {
    return PULSE_TO_DEG(currentPulse);
}

void Servo::startPulse() {
    pulsePin = 1;
    pulseTimer.attach(callback(this, &Servo::endPulse), currentPulse/1e6);
}

void Servo::endPulse() {
    pulsePin = 0;
}