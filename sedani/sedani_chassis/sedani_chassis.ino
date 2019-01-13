#include "pitch.h"
#include "SpeedLUT.h"
#include <Servo.h>

// Pins
const int rcEscPin = 2;
const int rcSteerPin = 3;
const int escPin = 4;
const int steerPin = 5;
const int isManualPin = 6;
const int speakerOutputPin = 9;
const int buttonEstopPin = 10;
const int wirelessPinC = 14;
const int wirelessPinB = 15;
const int wirelessPinD = 16;

// Control Limits
const float maxSpeed = 3; // maximum velocity
const float minSpeed = -1;

const int centerSpeedPwm = 1492;

const float maxSteeringAngle = 0.463; //Radians
const float minSteeringAngle = -0.463; //Radians

const int maxSteeringPwm = 1773;
const int minSteeringPwm = 1347;
const int centerSteeringPwm = 1560;

// Control Variables
float currentSteeringAngle = 0;
float desiredSteeringAngle = 0;
float currentSpeed = 0;
float desiredSpeed = 0;

// RC Smoothing Buffers
const int bufferSize = 10;
float speedBuffer[bufferSize] = {0, 0, 0, 0, 0};
float steerBuffer[bufferSize] = {0, 0, 0, 0, 0};
int speedIndex = 0;
int steerIndex = 0;
float speedSum = 0;
float steerSum = 0;

// Timeout Variables
unsigned long lastMessageTime;
bool isTimedOut = true;

// E-Stop Variables (true means the car can't move)
bool buttonEstopActive = false;
bool wirelessEstopActive = false;

// Wireless States
bool prevWirelessStateB = false;
bool prevWirelessStateC = false;
bool prevWirelessStateD = false;

// Songs!
int songInformation0[2] = {NOTE_C4, NOTE_C4};
int songInformation1[2] = {NOTE_C5, NOTE_C4};
int songInformation2[2] = {NOTE_C4, NOTE_C5};
int songInformation3[2] = {NOTE_C5, NOTE_C5};

// Manual Variables (true means human drives)
bool isManual = true;

// Servo objects
Servo esc;
Servo steering;

enum ChassisState {
  STATE_MANUAL,
  STATE_AUTONOMOUS,
  STATE_ESTOPPED,
  STATE_TIMEOUT
} currentState = STATE_ESTOPPED;

void setup()
{
    pinMode(wirelessPinB,INPUT);
    pinMode(wirelessPinC,INPUT);
    pinMode(wirelessPinD,INPUT);
    pinMode(buttonEstopPin, INPUT);
    pinMode(rcEscPin, INPUT);
    pinMode(rcSteerPin, INPUT);
    pinMode(isManualPin, INPUT);

    pinMode(speakerOutputPin, OUTPUT);
    pinMode(escPin, OUTPUT);
    pinMode(steerPin, OUTPUT);

    esc.attach(escPin);
    steering.attach(steerPin);

    steering.write(centerSteeringPwm);
    esc.write(centerSpeedPwm);

    Serial.begin(115200);

    lastMessageTime = millis();
    isTimedOut = true;

    playSong(3);
}

void loop()
{
  bool gotMessage = getMessage();

  //if we haven't received a message from the NUC in a while, stop driving
  if(gotMessage) {
    isTimedOut = false;
    lastMessageTime = millis();
  } else if ((lastMessageTime + 1000) < millis()) {
    isTimedOut = true;
  }

  bool wirelessStateB = digitalRead(wirelessPinB);
  bool wirelessStateC = digitalRead(wirelessPinC);
  bool wirelessStateD = digitalRead(wirelessPinD);
  isManual = digitalRead(isManualPin);
  buttonEstopActive = !digitalRead(buttonEstopPin);

  wirelessEstopActive = !(wirelessStateB || wirelessStateC || wirelessStateD);

  bool isEstopped = wirelessEstopActive || buttonEstopActive;

  /*
  // If you want to add more states based on the wireless remote, use state transition checks like these
  if(wirelessStateB && !prevWirelessStateB){
    playSong(2);
  }

  if(wirelessStateC && !prevWirelessStateC){
    playSong(1);
  }

  if(wirelessStateD && !prevWirelessStateD){
    playSong(0);
  }*/

  /*
   * STATE MACHINE TRANSITIONS
   */
  if(isEstopped && currentState != STATE_ESTOPPED) {
    currentState = STATE_ESTOPPED;
    steering.write(centerSteeringPwm);
    esc.write(centerSpeedPwm);
    playSong(0);
  } else if(isManual && !isEstopped && currentState != STATE_MANUAL) {
    currentState = STATE_MANUAL;
    steering.write(centerSteeringPwm);
    esc.write(centerSpeedPwm);
    playSong(3);
  } else if( (currentState == STATE_ESTOPPED || currentState == STATE_MANUAL) && (!isEstopped && !isManual)) {
    currentState = STATE_TIMEOUT;
    steering.write(centerSteeringPwm);
    esc.write(centerSpeedPwm);
    playSong(1);
  } else if(currentState == STATE_TIMEOUT && !isTimedOut && !isEstopped && !isManual) {
    currentState = STATE_AUTONOMOUS;
    desiredSteeringAngle = 0.0;
    desiredSpeed = 0.0;
    playSong(2);
  } else if(currentState == STATE_AUTONOMOUS && isTimedOut && !isEstopped && !isManual) {
    currentState = STATE_TIMEOUT;
    steering.write(centerSteeringPwm);
    esc.write(centerSpeedPwm);
    playSong(1);
  }

  /*
   * STATE MACHINE ACTIONS
   */
  switch(currentState) {
    case STATE_ESTOPPED:
      // Do nothing, because e-stops
      break;
    case STATE_MANUAL:
      runStateManual();
      break;
    case STATE_AUTONOMOUS:
      runStateAutonomous();
      break;
    case STATE_TIMEOUT:
      break;
    default:
      break;
  }

  if(gotMessage) {
    double values[] = {currentState, currentSpeed, currentSteeringAngle};
    sendFeedback(values, sizeof(values)/sizeof(double));
  }

  prevWirelessStateB = wirelessStateB;
  prevWirelessStateC = wirelessStateC;
  prevWirelessStateD = wirelessStateD;

  delay(25);
}

unsigned long escPwmFromMetersPerSecond(float velocity)
{
  if(velocity <= 0) {
    return SpeedLUT[0][0];
  }
  double prevLUTVelocity = 0.0;
  for(int i = 1; i < 86; i++) {
    double LUTVelocity = SpeedLUT[i][1];
    if(fabs(velocity - LUTVelocity) > fabs(velocity - prevLUTVelocity)) {
      return SpeedLUT[i-1][0];
    }
  }
  return SpeedLUT[85][0];
}

float metersPerSecondFromEscPwm(unsigned long pwm) {
  int index = pwm - centerSpeedPwm;
  if(index < 0) index = 0;
  if(index > 85) index = 85;
  return SpeedLUT[index][1];
}

float linearMap(float x, float in_min, float in_max, float out_min, float out_max) {
  //@note this is the same as the arduino map() function but with floating point math
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float radiansFromServoPwm(unsigned long pwm) {
  //@note This is split into two to ensure that a center pwm actually maps to our 0 exactly
  if(pwm < centerSteeringPwm) {
    return linearMap((float)pwm, minSteeringPwm, centerSteeringPwm, minSteeringAngle, 0.0);
  } else {
    return linearMap((float)pwm, centerSteeringPwm, maxSteeringPwm, 0.0, maxSteeringAngle);
  }
}

unsigned long servoPwmFromRadians(float radians) {
  //@note This is split into two to ensure that 0 radians actually maps to our center pwm exactly
  if (radians < 0.0) {
    return linearMap(radians, minSteeringAngle, 0.0, minSteeringPwm, centerSteeringPwm);
  } else {
    return linearMap(radians, 0.0, maxSteeringAngle, centerSteeringPwm, maxSteeringPwm);
  }
}


bool getMessage()
{
  bool gotMessage = false;
  while(Serial.available())
  {
    if(Serial.read() == '$')
    {
      gotMessage = true;
      desiredSpeed = Serial.parseFloat();
      desiredSteeringAngle = Serial.parseFloat();
      desiredSpeed = min(maxSpeed, max(desiredSpeed, minSpeed));
      desiredSteeringAngle = min(maxSteeringAngle, max(desiredSteeringAngle, minSteeringAngle));
    }
  }
  return gotMessage;
}

void sendFeedback(const double* feedbackValues, const int feedbackCount) {
  String message = "$";
  for (int i = 0; i < feedbackCount; i++) {
    message.concat(feedbackValues[i]);
    if(i < feedbackCount-1) {
      message.concat(",");
    }
  }
  Serial.println(message);
}

void playSong(int number){
  for (int thisNote = 0; thisNote < 2; thisNote++) {
    int noteDuration = 125;
    switch(number) {
      case 0:
        tone(speakerOutputPin, songInformation0[thisNote], noteDuration);
        break;
      case 1:
        tone(speakerOutputPin, songInformation1[thisNote], noteDuration);
        break;
      case 2:
        tone(speakerOutputPin, songInformation2[thisNote], noteDuration);
        break;
      case 3:
        tone(speakerOutputPin, songInformation3[thisNote], noteDuration);
        break;
      default:
        break;
    }
    int pauseBetweenNotes = 162;
    delay(pauseBetweenNotes);
    noTone(speakerOutputPin);
  }
}

void runStateManual() {
  unsigned long currentEscPwm = pulseIn(rcEscPin,HIGH);
  unsigned long currentSteerPwm = pulseIn(rcSteerPin,HIGH);
  speedBuffer[speedIndex] = metersPerSecondFromEscPwm(currentEscPwm);
  steerBuffer[steerIndex] = radiansFromServoPwm(currentSteerPwm);
  speedSum += speedBuffer[speedIndex];
  steerSum += steerBuffer[steerIndex];
  speedIndex = (speedIndex + 1) % bufferSize;
  steerIndex = (steerIndex + 1) % bufferSize;
  speedSum -= speedBuffer[speedIndex];
  steerSum -= steerBuffer[steerIndex];
  currentSpeed = speedSum / bufferSize;
  currentSteeringAngle = steerSum / bufferSize;
}

void runStateAutonomous() {
  if(currentSpeed != desiredSpeed) {
    currentSpeed = desiredSpeed;
    unsigned long newEscPwm = escPwmFromMetersPerSecond(desiredSpeed);
    esc.write(newEscPwm);
  }
  if(currentSteeringAngle != desiredSteeringAngle) {
    currentSteeringAngle = desiredSteeringAngle;
    unsigned long newSteerPwm = servoPwmFromRadians(desiredSteeringAngle);
    steering.write(newSteerPwm);
  }
}
