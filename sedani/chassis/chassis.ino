#include "pitch.h"
#include <Servo.h>

// Pins
const int rcEscPin = 2;
const int rcSteerPin = 3;
const int escPin = 4;
const int steerPin = 5;
const int manualStatePin = 6;
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

// Timeout Variables
int lastMessageTime;
bool timeout = false; 

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
bool manualState = true;

// Servo objects
Servo esc;
Servo steering;

enum ChassisState {
  STATE_MANUAL,
  STATE_AUTONOMOUS,
  STATE_ESTOPPED
} currentState = STATE_ESTOPPED;

void setup()
{
    pinMode(wirelessPinB,INPUT);
    pinMode(wirelessPinC,INPUT);
    pinMode(wirelessPinD,INPUT);
    pinMode(buttonEstopPin, INPUT);
    pinMode(rcEscPin, INPUT);
    pinMode(rcSteerPin, INPUT);
    pinMode(manualStatePin, INPUT);
    
    pinMode(speakerOutputPin, OUTPUT);
    pinMode(escPin, OUTPUT);
    pinMode(steerPin, OUTPUT);
    
    esc.attach(escPin);
    steering.attach(steerPin);

    steering.write(centerSteeringPwm);
    esc.write(centerSpeedPwm);

    Serial.begin(115200);
    
    lastMessageTime = millis();
    
    playSong(3);
}

void loop()
{   
  bool gotMessage = getMessage();

  //if we haven't received a message from the NUC in a while, stop driving
  if(gotMessage) {
    timeout = false;
    lastMessageTime = millis();
  } else if (lastMessageTime + 500 < millis()) {
    timeout = true;
  }
  
  bool wirelessStateB = digitalRead(wirelessPinB);
  bool wirelessStateC = digitalRead(wirelessPinC);
  bool wirelessStateD = digitalRead(wirelessPinD);
  manualState = digitalRead(manualStatePin);
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
  } else if(manualState && currentState != STATE_MANUAL) {
    currentState = STATE_MANUAL;
    steering.write(centerSteeringPwm);
    esc.write(centerSpeedPwm);
  } else if(timeout && currentState == STATE_AUTONOMOUS) { 
    currentState = STATE_ESTOPPED;
    steering.write(centerSteeringPwm);
    esc.write(centerSpeedPwm);
    playSong(0);
  } else if(currentState != STATE_AUTONOMOUS) {
    currentState = STATE_AUTONOMOUS;
    desiredSteeringAngle = 0.0;
    desiredSpeed = 0.0;
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
  
  delay(50);
}

unsigned long escPwmFromMetersPerSecond(float velocity)
{
  // TODO need new data
  return 0;
}

float radiansFromServoPwm(unsigned long pwm) {
  unsigned long distanceFromCenter = pwm - centerSteeringPwm;
  if(distanceFromCenter > 0) {
    float prop = distanceFromCenter / (maxSteeringPwm - centerSteeringPwm);
    return prop * maxSteeringAngle;
  } else {
    float prop = distanceFromCenter / (minSteeringPwm - centerSteeringPwm);
    return prop * minSteeringAngle;
  }
}

unsigned long servoPwmFromRadians(float radians) {
  // TODO
  return 0;
}

float metersPerSecondFromEscPwm(unsigned long pwm) {
  // TODO need new data
  return 0.0;
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
  currentSpeed = metersPerSecondFromEscPwm(currentEscPwm);
  currentSteeringAngle = radiansFromServoPwm(currentSteerPwm);
}

void runStateAutonomous() {
  currentSpeed = desiredSpeed;
  currentSteeringAngle = desiredSteeringAngle;
  unsigned long newEscPwm = escPwmFromMetersPerSecond(desiredSpeed);
  unsigned long newSteerPwm = servoPwmFromRadians(desiredSteeringAngle);
  esc.write(newEscPwm);
  steering.write(newSteerPwm);
}

