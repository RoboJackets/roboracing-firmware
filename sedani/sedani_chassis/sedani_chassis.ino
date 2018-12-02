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

//Speed Calculation
const int speedSensor = 2;

volatile float speed = 0.0;
volatile long pTime = micros();
volatile int interruptCount = 0;
int pInterruptCount = 0;

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
    pinMode(speedSensor, INPUT);
    attachInterrupt(digitalPinToInterrupt(speedSensor), calcSpeed, RISING);
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
    double values[] = {currentState, speed, currentSteeringAngle};
    sendFeedback(values, sizeof(values)/sizeof(double));
  }
  
  prevWirelessStateB = wirelessStateB;
  prevWirelessStateC = wirelessStateC;
  prevWirelessStateD = wirelessStateD;

  pInterruptCount = interruptCount;
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

float radiansFromServoPwm(unsigned long pwm) {
  float distanceFromCenter = ((int)pwm) - centerSteeringPwm;
  if(distanceFromCenter > 0) {
    float prop = distanceFromCenter / (maxSteeringPwm - centerSteeringPwm);
    return prop * maxSteeringAngle;
  } else {
    float prop = distanceFromCenter / (minSteeringPwm - centerSteeringPwm);
    return prop * minSteeringAngle;
  }
}

unsigned long servoPwmFromRadians(float radians) {
  if(radians > 0) {
    float prop = radians / maxSteeringAngle;
    return (prop * ( maxSteeringPwm - centerSteeringPwm)) + centerSteeringPwm;
  } else {
    float prop = radians / minSteeringAngle;
    return (prop * (minSteeringPwm - centerSteeringPwm)) + centerSteeringPwm;
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
  //currentSpeed = speedSum / bufferSize;
  if(interruptCount-pInterruptCount == 0){
    speed = 0;
  }
  currentSpeed = speed;
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

void calcSpeed(){
  long cTime = micros();
  if(pTime != cTime){
    speed = 1000000.0/(cTime - pTime);
  }
  else{
    speed = 0;
  }
  pTime = cTime;
  interruptCount ++;
}
