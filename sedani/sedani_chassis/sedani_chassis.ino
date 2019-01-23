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
unsigned long currentEscPwm = 0;

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
unsigned long timeoutDuration = 1000;

// E-Stop Variables (true means the car can't move)
bool buttonEstopActive = false;
bool wirelessEstopActive = false;

// Wireless States
bool prevWirelessStateB = false;
bool prevWirelessStateC = false;
bool prevWirelessStateD = false;

//Speed Calculation
const int speedSensorPin = 7;
float measuredSpeed = 0.0;
volatile unsigned long prevSampleTime = micros();
volatile unsigned long currSampleTime = micros();
volatile unsigned int interruptCount = 0;
unsigned int prevInterruptCount = 0;
//Use a speedScaling factor of 2426595.48 for RPM
//Converts frequency of sensor to m/s
const float speedScalingFactor = 9312.53;

//PID Speed Control
double integral = 0.0;
double derivative = 0.0;
double prevError = 0.0;
double kP = 0.0;
double kI = 0.0;
double kD = 0.0;


// Reverse
const unsigned long brakePwm = 1300;
const float minBrakingSpeed = 0.05;

unsigned int consecutiveZeroSpeed = 0;
const unsigned int minConsecutiveZeroSpeed = 3;

unsigned int consecutiveStop = 0;
const unsigned int minConsecutiveStop = 8;

const unsigned long reversePwm = 1375;
const unsigned long reverseHoldPwm = 1490;

bool reverseTag = false;

// Songs!
int songInformation0[2] = {NOTE_C4, NOTE_C4};
int songInformation1[2] = {NOTE_C5, NOTE_C4};
int songInformation2[2] = {NOTE_C4, NOTE_C5};
int songInformation3[2] = {NOTE_C5, NOTE_C5};
int songInformation4[2] = {NOTE_C4, NOTE_C6};
int songInformation5[2] = {NOTE_C6, NOTE_C6};
int songInformation6[2] = {NOTE_C6, NOTE_C5};
int songInformation7[2] = {NOTE_C5, NOTE_C6};
int songInformation8[2] = {NOTE_C4, NOTE_C6};

// Manual Variables (true means human drives)
bool isManual = true;

// Servo objects
Servo esc;
Servo steering;

// State machine possible states
enum ChassisState {
  STATE_DISABLED,
  STATE_TIMEOUT,
  STATE_MANUAL,
  STATE_FORWARD,
  STATE_FORWARD_BRAKING,
  STATE_REVERSE,
  STATE_REVERSE_COAST,
  STATE_REVERSE_TRANSITION,
  STATE_IDLE
} currentState = STATE_DISABLED;

void setup()
{
    pinMode(wirelessPinB,INPUT);
    pinMode(wirelessPinC,INPUT);
    pinMode(wirelessPinD,INPUT);
    pinMode(buttonEstopPin, INPUT);
    pinMode(rcEscPin, INPUT);
    pinMode(rcSteerPin, INPUT);
    pinMode(isManualPin, INPUT);
    pinMode(speedSensorPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(speedSensorPin), measureSpeed, FALLING);
    pinMode(speakerOutputPin, OUTPUT);
    pinMode(escPin, OUTPUT);
    pinMode(steerPin, OUTPUT);
    
    esc.attach(escPin);
    steering.attach(steerPin);

    steering.write(centerSteeringPwm);
    drive(centerSpeedPwm);

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
  } else if ((lastMessageTime + timeoutDuration) < millis()) {
    isTimedOut = true;
  }

  calculateSpeed();
  
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


  /////////////////////////////////
  // BEGINNING OF STATE MACHINE  //
  /////////////////////////////////
  switch(currentState){

    ////////////////////////////////////////////////////////
    //               (0)  DISABLED STATE                  //
    //   Centers steering angle and sets speed command    //
    //   to zero. Enter this state if the wireless remote //
    //   or emergency pushbutton are pressed.             //
    ////////////////////////////////////////////////////////
    case STATE_DISABLED:{     
      /*----------------------
              LOGIC
      ----------------------*/

        runHold();

      /*----------------------
            TRANSITIONS
      ----------------------*/
      // Transition to Manual State
      if (!isEstopped && isManual){
        currentState = STATE_MANUAL;
        playSong(2);
        break;
      }
      // Transition to Timeout State
      if (!isEstopped && !isManual && isTimedOut){
        currentState = STATE_TIMEOUT;
        playSong(1);
        break;
      }
      // Transition to Idle State (only transition if not moving)
      if (!isEstopped && !isManual && !isTimedOut && measuredSpeed == 0){
        currentState = STATE_IDLE;
        playSong(8);
        break;
      }
      // Default Loop State
      currentState = STATE_DISABLED;
      break;
    }

    ////////////////////////////////////////////////////////
    //              (1) TIMEOUT STATE                     //
    //    Sets Speed and Steering Angle to Zero. If       // 
    //    communication has not occured since             //
    //    timeoutDuration (in ms) and in Auto, go to this //
    //    state.                                          //
    ////////////////////////////////////////////////////////
    case STATE_TIMEOUT:{
      /*----------------------
              LOGIC
      ----------------------*/

        runHold();

      /*----------------------
            TRANSITIONS
      ----------------------*/
      // Transition to Disabled State
      if (isEstopped){
        currentState = STATE_DISABLED;
        playSong(0);
        break;
      }
      // Transition to Manual State
      if (!isEstopped && isManual){
        currentState = STATE_MANUAL;
        playSong(2);
        break;
      }
      // Transition to Idle State
      if (!isEstopped && !isManual && !isTimedOut && desiredSpeed == 0){
        currentState = STATE_IDLE;
        playSong(8);
        break;
      }      
      // Transition to Forward State
      if (!isEstopped && !isManual && !isTimedOut && desiredSpeed > 0){
        currentState = STATE_FORWARD;
        playSong(3);
        break;
      }
      // Default Loop Case
      currentState = STATE_TIMEOUT;
      break;
    }

    ////////////////////////////////////////////////////////
    //              (2)  MANUAL STATE                     //
    ////////////////////////////////////////////////////////
    case STATE_MANUAL:{
      /*----------------------
              LOGIC
      ----------------------*/

      runStateManual();      
          
      /*----------------------
            TRANSITIONS
      ----------------------*/
      // Transition to Disabled State
      if (isEstopped){
        currentState = STATE_DISABLED;
        playSong(0);
        break;
      }
      // Transition to Timeout State
      if (!isEstopped && !isManual && isTimedOut){
        currentState = STATE_TIMEOUT;
        playSong(1);
        break;
      }
      // Transition to Idle State
      if (!isEstopped && !isManual && !isTimedOut && desiredSpeed == 0){
        currentState = STATE_IDLE;
        playSong(8);
        break;
      }
      // Transition to Forward State
      if (!isEstopped && !isManual && !isTimedOut && desiredSpeed > 0){
        currentState = STATE_FORWARD;
        playSong(3);
        break;
      }
      // Default Loop Case
      currentState = STATE_MANUAL;
      break;
    }
       
    ////////////////////////////////////////////////////////
    //              (3) FORWARD STATE                     //
    ////////////////////////////////////////////////////////
    case STATE_FORWARD:{          
      /*----------------------
              LOGIC
      ----------------------*/

      runStateForward();
      
      /*----------------------
            TRANSITIONS
      ----------------------*/
      // Transition to Disabled State
      if (isEstopped){
        currentState = STATE_DISABLED;
        playSong(0);
        break;
      }
      // Transition to Manual State
      if (!isEstopped && isManual){
        currentState = STATE_MANUAL;
        playSong(2);
        break;
      }
      // Transition to Timeout State
      if (!isEstopped && !isManual && isTimedOut){
        currentState = STATE_TIMEOUT;
        playSong(1);
        break;
      }
      // Transition to Idle
      if (!isEstopped && !isManual && !isTimedOut && desiredSpeed == 0 && measuredSpeed == 0){
        currentState = STATE_IDLE;
        playSong(8);
        break;
      }
      // Transition to Forward Braking State
      if (!isEstopped && !isManual && !isTimedOut && desiredSpeed <= 0 && measuredSpeed > 0){
        currentState = STATE_FORWARD_BRAKING;
        consecutiveZeroSpeed = 0; // Reset zero speed cycle counter
        playSong(4);
        break;
      }
      // Default Loop Case
      currentState = STATE_FORWARD;
      break;
    }

    ////////////////////////////////////////////////////////
    //           (4) FORWARD BRAKING STATE                //
    ////////////////////////////////////////////////////////
    case STATE_FORWARD_BRAKING:{      
      /*----------------------
              LOGIC
      ----------------------*/

      runStateBraking();
      
      /*----------------------
            TRANSITIONS
      ----------------------*/
      // Transition to Disabled State
      if (isEstopped){
        currentState = STATE_DISABLED;
        playSong(0);
        break;
      }
      // Transition to Manual State
      if (!isEstopped && isManual){
        currentState = STATE_MANUAL;
        playSong(2);
        break;
      }
      // Transition to Timeout State
      if (!isEstopped && !isManual && isTimedOut){
        currentState = STATE_TIMEOUT;
        playSong(1);
        break;
      }
      // Transition to Forward State
      if (!isEstopped && !isManual && !isTimedOut && desiredSpeed > 0){
        currentState = STATE_FORWARD;
        playSong(3);
        break;
      }
      // Transition to Reverse Transition State
      if (!isEstopped && !isManual && !isTimedOut && desiredSpeed < 0 && measuredSpeed == 0 && consecutiveZeroSpeed > minConsecutiveZeroSpeed){
        currentState = STATE_REVERSE_TRANSITION;
        consecutiveStop = 0; // Reset stop cycle counter
        playSong(7);
        break;
      }
      //Transition to Idle State
      if (!isEstopped && !isManual && !isTimedOut && desiredSpeed == 0 && measuredSpeed == 0){
        currentState = STATE_IDLE;
        playSong(8);
        break;
      }
      // Default Loop Case
      currentState = STATE_FORWARD_BRAKING;
      break;
    } 

    ////////////////////////////////////////////////////////
    //           (5) REVERSE COAST STATE                  //
    ////////////////////////////////////////////////////////
    case STATE_REVERSE_COAST:{
      /*----------------------
              LOGIC
      ----------------------*/       
        runHold();
      /*----------------------
            TRANSITIONS
      ----------------------*/
      // Transition to Disabled State
      if (isEstopped){
        currentState = STATE_DISABLED;
        playSong(0);
        break;
      }
      // Transition to Manual State
      if (!isEstopped && isManual){
        currentState = STATE_MANUAL;
        playSong(2);
        break;
      }
      // Transition to Timeout State
      if (!isEstopped && !isManual && isTimedOut){
        currentState = STATE_TIMEOUT;
        playSong(1);
        break;
      }
      // Transition to Forward State
      if (!isEstopped && !isManual && !isTimedOut && desiredSpeed > 0){
        currentState = STATE_FORWARD;
        playSong(3);
        break;
      }
      // Transition to Reverse State
      if (!isEstopped && !isManual && !isTimedOut && desiredSpeed < 0 && measuredSpeed < 0){
        currentState = STATE_REVERSE;
        playSong(6);
        break;
      }
      // Transition to Idle
      if (!isEstopped && !isManual && !isTimedOut && desiredSpeed == 0 && measuredSpeed == 0){
        currentState = STATE_IDLE;
        playSong(8);
        break;
      }
      // Default Loop Case
      currentState = STATE_REVERSE_COAST;
      break;
    }
         
    ////////////////////////////////////////////////////////
    //           (6) REVERSE STATE                        //
    ////////////////////////////////////////////////////////
    case STATE_REVERSE:{
      /*----------------------
              LOGIC
      ----------------------*/ 

      runStateReverse();

      /*----------------------
            TRANSITIONS
      ----------------------*/
      // Transition to Disabled State
      if (isEstopped){
        currentState = STATE_DISABLED;
        playSong(0);
        break;
      }
      // Transition to Manual State
      if (!isEstopped && isManual){
        currentState = STATE_MANUAL;
        playSong(2);
        break;
      }
      // Transition to Timeout State
      if (!isEstopped && !isManual && isTimedOut){
        currentState = STATE_TIMEOUT;
        playSong(1);
        break;
      }
      // Transition to Forward State
      if (!isEstopped && !isManual && !isTimedOut && desiredSpeed > 0){
        currentState = STATE_FORWARD;
        playSong(3);
        break;
      }
      // Transition to Reverse Coast State
      if (!isEstopped && !isManual && !isTimedOut && desiredSpeed == 0 && measuredSpeed < 0){
        currentState = STATE_REVERSE_COAST;
        playSong(5);
        break;
      }
      // Default Loop Case
      currentState = STATE_REVERSE;
      break;          
    }

    ////////////////////////////////////////////////////////
    //           (7) REVERSE TRANSITION STATE             //
    ////////////////////////////////////////////////////////
    case STATE_REVERSE_TRANSITION:{
      /*----------------------
              LOGIC
      ----------------------*/       
      runStateStopped();
          
      /*----------------------
            TRANSITIONS
      ----------------------*/
      // Transition to Disabled State
      if (isEstopped){
        currentState = STATE_DISABLED;
        playSong(0);
        break;
      }
      // Transition to Manual State
      if (!isEstopped && isManual){
        currentState = STATE_MANUAL;
        playSong(2);
        break;
      }
      // Transition to Timeout State
      if (!isEstopped && !isManual && isTimedOut){
        currentState = STATE_TIMEOUT;
        playSong(1);
        break;
      }
      //Transition to Idle State
      if (!isEstopped && !isManual && !isTimedOut && desiredSpeed == 0 && measuredSpeed == 0){
        currentState == STATE_IDLE;
        playSong(8);
        break;
      }
      // Transition to Forward State
      if (!isEstopped && !isManual && !isTimedOut && desiredSpeed > 0){
        currentState = STATE_FORWARD;
        playSong(3);
        break;
      }
      // Transition to Reverse State (must wait for a minimum number of stop cycles before going to reverse)
      if (!isEstopped && !isManual && !isTimedOut && desiredSpeed < 0 && consecutiveStop > minConsecutiveStop){
        currentState = STATE_REVERSE;
        playSong(6);
        break;
      }
      // Default Loop Case
      currentState = STATE_REVERSE_TRANSITION;
      break;
    }         
      
     
    ////////////////////////////////////////////////////////
    //             (8) IDLE STATE                         //
    ////////////////////////////////////////////////////////
    case STATE_IDLE:{
      /*----------------------
              LOGIC
      ----------------------*/
      runHold();
          
     /*----------------------
            TRANSITIONS
      ----------------------*/
      // Transition to Disabled State
      if (isEstopped){
        currentState = STATE_DISABLED;
        playSong(0);
        break;
      }
      // Transition to Manual State
      if (!isEstopped && isManual){
        currentState = STATE_MANUAL;
        playSong(2);
        break;
      }
      // Transition to Timeout State
      if (!isEstopped && !isManual && isTimedOut){
        currentState = STATE_TIMEOUT;
        playSong(1);
        break;
      }
      // Transition to Forward State
      if (!isEstopped && !isManual && !isTimedOut && desiredSpeed > 0){
        currentState = STATE_FORWARD;
        playSong(3);
        break;
      }
      // Transition to Forward Braking
      if (!isEstopped && !isManual && !isTimedOut && desiredSpeed <= 0 && measuredSpeed > 0){
        currentState = STATE_FORWARD_BRAKING;
        consecutiveZeroSpeed = 0; // Reset zero speed cycle counter
        playSong(4);
        break;
      }
      // Transition to Reverse Transition
      if (!isEstopped && !isManual && !isTimedOut && desiredSpeed < 0 && measuredSpeed == 0){
        currentState = STATE_REVERSE_TRANSITION;
        consecutiveStop = 0; // Reset stop cycle counter
        playSong(7);
        break;
      }
      //Default Loop Case
      currentState = STATE_IDLE;
      break;
    }
  }
  ///////////////////////////
  // END OF STATE MACHINE  //
  ///////////////////////////


  if(gotMessage) {
    double values[] = {currentState, measuredSpeed, currentSteeringAngle};
    sendFeedback(values, sizeof(values)/sizeof(double));
  }
  
  prevWirelessStateB = wirelessStateB;
  prevWirelessStateC = wirelessStateC;
  prevWirelessStateD = wirelessStateD;

  delay(25);
}

unsigned long escPwmFromMetersPerSecond(float velocity) {
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

float maxPwm = escPwmFromMetersPerSecond(maxSpeed);

unsigned long escPwmPID(float velocity) {
  if(velocity <= 0) {
    return SpeedLUT[0][0];
  }
  else{
    double error = velocity - measuredSpeed;
    int pwm = kP * error + kI * integral + kD * derivative + escPwmFromMetersPerSecond(velocity);
    pwm = constrain(pwm, 0, maxPwm); //control limits
    if(pwm != maxPwm){ //integral windup protection
        integral += 0.025 * (prevError + error) * 0.25; //Trapezoid Rule integration.  Assumes 25ms execution time for every loop
    }
    derivative += (error - prevError)/ 0.025; //difference derivative.  Also assumes 25ms execution time
    prevError = error;
    return pwm + centerSpeedPwm;
  }

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
    unsigned long noteDuration = 125;
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
      case 4:
        tone(speakerOutputPin, songInformation4[thisNote], noteDuration);
        break;
      case 5:
        tone(speakerOutputPin, songInformation5[thisNote], noteDuration);
        break;
      case 6:
        tone(speakerOutputPin, songInformation6[thisNote], noteDuration);
        break;
      case 7:
        tone(speakerOutputPin, songInformation7[thisNote], noteDuration);
        break;
      case 8:
        tone(speakerOutputPin, songInformation8[thisNote], noteDuration);
        break;
      default:
        break;
    }
    int pauseBetweenNotes = 162;
    delay(pauseBetweenNotes);
    noTone(speakerOutputPin);
  }
}

// State functions

void runHold(){
  steering.write(centerSteeringPwm);
  drive(centerSpeedPwm);
}

void runStateManual() {
  //unsigned long currentEscPwm = pulseIn(rcEscPin,HIGH);
  unsigned long currentSteerPwm = pulseIn(rcSteerPin,HIGH);
  //speedBuffer[speedIndex] = metersPerSecondFromEscPwm(currentEscPwm);
  steerBuffer[steerIndex] = radiansFromServoPwm(currentSteerPwm);
  //speedSum += speedBuffer[speedIndex];
  steerSum += steerBuffer[steerIndex];
  //speedIndex = (speedIndex + 1) % bufferSize;
  steerIndex = (steerIndex + 1) % bufferSize;
  //speedSum -= speedBuffer[speedIndex];
  steerSum -= steerBuffer[steerIndex];
  //currentSpeed = speedSum / bufferSize;
  currentSteeringAngle = steerSum / bufferSize;
  runHold();
}

void runStateForward() {
  currentSpeed = desiredSpeed;
  drive(escPwmFromMetersPerSecond(desiredSpeed));
  steer();
}

void runStateBraking() {
  if(measuredSpeed > minBrakingSpeed){
    consecutiveZeroSpeed = 0;
    drive(brakePwm);
  }else if(measuredSpeed < -minBrakingSpeed){
    consecutiveZeroSpeed = 0;
    drive(centerSpeedPwm);
  }else{
    consecutiveZeroSpeed++;
  }
  
  steer();
}

void runStateStopped() {
  drive(reverseHoldPwm);
  consecutiveStop++;
  steer();
}

void runStateReverse() {
  drive(reversePwm);
  steer();
//  //Back up beeps >)
//  unsigned long noteDuration = 50;
//  tone(speakerOutputPin, NOTE_C7, noteDuration);
}

// Speed measurement

void measureSpeed(){
  currSampleTime = micros();  
  interruptCount++;
}

void calculateSpeed(){
  const unsigned long currTime = currSampleTime;
  const unsigned long prevTime = prevSampleTime;
  if(currTime != prevTime){
    measuredSpeed = (speedScalingFactor*(interruptCount-prevInterruptCount))/(currTime - prevTime);
    if(reverseTag){
      measuredSpeed *= -1.0;
    }
    
    prevSampleTime = currSampleTime;
  }else{
    measuredSpeed = 0;
    interruptCount = 0;
    if(currentEscPwm >= centerSpeedPwm){
      reverseTag = false;
    }
    else{
      reverseTag = true;
    }
  }
  
  prevInterruptCount = interruptCount;
}

// Steering

void steer(){
  currentSteeringAngle = desiredSteeringAngle;
  unsigned long newSteerPwm = servoPwmFromRadians(desiredSteeringAngle);
  steering.write(newSteerPwm);
}

// Driving

void drive(unsigned long desiredSentPwm){
  currentEscPwm = desiredSentPwm;
  esc.write(currentEscPwm);
}
