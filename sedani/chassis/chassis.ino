#include "pitch.h"
#include <Servo.h>

//forward method declarations
void updateHeading();
void updateSpeed();
float speedToPwm(float velocity);
void rcEscRead();
void rcSteerRead();
void encoderCallback();
bool getMessage();
void measureCurrentSpeed();

//control limits
static const float maxSpeed = 3; // maximum velocity 
static const float minSpeed = -1;
static const float maxHeading = 0.463; //Radians
static const float minHeading = -0.463; //Radians

static const int steerRangePwm = 213; //(Max Steering Pwm  - Min Steering Pwm) / 2
static int steerCenterPwm = 1560;

static const int escRangePwm = 90; //Max PWM chosen for safety
static int escCenterPwm = 1520;

//Setpoint and error tracking
static float currentHeading = 0;
static float desiredHeading = 0;
static volatile float currentSpeed = 0;
static float desiredSpeed = 0;

//stop conditions for the car
static int lastMessageTime;
static bool timeout = false; 

//Button E-Stop (true means E-Stop is off)
const int buttonEstopPin = 10;
bool buttonEstopOff = false;

//Wireless E-Stop (true means E-Stop is off)
const int wirelessEstopPin = 16;
bool wirelessEstopOff = false;

const int wirelessD0Pin = 16;
bool wirelessD0State = false;
bool wirelessD0Buffer = false;

const int wirelessD1Pin = 14;
bool wirelessD1State = false;
bool wirelessD1Buffer = false;

const int wirelessD2Pin = 15;
bool wirelessD2State = false;
bool wirelessD2Buffer = false;

const int speakerOutputPin = 9;

int songInformation0[2] = {NOTE_C4, NOTE_C4};
int songInformation1[2] = {NOTE_C5, NOTE_C4};
int songInformation2[2] = {NOTE_C4, NOTE_C5};

//autonomous or human control
const int manualStatePin = 6;
bool manualState = true;

//RC Remote Monitoring
static float currentEscVelocity = 0;
static float currentEscPwm = 0;
const int rcEscPin = 2;

static float currentSteerAngle = 0;
static float currentSteerPwm = 0;
const int rcSteerPin = 3;

//ESC and Steering objects
Servo esc;
const int escPin = 4;
Servo steering;
const int steerPin = 5;

void setup()
{
    pinMode(wirelessD0Pin,INPUT);
    pinMode(wirelessD1Pin,INPUT);
    pinMode(wirelessD2Pin,INPUT);
    pinMode(speakerOutputPin, OUTPUT);
  
    pinMode(buttonEstopPin, INPUT);
    pinMode(wirelessEstopPin, INPUT);
    
    pinMode(escPin, OUTPUT);
    esc.attach(escPin);
    
    pinMode(steerPin, OUTPUT);
    steering.attach(steerPin);

    //RC Remote Monitoring Init
    pinMode(rcEscPin, INPUT);
    pinMode(rcSteerPin, INPUT);

    pinMode(manualStatePin, INPUT);

    Serial.begin(115200);
    lastMessageTime = millis();

}

void loop()
{   
  wirelessD0State = digitalRead(wirelessD0Pin);
  wirelessD1State = digitalRead(wirelessD1Pin);
  wirelessD2State = digitalRead(wirelessD2Pin);

  if(wirelessD0State != wirelessD0Buffer && wirelessD0State){
    desiredSpeed = 0;
    desiredHeading = 0;
    Serial.println("Standby");
    playSong(0);
  }
 
  if(wirelessD1State != wirelessD1Buffer && wirelessD1State){
    desiredSpeed = 1;
    desiredHeading = 0;
    Serial.println("Move Forward Medium");
    playSong(1);
  }
  
  if(wirelessD2State != wirelessD2Buffer && wirelessD2State){
    desiredSpeed = 2;
    desiredHeading = 0;
    Serial.println("Move Forward Fast");
    playSong(2);
  }
  
  manualState = digitalRead(manualStatePin);
  buttonEstopOff = digitalRead(buttonEstopPin);
  wirelessEstopOff = digitalRead(wirelessEstopPin);


  //if we haven't received a message from the NUC in a while, stop driving
  if (lastMessageTime + 500 < millis()) {
    //timeout = true;
  }
  
  if (manualState || !buttonEstopOff || !wirelessEstopOff || timeout) {
//    desiredSpeed = 0;
//    currentSpeed = 0;
//    desiredHeading = 0;
//    currentHeading = 0;
//    escDrive(0);
  } else{
    updateSpeed();
    updateHeading();
  }

  updateHeading();
  updateSpeed();

  updateRcSteerRead();
  updateRcEscRead();
  wirelessD0Buffer = wirelessD0State;
  wirelessD1Buffer = wirelessD1State;
  wirelessD2Buffer = wirelessD2State;
  
  delay(50);
}


void updateHeading()
{
  currentHeading = min(maxHeading, max(desiredHeading, minHeading));
  int steerPwm = ((currentHeading/maxHeading) * steerRangePwm)+steerCenterPwm;
  steering.write(steerPwm);    
}

void updateSpeed()
{
  currentSpeed = min(maxSpeed, max(desiredSpeed, minSpeed));
  int escPwm = speedToPwm(currentSpeed);
  esc.write(escPwm);
}

float speedToPwm(float velocity)
{
  float outputPwm = 0.8675*(velocity*velocity) + 15.619*(velocity) + escCenterPwm;
  return outputPwm;
}

void updateRcSteerRead()
{
  //In RC remote mode, read Steer
  if(manualState){
    currentSteerPwm = float(pulseIn(rcSteerPin,HIGH));
    //1500 is the center of the PWM range of the reciever output (between 1000 to 2000)
    float currentSteerProp = min(max((currentSteerPwm - (steerCenterPwm))/steerRangePwm,-1),1);
    if(currentSteerProp > 0){
      currentSteerAngle = currentSteerProp*maxHeading; 
    } else{
      currentSteerAngle = -1*currentSteerProp*minHeading;
    }
  }
}

void updateRcEscRead()
{
  //In RC remote mode, read Esc
  if(manualState){
    currentEscPwm = float(pulseIn(rcEscPin,HIGH));
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
      timeout = false;
      lastMessageTime = millis();
      
      desiredSpeed = Serial.parseFloat();
      desiredHeading = Serial.parseFloat();
            
      String message = "$";
      //If in RC control
      if(manualState){
         message.concat("D, ");
         message.concat(currentEscPwm);
      }
      else{
        message.concat("A,");
        message.concat(currentSpeed);
        message.concat(",");
        message.concat(wirelessEstopOff);
        message.concat(",");
        message.concat(buttonEstopOff);
      }
      Serial.println(message);
    }
  }
  return gotMessage;
}


void playSong(int number){
    for (int thisNote = 0; thisNote < 2; thisNote++) {
      int noteDuration = 125;
      if(number == 0){
        tone(speakerOutputPin, songInformation0[thisNote], noteDuration);
      } else if(number == 1){
        tone(speakerOutputPin, songInformation1[thisNote], noteDuration);
      } else{
        tone(speakerOutputPin, songInformation2[thisNote], noteDuration);
      }
      int pauseBetweenNotes = 162;
      delay(pauseBetweenNotes);
      noTone(speakerOutputPin);
    }
}
