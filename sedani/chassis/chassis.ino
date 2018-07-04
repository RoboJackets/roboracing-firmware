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

//To keep track if just turned on
static bool justOn = false;

//control limits
static const float maxSpeed = 3; // maximum velocity 
static const float minSpeed = -1;
static const float maxHeading = 0.463; //Radians
static const float minHeading = -0.463; //Radians

static const int steerRangePwm = 213; //(Max Steering Pwm  - Min Steering Pwm) / 2
static int steerCenterPwm = 1560;

static int escCenterPwm = 1492;

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

const int bWPin = 15;
bool bWState = false;
bool bWBuffer = false;

const int cWPin = 14;
bool cWState = false;
bool cWBuffer = false;

const int dWPin = 16;
bool dWState = false;
bool dWBuffer = false;

//If true, e-stop is off
bool totalWStateOff = false;
bool totalWBuffer = false;

const int speakerOutputPin = 9;

int songInformation0[2] = {NOTE_C4, NOTE_C4};
int songInformation1[2] = {NOTE_C5, NOTE_C4};
int songInformation2[2] = {NOTE_C4, NOTE_C5};
int songInformation3[2] = {NOTE_C5, NOTE_C5};

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
    pinMode(bWPin,INPUT);
    pinMode(cWPin,INPUT);
    pinMode(dWPin,INPUT);
    
    
    pinMode(speakerOutputPin, OUTPUT);
  
    pinMode(buttonEstopPin, INPUT);
    
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
  if(!justOn){
    playSong(3);
    justOn = true;
  }
  
  bWState = digitalRead(bWPin);
  cWState = digitalRead(cWPin);
  dWState = digitalRead(dWPin);
  
  totalWStateOff = bWState || cWState || dWState;

  if(bWState != bWBuffer && bWState){
    desiredSpeed = 2;
    desiredHeading = 0;
    Serial.println("Move Forward Fast");
    playSong(2);
  }

  if(cWState != cWBuffer && cWState){
    desiredSpeed = 1;
    desiredHeading = 0;
    Serial.println("Move Forward Medium");
    playSong(1);
  }

  if(dWState != dWBuffer && dWState){
    desiredSpeed = 0;
    desiredHeading = 0;
    Serial.println("Standby");
    playSong(0);
  }

  if(totalWStateOff != totalWBuffer && !totalWStateOff){
    Serial.println("Stop");
    playSong(0);
  }
  

//  if(dWState != dWBuffer && dWState){
//    //desiredSpeed = 0;
//    escRangePwm = 0;
//    desiredHeading = 0;
//    Serial.println("Standby");
//    playSong(0);
//  }
// 
  
  manualState = digitalRead(manualStatePin);
  buttonEstopOff = digitalRead(buttonEstopPin);


  //if we haven't received a message from the NUC in a while, stop driving
  if (lastMessageTime + 500 < millis()) {
    //timeout = true;
  }
  
//  if (manualState || !buttonEstopOff || !totalWStateOff || timeout) {
//    desiredSpeed = 0;
//    currentSpeed = 0;
//    desiredHeading = 0;
//    currentHeading = 0;
//    escDrive(0);
//  } else{
//    updateSpeed();
//    updateHeading();
//  }

  updateHeading();
  updateSpeed();

  updateRcSteerRead();
  updateRcEscRead();
  
  bWBuffer = bWState;
  cWBuffer = cWState;
  dWBuffer = dWState;
  totalWBuffer = totalWStateOff;
  
  
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
  float outputPwm = 11.185*(pow(velocity,3)) - 53.673*(pow(velocity,2)) + 98.625*velocity + escCenterPwm;
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
        message.concat(totalWStateOff);
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
      } else if(number == 2){
        tone(speakerOutputPin, songInformation2[thisNote], noteDuration);
      } else{
        tone(speakerOutputPin, songInformation3[thisNote], noteDuration);
      }
      int pauseBetweenNotes = 162;
      delay(pauseBetweenNotes);
      noTone(speakerOutputPin);
    }
}
