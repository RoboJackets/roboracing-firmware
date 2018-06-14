#include <Servo.h>

//forward method declarations
void escDrive(int val);
void steerDrive(int val);
void limitDesiredSpeed(float& desiredSpeed);
void limitDesiredHeading(float& desiredHeading);
void updateHeading();
void updateSpeed();
int speedToPWM(float velocity);
void rcEscRead();
void rcSteerRead();
void encoderCallback();
bool getMessage();
void measureCurrentSpeed();

//control limits
static const float maxSpeed = 430; // maximum velocity 
static const float minSpeed = -90;
static const float maxHeading = 0.35;
static const float minHeading = -0.35;

static int trim = 0;

//Setpoint and error tracking
static volatile float currentSpeed = 0;
static float currentHeading = 0;
static float desiredHeading = 0;
static float desiredSpeed = 0;
static float currentEscPWM = 0;

//stop conditions for the car
static int lastMessageTime;
static bool timeout = false; 

//Button E-Stop (true means E-Stop is off)
const int buttonEstopPin = 10;
bool buttonEstopOff = false;

//Wireless E-Stop (true means E-Stop is off)
const int wirelessEstopPin = 16;
bool wirelessEstopOff = false;

//autonomous or human control
const int manualStatePin = 6;
bool manualState = true;

//RC Remote Monitoring
static float currentEscValue = 0;
const int rcEscPin = 2;
static float currentSteerValue = 0;
const int rcSteerPin = 3;

//ESC and Steering objects
Servo esc;
const int escPin = 4;
Servo steering;
const int steerPin = 5;

void setup()
{
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
      
    escDrive(0);
    steerDrive(0);

    Serial.begin(115200);
    lastMessageTime = millis();

}

void loop()
{   
  manualState = digitalRead(manualStatePin);
  buttonEstopOff = digitalRead(buttonEstopPin);
  wirelessEstopOff = digitalRead(wirelessEstopPin);

  if(getMessage()) {
      limitDesiredSpeed(desiredSpeed);
      limitDesiredHeading(desiredHeading); 
  }

  //if we haven't received a message from the NUC in a while, stop driving
  if (lastMessageTime + 500 < millis()) {
    timeout = true;
  }
  
  if (manualState || !buttonEstopOff || !wirelessEstopOff || timeout) {
    desiredSpeed = 0;
    currentSpeed = 0;
    desiredHeading = 0;
    currentHeading = 0;
    escDrive(0);
    steerDrive(0);
  } else{
    updateHeading();
    updateSpeed();
  }
  
  
  
  
  updateRcSteerRead();
//updateRcEscRead();
  
  
  delay(50);
}

void limitDesiredSpeed(float& desiredSpeed) {
  desiredSpeed = min(maxSpeed, max(desiredSpeed, minSpeed));
}

void limitDesiredHeading(float& desiredHeading) {
  desiredHeading = min(maxHeading, max(desiredHeading, minHeading));    
}

void updateHeading()
{
  currentHeading = desiredHeading;
  //TODO fix constants
  int steerPwm = ((currentHeading - minHeading) * 142.857) - 45;
  steerDrive(steerPwm);    
}

void updateSpeed()
{
    currentSpeed = desiredSpeed;
    int escPWM = speedToPWM(currentSpeed);
    escDrive(currentEscPWM);
}

int speedToPWM(float velocity)
{
  int outputPWM = 0;
  //TODO do table calc
  return outputPWM;
}

void rcSteerRead()
{
  //In RC remote mode, read Steer
  if(manualState){
    float currentSteerPWM = float(pulseIn(rcSteerPin,HIGH));
    currentSteerPWM = min(max((currentSteerPWM-1500.0)/500.0,-1),1);

    //TODO Clean up
    if(currentSteerPWM > 0){
      currentSteerValue = currentSteerPWM*maxHeading;
    } else{
      currentSteerValue = -1*currentSteerPWM*minHeading;
    }
  }
}

void rcEscRead()
{
  //In RC remote mode, read Esc
  if(manualState){
    float currentEscPWM = float(pulseIn(rcEscPin,HIGH));
    currentEscPWM = min(max((currentEscPWM-1500.0)/500.0,-1),1);
    //TODO Clean up
    if(currentEscPWM > 0){
      currentEscValue = currentEscPWM;
    } else{
      currentEscValue = -1*currentEscPWMg;
    }
  }
}

//TODO understand +90
void steerDrive(int val)
{
  steering.write(val + trim + 90);
}

//TODO understand +90
void escDrive(int val)
{
  esc.write(val + 90);
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
      pid_p = Serial.parseFloat();
      pid_i = Serial.parseFloat();
      pid_d = Serial.parseFloat();
      trim = Serial.parseInt();
            
      String message = "$";
      //If in RC control
      if(manualState){
         message.concat("D,");
         message.concat(currentSteerValue);
      }
      else{
        message.concat("A,");
        message.concat(currentSpeed);
        message.concat(",");
        message.concat(estop);
      }
      Serial.println(message);
    }
  }
  return gotMessage;
}
