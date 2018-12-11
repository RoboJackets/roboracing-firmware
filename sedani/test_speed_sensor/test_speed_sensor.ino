#include <Servo.h>

const int speedSensor = 7;
const int escPin = 4;

const int escPwm = 1600;
const float speedConstant = 1000000.0;
float currentSpeed = 0.0;
volatile unsigned long pTime = micros();
volatile unsigned long cTime = micros();
volatile unsigned int interruptCount = 0;
unsigned int pInterruptCount = 0;


Servo esc;

void setup() {
  // put your setup code here, to run once:
  pinMode(speedSensor, INPUT_PULLUP);
  
  pinMode(escPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(speedSensor), calcSpeed, FALLING);
  Serial.begin(9600);
  esc.attach(escPin);  
}

void loop() {
  // put your main code here, to run repeatedly:
  esc.write(escPwm);
//  if(interruptCount-pInterruptCount == 0){
//    //currentSpeed = 0;
//    //interruptCount = 0;
//  }
//  else{
      const unsigned long currTime = cTime;
      const unsigned long prevTime = pTime;
      if(cTime != pTime){
        currentSpeed = (speedConstant*(interruptCount-pInterruptCount))/(currTime - prevTime);
        pTime = cTime;
      }else{
        currentSpeed = 0;
        interruptCount = 0;
      }

      
//  }
  pInterruptCount = interruptCount;
  Serial.println(currentSpeed);
  delay(3);
}

void calcSpeed(){
  cTime = micros();
//  if(pTime != cTime){
//    speed = speedConstant/(cTime - pTime);
//  }
//  else{
//    speed = 0;
//  }
  
  
  interruptCount++;
}

