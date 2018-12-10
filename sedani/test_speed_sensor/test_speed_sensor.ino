#include <Servo.h>

const int speedSensor = 7;
const int escPin = 4;

const int escPwm = 1575;
const float speedConstant = 1000000.0;
volatile float speed = 0.0;
volatile long pTime = micros();
volatile int interruptCount = 0;
int pInterruptCount = 0;


Servo esc;

void setup() {
  // put your setup code here, to run once:
  pinMode(speedSensor, INPUT_PULLUP);
  
  pinMode(escPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(speedSensor), calcSpeed, RISING);
  Serial.begin(9600);
  esc.attach(escPin);  
}

void loop() {
  // put your main code here, to run repeatedly:
  esc.write(escPwm);
  if(interruptCount-pInterruptCount == 0){
    speed = 0;
  }
  pInterruptCount = interruptCount;
  interruptCount = 0;
  Serial.println(speed);
  delay(10);
}

void calcSpeed(){
  long cTime = micros();
  if(pTime != cTime){
    speed = speedConstant/(cTime - pTime);
  }
  else{
    speed = 0;
  }
  pTime = cTime;
  interruptCount++;
}

