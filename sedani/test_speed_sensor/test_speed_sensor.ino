#include <Servo.h>
#include <Encoder.h>

const int escPin = 4;

//Encoder pins must be interrupt capable
const byte encoderPinA = 7;
const byte encoderPinB = 8;

// Encoders
Encoder driveShaftEncoder(encoderPinA, encoderPinB);
float speedBuffer[20];
long prevEncoderPosition = 0;
unsigned long prevTime = 0;
unsigned long currTime = 0;
float measuredSpeed = 0.0;
const float metersPerEncoderTick = 1.0/9641.5;
const int millisPerSec = 1000;

//
const int escPwm = 1570;



Servo esc;

void setup() {
  // put your setup code here, to run once:
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(escPin, OUTPUT);

  //Initialize encoder position to 0;
  driveShaftEncoder.write(0);

  Serial.begin(9600);
  esc.attach(escPin);  
}

void loop() {
  // put your main code here, to run repeatedly:
  esc.write(escPwm);

  calculateSpeed();

  Serial.println(prevEncoderPosition*metersPerEncoderTick);
  delay(20);
}

void calculateSpeed(){
    long currentEncoderPosition = driveShaftEncoder.read();
    currTime = millis();    
    measuredSpeed = -(currentEncoderPosition - prevEncoderPosition)*metersPerEncoderTick/(currTime-prevTime)*millisPerSec;
    prevTime = currTime;
    prevEncoderPosition = currentEncoderPosition;
}
