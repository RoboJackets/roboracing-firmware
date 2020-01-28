#include "DriveBrake.h"
volatile long encoder0Pos=0;

void loop() {
  // put your main code here, to run repeatedly:

}

double accelerate(double a){
  
}

double brake(double targetSpeed){
  
}

double getSpeed(){  
  digitalWrite(encoder0PinA, HIGH);
  digitalWrite(encoder0PinB, HIGH);
  //encoder counts 64 every revolution
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), doEncoder, RISING); // interrupts encoderpin in 19
  int newposition = encoder0Pos;
  int newtime = (0.001*millis());
  int counts = (newposition-oldposition)/(newtime-oldtime);
  

}

void doencoder(){
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
}


void setMotorPWM(double voltage){
 byte PWM = ((0.556 - sqrt(0.556*0.556 - 0.00512*(62.1-voltage)))/0.00256); //see "rigatoni PWM to motor power" in drive for equation
 analogWrite(MOTOR_CONTROLLER_INPUT, PWM);
}

int getCurrent(){
  int counts = analogRead(CURR_DATA);
  return 200*5/counts; // 200A/V * 5V/count -> gives A
}
