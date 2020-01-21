#include "DriveBrake.h"



void loop() {
  // put your main code here, to run repeatedly:

}

double accelerate(double a){
  
}

double brake(){
  
}

double getSpeed(){  
  int counts = 
}

void setMotorPWM(double voltage){
 byte PWM = ((0.556 - sqrt(0.556*0.556 - 0.00512*(62.1-voltage)))/0.00256); //see "rigatoni PWM to motor power" in drive for equation
 analogWrite(MOTOR_CONTROLLER_INPUT, PWM);
}

int getCurrent(){
  int counts = analogRead(CURR_DATA);
  return 200*5/counts; // 200A/V * 5V/count -> gives A
}
