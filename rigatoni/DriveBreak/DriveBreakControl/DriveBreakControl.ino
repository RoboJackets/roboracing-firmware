#include "DriveBrake.h"
volatile long encoder0Pos=0;
float speed = 0;	// 0 - 255

void setup(){
  pinMode(RXLED, OUTPUT);

  pinMode(INT_ETH, INPUT);

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);  

  pinMode(ETH_RST, OUTPUT);


  
  pinMode(REVERSE_LED, OUTPUT);
  pinMode(USER_DEFINED_LED, OUTPUT);

  pinMode(MOTOR_CONTROL, OUTPUT);
  
  pinMode(BRAKE_EN, OUTPUT);
  pinMode(BRAKE_PWM, OUTPUT);

  pinMode(CURR_DATA, INPUT);

  Serial.begin(115200);
}


void loop() {
  // put your main code here, to run repeatedly:
	motorTest();
	Serial.println(speed);
	getSpeedMessage();
	
}

void motorTest() {
	analogWrite(MOTOR_CONTROLLER_INPUT, speed);
	
}

void getSpeedMessage() {
    while(Serial.available()){
      if (Serial.read() == '#'){
        speed = Serial.parseFloat();
      }
    }
}

double accelerate(double a){
  
}

double brake(double targetSpeed){
  
}

double getSpeed(){  
  digitalWrite(ENCODER_A, HIGH);
  digitalWrite(ENCODER_B, HIGH);
  //encoder counts 64 every revolution
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), doEncoder, RISING); // interrupts encoderpin in 19
  int newposition = encoder0Pos;
  int newtime = (0.001*millis());
  int counts = (newposition-oldposition)/(newtime-oldtime);
  

}

void doEncoder(){
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
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
  return (200*(5/1024)*counts) - 500; // 200A/V * 5V/count -> gives A
}

void executeStateMachine(){ 
	switch(currentState) { 
		case STATE_DISABLED:{
			
		}
		case STATE_TIMEOUT:{
			
		}
		case STATE_FORWARD:{
			
		}
		case STATE_FORWARD_BRAKE:{
			
		}
		case STATE_REVERSE:{
			
		}
		case STATE_REVERSE_BRAKE:{
			
		}
		case STATE_IDLE:{
			// create state transition conditions	
		}
	}
}
