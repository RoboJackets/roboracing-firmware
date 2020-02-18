#include "evgp_estop_motherboard.h"

// ALL STACK LIGHT COLOR OUTPUTS NEED TO BE CHANGED!

byte currentState = 0;
byte sensor1;      // input from sensor 1
byte sensor2;      // input from sensor 2
byte steeringIn;   // steering input from radio board
byte driveIn;      // drive input from radio board


void setup() {
  pinMode(INT, INPUT);
  pinMode(SAFE_RB, INPUT);
  pinMode(START_RB, INPUT);
  pinMode(SENSOR_1, INPUT);
  pinMode(SENSOR_2, INPUT);
  pinMode(STEERING_IN, INPUT);
  pinMode(DRIVE_IN, INPUT);
  pinMode(STEERING_EN, OUTPUT);
  pinMode(DRIVE_EN, OUTPUT);
  pinMode(BRAKE_EN, OUTPUT);
  pinMode(STACK_G, OUTPUT);
  pinMode(STACK_Y, OUTPUT);
  pinMode(STACK_R, OUTPUT);
  // pinMode(LED, OUTPUT);
  digitalWrite(DRIVE_EN, LOW);  // Initially start E-stopped
  digitalWrite(BRAKE_EN, LOW);  //
  digitalWrite(STACK_G, HIGH); // change these light settings
  digitalWrite(STACK_Y, HIGH);
  digitalWrite(STACK_R, HIGH); 
}

void loop() {
  sensor1 = digitalRead(SENSOR_1);
  sensor2 = digitalRead(SENSOR_2);
  steeringIn = digitalRead(STEERING_IN);
  driveIn = digitalRead(DRIVE_IN);
  if (!steeringIn && !driveIn){
    currentState = 1;  // everything disabled
  } else if(!steeringIn){
    currentState = 2;  // steering disabled, drive enabled
  } else if(!driveIn){
    currentState = 3;  // everything enabled
  } else {
    currentState = 0;
  }
  executeStateMachine();  
}

void executeStateMachine(){
  switch(currentState) {
    case 0:    // everything running
      steerDriveBrake(HIGH, HIGH, HIGH);
      stackLights(1, 0, 0);  
      break; 
    case 1:    // everything disabled and emergency break enabled
      steerDriveBrake(LOW, LOW, LOW);
      stackLights(0, 0, 1);
      break;
    case 2:    // steering disabled, drive enabled
      steerDriveBrake(LOW, HIGH, LOW);
      stackLights(0, 1, 0);    // CHANGE this
      break;
    case 3:    // steering enabled, drive disabled
      steerDriveBrake(HIGH, LOW, HIGH);
      stackLights(0, 1, 0);    // CHANGE this
      break;
  }
}

void stackLights(byte G, byte Y, byte R){
  digitalWrite(STACK_G, G);
  digitalWrite(STACK_Y, Y);
  digitalWrite(STACK_R, R);
}

void steerDriveBrake(byte steer, byte drive, byte brake){
  digitalWrite(STEERING_EN, steer);
  digitalWrite(DRIVE_EN, drive);
  digitalWrite(BRAKE_EN, brake);
}
