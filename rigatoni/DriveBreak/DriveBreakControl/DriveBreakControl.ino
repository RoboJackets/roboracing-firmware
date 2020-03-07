#include "DriveBrake.h"
#include "RJNet.h"
#include <Ethernet.h>

#define NUM_MAGNETS 24
#define WHEEL_DIA 0.27305 //meters

const static int PORT = 7;  //port RJNet uses

volatile long encoder0Pos = 0;
long encPrevPos = 0;
float currentSpeed;
long prevMillis;
long count = 0;

float desiredSpeed = 0;	
float maxSpeed = 10; //m/s




// State machine possible states
enum ChassisState {
    STATE_DISABLED = 0,
    STATE_TIMEOUT = 1,
    STATE_FORWARD = 2,
    STATE_FORWARD_BRAKING = 3,
    STATE_REVERSE = 4,
    STATE_REVERSE_BRAKING = 5,
    STATE_IDLE = 6
} 
currentState = STATE_DISABLED;

// Enter a MAC address and IP address for your board below
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE };
IPAddress ip(192, 168, 0, 177); //set the IP to find us at
EthernetServer server(PORT);

// Enter a IP address for other board below
IPAddress otherIP(192, 168, 0, 178); //set the IP of the NUC
EthernetClient otherBoard;	// client 


void setup(){
	pinMode(INT_ETH, OUTPUT);
	
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

	//********** Ethernet Initialization *************//
	Ethernet.init(INT_ETH); 	// SCLK pin from eth header
	Ethernet.begin(mac, ip); 	// initialize the ethernet device
	/* while (Ethernet.hardwareStatus() == EthernetNoHardware) {
		Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
		delay(500);
	}
	while(Ethernet.linkStatus() == LinkOFF) {
		Serial.println("Ethernet cable is not connected.");	// do something with this
		delay(500);	 	// TURN down delay to check/startup faster
	} */
	server.begin();	// launches server

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), doEncoder, FALLING);
}


void loop() {
	// put your main code here, to run repeatedly:
	// Ethernet stuff
	EthernetClient client = server.available();		// if there is a new message form client create client object, otherwise new client object null
	if (client) {
		String data = RJNet::readData(client);	// if i get string from RJNet buffer ($speed_value;)
		if (data.length() != 0) {			// if data exists (?)
			// get data from nuc/manual board/E-Stop (?) and do something with it
			desiredSpeed = data.toFloat();	// convert from string to int potentially
      // String reply = "$" + String(currentSpeed) + ";";
	  String reply = String(currentSpeed);
      RJNet::sendData(client, reply);
		}
	}
	
	//motorTest(desiredSpeed);
  currentSpeed = calcSpeed();
  Serial.println(currentSpeed);
  
  delay(100);
}

void motorTest(float speed) { //open loop test
	analogWrite(MOTOR_CONTROL, (int)(speed/maxSpeed)*255);
}


double accelerate(double a){
  
}

double brake(double targetSpeed){
  
}

/*
double getSpeed(){  
  digitalWrite(ENCODER_A, HIGH);
  digitalWrite(ENCODER_B, HIGH);
  //encoder counts 64 every revolution
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), doEncoder, RISING); // interrupts encoderpin in 19
  int newposition = encoder0Pos;
  int newtime = (0.001*millis());
  int counts = (newposition-oldposition)/(newtime-oldtime);
  

}
*/

void doEncoder(){
  /*
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
  */

  encoder0Pos++;
  count++;
  // Serial.println(count);
}

float calcSpeed() {
  float val = ((float)(encoder0Pos - encPrevPos)/(millis() - prevMillis)) * 1000* 3.1415 * WHEEL_DIA/NUM_MAGNETS;
  encoder0Pos = encPrevPos;
  prevMillis = millis();
  return val;
}


void setMotorPWM(double voltage){
 byte PWM = ((0.556 - sqrt(0.556*0.556 - 0.00512*(62.1-voltage)))/0.00256); //see "rigatoni PWM to motor power" in drive for equation
 analogWrite(MOTOR_CONTROL, PWM);
}

int getCurrent(){
  int counts = analogRead(CURR_DATA);
  return (200*(5/1024)*counts) - 500; // 200A/V * 5V/count -> gives A
}

/*

void executeStateMachine(){ 
	switch(currentState) { 
		case STATE_DISABLED:{
			runStateDisabled();
			//transitions
			currentState = STATE_DISABLED;
			break;
		}
		case STATE_TIMEOUT:{
			runStateTimeout();
			//transitions
			currentState = STATE_TIMEOUT;
			break;
		}
		case STATE_FORWARD:{
			runStateForward();
			//transitions
			currentState = STATE_FORWARD;
			break;
		}
		case STATE_FORWARD_BRAKE:{
			runStateForwaredBrake();
			//transitions
			currentState = STATE_FORWARD_BRAKE;
			break;
		}
		case STATE_REVERSE:{
			runStateReverse();
			//transitions
			currentState = STATE_REVERSE;
			break;
		}
		case STATE_REVERSE_BRAKE:{
			runStateReverseBrake();
			//transitions
			currentState = STATE_REVERSE_BRAKE;
			break;
		}
		case STATE_IDLE:{
			runStateIdle();
			//transitions
			currentState = STATE_IDLE;
			break;	
		}
	}
}

*/
void runStateDisabled(){}
void runStateTimeout(){}
void runStateForward(){}
void runStateForwaredBrake(){}
void runStateReverse(){}
void runStateReverseBrake(){}
void runStateIdle(){}
