#include "DriveBrake.h"
#include "RJNet.h"
#include <Ethernet.h>

#define NUM_MAGNETS 24
#define WHEEL_DIA 0.27305 //meters
#define MILLIS_PER_SEC 1000
#define PI_M 3.14159


////
// STATE MACHINE
////

// State machine possible states
enum ChassisState {
    STATE_DISABLED = 0,
    STATE_TIMEOUT = 1,
    STATE_FORWARD = 2,
    STATE_REVERSE = 3,
    STATE_BRAKE = 4
} 
currentState = STATE_DISABLED;

////
// ETHERNET
////

// Enter a MAC address and IP address for your board below
const byte driveMAC[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE };
const static int PORT = 7;  //port RJNet uses

IPAddress driveIP(192, 168, 0, 177); //set the IP to find us at
EthernetServer server(PORT);

// Enter a IP address for other board below
IPAddress otherIP(192, 168, 0, 178); //set the IP of the NUC
EthernetClient otherBoard;  // client 



////
// ENCODER
////

unsigned long startTime = 0; // the time used to run the loop
volatile unsigned long currEncoderCount = 0;
unsigned long prevEncoderCount = 0;
unsigned long prevMillis = 0;

////
// MOTOR
////
const float Kv = 75.7576; // RPM/V
const float inverseGearRatio = 1/2.909; // 64 / 22
const float wheelCircumference = WHEEL_DIA*PI_M;
// PID Speed Control
float integral = 0.0;
float derivative = 0.0;
float prevError = 0.0;
// 2 second response time, Transient Behaviour of 0.9
const float kP = 220.4;
const float kI = 22.5;
const float kD = 22.5;

// Control Limits
const float maxVelocity = 10.0; // maximum velocity in 
const int maxSpeedPwm = 127; // 50% PWM, 23% Voltage
const int minSpeedPwm = 255; // 100% PWM, 0% Voltage
const float maxVoltage = 19.4; // 

float desiredSpeed = 0;  
float currentSpeed = 0;


void setup(){
  // Ethernet Pins
	pinMode(ETH_INT_PIN, INPUT);
  pinMode(ETH_RST_PIN, OUTPUT);
  pinMode(ETH_CS_PIN, OUTPUT);

	// Encoder Pins
	pinMode(ENCODER_A_PIN, INPUT);
  
  // LED Pins
	pinMode(REVERSE_LED_PIN, OUTPUT);

  // Motor Pins
	pinMode(MOTOR_CNTRL_PIN, OUTPUT);
  pinMode(FORWARD_OUT_PIN, OUTPUT);
  pinMode(REVERSE_OUT_PIN, OUTPUT);

  // TODO BRAKE NO LONGER SERVO, CHANGING INTERFACE
  // Brake Pins
  // pinMode(BRAKE_EN, OUTPUT);
	// pinMode(BRAKE_PWM, OUTPUT);

  // Current Sensing Pins
	pinMode(CURR_DATA_PIN, INPUT);


  resetEthernet();

	//********** Ethernet Initialization *************//
	Ethernet.init(ETH_CS_PIN); 	// CS pin from eth header
	Ethernet.begin(driveMAC, driveIP); 	// initialize the ethernet device
	/* while (Ethernet.hardwareStatus() == EthernetNoHardware) {
		Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
		delay(500);
	}
	while(Ethernet.linkStatus() == LinkOFF) {
		Serial.println("Ethernet cable is not connected.");	// do something with this
		delay(500);	 	// TURN down delay to check/startup faster
	} */
	server.begin();	// launches server

  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), HallEncoderInterrupt, FALLING);
}


void loop() {
	// put your main code here, to run repeatedly:
  startTime = millis();
	// Ethernet stuff
	
	getSpeedMessage();
	
  currentSpeed = CalcCurrentSpeed();
  Serial.println(currentSpeed);

  executeStateMachine();


  
  delay(100);
}

////
// ETHERNET FUNCTIONS
////

void getSpeedMessage()
{
  EthernetClient client = server.available();    // if there is a new message form client create client object, otherwise new client object null
  if (client) {
    String data = RJNet::readData(client);  // if i get string from RJNet buffer ($speed_value;)
    if (data.length() != 0) {     // if data exists (?)
      // get data from nuc/manual board/E-Stop (?) and do something with it
      desiredSpeed = data.toFloat();  // convert from string to int potentially
      // String reply = "$" + String(currentSpeed) + ";";
    String reply = String(currentSpeed);
      RJNet::sendData(client, reply);
    }
  }
}

void resetEthernet(void){
    //Resets the Ethernet shield
    digitalWrite(ETH_RST_PIN, LOW);
    delay(500);
}


////
// ENCODER FUNCTIONS
////

void HallEncoderInterrupt(){
  currEncoderCount++;
}

float CalcCurrentSpeed() {
  float val = ((float)(currEncoderCount - prevEncoderCount)/(millis() - prevMillis)) * MILLIS_PER_SEC * PI_M * WHEEL_DIA/NUM_MAGNETS;
  prevEncoderCount = currEncoderCount;
  prevMillis = millis();
  return val;
}

////
// MOTOR FUNCTIONS
////

// Percentage 0.0 to 1.0 of max speed 
void motorTest(float percentage) { //open loop test
  if(percentage > 0 && percentage < 1.0)
  {
    byte writePercent = constrain(MotorPwmFromVoltage(maxVoltage*percentage),maxSpeedPwm,minSpeedPwm);
    analogWrite(MOTOR_CNTRL_PIN, writePercent);
  }
  else
  {
    analogWrite(MOTOR_CNTRL_PIN, minSpeedPwm);
  }
}

byte MotorPwmFromVoltage(double voltage){
  return (byte)((0.556 - sqrt(0.556*0.556 - 0.00512*(62.1-voltage)))/0.00256); //see "rigatoni PWM to motor power" in drive for equation
}

float LinearVelocityFromVoltage(float voltage)
{
  return ((Kv*voltage)/60.0)*inverseGearRatio*wheelCircumference;
}

float VoltageFromLinearVelocity(float velocity)
{
  return velocity*(60.0/(inverseGearRatio*wheelCircumference*Kv));
}

int MotorPwmFromVelocityPID(float velocity) { 
    if (velocity <= 0) {
        integral = 0;
        return 0;
    } else {
        const float dt = (float) startTime / MILLIS_PER_SEC;
        float error = velocity - (float)CalcCurrentSpeed();

        // protect against runaway integral accumulation
        float trapezoidIntegral = (prevError + error) * 0.5 * dt;
        if (abs(kI * integral) < maxSpeedPwm - minSpeedPwm || trapezoidIntegral < 0) {
            integral += trapezoidIntegral;
        }

        derivative = (error - prevError) / dt; //difference derivative

        float velocityOutput = kP * error + kI * integral + kD * derivative;
        int pwmCalculatedOutput = MotorPwmFromVoltage(VoltageFromLinearVelocity(velocityOutput)); 
        int writePwm = constrain(pwmCalculatedOutput, maxSpeedPwm, minSpeedPwm); //control limits *NOTE PWM is reverse (255 is min speed, 0 is max speed)

        prevError = error;
        return writePwm;
    }
}

////
// CURRENT SENSE FUNCTIONS
////

unsigned int GetMotorCurrent(){
  unsigned int currentAnalog = analogRead(CURR_DATA_PIN);
  return (200*(5/1024)*currentAnalog) - 500; // 200A/V * 5V/currentAnalog -> gives A
}

////
// STATE MACHINE FUNCTIONS
////

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
		case STATE_REVERSE:{
			runStateReverse();
			//transitions
			currentState = STATE_REVERSE;
			break;
		}
		case STATE_BRAKE:{
			runStateBrake();
			//transitions
			currentState = STATE_BRAKE;
			break;
		}
	}
}

void runStateDisabled(){}
void runStateTimeout(){}
void runStateForward(){}
void runStateReverse(){}
void runStateBrake(){}
