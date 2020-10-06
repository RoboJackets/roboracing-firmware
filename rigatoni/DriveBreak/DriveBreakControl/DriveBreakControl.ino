#include "DriveBrake.h"
#include "RJNet.h"
#include <Ethernet.h>
//#include <sstream>

#define NUM_MAGNETS 24
#define WHEEL_DIA 0.27305 //meters
#define MILLIS_PER_SEC 1000
#define PI_M 3.14159

#define ETH_NUM_SENDS 2
#define ETH_RETRANSMISSION_DELAY_MS 50  //Time before TCP tries to resend the packet
#define ETH_TCP_INITIATION_DELAY 50 //How long we wait before .connected() or .connect() fails.

#define REPLY_TIMEOUT_MS 400 //We have to receive a reply from Estop and Brake within this many MS of our message or we are timed out. NOT TCP timeout. Have to 
#define MIN_MESSAGE_SPACING 100  //Send messages to Brake and request state from e-stop at 10 Hz

/****************ETHERNET****************/
// See https://docs.google.com/document/d/10klaJG9QIRAsYD0eMPjk0ImYSaIPZpM_lFxHCxdVRNs/edit#
////

// Enter a MAC address and IP address for your board below
const static byte driveMAC[] = {0xDE, 0xAD, 0xBA, 0xEF, 0xFE, 0xEE};
const static int PORT = 7;  //port RJNet uses

const static IPAddress driveIP(192, 168, 0, 4); //set the IP to find us at
EthernetServer server(PORT);

// Our two clients: 
const static IPAddress brakeIP(192, 168, 0, 5); //set the IP of the brake
EthernetClient brakeBoard;  // client 

const static IPAddress estopIP(192, 168, 0, 3); //set the IP of the estop
EthernetClient estopBoard;  // client

//Manual board, which is not a client:
const static IPAddress manualIP(192, 168, 0, 6); //set the IP of the estop

/****************Messages from clients****************/
//Universal acknowledge message
const static String ackMsg = "R";

//Possible messages from e-stop
const static String estopStopMsg = "D";
const static String estopLimitedMsg = "L";
const static String estopGoMsg = "G";

//Speed request message
const static String speedRequestMsg = "S?";

//Manual RC speed command header
const static String manualStringHeader = "v=";

//Timestamps of replies from boards in ms
unsigned long lastEstopReply = 0;
unsigned long lastBrakeReply = 0;
unsigned long lastManualCommand = 0;
//Timestamps of our last messages to boards in ms
unsigned long estopRequestSent = 0;
unsigned long brakeCommandSent = 0;

/*
State Machine

We are disabled if:
 - Estop says we are
 - timed out waiting for message from estop, brake, or manual
 
Forward state if commanded velocity >=0. Backward if velocity < 0.
*/

// State machine possible states
enum ChassisState {
    STATE_DISABLED = 0,
    STATE_FORWARD = 1,
    STATE_REVERSE = 2
}
currentState = STATE_DISABLED;
bool motorEnabled = false;


////
// ENCODER
////

unsigned long loopStartTime = 0; // the time used to run the loop
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

float motorCurrent = 0;


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

    // Current Sensing Pins
	pinMode(CURR_DATA_PIN, INPUT);
    
    Serial.begin(115200);

	//********** Ethernet Initialization *************//
    resetEthernet();
    
	Ethernet.init(ETH_CS_PIN); 	// CS pin from eth header
	Ethernet.begin(driveMAC, driveIP); 	// initialize the ethernet device
    Ethernet.setRetransmissionCount(ETH_NUM_SENDS); //Set number resends before failure
    Ethernet.setRetransmissionTimeout(ETH_RETRANSMISSION_DELAY_MS);  //Set timeout delay before failure
    
	while (Ethernet.hardwareStatus() == EthernetNoHardware) {
		Serial.println("Ethernet shield was not found.");
		delay(50);
	}
	while(Ethernet.linkStatus() == LinkOFF) {
		Serial.println("Ethernet cable is not connected.");	// do something with this
		delay(50);	 	// TURN down delay to check/startup faster
	}
    
    server.begin();	// launches OUR server
    Serial.println("Server started");
    
    //Make sure we are connected to clients before proceeding.
    estopBoard.setConnectionTimeout(ETH_TCP_INITIATION_DELAY);
    brakeBoard.setConnectionTimeout(ETH_TCP_INITIATION_DELAY);
    
    byte estopConnected = estopBoard.connect(estopIP, PORT);
    byte brakeConnected = brakeBoard.connect(brakeIP, PORT);
    
    while(!estopConnected || !brakeConnected){
        if(!estopConnected){
            Serial.println("Estop not connected");
            estopConnected = estopBoard.connect(estopIP, PORT);
        }
        if(!brakeConnected){
            Serial.println("Brake not connected");
            estopConnected = brakeBoard.connect(brakeIP, PORT);
        }
    }

    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), HallEncoderInterrupt, FALLING);
}


void loop() {
    loopStartTime = millis();
    
	// Read new messages from WizNet and make necessary replies
    readAllNewMessages();
	
    currentSpeed = CalcCurrentSpeed();
    Serial.println(currentSpeed);

    executeStateMachine();

}

////
// ETHERNET FUNCTIONS
////

bool parseEstopMessage(const String msg){
    /*
    Parse a message from the estop board and determine if the drive motor is enabled or disabled.
    If state is GO, motor enabled; in all other cases, motor is disabled.
    */
    return estopGoMsg.equals(msg);
}

float parseSpeedMessage(const String speedMessage){
    //takes in message from manual and converts it to a float desired speed.
    return speedMessage.substring(2).toFloat();
}

void readAllNewMessages(){
    /*
    Reads all messages that have come in since we last checked and deals with them.
    If it is a request for the current speed, reply with the speed.
    Else, see if it is Estop telling us the state
    or Manual telling us the velocity
    or Brake acknowledging a command
    */
    EthernetClient client = server.available();  //if there is a new message form client create client object, otherwise new client object null
    while(client){
        String data = RJNet::readData(client);  //RJNet handles getting complete message
        
        if(data.length() != 0){  //Got valid data - string will be empty if message not valid
            if(speedRequestMsg.equals(data)){  //Somebody's asking for our speed
                //Send back our current speed
                RJNet::sendData(client, "v=" + String(currentSpeed) + " I=" + String(motorCurrent));
            }
            else if(estopIP == client.remoteIP()){
                //Message from Estop board
                motorEnabled = parseEstopMessage(data);
                lastEstopReply = millis();
            }
            else if(brakeIP == client.remoteIP()){
                //Message from brake board, should be acknoweldge
                if(ackMsg.equals(data)){
                    lastBrakeReply = millis();
                }
                else{
                    Serial.print("Invalid message from brake: ");
                    Serial.println(data);
                }
            }
            else if(manualIP == client.remoteIP()){
                //Message from manual board, should be speed command
                if(data.length() > 2){
                    if(manualStringHeader.equals(data.substring(0,2))){
                        desiredSpeed = parseSpeedMessage(data);
                        lastManualCommand = millis();
                        
                        //Acknowledge receipt of message
                        RJNet::sendData(client, ackMsg);
                    }
                    else{
                        Serial.print("Invalid message from manual: ");
                        Serial.println(data);
                    }
                }
                else{
                    Serial.print("Invalid message from manual: ");
                    Serial.println(data);
                }
            }
        }
        else{
            Serial.print("Empty/invalid message recieved from ");
            Serial.println(client.remoteIP());
        }
    }
}

void resetEthernet(void){
    //Resets the Ethernet shield
    digitalWrite(ETH_RST_PIN, LOW);
    delay(1);
    digitalWrite(ETH_RST_PIN, HIGH);
    delay(501);
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

float LinearVelocityFromVoltage(float voltage){
  return ((Kv*voltage)/60.0)*inverseGearRatio*wheelCircumference;
}

float VoltageFromLinearVelocity(float velocity){
  return velocity*(60.0/(inverseGearRatio*wheelCircumference*Kv));
}

int MotorPwmFromVelocityPID(float velocity) { 
    if (velocity <= 0) {
        integral = 0;
        return 0;
    } else {
        const float dt = (float) loopStartTime / MILLIS_PER_SEC;
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
	}
}

void runStateDisabled(){
  digitalWrite(FORWARD_OUT_PIN, LOW);
  digitalWrite(REVERSE_OUT_PIN, LOW);
    
}

void runStateTimeout(){
  digitalWrite(FORWARD_OUT_PIN, LOW);
  digitalWrite(REVERSE_OUT_PIN, LOW);

}
void runStateForward(){
  digitalWrite(FORWARD_OUT_PIN, HIGH);
  digitalWrite(REVERSE_OUT_PIN, LOW);
  
}

//void runStateReverse(){}
//void runStateBrake(){}
