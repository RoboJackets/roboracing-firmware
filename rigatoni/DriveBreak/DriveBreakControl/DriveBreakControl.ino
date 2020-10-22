#include <avr/wdt.h>
#include "DriveBrake.h"
#include "RJNet.h"
#include <Ethernet.h>

#define TIMER_MIN_RESOLUTION_US 8  //Timer minimum resolution of 8 us

#define NUM_MAGNETS 24
#define WHEEL_DIA 0.27305 //meters
const static unsigned long US_PER_SEC = 1000000;
#define PI_M 3.14159

#define ETH_NUM_SENDS 2
#define ETH_RETRANSMISSION_DELAY_MS 50  //Time before TCP tries to resend the packet
#define ETH_TCP_INITIATION_DELAY 50 //How long we wait before .connected() or .connect() fails.

#define REPLY_TIMEOUT_MS 4000 //We have to receive a reply from Estop and Brake within this many MS of our message or we are timed out. NOT TCP timeout. Have to 
#define MIN_MESSAGE_SPACING 100  //Send messages to Brake and request state from e-stop at 10 Hz

/*******FUNCTION HEADERS*****/
byte MotorPwmFromVoltage(double);

/****************ETHERNET****************/
// See https://docs.google.com/document/d/10klaJG9QIRAsYD0eMPjk0ImYSaIPZpM_lFxHCxdVRNs/edit#
////

// Enter a MAC address and IP address for your board below
const static byte driveMAC[] = {0x11, 0xAD, 0xBA, 0xEF, 0xFE, 0xEE};
const static int PORT = 7;  //port RJNet uses

const static IPAddress driveIP(192, 168, 0, 4); //set the IP to find us at
EthernetServer server(PORT);

// Our two clients: 
const static IPAddress brakeIP(192, 168, 0, 5); //set the IP of the brake
EthernetClient brakeBoard;  // client 

const static IPAddress estopIP(192, 168, 0, 3); //set the IP of the estop
EthernetClient estopBoard;  // client

//Manual board, which is not a client:
const static IPAddress manualIP(192, 168, 0, 144); //set the IP of the estop

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

//TCP Connection status to brake and estop
bool estopConnected = false;
bool brakeConnected = false;

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
    STATE_TIMEOUT = 1,
    STATE_FORWARD = 2,
    STATE_REVERSE = 3
}
currentState = STATE_DISABLED;
bool motorEnabled = false;

////
// ENCODER
////

unsigned long lastPIDTimeUS = 0; // the time used to run the loop
volatile unsigned long currEncoderCount = 0;
unsigned long prevEncoderCount = 0;
unsigned long lastSpeedReadTimeUs = 0;

float currentSpeed = 0;
float motorCurrent = 0;

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
const float maxVelocity = 10.0; // maximum velocity in m/s
const float maxVoltage = 19.4; //Maximum allowable voltage to motors

const byte maxSpeedPwm = MotorPwmFromVoltage(maxVoltage); // 50% PWM, 23% Voltage
const byte zeroSpeedPwm = 255; // 100% PWM, 0% Voltage

const byte maxBrakingForce = 255;

float desiredVelocity = 0;
byte brakingForce = 0;


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
    writeReversingContactorForward(true); //Start out with reversing contactor going forwards.

    // Current Sensing Pins
	pinMode(CURR_DATA_PIN, INPUT);
    
    Serial.begin(115200);

	//********** Ethernet Initialization *************//
    resetEthernet();
    
	Ethernet.init(ETH_CS_PIN); 	// CS pin from eth header
	Ethernet.begin(driveMAC, driveIP); 	// initialize the ethernet device
    
	while (Ethernet.hardwareStatus() == EthernetNoHardware) {
		Serial.println("Ethernet shield was not found.");
		delay(50);
	}
	while(Ethernet.linkStatus() == LinkOFF) {
		Serial.println("Ethernet cable is not connected.");	// do something with this
		delay(50);	 	// TURN down delay to check/startup faster
	}
    
    Ethernet.setRetransmissionCount(ETH_NUM_SENDS); //Set number resends before failure
    Ethernet.setRetransmissionTimeout(ETH_RETRANSMISSION_DELAY_MS);  //Set timeout delay before failure
    
    server.begin();	// launches OUR server
    Serial.println("Server started");
    
    //Make sure we are connected to clients before proceeding.
    estopBoard.setConnectionTimeout(ETH_TCP_INITIATION_DELAY);
    brakeBoard.setConnectionTimeout(ETH_TCP_INITIATION_DELAY);
    
    while(!estopConnected || !brakeConnected){
        if(!estopConnected){
            Serial.println("Connecting to Estop");
            estopConnected = estopBoard.connect(estopIP, PORT) > 0;
        }
        if(!brakeConnected){
            Serial.println("Connecting to Brake");
            brakeConnected = brakeBoard.connect(brakeIP, PORT) > 0;
        }
    }

    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), HallEncoderInterrupt, FALLING);
    
    // WATCHDOG TIMER
    wdt_reset();
    wdt_enable(WDTO_500MS);
}

const static int printDelayMs = 1000;
unsigned long lastPrintTime = 0;

void loop() {
    wdt_reset();
    
	// Read new messages from WizNet and make necessary replies
    readAllNewMessages();
	
    //Calculate current speed
    motorCurrent = GetMotorCurrent();
    currentSpeed = CalcCurrentSpeed();
    
    if (millis() > lastPrintTime + printDelayMs){
        Serial.print("Current speed: ");
        Serial.print(currentSpeed);
        Serial.print(" Motor Current: ");
        Serial.println(motorCurrent);
        
        lastPrintTime = millis();
    }

    executeStateMachine();
    
    //Now make requests to estop and brake
    sendToBrakeEstop();
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

void handleSingleClientMessage(EthernetClient otherBoard){
    /*
    Handles a single client with new data. We make no initial distinction about who the otherBoard is - it could be a server connection OR 
    a reply from Estop or Brake where WE have the EthernetClient object
    
    If it is a request for the current speed, reply with the speed.
    Else, see if it is Estop telling us the state
    or Manual telling us the velocity
    or Brake acknowledging a command
    */
    String data = RJNet::readData(otherBoard);  //RJNet handles getting complete message
    
    IPAddress otherIP = otherBoard.remoteIP();
        
    if(data.length() != 0){  //Got valid data - string will be empty if message not valid
        otherBoard.setConnectionTimeout(ETH_TCP_INITIATION_DELAY);   //Set connection delay so we don't hang
        
        if(speedRequestMsg.equals(data)){  //Somebody's asking for our speed
            //Send back our current speed
            RJNet::sendData(otherBoard, "v=" + String(currentSpeed) + " I=" + String(motorCurrent));
            Serial.print("Speed request from");
            Serial.println(otherIP);
        }
        else if(estopIP == otherIP){
            //Message from Estop board
            motorEnabled = parseEstopMessage(data);
            lastEstopReply = millis();
            Serial.print("Estop: motor enabled? ");
            Serial.println(motorEnabled);
        }
        else if(brakeIP == otherIP){
            //Message from brake board, should be acknoweldge
            if(ackMsg.equals(data)){
                lastBrakeReply = millis();
            }
            else{
                Serial.print("Invalid message from brake: ");
                Serial.println(data);
            }
        }
        else if(manualIP == otherIP){
            //Message from manual board, should be speed command
            if(data.length() > 2){
                if(manualStringHeader.equals(data.substring(0,2))){
                    desiredVelocity = parseSpeedMessage(data);
                    lastManualCommand = millis();
                    
                    //Acknowledge receipt of message
                    RJNet::sendData(otherBoard, ackMsg);
                    
                    Serial.print("New target speed: ");
                    Serial.println(desiredVelocity);
                }
                else{
                    Serial.print("Wrong prefix from manual: ");
                    Serial.println(data);
                }
            }
            else{
                Serial.print("Too short message from manual: ");
                Serial.println(data);
            }
        }
    }
    else{
        Serial.print("Empty/invalid message received from ");
        Serial.println(otherIP);
    }
}

void readAllNewMessages(){
    /*
    Checks the server, brake, and Estop for new messages and deals with them
    */
    EthernetClient client = server.available();  //if there is a new message form client create client object, otherwise new client object null
    while(client){
        handleSingleClientMessage(client);
        client = server.available();  //Go to next message
    }
    
    while(brakeBoard.available()){
        handleSingleClientMessage(brakeBoard);
    }
    
    while(estopBoard.available()){
        handleSingleClientMessage(estopBoard);
    }
}

void sendToBrakeEstop(void){
    /*
    Checks our connection to the brake and estop and attempts to reconnect if we are disconnected.
    If we are connected and sufficient time has elapsed both from our last request and their last reply, send new request to estop/brake
    */
    brakeConnected = brakeBoard.connected();
    if(!brakeConnected){
        //Lost TCP connection with the brake board. Takes 10 seconds for TCP connection to fail after disconnect.
        //Takes a very long time to time out
        brakeBoard.connect(brakeIP, PORT);
        Serial.println("Lost connection with brakes");
    }
    else{
        if(millis() > lastBrakeReply + MIN_MESSAGE_SPACING && millis() > brakeCommandSent + MIN_MESSAGE_SPACING){
            RJNet::sendData(brakeBoard, "B=" + String(brakingForce));
            brakeCommandSent = millis();
        }
    }
    
    estopConnected = estopBoard.connected();
    if(!estopConnected){
        //Lost TCP connection with the estop board
        estopBoard.connect(estopIP, PORT);
        Serial.println("Lost connection with estop");
    }
    else{
        if(millis() > lastEstopReply + MIN_MESSAGE_SPACING && millis() > estopRequestSent + MIN_MESSAGE_SPACING){
            RJNet::sendData(estopBoard, "S?");
            estopRequestSent = millis();
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
// STATE MACHINE FUNCTIONS
////

void executeStateMachine(){ 
    //Check what state we are in at the moment.
    //We are disabled if the motor is disabled, or any of our clients has timed out 
    unsigned long currTime = millis();
    if(!motorEnabled){
        currentState = STATE_DISABLED;
    }
    else if((!brakeConnected || currTime > lastBrakeReply + REPLY_TIMEOUT_MS) || (!estopConnected || currTime > lastEstopReply + REPLY_TIMEOUT_MS) || currTime > lastManualCommand + REPLY_TIMEOUT_MS){
        currentState = STATE_TIMEOUT;
    }
    else{
        if(desiredVelocity < 0){
            currentState = STATE_REVERSE;
        }
        else{
            currentState = STATE_FORWARD;
        }
    }
    //Now execute that state
	switch(currentState) { 
		case STATE_DISABLED:{
			runStateDisabled();
			break;
		}
        case STATE_TIMEOUT:{
			runStateTimeout();
			break;
		}
		case STATE_FORWARD:{
			runStateForward();
			break;
		}
		case STATE_REVERSE:{
			runStateReverse();
			break;
		}
	}
}

void runStateDisabled(){
    /*
    Motor off, brakes engaged. Do't care what reversing contactor is doing.
    */
    Serial.println("Motor disabled");
    writeMotorOff();
    brakingForce = maxBrakingForce;
}
void runStateTimeout(){
    /*
    Motor off, brakes disengaged. Don't care what reversing contactor is doing.
    */
    unsigned long currTime = millis();
    
    if (!brakeConnected){
        Serial.println("Lost brake TCP connection");
    }
    else if(currTime > lastBrakeReply + REPLY_TIMEOUT_MS){
        Serial.println("Brake timed out");
    }
    
    if (!estopConnected){
        Serial.println("Lost estop TCP connection");
    }
    else if(currTime > lastEstopReply + REPLY_TIMEOUT_MS){
        Serial.println("Estop timed out");
    }
    
    if (currTime > lastManualCommand + REPLY_TIMEOUT_MS){
        Serial.println("Manual timed out");
    }
    
    writeMotorOff();
    brakingForce = 0;
}

void runStateForward(){
    /*
    Going forward in normal operation.
    */
    writeReversingContactorForward(true);
    
    //TODO: controls math here
    motorTest(0.5);
    brakingForce = 0;
}

void runStateReverse(){
    /*
    Going reversed in normal operation.
    */
    writeReversingContactorForward(false);
    
    //TODO: controls math here
    motorTest(0.5);
    brakingForce = 0;
}


void writeReversingContactorForward(bool forward){
    /*
    Controls the reversing contactor. True means go forward, False means go backwards
    */
    digitalWrite(FORWARD_OUT_PIN, forward);
    digitalWrite(REVERSE_OUT_PIN, !forward);
}


////
// ENCODER FUNCTIONS
////

void HallEncoderInterrupt(){
    currEncoderCount++;
}

float CalcCurrentSpeed() {
    unsigned long currTime = micros();
    unsigned long timeDelta = max(currTime - lastSpeedReadTimeUs, TIMER_MIN_RESOLUTION_US);
    
    float val = (((float) currEncoderCount - prevEncoderCount)/timeDelta) * US_PER_SEC * PI_M * WHEEL_DIA/NUM_MAGNETS;
    prevEncoderCount = currEncoderCount;
    lastSpeedReadTimeUs = currTime;
    return val;
}

////
// MOTOR FUNCTIONS
////

// Percentage 0.0 to 1.0 of max speed 
void motorTest(float percentage) { //open loop test
    if(percentage > 0 && percentage < 1.0)
    {
        byte writePercent = constrain(MotorPwmFromVoltage(maxVoltage*percentage),maxSpeedPwm,zeroSpeedPwm);
        analogWrite(MOTOR_CNTRL_PIN, writePercent);
    }
    else
    {
        analogWrite(MOTOR_CNTRL_PIN, zeroSpeedPwm);
    }
}

//Write voltage to motor
void writeMotorOff(void){
    //Disables the motor by writing correct voltage
    analogWrite(MOTOR_CNTRL_PIN, zeroSpeedPwm);
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
        unsigned long currTime = micros();
        
        const float dt = ((float) min(currTime - lastPIDTimeUS, TIMER_MIN_RESOLUTION_US)) / US_PER_SEC;
        float error = velocity - (float)CalcCurrentSpeed();

        // protect against runaway integral accumulation
        float trapezoidIntegral = (prevError + error) * 0.5 * dt;
        if (abs(kI * integral) < maxSpeedPwm - zeroSpeedPwm || trapezoidIntegral < 0) {
            integral += trapezoidIntegral;
        }

        derivative = (error - prevError) / dt; //difference derivative

        float velocityOutput = kP * error + kI * integral + kD * derivative;
        int pwmCalculatedOutput = MotorPwmFromVoltage(VoltageFromLinearVelocity(velocityOutput)); 
        int writePwm = constrain(pwmCalculatedOutput, maxSpeedPwm, zeroSpeedPwm); //control limits *NOTE PWM is reverse (255 is min speed, 0 is max speed)

        prevError = error;
        lastPIDTimeUS = currTime;
        return writePwm;
    }
}

////
// CURRENT SENSE FUNCTIONS
////

const static float currentSensorMidpoint = 500.0;
const static int adcResolution = 1024;
const static float adcMaxVoltage = 5.0;
const static float currentSensorAmpsPerVolt = 200.0;
const static float ampsPerBit = currentSensorAmpsPerVolt*adcMaxVoltage/adcResolution;

float GetMotorCurrent(){
    unsigned int rawCurrentBits = analogRead(CURR_DATA_PIN);
    return (ampsPerBit*rawCurrentBits) - currentSensorMidpoint; // 200A/V * current voltage -> gives A
}