#include "RigatoniNetwork.h"
#include <avr/wdt.h>
#include "DriveControl.h"
#include "RJNet.h"
#include "controller_estimator.h"
#include <Ethernet.h>

#define NUM_MAGNETS 24
const static float US_PER_SEC = 1000000.0;

#define ETH_NUM_SENDS 2
#define ETH_RETRANSMISSION_DELAY_MS 50  //Time before TCP tries to resend the packet
#define ETH_TCP_INITIATION_DELAY 50     //How long we wait before .connected() or .connect() fails.

#define REPLY_TIMEOUT_MS 4000   //We have to receive a reply from Estop and Brake within this many MS of our message or we are timed out. NOT TCP timeout. Have to 
#define MIN_MESSAGE_SPACING 100 //Send messages to Brake and request state from e-stop at 10 Hz

/****************ETHERNET****************/
// See https://docs.google.com/document/d/10klaJG9QIRAsYD0eMPjk0ImYSaIPZpM_lFxHCxdVRNs/edit#
////

// Enter a MAC address and IP address for your board below


//const static IPAddress driveIP(192, 168, 0, 4); //set the IP to find us at
EthernetServer server(PORT);

// Our two clients: 
//const static IPAddress brakeIP(192, 168, 0, 7); //set the IP of the brake
EthernetClient brakeBoard;  // client 

//const static IPAddress estopIP(192, 168, 0, 3); //set the IP of the estop
EthernetClient estopBoard;  // client

//Manual board, which is not a client:
//const static IPAddress manualIP(192, 168, 0, 144); //set the IP of the estop

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
    STATE_DISABLED_FORWARD = 0,
    STATE_DRIVING_FORWARD = 1,
    STATE_DISABLED_REVERSE = 2,
    STATE_DRIVING_REVERSE = 3
}
currentState = STATE_DISABLED_FORWARD;
bool motorEnabled = false;

unsigned long lastControllerRunTime = 0;    //Last time the controller and speed estimator was executed

float motorCurrent = 0;                 //Current passing through the motor (can be + or -, depending on direction)
float desired_braking_force = 0;        //Desired braking force in N (should be >=0)
float softwareDesiredVelocity = 0;      //What Software is commanding us

//Motor translation parameters
static const float batteryVoltage = 48.0;
static const byte maxSpeedPwm = 255;
static const byte zeroSpeedPwm = 0; // 100% PWM, 0% Voltage

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
		delay(100);
	}

	while(Ethernet.linkStatus() == LinkOFF) {
		Serial.println("Ethernet cable is not connected.");	// do something with this
		delay(100);	 	// TURN down delay to check/startup faster
	}
    
    Ethernet.setRetransmissionCount(ETH_NUM_SENDS); //Set number resends before failure
    Ethernet.setRetransmissionTimeout(ETH_RETRANSMISSION_DELAY_MS);  //Set timeout delay before failure
    
    server.begin();	// launches OUR server
    Serial.println("Server started");
    Serial.println(Ethernet.localIP());


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
        delay(100);
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
	
    //How long since the last controller execution
    unsigned long currTime = micros();
    float loopTimeStep = (currTime - lastControllerRunTime)/US_PER_SEC;
    lastControllerRunTime = currTime;
    
    //Calculate current speed
    motorCurrent = GetMotorCurrent();
    estimate_vel(loopTimeStep, motorCurrent, desired_braking_force);

    executeStateMachine(loopTimeStep);
    
    //Print diagnostics
    if (millis() > lastPrintTime + printDelayMs){
        Serial.print("Current speed: ");
        Serial.print(get_speed());
        Serial.print(" Motor Current: ");
        Serial.println(motorCurrent);
        
        lastPrintTime = millis();
    }
    
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
            RJNet::sendData(otherBoard, "v=" + String(get_speed()) + " I=" + String(motorCurrent));
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
                    softwareDesiredVelocity = parseSpeedMessage(data);
                    lastManualCommand = millis();
                    
                    //Acknowledge receipt of message
                    RJNet::sendData(otherBoard, ackMsg);
                    
                    Serial.print("New target speed: ");
                    Serial.println(softwareDesiredVelocity);
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
            RJNet::sendData(brakeBoard, "B=" + String(desired_braking_force));
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

void executeStateMachine(float timeSinceLastLoop){ 
    //Check what state we are in at the moment.
    //We are disabled if the motor is disabled, or any of our clients has timed out 
    unsigned long currTime = millis();
    
    if(!motorEnabled){
        Serial.println("Motor disabled");
    }
    
    //Check if we are connected to all boards
    bool connected_to_all_board = true;
    if(!brakeConnected){
        Serial.println("Lost brake TCP connection");
        connected_to_all_board = false;
    }
    else if(currTime > lastBrakeReply + REPLY_TIMEOUT_MS){
        Serial.println("Brake timed out");
        connected_to_all_board = false;
    }
    else if(!estopConnected){
        Serial.println("Lost estop TCP connection");
        connected_to_all_board = false;
    }
    else if(currTime > lastEstopReply + REPLY_TIMEOUT_MS){
        Serial.println("Estop timed out");
        connected_to_all_board = false;
    }
    else if(currTime > lastManualCommand + REPLY_TIMEOUT_MS){
        Serial.println("Manual timed out");
        connected_to_all_board = false;
    }
    
    //State transitions. See .gv file.
    if(currentState == STATE_DRIVING_FORWARD){
        if(!motorEnabled || !connected_to_all_board){
            currentState = STATE_DISABLED_FORWARD;
        }
        else if(softwareDesiredVelocity < 0 && get_speed() < switch_direction_max_speed){
            currentState = STATE_DRIVING_REVERSE;
        }
    }
    else if(currentState == STATE_DRIVING_REVERSE){
        if(!motorEnabled || !connected_to_all_board){
            currentState = STATE_DISABLED_REVERSE;
        }
        else if(softwareDesiredVelocity > 0 && get_speed() < switch_direction_max_speed){
            currentState = STATE_DRIVING_FORWARD;
        }
    }
    else if(currentState == STATE_DISABLED_REVERSE){
        if(motorEnabled && connected_to_all_board){
            currentState = STATE_DRIVING_REVERSE;
        }
    }
    else{
        //State is STATE_DISABLED_FORWARD
        if(motorEnabled && connected_to_all_board){
            currentState = STATE_DRIVING_FORWARD;
        }
    }
    
    //Now execute that state
	switch(currentState) { 
		case STATE_DISABLED_FORWARD:{
			runStateDisabled();
			break;
		}
        case STATE_DISABLED_REVERSE:{
			runStateDisabled();
			break;
		}
		case STATE_DRIVING_FORWARD:{
			runStateForward(timeSinceLastLoop);
			break;
		}
		case STATE_DRIVING_REVERSE:{
			runStateReverse(timeSinceLastLoop);
			break;
		}
	}
}

void runStateDisabled(){
    /*
    Motor off, brakes engaged. Don't care what reversing contactor is doing.
    */
    writeMotorOff();
    desired_braking_force = maxBrakingForce;
    
    //Reset speed filter so we don't have problems coming out of this state
    reset_controller(get_speed());
}

void runStateForward(float timestep){
    /*
    Going forward in normal operation.
    */
    writeReversingContactorForward(true);
    
    float capped_target_speed = max(0, softwareDesiredVelocity);    //If softwareDesiredVelocity < 0, then this is 0, so we will brake in preparation for going backwards
    
    //controls math here
    FloatPair voltage_and_braking_force = gen_control_voltage_brake_force(timestep, get_speed(), capped_target_speed);
    float desired_motor_voltage = voltage_and_braking_force.first;
    desired_braking_force = voltage_and_braking_force.second;
    
    writeVoltageToMotor(desired_motor_voltage);
}

void runStateReverse(float timestep){
    /*
    Going reversed in normal operation.
    Remeber our controller controls SPEED and disregards direction. So to go backwards we flip the reversing contactor and make the speed positive
    */
    writeReversingContactorForward(false);
    
    float capped_target_speed = max(0, -softwareDesiredVelocity);    //If softwareDesiredVelocity > 0, then this is 0, so we will brake in preparation for going forwards
    
    //controls math here
    FloatPair voltage_and_braking_force = gen_control_voltage_brake_force(timestep, get_speed(), capped_target_speed);
    float desired_motor_voltage = voltage_and_braking_force.first;
    desired_braking_force = voltage_and_braking_force.second;
    
    writeVoltageToMotor(desired_motor_voltage);
}


void writeReversingContactorForward(bool forward){
    /*
    Controls the reversing contactor. True means go forward, False means go backwards
    */
    digitalWrite(FORWARD_OUT_PIN, forward);
    digitalWrite(REVERSE_OUT_PIN, !forward);
}

////
// Motor hardware interface
////
void writeMotorOff(void){
    //Disables the motor by writing correct voltage
    analogWrite(MOTOR_CNTRL_PIN, zeroSpeedPwm);
}

void writeVoltageToMotor(float voltage){
    if(voltage > batteryVoltage){
        analogWrite(MOTOR_CNTRL_PIN, maxSpeedPwm);
    }
    else if(voltage < 0){
        analogWrite(MOTOR_CNTRL_PIN, zeroSpeedPwm);
    }
    
    byte desiredPWM = (byte) maxSpeedPwm*voltage/batteryVoltage;
    analogWrite(MOTOR_CNTRL_PIN, desiredPWM);
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
