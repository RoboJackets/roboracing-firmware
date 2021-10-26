#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include "RigatoniNetwork.h"
#include <avr/wdt.h>
#include "DriveControl.h"
#include "RJNet.h"
#include "controller_estimator.h"
#include <Ethernet.h>

#define NUM_MAGNETS 9
const static float US_PER_SEC = 1000000.0;


const static unsigned int REPLY_TIMEOUT_MS = 4000;   //We have to receive a reply from Estop and Brake within this many MS of our message or we are timed out. NOT TCP timeout. Have to 
const static unsigned int COMMAND_CONNECTION_TIMEOUT_MS = 500; //If no message from manual in this long, assume auto mode if a message from NUC in this time

/****************ETHERNET****************/
// See https://docs.google.com/document/d/10klaJG9QIRAsYD0eMPjk0ImYSaIPZpM_lFxHCxdVRNs/edit#
////



EthernetServer server(PORT);

// Drive connects to Brake and Estop as a client (Brake and Estop are servers): 
EthernetClient brakeBoard; 
EthernetClient estopBoard;

/****************Messages from clients****************/
//Universal acknowledge message
const static String ackMsg = "R";

//Brake special acknowledge with current braking force
const static String brakeAckHeader = "R=";

//Possible messages from e-stop
const static String estopStopMsg = "D";
const static String estopLimitedMsg = "L";
const static String estopGoMsg = "G";
const static String estopRequestMsg = "S?";

//Speed request message
const static String speedRequestMsg = "S?";

//Manual RC speed command header
const static String speedCommandHeader = "v=";

//Timestamps of replies from boards in ms
unsigned long lastEstopReply = 0;
unsigned long lastBrakeReply = 0;

//Timestamps of the last messages from NUC and manual in MS
unsigned long lastNUCSpeedTime = 0;
unsigned long lastManualSpeedTime = 0;

//Timestamps of our last messages to boards in ms
unsigned long estopRequestSent = 0;
unsigned long brakeCommandSent = 0;

//End of startup. Needed so we don't connect for X seconds
unsigned long endOfStartupTime = 0;

//TCP Connection status to brake and estop
bool estopConnected = false;
bool brakeConnected = false;

/****************Current Sensing****************/
// Current Sensing
const static float currentSensorMidpoint = 502.0;
const static int adcResolution = 1024;
const static float adcMaxVoltage = 5.0;
const static float currentSensorAmpsPerVolt = 200.0;
const static float ampsPerBit = currentSensorAmpsPerVolt*adcMaxVoltage/adcResolution;
float motorCurrent = 0;                 //Current passing through the motor (can be + or -, depending on direction)

//Printing timing
const static int printDelayMs = 300;
unsigned long lastPrintTime = 0;

//For printing only
long totalEncoderTicks = 0;
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

float desired_braking_force = 0;        //Desired braking force in N (should be >=0)

/* We choose target velocity by using
 - manual speed if Manual commanded us recently (in manual mode).
 - NUC speed if NUC commanded us recently and Manual didn't (in autonomous mode).
 - 0 if neither NUC or Manual commanded us recently.
*/
enum currentSpeedCommander{
    MANUAL,
    NUC,
    NOBODY
};
currentSpeedCommander whoIsCommandingSpeed = NOBODY;

float targetVelocity = 0;
float nucTargetVelocity = 0;
float manualTargetVelocity = 0;

Encoder encoder(ENCODER_A_PIN,ENCODER_B_PIN);

//Motor translation parameters
static const float batteryVoltage = 48.0;
static const byte maxSpeedPwm = 130; // Changed to 130 from 255 to scale for motor controller throttle range
static const byte zeroSpeedPwm = 7;  //This much PWM to turn on motor
static const byte motorOffPwm = 0;
byte desiredPWM = motorOffPwm;   //Throttle setting

void setup(){
    // Ethernet Pins
    pinMode(ETH_INT_PIN, INPUT);
    pinMode(ETH_RST_PIN, OUTPUT);
    pinMode(ETH_CS_PIN, OUTPUT);
  
    // LED Pins
    pinMode(LED2_PIN, OUTPUT);
    //pinMode(USER_LED_PIN, OUTPUT); TODO doesn't work on v1.0 of board

    // Encoder Pins. You must call this to turn off the pullup resistors encoder library enables
    pinMode(ENCODER_A_PIN, INPUT);
    pinMode(ENCODER_B_PIN, INPUT);

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
    
    Ethernet.init(ETH_CS_PIN);  // CS pin from eth header
    Ethernet.begin(driveMAC, driveIP);  // initialize the ethernet device
    
    unsigned long loopCounter = 0;
    while(Ethernet.hardwareStatus() == EthernetNoHardware) {
        digitalWrite(LED2_PIN, loopCounter++ % 4 == 0);
        Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
        delay(100);
    }

    while(Ethernet.linkStatus() == LinkOFF) {
        digitalWrite(LED2_PIN, loopCounter++ % 4 > 0);
        Serial.println("Ethernet cable is not connected.");
        delay(100);
    }
    
    Ethernet.setRetransmissionCount(ETH_NUM_SENDS); //Set number resends before failure
    Ethernet.setRetransmissionTimeout(ETH_RETRANSMISSION_DELAY_MS);  //Set timeout delay before failure
    
    server.begin(); // launches OUR server
    Serial.print("Our address: ");
    Serial.println(Ethernet.localIP());

    estopBoard.setConnectionTimeout(ETH_TCP_INITIATION_DELAY);
    brakeBoard.setConnectionTimeout(ETH_TCP_INITIATION_DELAY);
    
    endOfStartupTime = millis();
    
    // WATCHDOG TIMER
    wdt_reset();
    wdt_enable(WDTO_500MS);
    
    //Reset encoder
    encoder.write(0);
}

void loop() {
    wdt_reset();
    digitalWrite(LED2_PIN, !digitalRead(LED2_PIN));
    
    // Read new messages from WizNet and make necessary replies
    readAllNewMessages();
    
    //How long since the last controller execution
    unsigned long currTime = micros();
    float loopTimeStep = (currTime - lastControllerRunTime)/US_PER_SEC;
    lastControllerRunTime = currTime;
    
    //Calculate current speed
    motorCurrent = getMotorCurrent();
    
    long encoderTicksSinceLastLoop = encoder.read();
    encoder.write(0);
    estimate_vel(loopTimeStep, motorCurrent, desired_braking_force, encoderTicksSinceLastLoop);
    totalEncoderTicks += encoderTicksSinceLastLoop;

    executeStateMachine(loopTimeStep);
    
    //Print diagnostics
    if (millis() - lastPrintTime >= printDelayMs){
        Serial.print("Drive speed from: ");
        switch (whoIsCommandingSpeed){
            case MANUAL:
                Serial.print("Manual. ");
                break;
            case NUC:
                Serial.print("NUC. ");
                break;
            case NOBODY:
                Serial.print("Nobody. ");
                break;
        }
        
        Serial.print("Estop: motor ");
        Serial.print(motorEnabled ? "enabled" : "disabled");
        Serial.print(" Manual cmd speed: ");
        Serial.print(targetVelocity);
        Serial.print(" Filtered target speed: ");
        Serial.print(get_curr_target_speed());
        Serial.print(" Throttle: ");
        Serial.print(desiredPWM);
        Serial.print(" Brake force: ");
        Serial.print(desired_braking_force);
        Serial.print(" Current speed: ");
        Serial.print(get_speed());
        Serial.print(" Encoder ticks:");
        Serial.print(totalEncoderTicks);
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
    return msg.equals(estopGoMsg);
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
    otherBoard.setConnectionTimeout(ETH_TCP_INITIATION_DELAY);   //Set connection delay so we don't hang
    
    IPAddress otherIP = otherBoard.remoteIP();
        
    if(data.length() != 0){  //Got valid data - string will be empty if message not valid
        
        if(data.equals(speedRequestMsg)){  //Somebody's asking for our speed
            //Send back our current speed
            RJNet::sendData(otherBoard, "v=" + String(get_speed()) + ",I=" + String(motorCurrent));
            Serial.print("Speed request from: ");
            Serial.println(otherIP);
        }
        else if(otherIP == estopIP){
            //Message from Estop board
            motorEnabled = parseEstopMessage(data);
            lastEstopReply = millis();
        }
        else if(otherIP == brakeIP){
            //Message from brake board, should be acknoweldge
            //Is in form "R=$float", 
            if(data.substring(0,1).equals(ackMsg)){
                lastBrakeReply = millis();
            }
            else{
                Serial.print("Invalid message from brake: ");
                Serial.println(data);
            }
        }
        else if(otherIP == manualIP){
            //Message from manual board, should be speed command
            if(data.length() > 2 && data.substring(0,2).equals(speedCommandHeader)){
                manualTargetVelocity = parseSpeedMessage(data);
                lastManualSpeedTime = millis();
            }
            else{
                Serial.print("Bad message from manual: ");
                Serial.println(data);
            }
        }
        else if(otherIP == nucIP){
            //Message from manual board, should be speed command
            if(data.length() > 2 && data.substring(0,2).equals(speedCommandHeader)){
                nucTargetVelocity = parseSpeedMessage(data);
                lastNUCSpeedTime = millis();
                //Send back our current speed
                RJNet::sendData(otherBoard, "v=" + String(get_speed()) + ",I=" + String(motorCurrent));
            }  
            else{
                Serial.print("Wrong message from NUC: ");
                Serial.println(data);
            }
        }
    }
    else{
        Serial.print("Empty/invalid message received from: ");
        Serial.println(otherIP);
    }
}

void readAllNewMessages(){
    /*
    Checks the server, brake, and Estop for new messages and deals with them
    */
    EthernetClient client = server.available();  //if there is a new message from client create client object, otherwise new client object null
    // These look for clients
    while(client){
        handleSingleClientMessage(client);
        client = server.available();  //Go to next message
    }

    // These look for message bytes
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
    if(millis() - endOfStartupTime < MS_AFTER_STARTUP_BEFORE_CLIENT_CONNECT){
        //Do nothing before MS_AFTER_STARTUP_BEFORE_CLIENT_CONNECT
        estopConnected = false;
        brakeConnected = false;
        return;
    }
    
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
            RJNet::sendData(estopBoard, estopRequestMsg);
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

void stateMachineForCurrentSpeed(){
    //If we have a recent command from manual, we are in manual mode. Return manual's command.
    //Else if we have a recent command from NUC but NOT a recent command from manual, we are in autonomous mode.
    //Else we are stopped; don't change angle
    unsigned long currentTime = millis();
    
    if(currentTime - lastManualSpeedTime <= COMMAND_CONNECTION_TIMEOUT_MS){
        targetVelocity = manualTargetVelocity;
        whoIsCommandingSpeed = MANUAL;
    }
    else if(currentTime - lastNUCSpeedTime <= COMMAND_CONNECTION_TIMEOUT_MS){
        targetVelocity = nucTargetVelocity;
        whoIsCommandingSpeed = NUC;
    }
    else{
        whoIsCommandingSpeed = NOBODY;
        targetVelocity = 0;
    }
}

void executeStateMachine(float timeSinceLastLoop){ 
    //Check what state we are in at the moment.
    //We are disabled if the motor is disabled, or any of our clients has timed out 
    unsigned long currTime = millis();
    
    //Check if we are connected to all boards
    bool connectedToAllBoards = true;
    if(!brakeConnected){
        Serial.println("Lost brake TCP connection");
        connectedToAllBoards = false;
    }
    else if(currTime > lastBrakeReply + REPLY_TIMEOUT_MS){
        Serial.println("Brake timed out");
        connectedToAllBoards = false;
    }
    if(!estopConnected){
        Serial.println("Lost estop TCP connection");
        connectedToAllBoards = false;
    }
    else if(currTime > lastEstopReply + REPLY_TIMEOUT_MS){
        Serial.println("Estop timed out");
        connectedToAllBoards = false;
    }
    
    //Check who is commanding the speed
    stateMachineForCurrentSpeed();
    if(whoIsCommandingSpeed == NOBODY){
        //Nobody giving valid speed commands
        connectedToAllBoards = false;
    }
    
    //State transitions. See .gv file.
    if(currentState == STATE_DRIVING_FORWARD){
        if(!motorEnabled || !connectedToAllBoards){
            currentState = STATE_DISABLED_FORWARD;
        }
        else if(targetVelocity < -0.1 && abs(get_speed()) < switch_direction_max_speed){
            currentState = STATE_DRIVING_REVERSE;
            //Reset controller to clear error integral
            reset_controller(0);
        }
    }
    else if(currentState == STATE_DRIVING_REVERSE){
        if(!motorEnabled || !connectedToAllBoards){
            currentState = STATE_DISABLED_REVERSE;
        }
        else if(targetVelocity > 0 && abs(get_speed()) < switch_direction_max_speed){
            currentState = STATE_DRIVING_FORWARD;
            //Reset controller to clear error integral
            reset_controller(0);
        }
    }
    else if(currentState == STATE_DISABLED_REVERSE){
        if(motorEnabled && connectedToAllBoards){
            currentState = STATE_DRIVING_REVERSE;
        }
    }
    else if(currentState == STATE_DISABLED_FORWARD){
        if(motorEnabled && connectedToAllBoards){
            currentState = STATE_DRIVING_FORWARD;
        }
    }
    else
    {
        Serial.println("INVALID STATE!");
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
    if(abs(get_speed()) > 0.2){
        desired_braking_force = 200;
    }
    else {
        desired_braking_force = 0;
    }
    
    //Reset speed filter so we don't have problems coming out of this state
    reset_controller(get_speed());
}

void runStateForward(float timestep){
    /*
    Going forward in normal operation.
    */
    writeReversingContactorForward(true);
    
    float capped_target_speed = max(0, targetVelocity);    //If targetVelocity < 0, then this is 0, so we will brake in preparation for going backwards
    
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
    
    float capped_target_speed = max(0, -targetVelocity);    //If targetVelocity > 0, then this is 0, so we will brake in preparation for going forwards
    
    //controls math here
    FloatPair voltage_and_braking_force = gen_control_voltage_brake_force(timestep, -get_speed(), capped_target_speed);
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
    desiredPWM = motorOffPwm;
    analogWrite(MOTOR_CNTRL_PIN, desiredPWM);
}

void writeVoltageToMotor(float voltage){
    if(voltage > batteryVoltage){
        desiredPWM = maxSpeedPwm;
    }
    else if(voltage < 0.01){
        //Basically off
        desiredPWM = motorOffPwm;
    }
    else{
        desiredPWM = (byte) (maxSpeedPwm - zeroSpeedPwm)*voltage/batteryVoltage + zeroSpeedPwm;
    }
    analogWrite(MOTOR_CNTRL_PIN, desiredPWM);
}

////
// CURRENT SENSE FUNCTIONS
////

float getMotorCurrent(){
    unsigned int rawCurrentBits = analogRead(CURR_DATA_PIN);
    return (ampsPerBit*rawCurrentBits) - currentSensorMidpoint; // 200A/V * current voltage -> gives A
}
