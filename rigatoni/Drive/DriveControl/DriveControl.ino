#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <avr/wdt.h>
#include "DriveControl.h"
#include "controller_simple_estimator_no_current.h"
#include <Ethernet.h>
#include <EthernetUdp.h>
#include "RJNetUDP.h"
#include "RigatoniNetworkUDP.h"

const static float US_PER_SEC = 1000000.0;


const static unsigned int REPLY_TIMEOUT_MS = 20000; //BOARD_RESPONSE_TIMEOUT_MS;   //We have to receive a reply from Estop and Brake within this many MS of our message or we are timed out. NOT TCP timeout.
const static unsigned int COMMAND_CONNECTION_TIMEOUT_MS = 500; //If no message from manual in this long, assume auto mode if a message from NUC in this time

/****************ETHERNET****************/
// This is the UDP version.
// See https://docs.google.com/document/d/1oRHm5xBQiod_YXESQ9omFy5joJqMfeQjY3NCvdzRTjk/edit
////

EthernetUDP Udp;

/****************Messages from clients****************/

//Brake special acknowledge with current braking force
const static String brakeForceHeader = "F=";

//Possible messages from e-stop
const static String estopStopMsg = "D";
const static String estopLimitedMsg = "L";
const static String estopGoMsg = "G";

//Manual message expected format
const static String speedCommandHeader = "v=";
const static String manualStateCommandHeader = "M=";
const static char autoModeCommand = 'A';
const static char manualModeCommand = 'M';

//Timestamps of messages from boards in ms
unsigned long lastEstopMessage = 0;
unsigned long lastBrakeMessage = 0;

//Timestamps of the last messages from NUC and manual in MS
unsigned long lastNUCSpeedTime = 0;
unsigned long lastManualSpeedTime = 0;

//Timestamps of our last messages to boards in ms
unsigned long brakeCommandSent = 0;

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
 - manual speed if in manual mode
 - NUC speed if in autonomous mode
 - 0 if in neither state (shouldn't happen)
*/
enum currentSpeedCommander{
    MANUAL,
    NUC,
    NOBODY
};
currentSpeedCommander whoIsCommandingSpeed = NUC;

float targetVelocity = 0;
float nucTargetVelocity = 0;
float manualTargetVelocity = 0;

Encoder encoder(ENCODER_A_PIN,ENCODER_B_PIN);
long lastLoopEncoderTicks = 0;

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
    
    //If you don't set retransmission to 0, the WIZNET will retry 8 times if it cannot resolve the
    //MAC address of the destination using ARP and block for a long time.
    //With this as 0, will only block for ETH_RETRANSMISSION_DELAY_MS
    Ethernet.setRetransmissionCount(0);
    //Ethernet.setRetransmissionTimeout(ETH_RETRANSMISSION_DELAY_MS);  //Set timeout delay before failure of ARP

    //server.begin();
    Udp.begin(RJNetUDP::RJNET_PORT);
    Serial.print("Our address: ");
    Serial.println(Ethernet.localIP());
    
    // WATCHDOG TIMER
    wdt_reset();
    wdt_enable(WDTO_500MS);
    
    //Reset encoder
    encoder.write(0);
}

void loop() {
    wdt_reset();
    digitalWrite(LED2_PIN, !digitalRead(LED2_PIN));
    
    // Read new messages from WizNet
    readAllNewMessages();
    
    //How long since the last controller execution
    unsigned long currTime = micros();
    float loopTimeStep = (currTime - lastControllerRunTime)/US_PER_SEC;
    lastControllerRunTime = currTime;
    
    
    long encoderTicksNow = encoder.read();
    estimate_vel(loopTimeStep, encoderTicksNow - lastLoopEncoderTicks);
    lastLoopEncoderTicks = encoderTicksNow;

    executeStateMachine(loopTimeStep);
    
    //Now write braking force to brake
    sendToBrake();
    
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
        
        Serial.print("Estop: ");
        Serial.print(motorEnabled ? "enabled" : "disabled");
        Serial.print(" Cmd vel: ");
        Serial.print(targetVelocity);
        Serial.print(" Filtered tgt vel: ");
        Serial.print(get_curr_target_speed());
        Serial.print(" Throttle: ");
        Serial.print(desiredPWM);
        Serial.print(" Brake force: ");
        Serial.print(desired_braking_force);
        Serial.print(" Curr vel: ");
        Serial.print(get_speed());
        Serial.print(" Enc ticks:");
        Serial.println(encoder.read());
        
        
        lastPrintTime = millis();
    }
}

////
// ETHERNET FUNCTIONS
////

bool parseEstopMessage(const String & msg){
    /*
    Parse a message from the estop board and determine if the drive motor is enabled or disabled.
    If state is GO, motor enabled; in all other cases, motor is disabled.
    */
    return msg.equals(estopGoMsg);
}

float parseSpeedMessage(const String & speedMessage, unsigned int startOfSpeed){
    //takes in message from manual and converts it to a float desired speed.
    //startOfSpeed is the index of the start of the floating-point number representing the speed.
    return atof(speedMessage.c_str() + startOfSpeed);
}

void readAllNewMessages(){
    /*
    Checks the server, brake, and Estop for new messages and deals with them
    */
    Message incomingMessage = RJNetUDP::receiveMessage(Udp);
    while(incomingMessage.received) {
        //For reasons that I don't understand, the UDP stuff seems to work better
        //when these print statements are here.
        Serial.print("Message from: ");
        Serial.print(incomingMessage.ipaddress);
        Serial.print(": ");
        Serial.println(incomingMessage.message);
        if(incomingMessage.ipaddress == estopIP){
            //Message from Estop board
            motorEnabled = parseEstopMessage(incomingMessage.message);
            lastEstopMessage = millis();
            //Serial.print("Got message from Estop. Motor enabled: ");
            //Serial.println(motorEnabled);
        }
        else if(incomingMessage.ipaddress == brakeIP){
            //Message from brake board, should be brake force
            if(incomingMessage.message.startsWith(brakeForceHeader)){
                lastBrakeMessage = millis();
            }
            else{
                Serial.print("Invalid message from brake: ");
                Serial.println(incomingMessage.message);
            }
        }
        else if(incomingMessage.ipaddress == manualIP){
            //Message from manual board, should be speed command
            //We know the message is of the form "M=%c v=%f", where %c is the character that defines the mode.
            //Message is known length until the float
            unsigned int startOfSpeedCommand = manualStateCommandHeader.length() + 1;
            if(incomingMessage.message.startsWith(manualStateCommandHeader) && incomingMessage.message.substring(startOfSpeedCommand, startOfSpeedCommand + speedCommandHeader.length()).equals(speedCommandHeader)){
                //Message from manual is valid
                
                //Parse current mode
                if(incomingMessage.message.charAt(manualStateCommandHeader.length()) == autoModeCommand){
                    whoIsCommandingSpeed = NUC;
                }
                else{
                    //Manual mode
                    whoIsCommandingSpeed = MANUAL;
                }
                
                //Parse current speed. We use pointer arithmetic to get a pointer to the start of the floating-point number
                manualTargetVelocity = parseSpeedMessage(incomingMessage.message, manualStateCommandHeader.length() + speedCommandHeader.length() + 2);
                
                lastManualSpeedTime = millis();
            }
            else{
                Serial.print("Bad message from manual: ");
                Serial.println(incomingMessage.message);
            }
        }
        else if(incomingMessage.ipaddress == nucIP){
            //Message from nuc, should be speed command
            if(incomingMessage.message.startsWith(speedCommandHeader)){
                nucTargetVelocity = parseSpeedMessage(incomingMessage.message, speedCommandHeader.length());
                lastNUCSpeedTime = millis();
                Serial.print("NUC speed: ");
                Serial.println(nucTargetVelocity);
            }  
            else{
                Serial.print("Wrong message from NUC: ");
                Serial.println(incomingMessage.message);
            }
        }
        else{
            Serial.print("Ignoring message from: ");
            Serial.print(incomingMessage.ipaddress);
            Serial.print(": ");
            Serial.println(incomingMessage.message);
        }
        //Get a new message
        incomingMessage = RJNetUDP::receiveMessage(Udp);
    }
}

void sendToBrake(void){
    /*
    If sufficient time has elapsed both from our last request and their last reply, send new command to brake
    */
    if(millis() > lastBrakeMessage + MIN_MESSAGE_SPACING && millis() > brakeCommandSent + MIN_MESSAGE_SPACING){
        String to_send = "B=" + String(desired_braking_force);
        RJNetUDP::sendMessage(to_send, Udp, brakeIP);
        brakeCommandSent = millis();
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
    
    //Check if we are connected to all boards
    bool connectedToAllBoards = true;

    /*
    if(currTime > lastBrakeMessage + REPLY_TIMEOUT_MS){
        Serial.println("Brake timed out");
        connectedToAllBoards = false;
    }
    */
    
    if(currTime - lastEstopMessage > REPLY_TIMEOUT_MS){
        Serial.println("Estop timed out");
        connectedToAllBoards = false;
    }
    
        //This checks to see what our target speed is
    switch (whoIsCommandingSpeed){
        case MANUAL:
            targetVelocity = manualTargetVelocity;
            if(currTime - lastManualSpeedTime > REPLY_TIMEOUT_MS){
                Serial.println("Manual commanding and timed out");
                connectedToAllBoards = false;
            }
            break;
        case NUC:
            targetVelocity = nucTargetVelocity;
            if(currTime - lastNUCSpeedTime > REPLY_TIMEOUT_MS){
                Serial.println("NUC commanding and timed out");
                connectedToAllBoards = false;
            }
            break;
        default:
            connectedToAllBoards = false;
            targetVelocity = 0;
            break;
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
// Currently current sensor not connected so thid does nothing.
////

float getMotorCurrent(){
    unsigned int rawCurrentBits = analogRead(CURR_DATA_PIN);
    return (ampsPerBit*rawCurrentBits) - currentSensorMidpoint; // 200A/V * current voltage -> gives A
}
