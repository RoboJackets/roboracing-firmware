#include "RigatoniNetwork.h"
#include "Steering.h"
#include "RJNet.h"
#include <Ethernet.h>
#include <SPI.h>
#include <avr/wdt.h>
#include <AccelStepper.h>

// Notes:
// Need to connect VDC GND and dir- to common GND
// Need to have feedback connected to function

/* Stepper*/ 

AccelStepper stepperMotor(AccelStepper::DRIVER, PULSE_PIN, DIR_PIN);

#define PI 3.141592653589793


bool steeringEnabled = true;
static const uint8_t PULSE_DURATION_US = 10; // us 
static const uint8_t DIR_DURATION_US = 20; // us

//Motor controller gives us 4x microstepping - 800 steps/rev
static const float STEPS_PER_MOTOR_REV = 800;
//1:47 gear box, 1:3 toothed pulleys
static const float GEAR_RATIO = 47*3;
static const float STEPPER_STEP_SIZE = 2*PI/(STEPS_PER_MOTOR_REV*GEAR_RATIO); //in rads

static const unsigned long STEPPER_TIMEOUT = 50; // ms
static const int MAX_SPEED_WHILE_HOMING = 30000;
static const int MAX_SPEED = 150000; // steps per second.
static const int ACCEL = 800000; // steps per second per second.

volatile bool limitSwitchCounterClockGood = true;
volatile bool limitSwitchClockGood = true; 

// TODO NEED TO SET STEPPER DISTANCE
// How many steps from limit switch to center on each side
static const int STEPPER_CCW_LIMIT_TO_ZERO_POS = 15000;
static const int STEPPER_CW_LIMIT_TO_ZERO_POS = 15000;

static const float MIN_ANGLE_RADS = -STEPPER_CW_LIMIT_TO_ZERO_POS*STEPPER_STEP_SIZE;
static const float MAX_ANGLE_RADS = STEPPER_CCW_LIMIT_TO_ZERO_POS*STEPPER_STEP_SIZE;

bool isCWDirection = true;

static const unsigned int COMMAND_CONNECTION_TIMEOUT_MS = 500;  //If we don't get a packet in this long, we assume this board is no longer commanding angle
float desiredAngle = 0;     //This will be nucDesiredAngle or manualDesiredAngle depending on 
float nucDesiredAngle = 0;
float manualDesiredAngle = 0;

enum currentAngleCommander{
    MANUAL,
    NUC,
    NOBODY
};
currentAngleCommander whoIsCommandingAngle = NOBODY;

/* Ethernet */
EthernetServer server(PORT);

EthernetClient estopBoard;
bool estopConnected = false;

//Timestamps of our last messages to boards in ms
unsigned long lastEstopRequest = 0;
unsigned long lastEstopReply = 0;

//Timestamps of the last messages from NUC and manual in MS
unsigned long lastNUCAngleTime = 0;
unsigned long lastManualAngleTime = 0;

//End of startup. Needed so we don't connect for X seconds
unsigned long endOfStartupTime = 0;

//Universal acknowledge message
const static String ackMsg = "R";

//Angle request message
const static String angleRequestMsg = "A?";

//Manual RC angle command header
const static String angleCommandHeader = "S=";

//Possible messages from e-stop
const static String estopStopMsg = "D";
const static String estopLimitedMsg = "L";
const static String estopGoMsg = "G";
const static String estopRequestMsg = "S?";
const static String estopSendError = "FAIL";

void setup() {
    Serial.begin(BAUDRATE);

    pinMode(LED_PIN, OUTPUT);

    /* Initialization for limit switches*/
    pinMode(LIMIT_SWITCH_CCW_PIN, INPUT_PULLUP); // If limit switch pressed, high
    pinMode(LIMIT_SWITCH_CW_PIN, INPUT_PULLUP); // If limit switch pressed, high

    limitSwitchCounterClockGood = !digitalRead(LIMIT_SWITCH_CCW_PIN);
    limitSwitchClockGood = !digitalRead(LIMIT_SWITCH_CW_PIN);

    attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_CCW_PIN), limitSwitchCCWChange,CHANGE);
    attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_CW_PIN), limitSwitchCWChange,CHANGE);

    /* Initialization for stepper*/
    pinMode(DIR_PIN, OUTPUT);
    pinMode(PULSE_PIN, OUTPUT);

    digitalWrite(PULSE_PIN, HIGH); // Active LOW
    digitalWrite(DIR_PIN, LOW);   // Default CW

    isCWDirection = true;

    //Verify values in testing with new motor
    stepperMotor.setMinPulseWidth(PULSE_DURATION_US);
    stepperMotor.setAcceleration(ACCEL);

    //Set slow speed for going to home
    stepperMotor.setMaxSpeed(MAX_SPEED_WHILE_HOMING);
    goToHome();
    
    //Set full speed for operation
    stepperMotor.setMaxSpeed(MAX_SPEED);

    /* Initialization for encoder*/
    pinMode(ETH_RST_PIN, OUTPUT);
    pinMode(ETH_CS_PIN, OUTPUT);

    /* Initialization for ethernet*/
    resetEthernet();

    Ethernet.init(ETH_CS_PIN);  // SCLK pin from eth header
    Ethernet.begin(steeringMAC, steeringIP); // initialize ethernet device

    unsigned long loopCounter = 0;
    while (Ethernet.hardwareStatus() == EthernetNoHardware) {
        digitalWrite(LED_PIN, loopCounter++ % 4 == 0);
        Serial.println("Ethernet shield was not found.");
        delay(100);
    }
    
    while(Ethernet.linkStatus() == LinkOFF) {
        digitalWrite(LED_PIN, loopCounter++ % 4 > 0);
        Serial.println("Ethernet cable is not connected."); // do something with this
        delay(100);    // TURN down delay to check/startup faster
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }

    server.begin();
    Serial.print("Our address: ");
    Serial.println(Ethernet.localIP());
  
    estopBoard.setConnectionTimeout(ETH_TCP_INITIATION_DELAY);

    endOfStartupTime = millis();

    wdt_reset();
    wdt_enable(WDTO_500MS);
}

unsigned long lastPrintTime = 0;
void loop() {
    wdt_reset();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    readEthernet();  // check for new angle from ethernet
    readEstopResponses();
    
    stateMachineForCurrentAngle();
    
    if(steeringEnabled)
    {
        goToPosition();
    }

    sendToEstop();
    
    if(millis() - lastPrintTime > 500){
        lastPrintTime = millis();
        if(!limitSwitchCounterClockGood){
            Serial.print("CCW limit hit. ");
        }
        if(!limitSwitchClockGood){
            Serial.print("CW limit hit. ");
        }
        
        if(steeringEnabled){
            Serial.print("Steering enabled ");
        }
        else{
            Serial.print("Steering DISABLED ");
        }
        
        Serial.print("Steering angle from: ");
        switch (whoIsCommandingAngle){
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
        
        Serial.print("Target position: ");
        Serial.print(stepperMotor.targetPosition());
        Serial.print(" Current position: ");
        Serial.println(stepperMotor.currentPosition());
    }
}

void readEthernet(){ 
    EthernetClient client = server.available();    // if there is a new message from client create client object, otherwise new client object null
    while (client) {
        String data = RJNet::readData(client);  // if i get string from RJNet buffer
        IPAddress otherIP = client.remoteIP();
        client.setConnectionTimeout(ETH_TCP_INITIATION_DELAY);   //Set connection delay so we don't hang
        Serial.print("Message: ");
        Serial.print(data);
        Serial.print(" From: ");
        Serial.println(otherIP);
        if (data.length() != 0) {// if data exists
            if(data.substring(0,2).equals(angleRequestMsg))
            {
                String reply = "A=" + String(checkCurrentAngle());   // reply with A= currentAngle
                RJNet::sendData(client, reply);
            }
            else if(otherIP == manualIP){
                if (data.substring(0,2).equals(angleCommandHeader)){    // if client is giving us new angle in the form of S=$float
                    manualDesiredAngle = constrain(data.substring(2).toFloat(), MIN_ANGLE_RADS, MAX_ANGLE_RADS); // set new angle, convert from str to float
                    lastManualAngleTime = millis();
                }
                else {
                    Serial.println("Invalid message received from manual");
                }
            }
            else if(otherIP == nucIP){
                if (data.substring(0,2).equals(angleCommandHeader)){    // if client is giving us new angle in the form of S=$float
                    nucDesiredAngle = constrain(data.substring(2).toFloat(), MIN_ANGLE_RADS, MAX_ANGLE_RADS); // set new angle, convert from str to float
                    lastNUCAngleTime = millis();
                    // reply with A= currentAngle
                    String reply = "A=" + String(checkCurrentAngle());   
                    RJNet::sendData(client, reply);
                    Serial.print("Angle from NUC: ");
                    Serial.println(nucDesiredAngle);
                }
                else {
                    Serial.println("Invalid message received from NUC");
                }
            }
            //Do not read Estop here - it is handled below
        } 
        else {
            Serial.print("Empty/invalid message received from: ");
            Serial.println(otherIP);
        }
        client = server.available();
    }
}

void readEstopResponses(){
    String data = RJNet::readData(estopBoard);  // if i get string from RJNet buffer
    if (data.length() != 0) {// if data exists
        if (!data.equals(estopStopMsg)){   // Covers both limited and go
            steeringEnabled = true;
        }
        else
        {
            steeringEnabled = false;
        }
        lastEstopReply = millis();
    }
}

void resetEthernet() {
    //Resets Ethernet shield
    digitalWrite(ETH_RST_PIN, LOW);
    delay(1);
    digitalWrite(ETH_RST_PIN, HIGH);
    delay(501);
}

void sendToEstop() {
    if(millis() - endOfStartupTime < MS_AFTER_STARTUP_BEFORE_CLIENT_CONNECT){
        //Do nothing before MS_AFTER_STARTUP_BEFORE_CLIENT_CONNECT
        estopConnected = false;
        return;
    }

    estopConnected = estopBoard.connected();
    if(!estopConnected){
        //Lost TCP connection with the estop board
        estopBoard.connect(estopIP, PORT);
        Serial.println("Lost connection with estop");
    }
    else{
        if(millis() > lastEstopReply + MIN_MESSAGE_SPACING && millis() > lastEstopRequest + MIN_MESSAGE_SPACING){
            // TODO May reimplement FAIL in the future
            RJNet::sendData(estopBoard, estopRequestMsg);
            lastEstopRequest = millis();
        }
    }
}

// Interrupt called when CCW limit switch changes
void limitSwitchCCWChange() {
    limitSwitchCounterClockGood = !digitalRead(LIMIT_SWITCH_CCW_PIN);
}

// Interrupt called when CW limit switch changes
void limitSwitchCWChange() {
    limitSwitchClockGood = !digitalRead(LIMIT_SWITCH_CW_PIN);
}

bool isStepperGoing(){
    //Returns True if not at target position.
    //The built-in .isRunning() also returns True if speed is 0
    return stepperMotor.distanceToGo() != 0;
}

void stepperPulse(){ // rotates stepper motor one step in the currently set direction
    // Only allow movement if not in the direction that a limit switch is triggered 
    if((isCWDirection && limitSwitchClockGood) || (!isCWDirection && limitSwitchCounterClockGood))
    {
        stepperMotor.run(); //steps towards relative target position.
    }
    else
    {
        Serial.println("At extents!");
        //This call sets the current position to the current position
        //So we don't change the position but we reset the motor speed to 0
        //since it isn't moving any more.
        stepperMotor.setCurrentPosition(stepperMotor.currentPosition());
    }
}

// Will attempt to home to 0 steering position
// KNOWN BEHAVIOR -> Should just loop if motor not enabled / limit switches not responding
void goToHome(){
    // Default CW
    //Go to -infinity and move till we hit the switch
    stepperMotor.move(-10000000);
    while(limitSwitchClockGood)
    {
        stepperPulse();
    }

    isCWDirection = false;
    stepperMotor.setCurrentPosition(0);
    
    //setting target position to get to an absolute position 0 (ie center).
    stepperMotor.moveTo(STEPPER_CW_LIMIT_TO_ZERO_POS);
    
    //Steps to the center.
    while(isStepperGoing())
    {
        stepperPulse();
    }
    stepperMotor.setCurrentPosition(0); // Center position now signified by value 0.

    Serial.println("Good to go!");   
}

void goToPosition(){
    unsigned long startTime = millis();

    long targetPosition = round(desiredAngle / STEPPER_STEP_SIZE);

    stepperMotor.moveTo(targetPosition);
    
    //if Positive desired Angle/target Position, move CCW
    //if Negative desired Angle/target Position, move CW
    isCWDirection = targetPosition > stepperMotor.currentPosition() ? false : true;

    while(isStepperGoing() && millis() - startTime < STEPPER_TIMEOUT)
    {
        stepperPulse();
    }
}

void stateMachineForCurrentAngle(){
    //If we have a recent command from manual, we are in manual mode. Return manual's command.
    //Else if we have a recent command from NUC but NOT a recent command from manual, we are in autonomous mode.
    //Else we are stopped; don't change angle
    unsigned long currentTime = millis();
    
    if(currentTime - lastManualAngleTime <= COMMAND_CONNECTION_TIMEOUT_MS){
        desiredAngle = manualDesiredAngle;
        whoIsCommandingAngle = MANUAL;
    }
    else if(currentTime - lastNUCAngleTime <= COMMAND_CONNECTION_TIMEOUT_MS){
        desiredAngle = nucDesiredAngle;
        whoIsCommandingAngle = NUC;
    }
    else{
        whoIsCommandingAngle = NOBODY;
    }
}

float checkCurrentAngle()
{
    return float(stepperMotor.currentPosition())*STEPPER_STEP_SIZE;
}
