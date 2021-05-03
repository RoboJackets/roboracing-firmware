#include "RigatoniNetwork.h"
#include "Steering.h"
#include "RJNet.h"
#include <Ethernet.h>
#include <SPI.h>
#include <avr/wdt.h>

// Notes:
// Need to connect VDC GND and dir- to common GND
// Need to have feedback connected to function

/* Stepper*/ 

#define STEPPER_TO_MOTOR_GEAR_RATIO 15.3
#define PI 3.141592653589793

// TODO determine
#define PER_STEP_DELAY_US 600 // us

bool steeringEnabled = false;
static const uint8_t PULSE_DURATION_US = 10; // us 
static const uint8_t DIR_DURATION_US = 20; // us
// Stepper changes by 0.1176 degrees (1.8deg of motor / 15.3 gear box)
static const float STEPPER_STEP_SIZE = 0.0020533; // rads
// Steering deadband to account for discrete stepper
static const float STEPPER_DEADBAND = 0.005; // rads
static const unsigned long STEPPER_TIMEOUT = 50; // ms

volatile bool limitSwitchCounterClockGood = true;
volatile bool limitSwitchClockGood = true; 

// TODO NEED TO SET STEPPER DISTANCE
// How many steps from limit switch to center on each side
static const int STEPPER_CCW_LIMIT_TO_ZERO_POS = 1700;
static const int STEPPER_CW_LIMIT_TO_ZERO_POS = 1700;

static const float MIN_ANGLE_RADS = -STEPPER_CW_LIMIT_TO_ZERO_POS*STEPPER_STEP_SIZE;
static const float MAX_ANGLE_RADS = STEPPER_CCW_LIMIT_TO_ZERO_POS*STEPPER_STEP_SIZE;

int currentStepPos = 0; // Stepper steps position
float currentAngle = 0;
float desiredAngle = 0;
bool isCWDirection = true;

/* Ethernet */
EthernetServer server(PORT);

EthernetClient estopBoard;
bool estopConnected = false;

//Timestamps of our last messages to boards in ms
unsigned long lastEstopRequest = 0;
unsigned long lastEstopReply = 0;

//End of startup. Needed so we don't connect for X seconds
unsigned long endOfStartupTime = 0;

//Universal acknowledge message
const static String ackMsg = "R";

//Angle request message
const static String angleRequestMsg = "A?";

//Manual RC angle command header
const static String manualStringHeader = "S=";

//Possible messages from e-stop
const static String estopStopMsg = "D";
const static String estopLimitedMsg = "L";
const static String estopGoMsg = "G";
const static String estopRequestMsg = "S?";
const static String estopSendError = "FAIL";

void setup() {
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

    /* Initialization for encoder*/
    pinMode(ETH_RST_PIN, OUTPUT);
    pinMode(ETH_CS_PIN, OUTPUT);

    Serial.begin(BAUDRATE);

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

    goToHome();

    endOfStartupTime = millis();

    wdt_reset();
    wdt_enable(WDTO_500MS);
}
 
void loop() {
    wdt_reset();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    readEthernet();  // check for new angle from ethernet

    if(steeringEnabled)
    {
        goToPosition();
    }

    sendToEstop();
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
                String reply = "A=" + String(currentAngle);   // reply with A= currentAngle
                RJNet::sendData(client, reply);
            }
            else if(otherIP == manualIP){
                if (data.substring(0,2).equals(manualStringHeader)){    // if client is giving us new angle in the form of S=$float
                    desiredAngle = constrain(data.substring(2).toFloat(), MIN_ANGLE_RADS, MAX_ANGLE_RADS); // set new angle, convert from str to float
                    RJNet::sendData(client, ackMsg);
                }
                else {
                    Serial.println("Invalid message received from manual");
                }
            }
            else if(otherIP == estopIP)
            {
                if (!data.equals(estopStopMsg)){   // Covers both limited and go
                    steeringEnabled = true;
                }
                else
                {
                    steeringEnabled = false;
                }
                lastEstopReply = millis();
                Serial.print("Estop: steering enabled? ");
                Serial.println(steeringEnabled);
            }
        } 
        else {
            Serial.print("Empty/invalid message received from: ");
            Serial.println(otherIP);
        }
        client = server.available();
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
            RJNet::sendData(estopBoard, estopSendError);
            lastEstopRequest = millis();
        }
    }
}

/* For setting direction of stepper motor */
// CCW is positive radians
// CW is negative radians
// TODO CHECK functionality 
void assignDirection(float goalAngle, float measuredAngle){  
    if (abs(goalAngle - measuredAngle) > STEPPER_DEADBAND){      // checks if motor needs to turn before changing direction
        bool setDirPin = HIGH; // setdirPIN to CW or CCW, HIGH is CW, LOW is CCW
        if (goalAngle > measuredAngle){  // CCW
            setDirPin = HIGH;
        } 
        else {  // CW
            setDirPin = LOW;
        }

        // Only change the pin if necessary
        if(setDirPin != isCWDirection){
            isCWDirection = setDirPin;
            digitalWrite(DIR_PIN, setDirPin);
            delayMicroseconds(DIR_DURATION_US);
        }
    }
    else
    {
        Serial.println("Close enough for direction");
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

void stepperPulse(){ // rotates stepper motor one step in the currently set direction
    // Only allow movement if not in the direction that a limit switch is triggered 
    if((isCWDirection && limitSwitchClockGood) || (!isCWDirection && limitSwitchCounterClockGood))
    {
        digitalWrite(PULSE_PIN, LOW);
        delayMicroseconds(PULSE_DURATION_US);
        digitalWrite(PULSE_PIN, HIGH);
        delayMicroseconds(PULSE_DURATION_US);

        delayMicroseconds(PER_STEP_DELAY_US);
    }
    else
    {
        Serial.println("At extents!");
    }
}

// Will attempt to home to 0 steering position
// KNOWN BEHAVIOR -> Should just loop if motor not enabled / limit switches not responding
void goToHome(){

    // Default CW
    while(limitSwitchClockGood)
    {
        stepperPulse();
    }

    // Once home, now switch to CCW direction to move to center
    isCWDirection = false;
    digitalWrite(DIR_PIN, HIGH);
    delayMicroseconds(DIR_DURATION_US);

    // Steps to the center
    for(int i = 0; i < STEPPER_CW_LIMIT_TO_ZERO_POS; i++)
    {
        stepperPulse();
    }

    Serial.println("Good to go!");   
}

void goToPosition(){
    unsigned long startTime = millis();
    checkCurrentAngle(); // Check current angle to avoid running if unnecessary

    // Tries to get location
    while(abs(desiredAngle - currentAngle) > STEPPER_DEADBAND && 
        millis() - startTime < STEPPER_TIMEOUT)
    {
        assignDirection(desiredAngle, currentAngle);
        stepperPulse();
        // TODO verify direction
        if(isCWDirection)
        {
            currentStepPos += 1;
        }
        else
        {
            currentStepPos -= 1;
        }

        checkCurrentAngle();
    }
}

void checkCurrentAngle()
{
    currentAngle = float(currentStepPos)*STEPPER_STEP_SIZE;
}
