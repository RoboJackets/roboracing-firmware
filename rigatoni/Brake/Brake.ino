#include "Brake.h"
#include "BrakeLUT.h"
#include "RigatoniNetwork.h"
#include "RJNet.h"
#include <Ethernet.h>
#include <SPI.h>
#include <avr/wdt.h>

// Notes:
// Need to connect VDC GND and dir- to common GND
// Need to have feedback connected to function

/* Stepper*/ 

#define PER_STEP_DELAY_US 200 // us

static const uint8_t PULSE_DURATION_US = 10; // us 
static const uint8_t DIR_DURATION_US = 20; // us
static const unsigned long STEPPER_TIMEOUT = 50; // ms

volatile bool awayFromHomeSwitch = true; // Used for homing
volatile bool limitSwitchGood = true; // Used for limits of e-stop

int desiredBrakeStepsFromHome = 0;
int currentBrakeStepsFromHome = 0;

bool isCWDirection = true;

/* Ethernet */
EthernetServer server(PORT);

EthernetClient estopBoard;
bool estopConnected = false;

//End of startup. Needed so we don't connect for X seconds
unsigned long endOfStartupTime = 0;

//Timestamps of our last messages to boards in ms
unsigned long lastEstopRequest = 0;
unsigned long lastEstopReply = 0;

//Universal acknowledge message
const static String ackMsg = "R";

//Brake force request message
const static String brakeForceRequestMsg = "F?";

//Drive RC angle command header
const static String driveStringHeader = "B=";

//Possible messages from e-stop
const static String estopStopMsg = "D";
const static String estopLimitedMsg = "L";
const static String estopGoMsg = "G";
const static String estopRequestMsg = "S?";
const static String estopSendError = "FAIL";

void setup() {
    pinMode(LED_PIN, OUTPUT);

    /* Initialization for limit switches*/
    pinMode(HOME_SWITCH_PIN, INPUT_PULLUP); // If limit switch pressed, high
    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP); // If limit switch pressed, high

    limitSwitchGood = !digitalRead(LIMIT_SWITCH_PIN);
    awayFromHomeSwitch = !digitalRead(HOME_SWITCH_PIN);

    attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), limitSwitchChange,CHANGE);
    attachInterrupt(digitalPinToInterrupt(HOME_SWITCH_PIN), homeSwitchChange,CHANGE);

    while(!limitSwitchGood)
    {
        Serial.println("LIMIT SWITCH FAULT!");
        delay(100);
        limitSwitchGood = !digitalRead(LIMIT_SWITCH_PIN);
    }

    /* Initialization for stepper*/
    pinMode(DIR_PIN, OUTPUT);
    pinMode(PULSE_PIN, OUTPUT);

    digitalWrite(PULSE_PIN, HIGH); // Active LOW
    digitalWrite(DIR_PIN, LOW);   // Default CW

    Serial.begin(BAUDRATE);

    /* Initialization for ethernet*/
    resetEthernet();

    pinMode(ETH_CS_PIN, OUTPUT);

    Ethernet.init(ETH_CS_PIN);  // SCLK pin from eth header
    Ethernet.begin(brakeMAC, brakeIP); // initialize ethernet device
  
    unsigned long loopCounter = 0;
    while(Ethernet.hardwareStatus() == EthernetNoHardware) {
        digitalWrite(LED_PIN, loopCounter++ % 4 == 0);
        Serial.println("Ethernet shield was not found.");
        delay(100);
    }

    while(Ethernet.linkStatus() == LinkOFF) {
        digitalWrite(LED_PIN, loopCounter++ % 4 > 0);
        Serial.println("Ethernet cable is not connected.");
        delay(100);
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

unsigned long lastPrintTime = 0;

void loop() {
    wdt_reset();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    readEthernet();  // check for new angle from ethernet
    
    if(limitSwitchGood)
    {
        goToPosition();
    }
    else {
        Serial.println("Limit switch fault; holding position");
    }
    
    if(millis() - lastPrintTime > 500){
        lastPrintTime = millis();
        Serial.print("Desired stepper position: ");
        Serial.print(desiredBrakeStepsFromHome);
        Serial.print(" Current stepper position: ");
        Serial.print(currentBrakeStepsFromHome);
        Serial.print(" Current stepper force: ");
        Serial.println(brakingForceFromCurrentPos(currentBrakeStepsFromHome));
    }
  
    sendToEstop();
}

void readEthernet(){ 
    EthernetClient client = server.available();    // if there is a new message from client create client object, otherwise new client object null
    while (client) {
        String data = RJNet::readData(client);  // if i get string from RJNet buffer (brake_value=%int)
        IPAddress otherIP = client.remoteIP();
        client.setConnectionTimeout(ETH_TCP_INITIATION_DELAY);   //Set connection delay so we don't hang
        
        Serial.print("Message: ");
        Serial.print(data);
        Serial.print(" From: ");
        Serial.println(otherIP);

        if (data.length() != 0) {   // if data exists
            if(data.substring(0,2).equals(brakeForceRequestMsg))
            {
                String reply = "F=" + String(currentBrakeStepsFromHome);  // reply with F=force
                RJNet::sendData(client, reply);
            }
            else if(otherIP == driveIP)
            {
                if (data.substring(0,2).equals(driveStringHeader)){    // if client is giving us new angle in the form of S=$float
                    // TODO These should be changed to forces
                    int desiredBrakingForce = constrain(data.substring(2).toInt(), 0, MAX_COMMAND_FORCE);
                    desiredBrakeStepsFromHome = stepperStepsFromHomeForForce(brakingForceFromCurrentPos(desiredBrakingForce));
                    RJNet::sendData(client, ackMsg);
                }
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
    /* Sends a request to Estop for the current car state*/
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
        //Don't spam server with messages
        if(millis() > lastEstopReply + MIN_MESSAGE_SPACING && millis() > lastEstopRequest + MIN_MESSAGE_SPACING){
            if(limitSwitchGood)
            {
                RJNet::sendData(estopBoard, estopRequestMsg);
            }
            else
            {
                RJNet::sendData(estopBoard, estopSendError);
            }
            lastEstopRequest = millis();
        }
    }
}

/* For setting direction of stepper motor */
// TODO CHECK functionality
void assignDirection(){      
    if(currentBrakeStepsFromHome-desiredBrakeStepsFromHome != 0)
    {
        bool setDirPin = HIGH; // setdirPIN to CW or CCW, HIGH is CW, LOW is CCW
        if(desiredBrakeStepsFromHome > currentBrakeStepsFromHome){      // set dirPIN to CW or CCW
            setDirPin = HIGH;   // CCW
        } else {
            setDirPin = LOW;    // CW
        }

        // Only change the pin if necessary
        if(setDirPin != isCWDirection){
            isCWDirection = setDirPin;
            digitalWrite(DIR_PIN, setDirPin);
            delayMicroseconds(DIR_DURATION_US);
        }
    }
}

// Will attempt to home to 0 brake position
// KNOWN BEHAVIOR -> Should just loop if motor not enabled
void goToHome()
{
    // Default CW    
    while(awayFromHomeSwitch)
    {
        stepperPulse();
        delayMicroseconds(500); // Extra small delay for homing
    }

    // Set to CCW
    isCWDirection = false;
    digitalWrite(DIR_PIN, HIGH);
    delayMicroseconds(DIR_DURATION_US);
    
    // Step away CCW until off home switch
    while(!awayFromHomeSwitch)
    {
        stepperPulse();
        delayMicroseconds(500); // Extra small delay for homing
    }
    currentBrakeStepsFromHome = 0;
    // Stays CCW since at the "zero" brake
    Serial.println("Good to go!");
}

void homeSwitchChange(){
    awayFromHomeSwitch = !digitalRead(HOME_SWITCH_PIN);
}

void limitSwitchChange() {
    limitSwitchGood = !digitalRead(LIMIT_SWITCH_PIN);
}

void stepperPulse(){ // rotates stepper motor one step in the currently set direction
    if((isCWDirection && awayFromHomeSwitch) || (!isCWDirection && limitSwitchGood))
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

void goToPosition(){
    unsigned long startTime = millis();

    // Tries to get location, but has a timeout
    while(currentBrakeStepsFromHome-desiredBrakeStepsFromHome != 0 && millis() - startTime < STEPPER_TIMEOUT){
        assignDirection();
        stepperPulse();
        if(isCWDirection) {
            currentBrakeStepsFromHome += 1;
        }
        else {
            currentBrakeStepsFromHome -= 1;
        }
    }
}

int stepperStepsFromHomeForForce(float brakingForce) {
    if(brakingForce <= 0) {
        return BrakeLUT[0][0];
    }

    for(byte i = 1; i < BrakeLUTLength; i++) {
        float LUTBrakingForce = BrakeLUT[i][1];
        if(LUTBrakingForce >= brakingForce) {
            //entry i is > than target, i-1 is < target. Linear interpolate
            float LUTBrakingForcePrevEntry = BrakeLUT[i-1][1];
            return BrakeLUT[i-1][0] + (brakingForce - LUTBrakingForcePrevEntry) * (BrakeLUT[i][0] - BrakeLUT[i-1][0])/(LUTBrakingForce - LUTBrakingForcePrevEntry);
        }
    }
    return BrakeLUT[BrakeLUTMaxIndex][0];
}

float brakingForceFromCurrentPos(int currStepsFromHome){
    if(currStepsFromHome <= 0) {
        return BrakeLUT[0][1];
    }

    for(byte i = 1; i < BrakeLUTLength; i++) {
        int LUTBrakeSteps = BrakeLUT[i][0];
        if(LUTBrakeSteps >= currStepsFromHome) {
            //entry i is > than target, i-1 is < target. Linear interpolate
            int LUTBrakeStepsPrevEntry = BrakeLUT[i-1][0];
            return BrakeLUT[i-1][1] + (currStepsFromHome - LUTBrakeStepsPrevEntry) * (BrakeLUT[i][1] - BrakeLUT[i-1][1])/(LUTBrakeSteps - LUTBrakeStepsPrevEntry);
        }
    }
    return BrakeLUT[BrakeLUTMaxIndex][1];
}
