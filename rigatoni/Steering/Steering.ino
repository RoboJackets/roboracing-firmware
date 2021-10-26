    #include "RigatoniNetwork.h"
#include "Steering.h"
#include "RJNet.h"
#include <Ethernet.h>
#include <SPI.h>
#include <avr/wdt.h>

// Notes:
// Need to connect VDC GND and dir- to common GND
// Need to have feedback connected to function
// make sure encoder switch is set to run mode

/* Stepper*/ 

#define STEPPER_TO_MOTOR_GEAR_RATIO 15.3
#define ENCODER_BAD_POSITION 0xFFFF
#define ENCODER_BAD_POSITION_ATTEMPTS 3
#define ENCODER_POSITIONS_PER_REVOLUTION 16384.0
#define PI 3.141592653589793

#define PER_STEP_DELAY_MS 5 // ms

// TODO check min and max
#define MIN_ANGLE_RADS -0.2 // rads
#define MAX_ANGLE_RADS 0.2  // rads

//#define ENCODER_ZERO // uncomment to set the zero of the encoder

bool steeringEnabled = false;
static const uint8_t PULSE_DURATION_US = 10; // us 
static const uint8_t DIR_DURATION_US = 20; // us
// Encoder is accurate to 0.2 degrees -> 0.003491 rads
// Stepper changes by 0.1176 degrees (1.8deg of motor / 15.3 gear box)  -> 0.0021 rads
// Steering deadband to account for discrete stepper + encoder
static const float STEPPER_DEADBAND = 0.006; // rads
static const unsigned long STEPPER_TIMEOUT = 50; // ms

volatile bool limitSwitch1Good = true;
volatile bool limitSwitch2Good = true; 

// TODO NEED TO SET ENCODER ZERO
static const int ENCODER_ZERO_POS = 1000;

float currentAngle = 0;
float desiredAngle = 0;
bool isCWDirection = true;
bool encoderGood = true;

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

    pinMode(LIMIT_SWITCH_1_PIN, INPUT_PULLUP);
    pinMode(LIMIT_SWITCH_2_PIN, INPUT_PULLUP);

    limitSwitch1Good = digitalRead(LIMIT_SWITCH_1_PIN);
    limitSwitch2Good = digitalRead(LIMIT_SWITCH_2_PIN);

    attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_1_PIN), limitSwitch1Hit,CHANGE);
    attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_2_PIN), limitSwitch2Hit,CHANGE);

    while(!limitSwitch1Good || !limitSwitch2Good)
    {
        Serial.println("LIMIT SWITCH FAULT!");
        delay(100);
    }

    /* Initialization for stepper*/
    pinMode(DIR_PIN, OUTPUT);
    pinMode(PULSE_PIN, OUTPUT);

    digitalWrite(PULSE_PIN, HIGH); // Active LOW
    digitalWrite(DIR_PIN, HIGH);   // Default CW

    /* Initialization for encoder*/
    pinMode(SPI_SCLK_PIN, OUTPUT);
    pinMode(SPI_MISO_PIN, OUTPUT);
    pinMode(SPI_MOSI_PIN, INPUT);
    pinMode(ENC_CS_PIN, OUTPUT);
    pinMode(ETH_RST_PIN, OUTPUT);
    pinMode(ETH_CS_PIN, OUTPUT);

    digitalWrite(ENC_CS_PIN, HIGH); // disable encoder on setup
    SPI.begin();

    // Will set encoder to zero
    // #ifdef ENCODER_ZERO
    //     setZeroSPI();
    //     while(1){
    //         Serial.println("Encoder Set to Zero. Reset and comment out ENCODER_ZERO to run.");
    //         delay(500);
    //     }
    // #endif

    /* Initialization for ethernet*/
    resetEthernet();

    Ethernet.init(ETH_CS_PIN);  // SCLK pin from eth header
    Ethernet.begin(steeringMAC, steeringIP); // initialize ethernet device
  
    Serial.begin(BAUDRATE);

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
 
void loop() {
    wdt_reset();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    readEthernet();  // check for new angle from ethernet

    if(steeringEnabled && encoderGood && limitSwitch1Good && limitSwitch2Good)
    {
        goToPosition();
    }
    
    if(!limitSwitch1Good)
    {
        Serial.println("LIMIT SWITCH 1 NOT GOOD!");
    }

    if(!limitSwitch2Good)
    {
        Serial.println("LIMIT SWITCH 2 NOT GOOD!");
    }

    if(!encoderGood)
    {
        Serial.println("ENCODER NOT GOOD!");
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
            if(limitSwitch1Good && limitSwitch2Good)
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
// CCW is positive radians
// CW is negative radians
// TODO CHECK functionality 
void assignDirection(float goalAngle, float measuredAngle){  
    if (abs(goalAngle - measuredAngle) > STEPPER_DEADBAND){      // checks if motor needs to turn before changing direction
        bool setDirPin = HIGH; // setdirPIN to CW or CCW, HIGH is CW, LOW is CCW
        if (goalAngle > measuredAngle){  // CCW
            setDirPin = LOW;
        } 
        else {  // CW
            setDirPin = HIGH;
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

void limitSwitch1Hit() {
    limitSwitch1Good = false;
}

void limitSwitch2Hit() {
    limitSwitch2Good = false;
}

void stepperPulse(){ // rotates stepper motor one step in the currently set direction
    digitalWrite(PULSE_PIN, LOW);
    delayMicroseconds(PULSE_DURATION_US);
    digitalWrite(PULSE_PIN, HIGH);
    delayMicroseconds(PULSE_DURATION_US);
}

void goToPosition(){
    unsigned long startTime = millis();
    currentAngle = getCurrentAngle(); // Check current angle to avoid running if unnecessary

    while(abs(desiredAngle - currentAngle) > STEPPER_DEADBAND && 
        millis() - startTime < STEPPER_TIMEOUT &&
        encoderGood &&
        limitSwitch1Good &&
        limitSwitch2Good)
    {
        assignDirection(desiredAngle, currentAngle);
        stepperPulse();
        currentAngle = getCurrentAngle();
        delay(PER_STEP_DELAY_MS);
    }
}

float getCurrentAngle(){
    int encoderAttemptCounts = 0;
    uint16_t currentPositionEncoder = getPositionSPI();
    float angle = 0;
    // Try to poll to get valid encoder value
    while (currentPositionEncoder == ENCODER_BAD_POSITION && 
        encoderAttemptCounts < ENCODER_BAD_POSITION_ATTEMPTS)
    {
        currentPositionEncoder = getPositionSPI(); //try again
        encoderAttemptCounts += 1;
    }

    if(currentPositionEncoder != ENCODER_BAD_POSITION)
    {
        // TODO UNCLEAR IF CW on encoder increases position or decreases it
        // Might need to invert to match CW 
        int currentPositionFromZero =  int(currentPositionEncoder) - ENCODER_ZERO_POS;
        angle = currentPositionFromZero*((2*PI)/ENCODER_POSITIONS_PER_REVOLUTION); // ticks * (rads/ticks) = rads
    }
    else
    {
        angle = 2*PI;
        encoderGood = false;
    }
    return angle;
}

/* Encoder Helper Functions */
void setCSLine(uint8_t csLine){   // enable or disable encoder
    digitalWrite(ENC_CS_PIN, csLine);
}

uint8_t spiWriteRead(uint8_t sendByte, uint8_t releaseLine){
    uint8_t data;

    //set cs low, cs may already be low but there's no issue calling it again except for extra time
    setCSLine(LOW);

    //There is a minimum time requirement after CS goes low before data can be clocked out of the encoder.
    delayMicroseconds(3);

    //send the command  
    data = SPI.transfer(sendByte);
    delayMicroseconds(3); //There is also a minimum time after clocking that CS should remain asserted before we release it
    setCSLine(releaseLine); //if releaseLine is high set it high else it stays low
  
    return data;
}

void setZeroSPI(){
    spiWriteRead(NOP, false); // NOP needs to be first byte before extended command can be set

    //this is the time required between bytes as specified in the datasheet.
    delayMicroseconds(3); 
  
    spiWriteRead(ZERO, true);
    delay(250); //250 ms delay to allow the encoder to reset
}

void resetAMT22(){
    spiWriteRead(NOP, false); // NOP needs to be first byte before extended command can be set

    //this is the time required between bytes as specified in the datasheet.
    delayMicroseconds(3); 
  
    spiWriteRead(RESET, true);
    delay(250); //250 ms delay to allow the encoder to start back up
}

uint16_t getPositionSPI(){
    uint16_t currentPosition;       //14-bit response from encoder
    bool binaryArray[16];           //after receiving the position we will populate this array and use it for calculating the checksum

    //get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
    currentPosition = spiWriteRead(NOP, false) << 8;   

    //this is the time required between bytes as specified in the datasheet.
    delayMicroseconds(3);

    //OR the low byte with the currentPosition variable. release line after second byte
    currentPosition |= spiWriteRead(NOP, true);
  
    //run through the 16 bits of position and put each bit into a slot in the array so we can do the checksum calculation
    for(int i = 0; i < 16; i++) binaryArray[i] = (0x01) & (currentPosition >> (i));

    //using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
    if ((binaryArray[15] == !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
          && (binaryArray[14] == !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
    {
        //we got back a good position, so just mask away the checkbits
        currentPosition &= 0x3FFF;
    }
    else
    {
        currentPosition = ENCODER_BAD_POSITION; //bad position
    }

    // Don't need to shift for 14 bit version

    return currentPosition;
}
