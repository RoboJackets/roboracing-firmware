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

bool steeringEnabled = false;
static const uint8_t PULSE_DURATION = 10; // us 
static const uint8_t DIR_DURATION = 20; // us 
static const float STEPPER_DEADBAND = 0.035; // rads. Because stepper motor 1.8 deg per step -> slightly more
static const unsigned long STEPPER_TIMEOUT = 5; // ms

// TODO need to set encoder zero
static const uint16_t ENCODER_ZERO = 1000;
float currentAngle = 0;
float desiredAngle = 0;

/* Ethernet */
EthernetServer server(PORT);

EthernetClient estopBoard;
bool estopConnected = false;

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

void setup() {

    /* Initialization for stepper*/
  
    pinMode(dirPin, OUTPUT);
    pinMode(pulsePin, OUTPUT);
    pinMode(commandInterruptPin, INPUT_PULLUP);
    digitalWrite(pulsePin, HIGH);


    /* Initialization for encoder*/
    //  pinMode(SPI_SCLK, OUTPUT);
    //  pinMode(SPI_MOSI, OUTPUT);
    //  pinMode(SPI_MISO, INPUT);
    pinMode(ENC_PIN, OUTPUT);
    pinMode(RST_ETH, OUTPUT);
    pinMode(CS_ETH, OUTPUT);

    digitalWrite(ENC_PIN, HIGH); // disable encoder on setup
    SPI.begin();


    /* Initialization for ethernet*/
    resetEthernet();

    Ethernet.init(CS_ETH);  // SCLK pin from eth header
    Ethernet.begin(steeringMAC, steeringIP); // initialize ethernet device
  
    Serial.begin(BAUDRATE);
  
    while (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println("Ethernet shield was not found.");
        delay(100);
    }
    
    while(Ethernet.linkStatus() == LinkOFF) {
        Serial.println("Ethernet cable is not connected."); // do something with this
        delay(100);    // TURN down delay to check/startup faster
    }

    server.begin();
    Serial.print("Our address: ");
    Serial.println(Ethernet.localIP());
  
    estopBoard.setConnectionTimeout(ETH_TCP_INITIATION_DELAY);
    estopConnected = estopBoard.connect(estopIP, PORT) > 0;


    wdt_reset();
    wdt_enable(WDTO_500MS);
}
 
void loop() {
    wdt_reset();

    readEthernet();  // check for new angle from ethernet

    if (steeringEnabled)
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
                    desiredAngle = data.substring(2).toFloat(); // set new angle, convert from str to float
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
    digitalWrite(RST_ETH, LOW);
    delay(1);
    digitalWrite(RST_ETH, HIGH);
    delay(501);
}

void sendToEstop() {
    estopConnected = estopBoard.connected();
    if(!estopConnected){
        //Lost TCP connection with the estop board
        estopBoard.connect(estopIP, PORT);
        Serial.println("Lost connection with estop");
    }
    else{
        if(millis() > lastEstopReply + MIN_MESSAGE_SPACING && millis() > lastEstopRequest + MIN_MESSAGE_SPACING){
            RJNet::sendData(estopBoard, estopRequestMsg);
            lastEstopRequest = millis();
        }
    }
}

/* For setting direction of stepper motor */
void assignDirection(){       
    currentAngle = getCurrentAngle();         // read current position from encoder
    if (abs(abs(desiredAngle) - abs(currentAngle)) > STEPPER_DEADBAND){      // checks if motor needs to turn
        if (desiredAngle < currentAngle){      // set dirPIN to CW or CCW
            digitalWrite(dirPin, LOW);
        } else {
            digitalWrite(dirPin, HIGH);
        }
        delayMicroseconds(DIR_DURATION);
    }
}

void stepperPulse(){ // rotates stepper motor one step in the currently set direction
    digitalWrite(pulsePin, LOW);
    delayMicroseconds(PULSE_DURATION);
    digitalWrite(pulsePin, HIGH);
    delayMicroseconds(PULSE_DURATION);
}

void goToPosition(){
    unsigned long startTime = millis();
    while(abs(abs(desiredAngle) - abs(currentAngle)) > STEPPER_DEADBAND || millis() - startTime > STEPPER_TIMEOUT)
    {
        assignDirection();
        stepperPulse();
        currentAngle = getCurrentAngle(); 
    }
}

void setCSLine(uint8_t csLine){   // enable or disable encoder
    digitalWrite(ENC_PIN, csLine);
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
    delay(250); //250 second delay to allow the encoder to reset
}

void resetAMT22(){
    spiWriteRead(NOP, false); // NOP needs to be first byte before extended command can be set

    //this is the time required between bytes as specified in the datasheet.
    delayMicroseconds(3); 
  
    spiWriteRead(RESET, true);
    delay(250); //250 second delay to allow the encoder to start back up
}

float getCurrentAngle(){

    uint16_t currentPosition = getPositionSPI();


}



uint16_t getPositionSPI(){
    uint16_t currentPosition;       //16-bit response from encoder
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
        currentPosition = 0xFFFF; //bad position
    }

    return currentPosition;
}
