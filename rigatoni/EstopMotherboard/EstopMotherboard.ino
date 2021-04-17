#include "RigatoniNetwork.h"
#include <avr/wdt.h>
#include <Ethernet.h>
#include "EstopMotherboard.h"
#include "RJNet.h"


const static String stopMsg = "D";
const static String limitedMsg = "L";
const static String goMsg = "G";
const static String nucRequestStateMsg = "S?";

const static String nucResponseGo = "G";
const static String nucResponseHalt = "H";


const static String genRequestStateMsg = "S?";

//If we receive this, indicates a hardware failure and we should permanently stop
const static String hardwareFailure = "FAIL";

bool isPermanentlyStopped = false;
byte currentState = STOP;  // default state is STOP

//byte sensor1;           // input from sensor 1 TODO Not implemented on current version
//byte sensor2;           // input from sensor 2 TODO Not implemented on current version
byte steeringIn;          // steering input from radio board
byte driveIn;             // drive input from radio board
byte remoteState = STOP;  


EthernetServer server(PORT);

// NUC 
const int timeToNucTimeoutMS = 500;
unsigned long lastNUCReply = 0;
bool nucConnected = false;

unsigned long lastTimePrintedNucConnFail = 0;
unsigned long timeAtEndOfStartup = 0;

byte nucState = GO; // Default state GO to operate without NUC connected 


void stackLights(byte green, byte yellow, byte red) {
    /*
    Sets the stack light to the specified state. Each byte is ON, OFF, or BLINK (predefined constants)
    We handle blinking by checking to see what time it is every time this function is called
    So if this function isn't being called, blinking lights won't blink
    */
    bool blink_status = (millis()/BLINK_PERIOD_MS) % 2;  //If blinking lights should be on or off right now
    
    if(green == BLINK){
        digitalWrite(STACK_G, blink_status);
    }
    else{
        digitalWrite(STACK_G, green == ON);
    }
    
    if(yellow == BLINK){
        digitalWrite(STACK_Y, blink_status);
    }
    else{
        digitalWrite(STACK_Y, yellow == ON);
    }
    
    if(red == BLINK){
        digitalWrite(STACK_R, blink_status);
    }
    else{
        digitalWrite(STACK_R, red == ON);
    }
}

void steerDrive(byte steer, byte drive) {
    digitalWrite(STEERING_EN, steer);
    digitalWrite(DRIVE_EN, drive);
}


void writeOutCurrentState() {  // CHANGE, STATUS NEEDS TO GO THROUGH NUC FIRST
    switch(currentState) {
    case GO:    // everything enabled
        steerDrive(HIGH, HIGH);
        stackLights(ON, OFF, OFF);  
        break; 
    case STOP:    // everything disabled
        steerDrive(LOW, LOW);
        stackLights(OFF, OFF, ON);
        break;
    case LIMITED:    // steering enabled, drive disabled
        steerDrive(HIGH, LOW);
        stackLights(OFF, ON, OFF);
        break;
    }
}


void respondToClient() {
    EthernetClient client = server.available();
    while (client) {
        client.setConnectionTimeout(ETH_TCP_INITIATION_DELAY);   //Set connection delay so we don't hang
        String data = RJNet::readData(client);
        if (data.length() != 0) {
            Serial.print(data); //show us what we read 
            Serial.print(" From client ");
            Serial.print(client.remoteIP());
            Serial.print(":");
            Serial.println(client.remotePort());

            if(data.equals(hardwareFailure))
            {
                isPermanentlyStopped = true;
                currentState = STOP;
                Serial.print("HARDWARE FAULT: MESSAGE: ");
            }
            else if(client.remoteIP() == nucIP && data.equals(nucResponseGo)){
                nucState = GO;
                lastNUCReply = millis();
                sendStateToClient(client);
            }
            else if(client.remoteIP() == nucIP && data.equals(nucResponseHalt)){
                nucState = STOP;
                lastNUCReply = millis();
                sendStateToClient(client);
            }
            else if(data.equals(genRequestStateMsg))
            {
                sendStateToClient(client);
            }
            else
            {
                Serial.println("Invalid message recieved.");
            }
        }
        else
        {
            Serial.println("Empty message recieved.");
        }
        client = server.available();
    }
}

void sendStateToClient(EthernetClient the_client){
    if(the_client.connected())
    {
        switch(currentState) {
            case GO:    // everything enabled
                RJNet::sendData(the_client, goMsg); 
                break; 
            case STOP:    // everything disabled
                RJNet::sendData(the_client, stopMsg);
                break;
            case LIMITED:    // steering enabled, drive disabled
                RJNet::sendData(the_client, limitedMsg);
                break;
        }
    }
}


void resetEthernet(void){
    //Resets the Ethernet shield
    digitalWrite(ETH_RST, LOW);
    delay(1);
    digitalWrite(ETH_RST, HIGH);
    delay(501);
}

void evaluateState(void){

    //sensor1 = digitalRead(SENSOR_1); TODO Not implemented on current version
    //sensor2 = digitalRead(SENSOR_2); TODO Not implemented on current version
    steeringIn = digitalRead(STEERING_IN);
    driveIn = digitalRead(DRIVE_IN);

    // Check remote state
    if(steeringIn && driveIn)
    {
        remoteState = GO; // everything Enabled
    }
    else if(steeringIn && !driveIn)
    {
        remoteState = LIMITED; // steering enabled, drive disabled (Limited)
    }
    else if(!steeringIn && !driveIn)
    { 
        remoteState = STOP;  // everything Disabled
    }
    else
    {
        remoteState = STOP; 
        Serial.println("INVALID REMOTE STATE!");
    }

    // Evaluate remote and nuc states
    if (isPermanentlyStopped)
    {
        //hardware fault. Permanently stopped.
        currentState = STOP;
        Serial.println("HARDWARE FAULT!");
    }
    else
    {
        // Written out more explicitly for clarity of state machine
        if(remoteState == STOP)  // Remote stop takes precedence over NUC no matter what
        {
            currentState = STOP;
        }
        else if((remoteState == GO || remoteState == LIMITED) && nucState == STOP) // If NUC is on and connected to say STOP, stop. NOTE: nucState default GO on start up
        {
            currentState = STOP;
        }
        else if(remoteState == LIMITED && nucState == GO)
        {
            currentState = LIMITED;
        }
        else if(remoteState == GO && nucState == GO) // If both Go, GO!
        {
            currentState = GO;
        }
        else
        {
            currentState = STOP;
            Serial.println("UNEXPECTED STATE!");
        }

    } 

}


void setup() {
    pinMode(INT, INPUT);
    pinMode(SAFE_RB, INPUT);
    pinMode(SENSOR_1, INPUT);
    pinMode(SENSOR_2, INPUT);
    pinMode(STEERING_IN, INPUT);
    pinMode(DRIVE_IN, INPUT);
    pinMode(STEERING_EN, OUTPUT);
    pinMode(DRIVE_EN, OUTPUT);
    pinMode(STACK_G, OUTPUT);
    pinMode(STACK_Y, OUTPUT);
    pinMode(STACK_R, OUTPUT);
    pinMode(LED1, OUTPUT);
    pinMode(ETH_RST, OUTPUT);
    //pinMode(ETH_CS_PIN, OUTPUT);
    //pinMode(17, OUTPUT);   //Default SS pin as output
    digitalWrite(STEERING_EN, LOW);  // Initially start E-stopped
    digitalWrite(DRIVE_EN, LOW);  // Initially start E-stopped
    digitalWrite(STACK_G, HIGH); // change these light settings
    digitalWrite(STACK_Y, HIGH);
    digitalWrite(STACK_R, HIGH); 
    
    Serial.begin(115200);

	//********** Ethernet Initialization *************//
    resetEthernet();
    // In case your RJ board wires the chip in an odd config,
    // otherwise, leave commented out
    // You can use Ethernet.init(pin) to configure the CS pin

    Ethernet.init(ETH_CS_PIN);
    Ethernet.begin(estopMAC, estopIP);

    unsigned long loopCounter = 0;
    while(Ethernet.hardwareStatus() == EthernetNoHardware) {
        digitalWrite(LED1, loopCounter++ % 4 == 0);
        Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
        delay(100);
    }

    while(Ethernet.linkStatus() == LinkOFF) {
        digitalWrite(LED1, loopCounter++ % 4 > 0);
        Serial.println("Ethernet cable is not connected.");
        delay(100);
    }

    Ethernet.setRetransmissionCount(ETH_NUM_SENDS); //Set number resends before failure
    Ethernet.setRetransmissionTimeout(ETH_RETRANSMISSION_DELAY_MS);  //Set timeout delay before failure

    server.begin();
    Serial.print("Our address: ");
    Serial.println(Ethernet.localIP());

    // WATCHDOG TIMER
    wdt_reset();
    wdt_enable(WDTO_500MS);
}

void loop() {
    wdt_reset();
    evaluateState();

    digitalWrite(LED1, !digitalRead(LED1));

    writeOutCurrentState();
    respondToClient();
    
    nucConnected = millis() - lastNUCReply < timeToNucTimeoutMS; 
    
    if(!nucConnected && (millis() - 500 > lastTimePrintedNucConnFail)){
        Serial.println("No connection with NUC.");
    }
}
