#include <avr/wdt.h>
#include "EstopMotherboard.h"
#include <Ethernet.h>
#include <EthernetUdp.h>
#include "RJNetUDP.h"
#include "RigatoniNetworkUDP.h"


const static char stopMsg[] = "D";
const static char limitedMsg[] = "L";
const static char goMsg[] = "G";

//If we receive this, indicates a hardware failure and we should permanently stop
const static String hardwareFailure = "FAIL";

bool isPermanentlyStopped = false;
byte currentState = STOP;  // default state is STOP

byte steeringIn;          // steering input from radio board
byte driveIn;             // drive input from radio board
byte remoteState = STOP;  


unsigned long lastTimePrintedStatus = 0;
unsigned long start_time = 0;

byte nucState = GO; // Default state GO to operate without NUC connected 

EthernetUDP Udp;


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

void broadcastState() {
    if (millis() - start_time >= MIN_MESSAGE_SPACING) {
        //So we are treating the C-style string as a pointer here because it
        //decays to a pointer when we pass it into a function.
        const char * currentStateMessage;
        switch(currentState) {
            case GO:    // everything enabled
                currentStateMessage = goMsg;
                break; 
            case STOP:    // everything disabled
                currentStateMessage = limitedMsg;
                break;
            case LIMITED:    // steering enabled, drive disabled
                currentStateMessage = stopMsg;
                break;
            }

        RJNetUDP::sendMessage(currentStateMessage, Udp, broadcastIP);
        start_time = millis();
    }
}

void checkAllMessages(){
    //Reads all UDP messages to prevent buffer from filling. Doesn' do anything with them though
    Message message1 = RJNetUDP::receiveMessage(Udp);
    while(message1.received) {
        Serial.print("Message from IP: ");
        Serial.print(message1.ipaddress);
        Serial.print(" : ");
        Serial.println(message1.message);
        message1 = RJNetUDP::receiveMessage(Udp);
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
    
    delay(100);
    
    Serial.begin(115200);

	//********** Ethernet Initialization *************//
    resetEthernet();
    // In case your RJ board wires the chip in an odd config,
    // otherwise, leave commented out
    // You can use Ethernet.init(pin) to configure the CS pin
    
    delay(10);

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

    //If you don't set retransmission to 0, the WIZNET will retry 8 times if it cannot resolve the
    //MAC address of the destination using ARP and block for a long time.
    //With this as 0, will only block for ETH_RETRANSMISSION_DELAY_MS
    Ethernet.setRetransmissionCount(0);
    Ethernet.setRetransmissionTimeout(ETH_RETRANSMISSION_DELAY_MS);  //Set timeout delay before failure of ARP

    //server.begin();
    Udp.begin(RJNetUDP::RJNET_PORT);
    Serial.print("Our address: ");
    Serial.println(Ethernet.localIP());

    // WATCHDOG TIMER
    wdt_reset();
    wdt_enable(WDTO_1S);
}

void loop() {
    wdt_reset();
    evaluateState();

    digitalWrite(LED1, !digitalRead(LED1));

    writeOutCurrentState();
    broadcastState();
    checkAllMessages();
    
    
    if(millis() - 500 > lastTimePrintedStatus){
        lastTimePrintedStatus = millis();
        if(isPermanentlyStopped){
            Serial.print("HARDWARE FAULT ON ");
            //Serial.print(hardwareFaultIP);
        }
        
        Serial.print("Overall state: ");
        Serial.println(currentState);
    }
}
