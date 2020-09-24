#include <avr/wdt.h>
#include <Ethernet.h>
#include "evgp_estop_motherboard.h"
#include "RJNet.h"

/*
A simplified sketch for the estop motherboard. It does not attempt to display anthing on the stack light
except the state from the radio board. It provides the current state to anything that requests it via ethernet.
*/

const static String stopMsg = "D";
const static String testingMsg = "L";
const static String goMsg = "G";
  
byte currentState = STOP;  // default state is STOP

byte sensor1;             // input from sensor 1
byte sensor2;             // input from sensor 2
byte steeringIn;          // steering input from radio board
byte driveIn;             // drive input from radio board

const static byte PORT = 7; //port RJNet uses
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x03};
IPAddress ip(192, 168, 0, 3); //set the IP to find us at
EthernetServer server(PORT);


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


void steerDriveBrake(byte steer, byte drive, byte brake_release) {
    digitalWrite(STEERING_EN, steer);
    digitalWrite(DRIVE_EN, drive);
    digitalWrite(BRAKE_EN, brake_release);
}


void writeOutCurrentState() {  // CHANGE, STATUS NEEDS TO GO THROUGH NUC FIRST
    switch(currentState) {
    case GO:    // everything enabled
        steerDriveBrake(HIGH, HIGH, HIGH);
        stackLights(ON, OFF, OFF);  
        break; 
    case STOP:    // everything disabled
        steerDriveBrake(LOW, LOW, LOW);
        stackLights(OFF, OFF, ON);
        break;
    case TESTING:    // steering enabled, drive disabled
        steerDriveBrake(HIGH, LOW, HIGH);
        stackLights(OFF, ON, OFF);
        break;
    }
}


void sendStateToClient() {
    EthernetClient client = server.available();
    if (client) {
        String data = RJNet::readData(client);
        if (data.length() != 0) {\
            
            Serial.print(data); //show us what we read 
            Serial.print(" From client ");
            Serial.print(client.remoteIP());
            Serial.print(":");
            Serial.println(client.remotePort());
            
            
            //Doesn't matter what they send us, we'll just send the state
            switch(currentState) {
            case GO:    // everything enabled
                RJNet::sendData(client, goMsg); 
                break; 
            case STOP:    // everything disabled
                RJNet::sendData(client, stopMsg);
                break;
            case TESTING:    // steering enabled, drive disabled
                RJNet::sendData(client, testingMsg);
                break;
            }
        }
        else Serial.println("Empty message recieved.");
    }
}

void resetEthernet(void){
    //Resets the Ethernet shield
    digitalWrite(ETH_RST, LOW);
    delay(1);
    digitalWrite(ETH_RST, HIGH);
    delay(501);
}


void setup() {
    pinMode(INT, INPUT);
    pinMode(SAFE_RB, INPUT);
    pinMode(START_RB, INPUT);
    pinMode(SENSOR_1, INPUT);
    pinMode(SENSOR_2, INPUT);
    pinMode(STEERING_IN, INPUT);
    pinMode(DRIVE_IN, INPUT);
    pinMode(STEERING_EN, OUTPUT);
    pinMode(DRIVE_EN, OUTPUT);
    pinMode(BRAKE_EN, OUTPUT);
    pinMode(STACK_G, OUTPUT);
    pinMode(STACK_Y, OUTPUT);
    pinMode(STACK_R, OUTPUT);
    pinMode(LED1, OUTPUT);
    pinMode(ETH_RST, OUTPUT);
    digitalWrite(DRIVE_EN, LOW);  // Initially start E-stopped
    digitalWrite(BRAKE_EN, LOW);  //
    digitalWrite(STACK_G, HIGH); // change these light settings
    digitalWrite(STACK_Y, HIGH);
    digitalWrite(STACK_R, HIGH); 
    
    Serial.begin(115200);

    // ETHERNET STUFF
    resetEthernet();
    // In case your RJ board wires the chip in an odd config,
    // otherwise, leave commented out
    // You can use Ethernet.init(pin) to configure the CS pin
    Ethernet.init(11);
    Ethernet.begin(mac, ip);
    while(Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
        digitalWrite(LED1, !digitalRead(LED1));
        delay(500);
    }
    while(Ethernet.linkStatus() == LinkOFF) {
        Serial.println("Ethernet cable is not connected.");
        digitalWrite(LED1, !digitalRead(LED1));
        delay(100);
    }
    server.begin();
    Serial.print("Our address: ");
    Serial.println(Ethernet.localIP());

    // WATCHDOG TIMER
    wdt_reset();
    wdt_enable(WDTO_500MS);
}


void loop() {
  wdt_reset();
  
  sensor1 = digitalRead(SENSOR_1);
  sensor2 = digitalRead(SENSOR_2);
  steeringIn = digitalRead(STEERING_IN);
  driveIn = digitalRead(DRIVE_IN);

  if(steeringIn && driveIn) {
    currentState = GO; // everything Enabled
  } else if(!driveIn) { // includes steering enabled and disabled
    currentState = STOP;  // everything Disabled
  } else {
    currentState = TESTING; // steering enabled, drive disabled (Limited)
  }
  
  digitalWrite(LED1, (millis()/1000)%2);
  
  writeOutCurrentState();
  sendStateToClient();
}
