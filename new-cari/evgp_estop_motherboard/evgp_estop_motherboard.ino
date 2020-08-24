#include <avr/wdt.h>
#include <Ethernet.h>
#include "evgp_estop_mnucServer.h"
#include "RJNet.h"

// ALL STACK LIGHT COLOR OUTPUTS NEED TO BE CHANGED!
/*
State numbering:
0: GO. Everything enabled.
1: Stop. Motor + steering disabled, emergency brake enabled.
2: Testing. Steering enabled, drive motor disabled.
*/
byte oldState = 1;        // default state is STOP
byte currentState = 1;
byte nucState = 1;
int respondStartTime;
int stateStartTime = millis();
bool sentAcknowledged = 0;
bool receivedAcknowledged = 0;
byte sensor1;             // input from sensor 1
byte sensor2;             // input from sensor 2
byte steeringIn;          // steering input from radio board
byte driveIn;             // drive input from radio board

const static byte PORT = 7; //port RJNet uses
double counter = 0;
long int startTime = 0;
String stateMsg;
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x03};
IPAddress ip(192, 168, 0, 3); //set the IP to find us at
EthernetServer server(PORT);
IPAddress nucIP(192, 168, 0, 175); //set the IP to find the other board at
EthernetClient nucServer;


void stackLights(byte G, byte Y, byte R) {
  digitalWrite(STACK_G, G);
  digitalWrite(STACK_Y, Y);
  digitalWrite(STACK_R, R);
}


void steerDriveBrake(byte steer, byte drive, byte brake) {
  digitalWrite(STEERING_EN, steer);
  digitalWrite(DRIVE_EN, drive);
  digitalWrite(BRAKE_EN, brake);
}


void executeStateMachine() {  // CHANGE, STATUS NEEDS TO GO THROUGH NUC FIRST
  switch(nucState) {
    case 0:    // everything enabled
      steerDriveBrake(HIGH, HIGH, HIGH);
      stackLights(1, 0, 0);  
      break; 
    case 1:    // everything disabled and emergency break enabled
      steerDriveBrake(LOW, LOW, LOW);
      stackLights(0, 0, 1);
      break;
    case 2:    // steering enabled, drive disabled
      steerDriveBrake(HIGH, LOW, HIGH);
      stackLights(0, 1, 0);
      break;
  }
}


void executeEthernetStuff() {
  // send state change to NUC
  //This is wierd - if we are not connected, we attempt to connect and don't send the data. But below we still wait for the NUC's response to that data.
  if(oldState != currentState) {
    if(!nucServer.connected()) {
      nucServer.connect(nucIP, PORT);
    }
    else {
      RJNet::sendData(nucServer, stateMsg);
      sentAcknowledged = 1; // now wait for NUC to respond
      respondStartTime = millis();
    }
  }

  // Wait for NUC to respond to sent state change
  //we don't execute this code block when we have just sent a new state
  if(millis() - respondStartTime >= 100 && sentAcknowledged) {
    if(nucServer.available()){
      String serversMessage = RJNet::readData(nucServer);
      respondStartTime = millis();
      if(serversMessage == "R") {
          //If server message not R, we have an infinite loop here
        sentAcknowledged = 0;
      }
    }
  }

  // Request state from NUC
  if(millis() - stateRespondTime >= 100) {
    if(!nucServer.connected()) {
      nucServer.connect(nucIP, PORT);
    }
    else {
      RJNet::sendData(nucServer, "S?");
      receivedAcknowledged = 1; // now wait for NUC to respond
    }
  }

  // Receive state from NUC
  if(receivedAcknowledged && nucServer.available()) {
    String serversMessage = RJNet::readData(nucServer);
    if(serversMessage == "G") { // everything ENABLED
      nucStatus = 0;
    }
    else if(serversMessage = "H") { // everything DISABLED
      nucStatus = 1;
    }
//    else if(serversMessage = "??????CHANGE") { // LIMITED
//      nucStatus = 2;
//    }
    receivedAcknowledged = 0;
  }
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
//  pinMode(LED, OUTPUT);
  digitalWrite(DRIVE_EN, LOW);  // Initially start E-stopped
  digitalWrite(BRAKE_EN, LOW);  //
  digitalWrite(STACK_G, HIGH); // change these light settings
  digitalWrite(STACK_Y, HIGH);
  digitalWrite(STACK_R, HIGH); 
//  TXLED0;

  // ETHERNET STUFF
  Serial.begin(115200);
  // In case your RJ board wires the chip in an odd config,
  // otherwise, leave commented out
  // You can use Ethernet.init(pin) to configure the CS pin
  Ethernet.init(10);  // Most Arduino shields  CHANGE?
  Ethernet.begin(mac, ip);
  while(Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    delay(500);
  }
  while(Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
    delay(500);
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

  oldState = currentState;
  if(steeringIn && driveIn) {
    currentState = 0; // everything Enabled
    stateMsg = "E";
  } else if(!driveIn) { // includes steering enabled and disabled
    currentState = 1;  // everything Disabled
    stateMsg = "D";
  } else {
    currentState = 2; // steering enabled, drive disabled (Limited)
    stateMsg = "L";
  } 
  
//  executeStateMachine();  // CHANGE, STATUS NEEDS TO COME FROM NUC
  executeEthernetStuff();
}
