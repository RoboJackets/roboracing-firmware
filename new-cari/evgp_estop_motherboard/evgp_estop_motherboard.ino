#include <avr/wdt.h>
#include <Ethernet.h>
#include "evgp_estop_motherboard.h"
#include "RJNet.h"

// ALL STACK LIGHT COLOR OUTPUTS NEED TO BE CHANGED!
byte oldState = 1;        // default state is STOP
byte currentState = 1;
int respondStartTime;
int stateStartTime = millis();
bool hasResponded = 1;
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
IPAddress otherIP(192, 168, 0, 175); //set the IP to find the other board at
EthernetClient otherBoard;


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
  switch(currentState) {
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
  // send state to NUC when state changes
  if(oldState != currentState) {
    if(!otherBoard.connected()) {
      otherBoard.connect(otherIP, PORT);
    }
    else {
      RJNet::sendData(otherBoard, stateMsg);
      isAcknowledged = 0; // now wait for NUC to respond
      respondStartTime = millis();
    }
  }

  // NUC has not responded yet
  if(millis() - respondStartTime >= 100 && !isAcknowledged) {
    respondStartTime = millis();
    if(otherBoard.available()){
      //Server sent us a message
      String serversMessage = RJNet::readData(otherBoard);
      if(serversMessage == "R") {
        isAcknowledged = 1;
      }
    }
  }

  if(millis() - stateRespondTime >= 100) {
    if(!otherBoard.connected()) {
      otherBoard.connect(otherIP, PORT);
    }
    else {
      RJNet::sendData(otherBoard, "S?");
      stateRespondTime = millis();
    }
  }

  // GET STATUS FROM NUC
  
  if(otherBoard.available()){
    //Server sent us a message
    String serversMessage = RJNet::readData(otherBoard);
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

  if(steeringIn && driveIn) {
    oldState = currentState;
    currentState = 0; // everything Enabled
    stateMsg = "E";
  } else if(!driveIn) { // includes steering enabled and disabled
    oldState = currentState;
    currentState = 1;  // everything Disabled
    stateMsg = "D";
  } else {
    oldState = currentState;
    currentState = 2; // steering enabled, drive disabled (Limited)
    stateMsg = "L";
  } 
  
//  executeStateMachine();  // CHANGE, STATUS NEEDS TO COME FROM NUC
  executeEthernetStuff();
}
