#include "pins.h"
#include "Ethernet.h"
#include "RJNet.h"

// Interrupt stuff
volatile unsigned long pwm_value_ch_1 = 0;
volatile unsigned long pwm_value_ch_2 = 0;
volatile unsigned long pwm_value_ch_3 = 0;

volatile unsigned long prev_time_ch_1 = 0;
volatile unsigned long prev_time_ch_2 = 0;
volatile unsigned long prev_time_ch_3 = 0;

unsigned long startTime;

// PWM limits from RC
#define ch_1_lower 1536
#define ch_1_upper 1852
#define ch_2_lower 1494
#define ch_2_upper 2020
#define ch_3_mid 1494

// RJNet ethernet port
#define PORT 7

// Ethernet stuffs
#define ETH_NUM_SENDS 2
#define ETH_RETRANSMISSION_DELAY_MS 50  //Time before TCP tries to resend the packet
#define ETH_TCP_INITIATION_DELAY 50 //How long we wait before .connected() or .connect() fails.

bool led_1_state = true;
bool led_2_state = false;

bool rc_present_state = false;
bool rc_prev_state = true;

bool manual_state = false; // true if RC controlled, false if NUC controlled

float value_ch_1;
float value_ch_2;
bool value_ch_3;

float nucSpeed = 0;
float nucSteering = 0;

// float value_throttle; For manual throttle, not implemented in this version

/* Ethernet */
const static byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 6}; // manual board's mac address
const static IPAddress manualIP(192, 168, 0, 6); // manual board's IP address
EthernetServer manualServer(PORT);

// Enter a IP address for nuc below, nuc client to manual
const static IPAddress nucIP(192, 168, 0, 2); //set the IP of the NUC
EthernetClient nuc;  // client 

// Enter a IP address for steering below, manual client to steering and drive
const static IPAddress steeringIP(192, 168, 0, 5); //set the IP of the steering board
EthernetClient steeringBoard; // client

const static IPAddress driveIP(192, 168, 0, 7); //set the IP of the drive board
EthernetClient driveBoard; // client

//TCP Connection status to brake and estop
bool driveConnected = false;
bool steeringConnected = false;

/****************Messages from clients****************/
//Universal acknowledge message
const static String ackMsg = "R";

//NUC commanded speed header
const static String nucDriveStringHeader = "v=";

//NUC commanded steering header
const static String nucSteeringStringHeader = " a=";

/****************Messages to clients****************/
//Manual RC speed command header
const static String manualDriveStringHeader = "v=";

//Manual RC steering angle command header
const static String manualSteeringStringHeader = "S=";

// Interrupt stuff
void isr_rising_ch_1() {
  prev_time_ch_1 = micros();
  attachInterrupt(digitalPinToInterrupt(CH_1), isr_falling_ch_1, FALLING);
}
 
void isr_falling_ch_1() {
  pwm_value_ch_1 = micros()-prev_time_ch_1;
  attachInterrupt(digitalPinToInterrupt(CH_1), isr_rising_ch_1, RISING);
}

void isr_rising_ch_2() {
  prev_time_ch_2 = micros();
  attachInterrupt(digitalPinToInterrupt(CH_2), isr_falling_ch_2, FALLING);
}
 
void isr_falling_ch_2() {
  pwm_value_ch_2 = micros()-prev_time_ch_2;
  attachInterrupt(digitalPinToInterrupt(CH_2), isr_rising_ch_2, RISING);
}

void isr_rising_ch_3() {
  prev_time_ch_3 = micros();
  attachInterrupt(digitalPinToInterrupt(CH_3), isr_falling_ch_3, FALLING);
}
 
void isr_falling_ch_3() {
  pwm_value_ch_3 = micros()-prev_time_ch_3;
  attachInterrupt(digitalPinToInterrupt(CH_3), isr_rising_ch_3, RISING);
}

void pin_assign(){
    pinMode(RC_IN, INPUT);
  
//    pinMode(THROTTLE, INPUT); NOT USED
//    pinMode(SWITCH, INPUT);   NOT USED
  
    pinMode(LED_1, OUTPUT);
    pinMode(LED_2, OUTPUT);

    pinMode(CH_1, INPUT);
    pinMode(CH_2, INPUT);
    pinMode(CH_3, INPUT);

    // Ethernet pins
    pinMode(ETH_INT_PIN, INPUT);
    //pinMode(ETH_RST_PIN, INPUT);  NOT POSSIBLE Not connected in current version
    pinMode(ETH_CS_PIN, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(CH_1), isr_rising_ch_1, RISING);
    attachInterrupt(digitalPinToInterrupt(CH_2), isr_rising_ch_2, RISING);
    attachInterrupt(digitalPinToInterrupt(CH_3), isr_rising_ch_3, RISING);
}

void rc_missing();
void evaluate_manual();
void evaluate_ch_1();
void evaluate_ch_2();
void evaluate_ch_3();
void set_led_1(bool);
void set_led_2(bool);

void setup(){
    pin_assign();
    Serial.begin(115200);
    startTime = 0;

    /* Initialization for ethernet*/
    // NOT POSSIBLE - Reset for ethernet is not broken out on microcontroller
    // resetEthernet();
    Ethernet.init(ETH_CS_PIN);  // SCLK pin from eth header
    Ethernet.begin(mac, manualIP); // initialize ethernet device

    
    while (Ethernet.hardwareStatus() == EthernetNoHardware) {
         Serial.println("Ethernet shield was not found.");
         delay(50);
    }
    while(Ethernet.linkStatus() == LinkOFF) {
         Serial.println("Ethernet cable is not connected."); // do something with this
         delay(50);    // TURN down delay to check/startup faster
    }

    Ethernet.setRetransmissionCount(ETH_NUM_SENDS);
    Ethernet.setRetransmissionTimeout(ETH_RETRANSMISSION_DELAY_MS);
    manualServer.begin();
    startTime = millis();

    
}



void loop() {
    readAllNewMessages();
    
    rc_missing();
    if(rc_present_state){
      evaluate_ch_1();
      evaluate_ch_2();
      evaluate_ch_3();
      Serial.print(" CH_1 ");
      Serial.println(value_ch_1);
      Serial.print(" CH_2 ");
      Serial.println(value_ch_2);
      Serial.print(" CH_3 ");
      Serial.println(value_ch_3);
      led_1_state = !led_1_state;
      led_2_state = !led_2_state;
      set_led_1(led_1_state);
      set_led_2(led_2_state);
    }

    evaluate_manual();

    if(millis() - startTime >= 500){
      sendNewMessages();
    }
    startTime = millis();
    
    delay(50);
}

// TODO Verify with desired functionality 
void readAllNewMessages(){ 
  EthernetClient client = manualServer.available();    // if there is a new message from client create client object, otherwise new client object, if evaluated, is false
  while (client) {
    String data = RJNet::readData(client);  // if i get string from RJNet buffer (v= $float a=$float)
    IPAddress clientIP = client.remoteIP();
    if (data.length() != 0) {   // if data exists
      client.setConnectionTimeout(ETH_TCP_INITIATION_DELAY);   //Set connection delay so we don't hang
      if (clientIP == nucIP) {
        if (manualDriveStringHeader.equals(data.substring(0,2))){    // if client is giving us new angle
          String reply = ackMsg;
  //        if (rc_present_state){         // if in manual mode
  //          reply = "M";
  //        } else {                       // if in autonomous mode
  //          reply = "A";
  //        }
          RJNet::sendData(client, reply);
          // TODO parse data message   
               
        }
        else {
          Serial.print("Invalid message received from nuc");  
        }
      }
      else
      {
        Serial.print("Invalid message received from ");
        Serial.println(clientIP);
      }
    }
    else {
      Serial.print("Empty message received from ");
      Serial.println(clientIP); 
    }
    client = manualServer.available();  //Go to next message
  }
}

void sendNewMessages() { // now manual board is acting as a client
      if (!nuc.connected()) {
        nuc.connect(nucIP, PORT);
        Serial.println("Lost connection with nuc");
      }
//      else if (value_ch_3) // in manual mode
//      {
//        RJNet::sendData(steeringBoard, manualSteeringStringHeader + String(value_ch_1)); // sending RC angle to nuc
//        RJNet::sendData(driveBoard, manualDriveStringHeader + String(value_ch_2)); // sending RC velocity to nuc
//      }

      if (!steeringBoard.connected()) {
        steeringBoard.connect(steeringIP, PORT);
        Serial.println("Lost connection with steering");
      }
      else if (manual_state)
      {
        RJNet::sendData(steeringBoard, manualSteeringStringHeader + String(value_ch_1)); // sending an angle to steering board
      }
      else
      {
        RJNet::sendData(steeringBoard, manualSteeringStringHeader + String(nucSteering)); // sending an angle to steering board
      }
    
      if (!driveBoard.connected()) {
        driveBoard.connect(driveIP, PORT);
        Serial.println("Lost connection with drive");
      }
      else if (manual_state)
      {
        RJNet::sendData(driveBoard, manualDriveStringHeader + String(value_ch_2)); // sending a velocity to the drivebrake board
      }
      else
      {
        RJNet::sendData(driveBoard, manualDriveStringHeader + String(nucSpeed)); // sending a velocity to the drivebrake board
      }
      
}


// Set status LEDs
void set_led_1(bool on){
    digitalWrite(LED_1, on);
}

void set_led_2(bool on){
    digitalWrite(LED_2, on);
}

/*  NOT POSSIBLE Manual state not used in this version
void evaluate_manual_switch(){
    manual_state = digitalRead(SWITCH);
} */

void rc_missing(){
    rc_present_state = digitalRead(RC_IN);
    if(!rc_present_state){
        Serial.println("RC Reciever Missing");
        pwm_value_ch_1 = 0;
        pwm_value_ch_2 = 0;
        pwm_value_ch_3 = 0;
        prev_time_ch_1 = 0;
        prev_time_ch_2 = 0;
        prev_time_ch_3 = 0;
        if(rc_prev_state){
            detachInterrupt(digitalPinToInterrupt(CH_1));
            detachInterrupt(digitalPinToInterrupt(CH_2));
            detachInterrupt(digitalPinToInterrupt(CH_3));
        }
    } else if (rc_present_state && !rc_prev_state){
        attachInterrupt(digitalPinToInterrupt(CH_1), isr_rising_ch_1, RISING);
        attachInterrupt(digitalPinToInterrupt(CH_2), isr_rising_ch_2, RISING);
        attachInterrupt(digitalPinToInterrupt(CH_3), isr_rising_ch_3, RISING);
    }
    rc_prev_state = rc_present_state;
}

void evaluate_manual(){
    if(rc_present_state && value_ch_3){
      manual_state = true;
      }
    else{
      manual_state = false;
      }
}

// TODO Review
void evaluate_ch_1() {
  value_ch_1 = map(pwm_value_ch_1, ch_1_lower, ch_1_upper, 0, 100) / 100.0;
  value_ch_1 = value_ch_1 * (abs(value_ch_1) > 0.02); // dead zone of 2%, becomes zero if false
}

// TODO Review
void evaluate_ch_2() {
  value_ch_2 = map(pwm_value_ch_2, ch_2_lower, ch_2_upper, 0, 100) / 100.0;
  value_ch_2 = value_ch_2 * (abs(value_ch_2) > 0.02);
  if (value_ch_2 < 0) {
    value_ch_2 = value_ch_2 * 10;
  } else {
    value_ch_2 = value_ch_2 * 40;
  }
}

void evaluate_ch_3() {
  value_ch_3 = pwm_value_ch_3 > ch_3_mid;
}

/*  NOT POSSIBLE For manual drive, not used in current version
void evaluate_throttle() {
  value_throttle = map(analogRead(THROTTLE), 0, 1023, 0, 100) / 100.0;
}*/

//  NOT POSSIBLE ETH Not resetable in current board version
//void resetEthernet(void){
//    //Resets the Ethernet shield
//    digitalWrite(ETH_RST_PIN, LOW);
//    delay(1);
//    digitalWrite(ETH_RST_PIN, HIGH);
//    delay(501);
//}
