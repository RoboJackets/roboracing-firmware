#include "pins.h"
#include "Ethernet.h"
#include "RJNet.h"

// Interrupt stuff
volatile unsigned long pwm_rc_angle = 0;
volatile unsigned long pwm_rc_speed = 0;
volatile unsigned long pwm_rc_control = 0;

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

float rc_angle = 0;
float rc_speed = 0;
bool rc_control;

float nuc_speed = 0;
float nuc_angle = 0;

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
const static String nucAngleStringHeader = " a=";

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
  pwm_rc_angle = micros()-prev_time_ch_1;
  attachInterrupt(digitalPinToInterrupt(CH_1), isr_rising_ch_1, RISING);
}

void isr_rising_ch_2() {
  prev_time_ch_2 = micros();
  attachInterrupt(digitalPinToInterrupt(CH_2), isr_falling_ch_2, FALLING);
}
 
void isr_falling_ch_2() {
  pwm_rc_speed = micros()-prev_time_ch_2;
  attachInterrupt(digitalPinToInterrupt(CH_2), isr_rising_ch_2, RISING);
}

void isr_rising_ch_3() {
  prev_time_ch_3 = micros();
  attachInterrupt(digitalPinToInterrupt(CH_3), isr_falling_ch_3, FALLING);
}
 
void isr_falling_ch_3() {
  pwm_rc_control = micros()-prev_time_ch_3;
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
      Serial.println(rc_angle);
      Serial.print(" CH_2 ");
      Serial.println(rc_speed);
      Serial.print(" CH_3 ");
      Serial.println(rc_control);
      led_1_state = !led_1_state;
      led_2_state = !led_2_state; // TODO can we use one of these leds for manual_state?
      set_led_1(led_1_state);
      set_led_2(led_2_state);
    }

    evaluate_state();

    if(millis() - startTime >= 100){ // Spec calls for sending at 10 Hz
      sendNewMessages();
      startTime = millis();
    }
    
    delay(50); // TODO should this be decreased?
}

// TODO Verify with desired functionality 
void readAllNewMessages(){ 
  EthernetClient client = manualServer.available();    // if there is a new message from client create client object, otherwise new client object, if evaluated, is false
  while (client) {
    String data = RJNet::readData(client);  // if i get string from RJNet buffer (v= $float or a=$float)
    IPAddress clientIP = client.remoteIP();
    if (data.length() != 0) {   // if data exists
      client.setConnectionTimeout(ETH_TCP_INITIATION_DELAY);   //Set connection delay so we don't hang
      if (clientIP == nucIP) {
        if (nucDriveStringHeader.equals(data.substring(0,2))){    // if client is giving us new angle
          String reply = ackMsg;
          RJNet::sendData(client, reply); // Reply "R"
          
          nuc_speed = parseSpeedMessage(data); 
          }
        else if (nucAngleStringHeader.equals(data.substring(0,2))){    // if client is giving us new steering
          String reply = ackMsg;
          RJNet::sendData(client, reply); // Reply "R"
          
          nuc_angle = parseSpeedMessage(data); 
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
//      else if (rc_control) // in manual mode
//      {
//        RJNet::sendData(steeringBoard, manualSteeringStringHeader + String(rc_angle)); // sending RC angle to nuc
//        RJNet::sendData(driveBoard, manualDriveStringHeader + String(rc_speed)); // sending RC velocity to nuc
//      }

      if (!steeringBoard.connected()) {
        steeringBoard.connect(steeringIP, PORT);
        Serial.println("Lost connection with steering");
      }
      else if (manual_state)
      {
        RJNet::sendData(steeringBoard, manualSteeringStringHeader + String(rc_angle)); // sending an angle to steering board from rc
      }
      else
      {
        RJNet::sendData(steeringBoard, manualSteeringStringHeader + String(nuc_angle)); // sending an angle to steering board from nuc
      }
    
      if (!driveBoard.connected()) {
        driveBoard.connect(driveIP, PORT);
        Serial.println("Lost connection with drive");
      }
      else if (manual_state)
      {
        RJNet::sendData(driveBoard, manualDriveStringHeader + String(rc_speed)); // sending a velocity to the drivebrake board from rc
      }
      else
      {
        RJNet::sendData(driveBoard, manualDriveStringHeader + String(nuc_speed)); // sending a velocity to the drivebrake board from nuc
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
        pwm_rc_angle = 0;
        pwm_rc_speed = 0;
        pwm_rc_control = 0;
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

void evaluate_state(){ // Determines if RC-controlled or NUC-controlled
    if(rc_present_state && rc_control){ // TODO should there be anything else here??
      manual_state = true;
      }
    else{
      manual_state = false;
      }
}

// TODO Review
void evaluate_ch_1() {
  rc_angle = map(pwm_rc_angle, ch_1_lower, ch_1_upper, 0, 100) / 100.0;
  rc_angle = rc_angle * (abs(rc_angle) > 0.02); // dead zone of 2%, becomes zero if false
}

// TODO Review
void evaluate_ch_2() {
  rc_speed = map(pwm_rc_speed, ch_2_lower, ch_2_upper, 0, 100) / 100.0;
  rc_speed = rc_speed * (abs(rc_speed) > 0.02);
  if (rc_speed < 0) {
    rc_speed = rc_speed * 10;
  } else {
    rc_speed = rc_speed * 40;
  }
}

void evaluate_ch_3() {
  rc_control = pwm_rc_control > ch_3_mid;
}

float parseSpeedMessage(const String speedMessage){
    //takes in message from nuc and converts it to a float desired speed.
    return speedMessage.substring(2).toFloat();
}

float parseAngleMessage(const String AngleMessage){
    //takes in message from nuc and converts it to a float desired speed.
    return AngleMessage.substring(2).toFloat();
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
