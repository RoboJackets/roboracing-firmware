#include "RigatoniNetwork.h"
#include <avr/wdt.h>
#include "pins.h"
#include "Ethernet.h"
#include "RJNet.h"

// PWM limits from RC
#define PWM_DEADBAND 20

#define PWM_CH_1_LOWER 1000
#define PWM_CH_1_MID 1500
#define PWM_CH_1_UPPER 2040

#define PWM_CH_2_LOWER 1020
#define PWM_CH_2_MID 1500
#define PWM_CH_2_UPPER 2040

#define PWM_CH_3_MID 1494

#define MAX_VELOCITY_FORWARD 10 // m/s
#define MAX_VELOCITY_BACKWARD -2 // m/s

#define MAX_ANGLE 0.52 // radians

#define NUC_TIMEOUT_MS 1000
#define STEERING_TIMEOUT_MS 1000
#define DRIVE_TIMEOUT_MS 1000 

#define REMOTE_TIMEOUT_COUNT 20

// Interrupt stuff
volatile unsigned long pwm_rc_angle = 0;
volatile unsigned long pwm_rc_speed = 0;
volatile unsigned long pwm_rc_control = 0;

unsigned long last_pwm_rc_angle = 0;
unsigned long current_remote_timeout_count = 0;

volatile unsigned long prev_time_ch_1 = 0;
volatile unsigned long prev_time_ch_2 = 0;
volatile unsigned long prev_time_ch_3 = 0;

bool led_1_state = true;
bool led_2_state = false;

bool remote_present_state = false;
bool rx_present_state = false;
bool rx_prev_state = true;

bool rcrx_present_state = false;

bool manual_state = true; // true if RC controlled, false if NUC controlled

float rc_angle = 0;
float rc_speed = 0;
bool rc_control;

float nuc_speed = 0;
float nuc_angle = 0;

unsigned long lastNUCCommand = 0;
unsigned long lastDriveReply = 0;
unsigned long lastSteeringReply = 0;

unsigned long lastDriveCommand = 0;
unsigned long lastSteeringCommand = 0;

unsigned long lastPrintTime = 0;

//End of startup. Needed so we don't connect for X seconds
unsigned long endOfStartupTime = 0;

// float value_throttle; For manual throttle, not implemented in this version

/* Ethernet */
EthernetServer manualServer(PORT);

// Enter a IP address for steering below, manual client to steering and drive
EthernetClient steeringBoard; // manual is client to steering

EthernetClient driveBoard; // manual is client to drive

//TCP Connection status to drive and steering
bool driveConnected = false;
bool steeringConnected = false;

/****************Messages from clients****************/
//Universal acknowledge message
const static String ackMsg = "R";

//NUC commanded speed header
const static String nucDriveStringHeader = "v=";

//NUC commanded steering header
const static String nucAngleStringHeader = "a=";

//NUC requested mode
const static String nucModeRequest = "S?";
const static String nucAutoMode = "A";
const static String nucManualMode = "M";

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

void rcrx_missing();
void evaluate_manual();
void evaluate_ch_1();
void evaluate_ch_2();
void evaluate_ch_3();
void set_led_1(bool);
void set_led_2(bool);

void setup(){
    pin_assign();

    Serial.begin(115200);

    /* Initialization for ethernet*/
    // NOT POSSIBLE - Reset for ethernet is not broken out on microcontroller
    // resetEthernet();
    Ethernet.init(ETH_CS_PIN);  // SCLK pin from eth header
    Ethernet.begin(manualMAC, manualIP); // initialize ethernet device

    unsigned long loopCounter = 0;
    while(Ethernet.hardwareStatus() == EthernetNoHardware) {
        set_led_1(loopCounter++ % 4 == 0);
        Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
        delay(100);
    }

    while(Ethernet.linkStatus() == LinkOFF) {
        set_led_1(loopCounter++ % 4 > 0);
        Serial.println("Ethernet cable is not connected.");
        delay(100);
    }

    Ethernet.setRetransmissionCount(ETH_NUM_SENDS);
    Ethernet.setRetransmissionTimeout(ETH_RETRANSMISSION_DELAY_MS);

    manualServer.begin();

    steeringBoard.setConnectionTimeout(ETH_TCP_INITIATION_DELAY);
    driveBoard.setConnectionTimeout(ETH_TCP_INITIATION_DELAY);

    endOfStartupTime = millis();
    
    // WATCHDOG TIMER
    wdt_reset();
    wdt_enable(WDTO_500MS);
    
}

void loop() {
    wdt_reset();
    readAllNewMessages();
    sendNewMessages();
    
    rcrx_missing();
    if(rcrx_present_state){
        evaluate_ch_1();
        evaluate_ch_2();
        evaluate_ch_3();
        led_1_state = !led_1_state;
        set_led_1(led_1_state);
    }

    evaluate_state();
    set_led_2(manual_state);

    if(millis() - lastNUCCommand > NUC_TIMEOUT_MS){ // Check fail condition timer
        nuc_speed = 0;
        nuc_angle = 0;
        Serial.println("NUC timed out");
    }

    
    if(millis() - 500 > lastPrintTime){
        lastPrintTime = millis();
        if(manual_state){
            Serial.print("Manual mode. ");
        }
        else{
            Serial.print("Autonomous mode. ");
        }
        Serial.print(" RC speed: ");
        Serial.print(rc_speed);
        Serial.print(" NUC speed: ");
        Serial.print(nuc_speed);
        Serial.print(" RC angle: ");
        Serial.print(rc_angle);
        Serial.print(" NUC angle: ");
        Serial.println(nuc_angle);
    }
}

void readAllNewMessages(){ 
    EthernetClient client = manualServer.available();    // if there is a new message from client create client object, otherwise new client object, if evaluated, is false
    while (client) {
        String data = RJNet::readData(client);  // if i get string from RJNet buffer (v= $float or a=$float)
        IPAddress clientIP = client.remoteIP();
        client.setConnectionTimeout(ETH_TCP_INITIATION_DELAY);   //Set connection delay so we don't hang
        if (data.length() != 0) {   // if data exists
            Serial.println(data);
            if (clientIP == nucIP)
            {
                if (data.substring(0,2).equals(nucModeRequest))
                {
                    String fullReply = "v="+String(rc_speed)+",a="+String(rc_angle)+",m=";
                    if(manual_state)
                    {
                        fullReply = fullReply + nucManualMode;
                    }
                    else
                    {
                        fullReply = fullReply + nucAutoMode;
                    }
                    
                    RJNet::sendData(client, fullReply);
                    lastNUCCommand = millis();
                    Serial.print("Responded with: ");
                    Serial.println(fullReply);
                }
                else if (parseValidateSpeedAngleMessage(data) == 0)
                {  
                    //New angle + speed command is valid
                    //parseValidateSpeedAngleMessage sets variables directly
                    RJNet::sendData(client, ackMsg); // Reply "R"
                    lastNUCCommand = millis(); // Reset fail condition timer
                }
                else
                {
                    Serial.print("Invalid message received from NUC");  
                }
            }
            else
            {
                Serial.print("Invalid message received from ");
                Serial.println(clientIP);
            }     
        }
        else
        {
            Serial.print("Empty message received from ");
            Serial.println(clientIP); 
        }
        client = manualServer.available();  //Go to next client for a message
    }

    while (driveBoard.available())  // Check for messages from drive client
    {
        String data = RJNet::readData(driveBoard);
        if (data.length() == 0 || data.substring(0,1) != ackMsg)
        {
            Serial.println("INVALID MESSAGE FROM DRIVE");
        }
        else
        {
            lastDriveReply = millis();
        }
    }

    while (steeringBoard.available())  // Check for messages from steering client
    {
        String data = RJNet::readData(steeringBoard);
        if (data.length() == 0 || data.substring(0,1) != ackMsg)
        {
            Serial.println("INVALID MESSAGE FROM STEERING");
        }
        else
        {
            lastSteeringReply = millis();
        }
    }

}

void sendNewMessages() {
    if(millis() - endOfStartupTime < MS_AFTER_STARTUP_BEFORE_CLIENT_CONNECT){
        //Do nothing before MS_AFTER_STARTUP_BEFORE_CLIENT_CONNECT
        steeringConnected = false;
        driveConnected = false;
        return;
    }

    // Steering message send
    steeringConnected = steeringBoard.connected();
    if (!steeringConnected){
        steeringBoard.connect(steeringIP, PORT);
        Serial.println("Lost connection with steering");
    }
    else if (millis() > lastSteeringReply + MIN_MESSAGE_SPACING && millis() > lastSteeringCommand + MIN_MESSAGE_SPACING){
        lastSteeringCommand = millis();
        RJNet::sendData(steeringBoard, manualSteeringStringHeader + String(manual_state ? rc_angle : nuc_angle));
    }
    
    // Drive message send
    driveConnected = driveBoard.connected();
    if (!driveConnected){
        driveBoard.connect(driveIP, PORT);
        Serial.println("Lost connection with drive");
    }
    else if (millis() > lastDriveReply + MIN_MESSAGE_SPACING && millis() > lastDriveCommand + MIN_MESSAGE_SPACING){
        lastDriveCommand = millis();
        RJNet::sendData(driveBoard, manualDriveStringHeader + String(manual_state ? rc_speed : nuc_speed)); // sending a velocity to the drivebrake board from rc
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


// Issue with turning the remote off, it sends the last command for CH1 and CH3.
// This function takes advantage of the noise of the PWM when the remote is on
// to act as a timeout if repeating PWM is seen (indicating remote is off / out of reach).
void remote_missing()
{
    if(last_pwm_rc_angle == pwm_rc_angle)
    {
        if(remote_present_state)
        {
            current_remote_timeout_count = current_remote_timeout_count + 1;
            if(current_remote_timeout_count > REMOTE_TIMEOUT_COUNT)
            {
                remote_present_state = false;
            }
        }
    }
    else
    {
        last_pwm_rc_angle = pwm_rc_angle;
        current_remote_timeout_count = 0;
        remote_present_state = true;
    }
}

// KNOWN ISSUE
// When the receiver gets reattached, the board reboots
void rcrx_missing(){

    remote_missing();
    rx_present_state = digitalRead(RC_IN);
    if(!rx_present_state){
        Serial.println("RC Receiver Missing");
        pwm_rc_angle = 0;
        pwm_rc_speed = 0;
        pwm_rc_control = 0;

        prev_time_ch_1 = 0;
        prev_time_ch_2 = 0;
        prev_time_ch_3 = 0;
        if(rx_prev_state){
            detachInterrupt(digitalPinToInterrupt(CH_1));
            detachInterrupt(digitalPinToInterrupt(CH_2));
            detachInterrupt(digitalPinToInterrupt(CH_3));
        }
    } else if (rx_present_state && !rx_prev_state){
        Serial.println("RC Receiver is back!");
        attachInterrupt(digitalPinToInterrupt(CH_1), isr_rising_ch_1, RISING);
        attachInterrupt(digitalPinToInterrupt(CH_2), isr_rising_ch_2, RISING);
        attachInterrupt(digitalPinToInterrupt(CH_3), isr_rising_ch_3, RISING);
    }
    rx_prev_state = rx_present_state;

    if(!rx_present_state || !remote_present_state){
        // Set actionable values to safe defaults
        rc_angle = 0;
        rc_speed = 0;
        manual_state = true;
        rcrx_present_state = false;
    }
    else
    {
        rcrx_present_state = true;
    }
}

void evaluate_state(){ // Determines if RC-controlled or NUC-controlled
    if(rx_present_state && rc_control){ 
        manual_state = true;
    }
    else{
        manual_state = false;
    }
}

void evaluate_ch_1() {
    // Scales from PWM to m/s
    if(pwm_rc_angle < PWM_CH_1_LOWER || pwm_rc_angle > PWM_CH_1_UPPER)
    {   //checks if invalid PWM
        rc_angle = 0;
    }
    else if(abs(pwm_rc_angle-PWM_CH_1_MID) < PWM_DEADBAND)
    {   //deadband to prevent PWM jitter from changing speed
        rc_angle = 0;
    }
    else if(pwm_rc_angle > PWM_CH_1_MID)
    {   //counter clockwise mapping
        rc_angle = map(pwm_rc_angle, PWM_CH_1_MID, PWM_CH_1_LOWER, 0, MAX_ANGLE*1000)/1000.0;
    }
    else if(pwm_rc_angle < PWM_CH_1_MID)
    {   //clockwise mapping
        rc_angle = map(pwm_rc_angle, PWM_CH_1_MID, PWM_CH_1_UPPER, 0, -MAX_ANGLE*1000)/1000.0;
    }
    else
    {
        Serial.println("INVALID CH1!");
    }
}

void evaluate_ch_2() {
    // Scales from PWM to m/s
    if(pwm_rc_speed < PWM_CH_2_LOWER || pwm_rc_speed > PWM_CH_2_UPPER)
    {   //checks if invalid PWM
        rc_speed = 0;
    }
    else if(abs(pwm_rc_speed-PWM_CH_2_MID) < PWM_DEADBAND)
    {   //deadband to prevent PWM jitter from changing speed
        rc_speed = 0;
    }
    else if(pwm_rc_speed > PWM_CH_2_MID)
    {   //forward mapping
        rc_speed = map(pwm_rc_speed, PWM_CH_2_MID, PWM_CH_2_UPPER, 0, MAX_VELOCITY_FORWARD*100.0)/100.0;
    }
    else if(pwm_rc_speed < PWM_CH_2_MID)
    {   //backward mapping
        rc_speed = map(pwm_rc_speed, PWM_CH_2_MID, PWM_CH_2_LOWER, 0, MAX_VELOCITY_BACKWARD*100.0)/100.0;
    }
    else
    {
        Serial.println("INVALID CH2!");
    }
}

void evaluate_ch_3() {
    rc_control = pwm_rc_control < PWM_CH_3_MID; // manual mode in power-on state
}

int parseValidateSpeedAngleMessage(const String speedMessage){
    //takes in message from nuc and sets nuc_angle and nuc_speed
    //Returns 1 for invalid message, 0 for valid message. If message invalid, does not modify
    //target speed or angle
    int message_break_pt = speedMessage.indexOf(nucAngleStringHeader);  //Returns -1 if not present
    if(speedMessage.substring(0,2).equals(nucDriveStringHeader) && message_break_pt > 0){
        nuc_speed = speedMessage.substring(2).toFloat();
        nuc_angle = speedMessage.substring(message_break_pt + 2).toFloat();
        return 0;
    }
    Serial.print("Invalid command message from NUC: ");
    Serial.println(speedMessage);
    return 1;
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
