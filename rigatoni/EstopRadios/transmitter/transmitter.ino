//EAGLE RFM69HCW model: https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/downloads
//Hookup Guide: https://learn.sparkfun.com/tutorials/rfm69hcw-hookup-guide
//Modified from sample sketch by Felix Rusu 2016, http://www.LowPowerLab.com/contact
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#include <RFM69.h>         //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69registers.h>
#include <SPI.h>           //included with Arduino IDE install (www.arduino.cc)
#include <avr/wdt.h>

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE ************
//*********************************************************************************************
#define NODEID        2    //must be unique for each node on same network (range up to 254, 255 is used for broadcast)
#define NETWORKID     101  //the same on all nodes that talk to each other (range up to 255)
#define GATEWAYID     1
//Match frequency to the hardware version of the radio:
#define FREQUENCY     RF69_915MHZ   //This identifies what module we have - 868 or 915MHz
#define FREQ_ADJUST   928000000     //This is the carrier frequency we want

#define IS_RFM69HW_HCW  //Only for RFM69HW/HCW! Leave out if you have RFM69W/CW!

#define SERIAL_BAUD   115200

//MAKE SURE TO KEEP THESE CODES THE SAME AS RECIEVER

const static uint8_t dieCode = 'd';
const static uint8_t stationaryCode = 's';
const static uint8_t eStopCode = 'e';
const static uint8_t resetCode = 'r';
const static uint8_t goCode = 'g';
const static byte expectedMessageLength = 1;

uint8_t state = resetCode; // initial state is reset

//Payload is the code (go or stop) to send with the radio.
const static byte payloadLength = 1;
uint8_t payload[payloadLength];

#define TRANSMIT_PERIOD 200 //wait this long after a successful send  to send again (in ms)
#define TRANSMIT_FAILED_PERIOD 20 //wait this long if failed to send a burst (ms)
#define RETRY_DELAY 50  //how many ms to wait before declaring sending failed and retrying
#define RETRIES 3  //Retry how many times before failure of a burst. 0 means send only once.

//Pins

#define LED4 1
#define LED4_SETUP()  pinMode(LED4, OUTPUT)
#define LED4_ON() digitalWrite(LED4, HIGH)
#define LED4_OFF() digitalWrite(LED4, LOW)

#define LED3_ON() TXLED0
#define LED3_OFF() TXLED1

RFM69 radio(RF69_SPI_CS, 3);


/***************PINS***************/
//LEDS on the controller
#define BLUE_LED 13
#define GREEN_LED 5
#define YELLOW_LED 10
#define RED_LED 9

//Buttons on the controller
#define GO_BUTTON 6
#define LIMITED_BUTTON 8
#define STOP_BUTTON 12

//Radio reset pin
#define RADIO_RESET A5 //Not needed for UNO, but doesn't hurt anything

void resetRadio();
void setUpRemoteIOPins();
void showStateOnLEDs(uint8_t);
void writeToRemoteLEDs(bool, bool, bool);

void setup() {
    delay(1);
    
    // Setting LED ouput pin and turning LEDs off
    LED4_SETUP();
    LED3_OFF();
    LED4_OFF();
    
    setUpRemoteIOPins();
    
    //Light ALL LEDS to indicate setup
    digitalWrite(BLUE_LED, HIGH);
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(YELLOW_LED, HIGH);
    digitalWrite(RED_LED, HIGH);
    
    Serial.begin(SERIAL_BAUD);
    
    pinMode(RADIO_RESET, OUTPUT);
    resetRadio();
    
    radio.initialize(FREQUENCY,NODEID,NETWORKID);
    radio.setHighPower(); //must include this only for RFM69HW/HCW!
    radio.setFrequency(FREQ_ADJUST);
    
    //Dial down transmit speed for increased range.
    //Causes sporadic connection losses
    radio.writeReg(REG_BITRATEMSB, RF_BITRATEMSB_19200);
    radio.writeReg(REG_BITRATELSB, RF_BITRATELSB_19200);
    
    LED4_ON();

    digitalWrite(BLUE_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(RED_LED, LOW);

    wdt_reset();
    wdt_enable(WDTO_1S);
}

bool lastSendSuccessful = false;
unsigned long startSendingTime = 0;

void loop() {
    wdt_reset();
    int delayTime = 0;
    
    //Get state from push-buttons
    state = readStateFromButtons(state);
    showStateOnLEDs(state);
    
    //Create the payload
    payload[0] = state;
    
    //Print what we are sending
    Serial.print("Sending: ");
    for(byte i = 0; i < payloadLength; i++){
        Serial.println((char)payload[i]);
    }
    
    //Send the data. If successful, delay.
    startSendingTime = micros();
    if (radio.sendWithRetry(GATEWAYID, payload, payloadLength, RETRIES, RETRY_DELAY)){
        //Serial.print(" ok! RSSI: ");
        //Serial.print(radio.RSSI);
        //Serial.print(". ms to get an ACK: ");
        //Serial.println((micros() - startSendingTime)/1000);
        lastSendSuccessful = true;
        delayTime = TRANSMIT_PERIOD;
    }
    else {
        //Serial.print(" SENDING FAILED. ms to FAIL: ");
        //Serial.println((micros() - startSendingTime)/1000);
        lastSendSuccessful = false;
        delayTime = TRANSMIT_FAILED_PERIOD;
    }
    
    
    lastSendSuccessful ? LED3_ON() : LED3_OFF();
    digitalWrite(BLUE_LED, lastSendSuccessful ? HIGH : LOW);
    
    delay(delayTime);   
}

void resetRadio(){
    //Ensure we don't call reset too quickly
    digitalWrite(RADIO_RESET, LOW);
    delayMicroseconds(5100);
    //Execute reset
    digitalWrite(RADIO_RESET, HIGH);
    delayMicroseconds(150);
    digitalWrite(RADIO_RESET, LOW);
    delayMicroseconds(5100);
}

void setUpRemoteIOPins(){
    pinMode(BLUE_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    pinMode(YELLOW_LED, OUTPUT);
    pinMode(RED_LED, OUTPUT);
    
    pinMode(GO_BUTTON, INPUT_PULLUP);
    pinMode(LIMITED_BUTTON, INPUT_PULLUP);
    pinMode(STOP_BUTTON, INPUT_PULLUP);
}

bool buttonsStillPressedStationary = false;
bool buttonsStillPressedEStop = false;

uint8_t readStateFromButtons(uint8_t curr_state){
    //Buttons are LOW when pushed
    bool go_button = !digitalRead(GO_BUTTON);
    bool limited_button = !digitalRead(LIMITED_BUTTON);
    bool stop_button = !digitalRead(STOP_BUTTON);
    
    //This is true from the time when the DIE code was pressed until all buttons are released
    buttonsStillPressedStationary = buttonsStillPressedStationary && (go_button || limited_button);
    buttonsStillPressedEStop = buttonsStillPressedEStop && (limited_button || stop_button);
    
    if(curr_state == stationaryCode && buttonsStillPressedStationary){
        //Commanded to die and all remote buttons have not yet been released
        return stationaryCode;
    }
    else if(curr_state == eStopCode && buttonsStillPressedEStop){
        //Commanded to die and all remote buttons have not yet been released
        return eStopCode;
    }
    else if (limited_button && stop_button) {
        buttonsStillPressedEStop = true;
        return eStopCode;
    }
    else if (go_button && limited_button){
        buttonsStillPressedStationary = true;
        return stationaryCode;
    }
    else if (stop_button){
        return dieCode;
    }
    else if (limited_button){
        return resetCode;
    }
    else if (go_button){
        return goCode;
    }
    return curr_state;
}

void showStateOnLEDs(uint8_t curr_state){
    if (curr_state == dieCode){
        writeToRemoteLEDs(false, false, true);
    }
    else if (curr_state == eStopCode){
        writeToRemoteLEDs(false, true, true);
    }
    else if (curr_state == stationaryCode){
        writeToRemoteLEDs(true, true, false);
    }
    else if (curr_state == resetCode){
        writeToRemoteLEDs(false, true, false);
    }
    else if (curr_state == goCode){
        writeToRemoteLEDs(true, false, false);
    }
    else {
        //Internal code fault
        Serial.print("WARNING: INVALID STATE");
        Serial.println(curr_state);
        writeToRemoteLEDs(true, true, true);
    }
}

void writeToRemoteLEDs(bool green, bool yellow, bool red){
    digitalWrite(GREEN_LED, green);
    digitalWrite(YELLOW_LED, yellow);
    digitalWrite(RED_LED, red);
}
    
