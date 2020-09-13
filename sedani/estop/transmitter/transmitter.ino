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
#include <RFM69_ATC.h>     //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69registers.h>
#include <SPI.h>           //included with Arduino IDE install (www.arduino.cc)

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE ************
//*********************************************************************************************
#define NODEID        2    //must be unique for each node on same network (range up to 254, 255 is used for broadcast)
#define NETWORKID     101  //the same on all nodes that talk to each other (range up to 255)
#define GATEWAYID     1
//Match frequency to the hardware version of the radio:
#define FREQUENCY     RF69_915MHZ

#define IS_RFM69HW_HCW  //Only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
//*********************************************************************************************
//Auto Transmission Control - dials down transmit power to save battery
//Usually you do not need to always transmit at max output power
//By reducing TX power even a little you save a significant amount of battery power
//This setting enables this gateway to work with remote nodes that have ATC enabled to
//dial their power down to only the required level (ATC_RSSI)
//#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
//#define ATC_RSSI      -60
//*********************************************************************************************
#define SERIAL_BAUD   115200

//MAKE SURE TO KEEP THESE CODES THE SAME AS RECIEVER

const static uint8_t eStopCode = 's';
const static uint8_t limitedCode = 'l';
const static uint8_t goCode = 'g';
const static byte expectedMessageLength = 1;

uint8_t state = eStopCode;

//Payload is the code (go or stop) to send with the radio.
const static byte payloadLength = 1;
uint8_t payload[payloadLength];

#define TRANSMIT_PERIOD 200 //wait this long after a successful send (in ms)
#define TRANSMIT_FAILED_PERIOD 20 //wait this long if failed to send a burst (ms)
//Each time we try to send a packet, try this many times
#define RETRY_DELAY 50  //how many ms to wait before a retry
#define RETRIES 0  //Retry how many times before failure. 0 means send only once.

//Pins

#ifdef UNO
    
#define LED4_SETUP()  pinMode(13, OUTPUT)
#define LED4_ON()  digitalWrite(13, HIGH)
#define LED4_OFF()  digitalWrite(13, LOW)

#define LED3_ON() 
#define LED3_OFF()

#ifdef ENABLE_ATC
RFM69_ATC radio;
#else
RFM69 radio;
#endif

#else

#define LED4 1
#define LED4_SETUP()  pinMode(LED4, OUTPUT)
#define LED4_ON() digitalWrite(LED4, HIGH)
#define LED4_OFF() digitalWrite(LED4, LOW)

#define LED3_ON() TXLED0
#define LED3_OFF() TXLED1

#ifdef ENABLE_ATC
RFM69_ATC radio(RF69_SPI_CS, 3);
#else
RFM69 radio(RF69_SPI_CS, 3);
#endif

#endif



#define RADIO_RESET A5 //Not needed for UNO, but doesn't hurt anything

void resetRadio();

void setup() {
    delay(1);
    Serial.begin(SERIAL_BAUD);
    
    // Setting LED ouput pin and turning LEDs off
    LED4_SETUP();
    LED3_OFF();
    LED4_OFF();
    
    
    pinMode(RADIO_RESET, OUTPUT);
    resetRadio();
    
    radio.initialize(FREQUENCY,NODEID,NETWORKID);
    radio.setHighPower(); //must include this only for RFM69HW/HCW!
    
    //Dial down transmit speed for increased range.
    //Causes sporadic connection losses
    radio.writeReg(REG_BITRATEMSB, RF_BITRATEMSB_19200);
    radio.writeReg(REG_BITRATELSB, RF_BITRATELSB_19200);
    
    LED4_SETUP();
    //radio.setFrequency(919000000); //set frequency to some custom frequency
    LED4_ON();

//Auto Transmission Control - dials down transmit power to save battery (-100 is the noise floor, -90 is still pretty good)
//For indoor nodes that are pretty static and at pretty stable temperatures (like a MotionMote) -90dBm is quite safe
//For more variable nodes that can expect to move or experience larger temp drifts a lower margin like -70 to -80 would probably be better
//Always test your ATC mote in the edge cases in your own environment to ensure ATC will perform as you expect
#ifdef ENABLE_ATC
    radio.enableAutoPower(ATC_RSSI);
    Serial.println("RFM69_ATC Enabled (Auto Transmission Control)\n");
#endif
}

bool lastSendSuccessful = false;
unsigned long startSendingTime = 0;

void loop() {
    int delayTime = 0;
    
    //Create the payload
    if (state == goCode){  //Copy goCode into payload
        payload[0] = goCode;
    }
    else if (state == limitedCode){
        payload[0] = limitedCode;
    }
    else{  //E_STOPPED: copy the e-stop code into payload
        payload[0] = eStopCode;
    }
    
    //Print what we are sending
    Serial.print("Sending: ");
    for(byte i = 0; i < payloadLength; i++){
        Serial.print((char)payload[i]);
    }
    
    //Send the data. If successful, delay.
    startSendingTime = micros();
    if (radio.sendWithRetry(GATEWAYID, payload, payloadLength, RETRIES, RETRY_DELAY)){
        Serial.print(" ok! RSSI: ");
        Serial.print(radio.RSSI);
        Serial.print(". ms to get an ACK: ");
        Serial.println((micros() - startSendingTime)/1000);
        lastSendSuccessful = true;
        delayTime = TRANSMIT_PERIOD;
    }
    else {
        Serial.print(" SENDING FAILED. ms to FAIL: ");
        Serial.println((micros() - startSendingTime)/1000);
        lastSendSuccessful = false;
        delayTime = TRANSMIT_FAILED_PERIOD;
    }
    
    
    if((millis()/497) % 3 == 0) {
        LED4_ON();
        state = goCode;
    }
    else if((millis()/497) % 3 == 1) {
        LED4_ON();
        state = limitedCode;
    }
    else {
        state = eStopCode;
        LED4_OFF();
    }
    
    lastSendSuccessful ? LED3_ON() : LED3_OFF();
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