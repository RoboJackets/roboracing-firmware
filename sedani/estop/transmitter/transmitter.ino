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
#include <SPI.h>           //included with Arduino IDE install (www.arduino.cc)

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE ************
//*********************************************************************************************
#define NODEID        2    //must be unique for each node on same network (range up to 254, 255 is used for broadcast)
#define NETWORKID     101  //the same on all nodes that talk to each other (range up to 255)
#define GATEWAYID     1
//Match frequency to the hardware version of the radio:
#define FREQUENCY     RF69_915MHZ

//exactly the same 16 characters/bytes on all nodes!
//Currently disabled, as no point to it
#define ENCRYPTKEY    null

#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
//*********************************************************************************************
//Auto Transmission Control - dials down transmit power to save battery
//Usually you do not need to always transmit at max output power
//By reducing TX power even a little you save a significant amount of battery power
//This setting enables this gateway to work with remote nodes that have ATC enabled to
//dial their power down to only the required level (ATC_RSSI)
//#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -60
//Possible antenna https://arduinodiy.wordpress.com/2015/07/25/coil-loaded-433-mhz-antenna/
//*********************************************************************************************
#define SERIAL_BAUD   9600

//MAKE SURE TO KEEP THIS THE SAME AS RECIEVER
#define CODE_LENGTH 7
const static uint8_t eStopCode[CODE_LENGTH] = {118, 187, 180, 208, 238, 135, 85};
const static uint8_t goCode[CODE_LENGTH] = {197, 254, 146, 31, 32, 106, 81};

//Payload is the code (go or stop) to send with the radio
const static byte payloadLength = CODE_LENGTH;
uint8_t payload[payloadLength];

#define TRANSMIT_PERIOD 200 //transmit a packet to gateway so often (in ms)
#define RETRY_DELAY 20  //how many ms to wait before a retry
#define RETRIES 3  //Retry how many times before failure

//Not sure what this does
char buff[20];

#define OUPTUT_LED A0

#ifdef ENABLE_ATC
RFM69_ATC radio;
#else
RFM69 radio;
#endif

void setup() {
    Serial.begin(SERIAL_BAUD);
    radio.initialize(FREQUENCY,NODEID,NETWORKID);
    radio.setHighPower(); //must include this only for RFM69HW/HCW!
    radio.encrypt(ENCRYPTKEY);
    
    pinMode(OUPTUT_LED, OUTPUT);
    //radio.setFrequency(919000000); //set frequency to some custom frequency

//Auto Transmission Control - dials down transmit power to save battery (-100 is the noise floor, -90 is still pretty good)
//For indoor nodes that are pretty static and at pretty stable temperatures (like a MotionMote) -90dBm is quite safe
//For more variable nodes that can expect to move or experience larger temp drifts a lower margin like -70 to -80 would probably be better
//Always test your ATC mote in the edge cases in your own environment to ensure ATC will perform as you expect
#ifdef ENABLE_ATC
    radio.enableAutoPower(ATC_RSSI);
#endif

    char buff[50];
    sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
    Serial.println(buff);

#ifdef ENABLE_ATC
    Serial.println("RFM69_ATC Enabled (Auto Transmission Control)\n");
#endif
}

bool linkEstablished = false;
bool go = false;
unsigned int lastPeriod = 0;
unsigned int currPeriod;

void loop() {

    //check for any received packets
    if (radio.receiveDone())
    {
        Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
        for (byte i = 0; i < radio.DATALEN; i++) {
            Serial.print((char)radio.DATA[i]);
        }
        Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);Serial.print("]");
  
        if (radio.ACKRequested()) {
            radio.sendACK();
            Serial.print(" - ACK sent");
        }
        Serial.println();
    }
    
     //Only send data every TRANSMIT_PERIOD ms
    currPeriod = millis()/TRANSMIT_PERIOD;
    if (currPeriod != lastPeriod) {
        lastPeriod=currPeriod;
        
        //Create the payload
        if (go){  //Copy goCode into payload
            for(byte i = 0; i < CODE_LENGTH; i++){
                payload[i] = goCode[i];
            }
        }
        else{  //E_STOPPED: copy the e-stop code into payload
            for(byte i = 0; i < CODE_LENGTH; i++){
                payload[i] = goCode[i];
            }
        }
        
        Serial.print("Sending: ");
        for(byte i = 0; i < payloadLength; i++){
            Serial.print((char)payload[i]);
        }
  
        if (radio.sendWithRetry(GATEWAYID, payload, payloadLength, RETRIES, RETRY_DELAY)){
            Serial.print(" ok!");
            linkEstablished = true;
        }
        else {
            Serial.print(" SENDING FAILED");
            linkEstablished = false;
        }
    
        Serial.println();
    }
    
    digitalWrite(OUPTUT_LED, linkEstablished);
}