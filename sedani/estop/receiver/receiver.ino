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
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NODEID        1    //unique for each node on same network
#define NETWORKID     101  //the same on all nodes that talk to each other
//Match frequency to the hardware version of the radio:
#define FREQUENCY     RF69_915MHZ

//exactly the same 16 characters/bytes on all nodes!
//Currently disabled, as no point to encryption
#define ENCRYPTKEY    null

#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
//*********************************************************************************************
//Auto Transmission Control - dials down transmit power to save battery
//Usually you do not need to always transmit at max output power
//By reducing TX power even a little you save a significant amount of battery power
//This setting enables this gateway to work with remote nodes that have ATC enabled to
//dial their power down to only the required level
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
//*********************************************************************************************

//How many ms before we decide the connection is lost
//And shut down the motor
#define E_STOP_TIMEOUT 500

/*
MAKE SURE TO KEEP THESE CODES THE SAME AS RECIEVER
We are using 7-byte codes for stop and go to hopefully prevent the radio from 
receiving a stop or go signal by random chance
*/
#define CODE_LENGTH 7
const static byte expectedMessageLength = CODE_LENGTH;
const static uint8_t eStopCode[CODE_LENGTH] = {118, 187, 180, 208, 238, 135, 85};
const static uint8_t goCode[CODE_LENGTH] = {197, 254, 146, 31, 32, 106, 81};

const static bool promiscuousMode = false; //set to 'true' to sniff all packets on the same network
#define SERIAL_BAUD   9600

#ifdef ENABLE_ATC
RFM69_ATC radio;
#else
RFM69 radio;
#endif

#define OUPTUT_LED A0

bool arrayCompare(uint8_t, uint8_t, unsigned int);

void setup() {
    pinMode(OUPTUT_LED, OUTPUT);
    
    Serial.begin(SERIAL_BAUD);
    delay(10);
    
    radio.initialize(FREQUENCY,NODEID,NETWORKID);
    radio.setHighPower(); //must include this only for RFM69HW/HCW!
    radio.encrypt(ENCRYPTKEY);
    radio.promiscuous(promiscuousMode);
    //radio.setFrequency(919000000); //set frequency to some custom frequency
    char buff[50];
    sprintf(buff, "\nListening at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
    Serial.println(buff);
        
#ifdef ENABLE_ATC
    Serial.println("RFM69_ATC Enabled (Auto Transmission Control)");
#endif
}

byte ackCount=0;
uint32_t packetCount = 0;

//Record when the last command was in ms
unsigned long lastCommandTime = 0;

//Record if the message is valid
bool messageValid = false;

//Should the car go or not?
bool go = false;

void loop() {

    if (radio.receiveDone()){
        
        
        //Print out sender, message, and RSSI signal strength
        Serial.print("#[");
        Serial.print(++packetCount);
        Serial.print(']');
        Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
        
        for (byte i = 0; i < radio.DATALEN; i++) {
            Serial.print((char)radio.DATA[i]);
        }
        Serial.print("     [RX_RSSI:");Serial.print(radio.RSSI);Serial.print("]");
        
        if (radio.ACKRequested())
        {
            byte theNodeID = radio.SENDERID;
            radio.sendACK();
            Serial.print(" - ACK sent.");

            // When a node requests an ACK, respond to the ACK
            // and also send a packet requesting an ACK (every 3rd one only)
            // This way both TX/RX NODE functions are tested on 1 end at the GATEWAY
            //May eliminate in final e-stop code
            /*
            if (ackCount++%3==0)
            {
                Serial.print(" Pinging node ");
                Serial.print(theNodeID);
                Serial.print(" - ACK...");
                delay(3); //need this when sending right after reception .. ?
                if (radio.sendWithRetry(theNodeID, "ACK TEST", 8, 0)){    // 0 = only 1 attempt, no retries
                    Serial.println("ok!");
                }
                else{
                    Serial.println("nothing");
                }
            }
            */
        }
        
        //Decode message and verify it
        messageValid = false;
        //If message wrong length, fail immediately
        if (expectedMessageLength >= radio.DATALEN){
            if(arrayCompare(radio.DATA, goCode, CODE_LENGTH)){
                go = true;
                messageValid = true;
            }
            else if (arrayCompare(radio.DATA, eStopCode, CODE_LENGTH)){
                go = false;
                messageValid = true;
            }
            else{
                Serial.println("Invalid message (of correct length) received");
            }
        }
        else {
            Serial.println("Message length too short");
        }
        
        
        //Update lastCommandTime if message was valid
        if (messageValid){
            lastCommandTime = millis();
        }
                
        Serial.println();
    }
    
    //Check if E_STOP is timed out
    if (millis() > lastCommandTime + E_STOP_TIMEOUT){
        go = false;
        Serial.println("LOST SIGNAL");
    }
    
    //Write the signal out to the pins
    if (go){
        digitalWrite(OUPTUT_LED, HIGH);
    }
    else {
        digitalWrite(OUPTUT_LED, LOW);
    }
}
//Compares if two arrays are element-for-element the same
//From https://forum.arduino.cc/index.php?topic=5157.0
//numberOfElements MUST be less than the length of the two arrays
bool arrayCompare(volatile uint8_t *a, uint8_t *b, unsigned int numberOfElements){
    for (unsigned int n=0;n<numberOfElements;n++){
        if (a[n]!=b[n]){
            return false;
        }
    }
    return true;
}