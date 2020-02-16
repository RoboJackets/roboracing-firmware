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
//#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
//#define ATC_RSSI      -60
//*********************************************************************************************

//How many ms before we decide the connection is lost
//And shut down the motor
#define E_STOP_TIMEOUT 500

/*
MAKE SURE TO KEEP THESE CODES THE SAME AS RECIEVER
*/
const static uint8_t eStopCode = 98;
const static uint8_t goCode = 97;
const static byte expectedMessageLength = 1;

const static bool promiscuousMode = false; //set to 'true' to sniff all packets on the same network
#define SERIAL_BAUD   115200

//#define UNO

#ifdef UNO
    
#define LED1_SETUP  pinMode(13, OUTPUT)
#define LED1_ON  digitalWrite(13, HIGH)
#define LED1_OFF  digitalWrite(13, LOW)

#define LED2_ON 
#define LED2_OFF

#ifdef ENABLE_ATC
RFM69_ATC radio;
#else
RFM69 radio;
#endif

#else

#define LED1 1
#define LED1_SETUP  pinMode(LED1, OUTPUT)
#define LED1_ON digitalWrite(LED1, HIGH)
#define LED1_OFF digitalWrite(LED1, LOW)

#define LED2_ON TXLED0
#define LED2_OFF TXLED1

#ifdef ENABLE_ATC
RFM69_ATC radio(RF69_SPI_CS, 3);
#else
RFM69 radio(RF69_SPI_CS, 3);
#endif

#endif

#define RADIO_RESET A5 //Not needed for UNO, but doesn't hurt anything



void resetRadio();
bool compareData(uint8_t, uint8_t, unsigned int);

void setup() {
    // Setting LED ouput pin
    LED1_SETUP;
    
    Serial.begin(SERIAL_BAUD);
    delay(10);
    
    radio.initialize(FREQUENCY,NODEID,NETWORKID);
    radio.setHighPower(); //must include this only for RFM69HW/HCW!
    
    //Dial down transmit speed for increased range.
    //Causes sporadic connection losses
    radio.writeReg(REG_BITRATEMSB, RF_BITRATEMSB_19200);
    radio.writeReg(REG_BITRATELSB, RF_BITRATELSB_19200);
        
#ifdef ENABLE_ATC
    radio.enableAutoPower(ATC_RSSI);
    Serial.println("RFM69_ATC Enabled (Auto Transmission Control)");
#endif
}

byte ackCount=0;
uint32_t packetCount = 0;

//Record when the last command was in ms
unsigned long lastCommandTime = 0;

//Record if the message is valid
bool messageValid = false;

//Record if we currently have a connection
bool connectionEstablished = false;

//Should the car go or not?
bool go = false;

// Number of loops per status LED state change
const static uint32_t blinkLoopsReceiver = 100;
uint32_t blinkCount = 0;
bool blinkState = false;

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
        }
        
        //Decode message and verify it
        messageValid = false;
        //If message wrong length, fail immediately
        if (expectedMessageLength >= radio.DATALEN){
            if(radio.DATA[0] == goCode){
                go = true;
                messageValid = true;
            }
            else if (radio.DATA[0] == eStopCode){
                go = false;
                messageValid = true;
            }
            else{
                Serial.println("Invalid message received");
            }
        }
        else {
            Serial.println("Message length too short");
        }
        
        
        //Update lastCommandTime if message was valid
        if (messageValid){
            lastCommandTime = millis();
            connectionEstablished = true;
        }
                
        Serial.println();
    }
    
    //Check for millis() rollovers. If it did, reset lastCommandTime to 0.
    if (millis() < lastCommandTime){
        lastCommandTime = 0;
    }
    
    //Check if E_STOP is timed out
    if (millis() > lastCommandTime + E_STOP_TIMEOUT){
        if (connectionEstablished){
            Serial.println("LOST SIGNAL");
        }
        connectionEstablished = false;
        go = false;
    }
    
    //Write the signal out to the pins
    if (go){
        LED1_ON;
        //Serial.println("GO");
    }
    else {
        // Status LED blinks if e-stopped
        if (blinkCount < blinkLoopsReceiver){
            blinkCount++;
        }
        else {
            blinkState = !blinkState;
            if(blinkState){
                LED1_ON;
            } else {
                LED1_OFF;
            }
            blinkCount = 0;
        }
    }
    //Serial.println("Finished");
}

//Compares if two arrays are element-for-element the same
//From https://forum.arduino.cc/index.php?topic=5157.0
//numberOfElements MUST be less than the length of the two arrays
bool compareData(volatile uint8_t *a, uint8_t *b, unsigned int numberOfElements){
    for (unsigned int n=0;n<numberOfElements;n++){
        if (a[n]!=b[n]){
            return false;
        }
    }
    return true;
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