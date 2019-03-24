/*
TODO:
get a calibration curve for the lidars

*/

#include <Wire.h>
#include <LIDARLite.h>

#define NUMBER_OF_LIDARS 2

#define DEBUG   //If you comment this line, the DPRINT & DPRINTLN lines are defined as blank.
#ifdef DEBUG    //Macros are usually in all capital letters.
  #define DPRINT(...)    Serial.print(__VA_ARGS__)     //DPRINT is a macro, debug print
  #define DPRINTLN(...)  Serial.println(__VA_ARGS__)   //DPRINTLN is a macro, debug print with new line
#else
  #define DPRINT(...)     //now defines a blank line
  #define DPRINTLN(...)   //now defines a blank line
#endif

//Define pin array to be scanned for valid lidar scanners
const int lidarEnablePinArray[] = {A5,A4};
byte activeLidarCounter = 0;

const byte loopsPerRecalibration = 50;

const byte lidarDefaultAddress = 0x62;        //default address
byte recalibrationCounter = 0;

// Timeout Variables
unsigned long lastMessageTime;
const unsigned long timeoutDuration = 1000;
bool isTimedOut = true; 

struct LIDAR
{
	LIDARLite myLidarLite;
    byte address;
    byte pin;
	unsigned int currentSumReads = 0;
	unsigned int currentNumReads = 0;
};

LIDAR lidarUnits[NUMBER_OF_LIDARS];            //Array of lidarUnits to store configuration data

void initializeLidars();

void setup() 
{
    Wire.begin();

	//Start serial communications
    Serial.begin(115200);
    while(!Serial){
        delay(1);
    }

    // Setup enable pins
   	for(int i=0; i < NUMBER_OF_LIDARS; i++){
        pinMode(lidarEnablePinArray[i],OUTPUT);
        digitalWrite(lidarEnablePinArray[i], LOW);
    }

    initializeLidars();

    delay(100);
    DPRINT(activeLidarCounter);
    DPRINTLN(" devices detected and readdressed");
}



void loop()
{
	
    //Loop through each Lidar unit and read distance data
    while(!getMessage()){
        if(isTimedOut){
            DPRINTLN("TIMED OUT!");
            break;
        }
        
    	for(int i = 0; i < activeLidarCounter; i++)
		{                
			if (recalibrationCounter == i){
				lidarUnits[i].currentSumReads += lidarUnits[i].myLidarLite.distance(true, lidarUnits[i].address);
			}
			else{
				lidarUnits[i].currentSumReads += lidarUnits[i].myLidarLite.distance(false, lidarUnits[i].address);
			}
			lidarUnits[i].currentNumReads++;
		}
		recalibrationCounter++;
		recalibrationCounter = recalibrationCounter % loopsPerRecalibration;
    }

	//print data
	if (lidarUnits[0].currentNumReads == 0){
		DPRINTLN("NO DATA ERROR!");
		DPRINTLN(micros());
	}
	
	DPRINTLN("Number of reads: " + String(lidarUnits[0].currentNumReads));
	
	//Replace with Ethernet stuff
	for(int i = 0; i < activeLidarCounter; i++){
		Serial.println("Average LIDAR " + String(i) + " value: " + String(lidarUnits[i].currentSumReads/lidarUnits[i].currentNumReads));
		lidarUnits[i].currentSumReads = 0;
		lidarUnits[i].currentNumReads = 0;
	}
}

void initializeLidars(){
    //Sequentially Enable the enable pins to detect active Lidar sensors
    for(int i=0; i < NUMBER_OF_LIDARS; i++) 
    {
    	//enable the enable pin
        digitalWrite(lidarEnablePinArray[i], HIGH); 
        //Wait 25ms to enable comms to Lidar unit
        delay(25);     
        
        //Attempt to communicate to device
        Wire.beginTransmission(lidarDefaultAddress);
        delay(10);
        //Comm error
        int error = Wire.endTransmission();
        //Communications attempt successful
        if(error == 0){
            lidarUnits[activeLidarCounter].pin = lidarEnablePinArray[i]; //Set pin number
            // << 1 for 7 bit address
            lidarUnits[activeLidarCounter].address = lidarDefaultAddress - ((activeLidarCounter + 1) << 1);  //Set device address
            DPRINT("Device found on pin ");
            DPRINTLN(lidarUnits[activeLidarCounter].pin);
            //Write address to LIDAR
            lidarUnits[activeLidarCounter].myLidarLite.begin(0,true);
            //TODO Verify necessity of configure line
            lidarUnits[activeLidarCounter].myLidarLite.configure(0);
            // Sets new address of enabled lidar
            lidarUnits[activeLidarCounter].myLidarLite.setI2Caddr(lidarUnits[activeLidarCounter].address, 1, lidarDefaultAddress);

            //checking address change
            // TODO Verify most effective way to change address
            for(char a=0;a<255;a+=2){
                Wire.beginTransmission(a);    //Attempt to communicate to device
                delay(1);
                int error = Wire.endTransmission();     //Comm error
                if(error == 0){
                    DPRINT("changed to ");
                    // TODO Verify cast to int?
                    DPRINTLN((int)a);
                    break;
                }
            }

            activeLidarCounter += 1;
        }
        else{
            DPRINT("No device found on ");
            DPRINTLN(lidarEnablePinArray[i]);
			digitalWrite(lidarEnablePinArray[i], LOW);
        }
    }

}

bool getMessage()
{
    bool gotMessage = false;
    if((lastMessageTime + timeoutDuration) < millis()){
        isTimedOut = true;
    }
    while(Serial.available()) {
        if(Serial.read() == '$') {
            gotMessage = true;
            isTimedOut = false;
            lastMessageTime = millis();
            //TODO Handle requests from specific lidars or all
        }
    }
    return gotMessage;
}
