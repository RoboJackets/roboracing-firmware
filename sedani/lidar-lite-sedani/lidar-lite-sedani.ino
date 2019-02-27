/*
TO DO:
get a calibration curve for the lidars

*/

#include <Wire.h>
#include <LIDARLite.h>

#define NUMBER_OF_LIDARS 2

//Define pin array to be scanned for valid lidar scanners
static const int lidarEnablePinArray[] = {A5,A4};
byte unitCounter = 0;

static const unsigned int loopToleranceTime = 100; //us. Extra slop time to keep timing constant
static const unsigned long usPerLoop = 100000;
static const byte loopsPerRecalibration = 50;

static const byte lidarDefaultAddress = 0x62;        //default address

byte recalibrationCounter = 0;

//microseconds
unsigned long startTime;
unsigned long endReadTime;
float timeToTransmit = 8000;  //Magic number guess from Serial

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
        digitalWrite(lidarEnablePinArray[i],0);
    }

    initializeLidars();

    delay(100);
    Serial.print(unitCounter);
    Serial.println(" devices detected and readdressed");
}



void loop()
{
	startTime = micros();
	endReadTime = startTime + usPerLoop - loopToleranceTime - (int)timeToTransmit;
	
    //Loop through each Lidar unit and read distance data
	do{
		for(int i = 0; i < unitCounter; i++)
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
	}while (micros() > startTime && micros() < endReadTime);
	//print data
	if (lidarUnits[0].currentNumReads == 0){
		Serial.println("NO DATA ERROR!");
		Serial.println(startTime);
		Serial.println(endReadTime);
		Serial.println(micros());
	}
	
	Serial.println("Time to transmit (us): "+ String(timeToTransmit));
	Serial.println("Number of reads: " + String(lidarUnits[0].currentNumReads));
	
	//Replace with Ethernet stuff
	for(int i = 0; i < unitCounter; i++){
		Serial.println("Average LIDAR " + String(i) + " value: " + String(lidarUnits[i].currentSumReads/lidarUnits[i].currentNumReads));
		lidarUnits[i].currentSumReads = 0;
		lidarUnits[i].currentNumReads = 0;
	}
	//if statement to make it rollover safe
	//Could have problem if endReadTime rolls over but micros() does not
	if(micros() > startTime){
		//exponential filter to estimate the time it takes to send back data
        //0.05 magic number
		timeToTransmit += 0.05*((micros() - endReadTime) - timeToTransmit);
	}
	//make timing stable
	while(micros() > startTime && micros() < (startTime + usPerLoop));
}

void initializeLidars(){
    //Sequentially Enable the enable pins to detect active Lidar sensors
    for(int i=0; i < NUMBER_OF_LIDARS; i++) 
    {
    	//enable the enable pin
        digitalWrite(lidarEnablePinArray[i],1); 
        //Wait 25ms to enable comms to Lidar unit
        delay(25);     
        
        //Attempt to communicate to device
        Wire.beginTransmission(lidarDefaultAddress);
        delay(10);
        //Comm error
        int error = Wire.endTransmission();
        //Communications attempt successful
        if(error == 0){
            lidarUnits[unitCounter].pin = lidarEnablePinArray[i]; //Set pin number
            //<< 1 for 7 bit address
            lidarUnits[unitCounter].address = lidarDefaultAddress - ((unitCounter + 1) << 1);  //Set device address
            Serial.print("Device found on pin ");
            Serial.println(lidarUnits[unitCounter].pin);
            //Write address to LIDAR
            lidarUnits[unitCounter].myLidarLite.begin(0,true);
            lidarUnits[unitCounter].myLidarLite.configure(0);
            
            lidarUnits[unitCounter].myLidarLite.setI2Caddr(lidarUnits[unitCounter].address, 1, lidarDefaultAddress);

            //checking address change
            for(char a=0;a<255;a+=2){
                Wire.beginTransmission(a);    //Attempt to communicate to device
                delay(1);
                int error = Wire.endTransmission();     //Comm error
                if(error == 0){
                    Serial.print("changed to ");
                    Serial.println((int)a);
                    break;
                }
            }
            unitCounter += 1;
        }
        else{
            Serial.print("No device found on ");
            Serial.println(lidarEnablePinArray[i]);
			digitalWrite(lidarEnablePinArray[i], LOW);
        }
    }

}
