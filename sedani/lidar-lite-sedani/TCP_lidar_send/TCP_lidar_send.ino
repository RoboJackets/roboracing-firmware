/*
Ethernet built on ChatServer example.

This assumes that the board is the server.

 */

#include <SPI.h>
#include <Ethernet.h>
#include <Wire.h>
#include <LIDARLite.h>

//Ethernet stuff
// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network.
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress lidar_ip(192, 168, 1, 177);
static const int port = 8888;
//IPAddress nuc_ip(192, 168, 1, 10); We don't need to know the NUC's IP

EthernetServer server(port);


//Misc loop timing and debugging stuff
static const char led = LED_BUILTIN;
unsigned static const int loopSpeed = 100; //Hertz
unsigned static const int usPerSec = 1000000;

//LIDAR stuff
#define NUMBER_OF_LIDARS 8

//Define pin array to be scanned for valid lidar scanners
static const int pinArray[] = {2,3,4,5,6,7,8,9};
int unitCounter = 0;

struct LIDAR
{
    int address;
    int pin;
    LIDARLite myLidarLite;
};

LIDAR lidarUnits[NUMBER_OF_LIDARS];            //Array of lidarUnits to store configuration data

static const int defAddress = 0x62;        //default address

void change_address(char address, LIDARLite lidar);

void setup() {
    
	pinMode(led, OUTPUT);
	
    // initialize the ethernet device
    Ethernet.begin(mac, lidar_ip);

    // Open serial communications:
    Serial.begin(115200);

    // Check for Ethernet hardware present
    while (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println("Ethernet shield was not found. Sorry, can't run without hardware. :(");
		digitalWrite(led, HIGH);
		delay(100);
		digitalWrite(led, LOW);
		delay(200);
		
        
    }
	
	//wait for Ethernet cable to be connected
    while (Ethernet.linkStatus() == LinkOFF) {
        Serial.println("Ethernet cable is not connected.");
		digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
		delay(100);
    }

    // start listening
    server.begin();

    Serial.print("Ethernet connected. LIDAR board address: ");
    Serial.println(Ethernet.localIP());
	
	Serial.println("Begining to set up LIDARS");
	
	
	//Begin scanning bus. Copy/pasted from Lidar_ShopTest
	    for(int i=0; i < NUMBER_OF_LIDARS; i++) // Setup enable pins
    {
        pinMode(pinArray[i],OUTPUT);
        digitalWrite(pinArray[i],0);
    }

    delay(100);

    
    //Sequentially Enable the enable pins to detect active Lidar sensors
    for(int i=0; i < NUMBER_OF_LIDARS; i++) 
    {
        digitalWrite(pinArray[i],1); //enable the enable pin
        delay(100);     //Wait 20ms to enable comms to Lidar unit
                
        Wire.beginTransmission(defAddress);    //Attempt to communicate to device
        delay(10);
        int error = Wire.endTransmission();   //Comm error
        if(error == 0)     //Communications attempt successful
        {
            lidarUnits[unitCounter].pin = pinArray[i]; //Set pin number
            lidarUnits[unitCounter].address = defAddress - unitCounter*2 - 10;  //Set device address
            Serial.print("Device found on pin ");
            Serial.println(lidarUnits[unitCounter].pin);
            //Write address to LIDAR
	        lidarUnits[unitCounter].myLidarLite.begin(0,true);
            lidarUnits[unitCounter].myLidarLite.configure(0);
            
            change_address(lidarUnits[unitCounter].address, lidarUnits[unitCounter].myLidarLite);
            //checking address change
            for(char a=0;a<255;a+=2){
                Wire.beginTransmission(a);    //Attempt to communicate to device
                delay(1);
                int error = Wire.endTransmission();     //Comm error
                if(error == 0){
                    Serial.print("changed to");
					Serial.println((int)a);
					break;
                    }
                }
                
             
            unitCounter += 1;
        }
        else
        {
            Serial.print("No device found on ");
            Serial.println(pinArray[i]);
        }
        
    }

    delay(100);
    Serial.print(unitCounter);
    Serial.println(" devices detected and readdressed");
}


#define MAX_CLIENTS 8
EthernetClient clients[MAX_CLIENTS];  //list of clients we are sending data to

String dataString = "Filler Data";  //Build all the data into a string before we send it

unsigned long startTime = 0;
unsigned long ethernetStartTime = 0;
unsigned long ethernetTimeTaken = 0;

void loop() {
	startTime = micros();  //start timing the loop

    //Code to read the lidars goes here
	//Loop through each Lidar unit and read distance data
    for(int i = 0; i < unitCounter; i++)
    {                
        //Serial.println(myLidarLite.distance(false, lidarUnits[i].address));
        Serial.print("reading lidar ");
        Serial.print(i);
        Serial.print(" on address ");
        Serial.println(lidarUnits[i].address);
        Serial.println(lidarUnits[i].myLidarLite.distance(false, lidarUnits[i].address));
        delay(10);
        }
	
	dataString = "Filler Data";  //Build all the data into a string before we send it
	
	ethernetStartTime = micros();
	
	// check for any new client connecting, and say hello (before any incoming data)
    EthernetClient newClient = server.accept();
    
    //If we have a new client, iterate over the list of clients and put it in the first open spot
    if (newClient) {
        for (byte i = 0; i < MAX_CLIENTS; i++) {
            if (!clients[i]) {
                clients[i] = newClient;
				Serial.println("Found new client at " + String(newClient.remoteIP()));
                break;
            }
        }
    }
	
	//Send the data to all available clients
	for (byte i = 0; i < MAX_CLIENTS; i++){
		for (byte j=0; j < 8; j++) {
            if (clients[j].connected()){
				clients[j].println(dataString);
			}
		}
	}
	

    // stop any clients which disconnect
    for (byte i = 0; i < MAX_CLIENTS; i++) {
        if (clients[i] && !clients[i].connected()) {
			Serial.print("disconnect client at IP ");
			Serial.println(clients[i].remoteIP());
            clients[i].stop();
        }
    }
	ethernetTimeTaken = micros() - ethernetStartTime;
	
	//to keep loop timing consistent
	//The micros() > startTime is to deal with rollover
	while ((micros() < startTime + usPerSec/loopSpeed)&&(micros() > startTime));
}

//changes LIDARlite address following procedure outlined in manual pg 5
void change_address(char address, LIDARLite lidar){
  byte serial_number[2];
  byte lidar_default_address = 0x62;
  Serial.println("Reading Serial number");
  lidar.read(0x96, 2, serial_number, false, lidar_default_address);
  Serial.println("Got Serial number ");
  lidar.write(0x18, serial_number[0]);
  Serial.println("Written first byte");
  lidar.write(0x19, serial_number[1]);
  Serial.println("Written second byte");
  lidar.write(0x1a, address);
  Serial.println("Written address");
  lidar.write(0x1e, 0x08);
  Serial.println("Overwritten 0x1e");
}

