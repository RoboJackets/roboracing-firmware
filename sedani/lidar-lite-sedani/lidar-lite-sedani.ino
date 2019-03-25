/*
TODO:
get a calibration curve for the lidars

*/

#include <Wire.h>
#include <LIDARLite.h>
#include <EthernetUdp.h>
//#include <RJNet.h>

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
unsigned long currTimeoutTime;
bool isTimedOut = true; 

const int distanceBufferSize = 3;


struct LIDAR
{
    LIDARLite myLidarLite;
    byte address;
    byte pin;
    unsigned long distanceData[distanceBufferSize];
};

char replyBuffer[255];                         // Array to hold the response message

LIDAR lidarUnits[NUMBER_OF_LIDARS];            //Array of lidarUnits to store configuration data

void initializeLidars();

unsigned long averageDistance(LIDAR &lidarUnit);

// Ethernet Related Variables
// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 1, 177);

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,

// Instantiate an Ethernet UDP instance
EthernetUDP Udp;

unsigned int localPort = 8888;      // local port to listen on


void setup() {
    Wire.begin();

	//Start serial communications
    Serial.begin(115200);
    while(!Serial){
        delay(1);
    }

    initializeLidars();

    delay(100);
    DPRINT(activeLidarCounter);
    DPRINTLN(" devices detected and readdressed");

    // You can use Ethernet.init(pin) to configure the CS pin
    Ethernet.init(17);
  
    // Start the Ethernet
    Ethernet.begin(mac, ip);

    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println("Ethernet shield was not found. Sorry, can't run without hardware. :(");
        /*while (true) {
            // BLINK LED WITH ERROR CODE
        }*/
    }

    if (Ethernet.linkStatus() == LinkOFF) {
        Serial.println("Ethernet cable is not connected.");
        // LED ETHERNET LINK DOWN
    }
  
    // start UDP
    Udp.begin(localPort);
 }

void loop() {

    // if there's data available, read a packet
    int packetSize = Udp.parsePacket();
    if (packetSize) {
        currTimeoutTime = 0;
        lastMessageTime = millis();
        DPRINT("Received packet of size ");
        DPRINTLN(packetSize);
        DPRINT("From ");
        IPAddress remote = Udp.remoteIP();
        for (int i=0; i < 4; i++) {
            DPRINT(remote[i], DEC);
            if (i < 3) {
                DPRINT(".");
            }
        }
        DPRINT(", port ");
        DPRINTLN(Udp.remotePort());

        // Read the packet into packetBufffer
        Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    
      DPRINTLN("Contents:");
      DPRINTLN(packetBuffer);

      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());

      // Construct response message
     // if (packetBuffer == "Request Distance")
          //{
        for(int i = 0; i < activeLidarCounter; i++){
          // send a reply to the IP address and port that sent us the packet we received
          unsigned long dist = averageDistance(lidarUnits[i]);
          sprintf(replyBuffer,"LIDAR %d value: %d;", i, dist);
          Udp.write(replyBuffer);
        }
      //}
    /*else if (packetBuffer = "Request State"){
    
    }*/
      Udp.endPacket();
      for (int i = 0; i < sizeof(replyBuffer); i++){
        replyBuffer[i] = "";
      }
    }
    
    else{
        currTimeoutTime += millis() - lastMessageTime;
        if (currTimeoutTime >= timeoutDuration){
            //DPRINTLN("Timeout");
        }
    }
  
    
    // Read data from all active lidar units and store in FIFO array
    for(int i = 0; i < activeLidarCounter; i++){ 
        runningAverageShift(lidarUnits[i]);             // Update the FIFO array with new distance readings          
        if (recalibrationCounter == i){                 // Run calibration - needed to properly bias the distance readings for accuracy
            lidarUnits[i].distanceData[0] = lidarUnits[i].myLidarLite.distance(true, lidarUnits[i].address);
        }
        else{
            lidarUnits[i].distanceData[0] = lidarUnits[i].myLidarLite.distance(false, lidarUnits[i].address);
        }
    }
    recalibrationCounter++;
    recalibrationCounter = recalibrationCounter % loopsPerRecalibration;

}
  
   

void initializeLidars(){
    // Setup enable pins
    for(int i=0; i < NUMBER_OF_LIDARS; i++) {
        pinMode(lidarEnablePinArray[i],OUTPUT);
        digitalWrite(lidarEnablePinArray[i], LOW);
    }

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


// Averages the distance values stored in the distance data 
unsigned long averageDistance(struct LIDAR &lidarUnit){
    unsigned long avgDist = 0;
    for(int i = 0; i < distanceBufferSize; i++){
        avgDist += lidarUnit.distanceData[i];
    }
    avgDist = avgDist / distanceBufferSize;
    return avgDist;  
}

// Shifts values for running average to make space for new distance
void runningAverageShift(struct LIDAR &lidarUnit){
    for(int j = distanceBufferSize - 1; j > 0; j--){
        lidarUnit.distanceData[j] = lidarUnit.distanceData[j-1];
    }
}
