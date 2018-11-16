/*
Ethernet built on ChatServer example.

This assumes that the board is the server.

 */

#include <SPI.h>
#include <Ethernet.h>

static const char led = LED_BUILTIN;

unsigned static const int loopSpeed = 100; //Hertz
unsigned static const int usPerSec = 1000000;

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network.
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress lidar_ip(192, 168, 1, 177);
static const int port = 8888;
//IPAddress nuc_ip(192, 168, 1, 10); We don't need to know the NUC's IP

EthernetServer server(port);

void setup() {
    
	pinMode(led, OUTPUT);
	
    // initialize the ethernet device
    Ethernet.begin(mac, lidar_ip);

    // Open serial communications and wait for port to open:
    Serial.begin(9600);

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
	
	Serial.println("Waiting for a client");
}

#define MAX_CLIENTS 8
EthernetClient clients[MAX_CLIENTS];  //list of clients we are sending data to

String dataString = "Filler Data";  //Build all the data into a string before we send it

unsigned long startTime = 0;

void loop() {
	startTime = micros();  //start timing the loop
	
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

    //Code to read the lidars goes here
	
	dataString = "Filler Data";  //Build all the data into a string before we send it
	
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
	
	//to keep loop timing consistent
	while (micros() < startTime + usPerSec/loopSpeed);
}


