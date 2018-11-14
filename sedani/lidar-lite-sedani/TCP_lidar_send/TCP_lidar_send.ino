/*
Ethernet copy/pasted from ChatServer example.

This assumes that the board is the server.
Probably, we want the NUC to be the server.

 */

#include <SPI.h>
#include <Ethernet.h>

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network.
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress lidar_ip(192, 168, 1, 177);
IPAddress nuc_ip(192, 168, 1, 10);

static const int port = 8888
EthernetServer server(port);
boolean alreadyConnected = false; // whether or not the NUC was connected previously

void setup() {
    
	pinMode(LED_BUILTIN, output);
	
    // initialize the ethernet device
    Ethernet.begin(mac, lidar_ip);

    // Open serial communications and wait for port to open:
    Serial.begin(9600);
     while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }

    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println("Ethernet shield was not found.    Sorry, can't run without hardware. :(");
        while (true) {
            delay(1); // do nothing, no point running without Ethernet hardware
        }
    }
	
	//wait for Ethernet cable to be connected
    while (Ethernet.linkStatus() == LinkOFF) {
        Serial.println("Ethernet cable is not connected.");
		digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
		delay(100);
    }

    // start listening
    server.begin();

    Serial.print("LIDAR board address: ");
    Serial.println(Ethernet.localIP());
}

void loop() {
    // wait for a new client:
    EthernetClient client = server.available();

    // when the client sends the first byte, say hello:
    if (client) {
        if (!alreadyConnected) {
            // clear out the input buffer:
            client.flush();
            Serial.print("Found new node at IP: ");
            client.println("Hello, client!");
            alreadyConnected = true;
        }

        if (client.available() > 0) {
            // read the bytes incoming from the client:
            char thisChar = client.read();
            // echo the bytes back to the client:
            server.write(thisChar);
            // echo the bytes to the server as well:
            Serial.write(thisChar);
        }
    }
}


