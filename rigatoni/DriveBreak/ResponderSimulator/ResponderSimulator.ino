/*
 * This runs a server that listens for messages. It responds to messages with the specified string and prints the message out.
 * This uses RJNet
 * Modified 19 Oct 2020
 * by Brian Cochran and Peter Wilson
 * 
 */

#include <Ethernet.h>
#include "RJNet.h" //make sure to install the RJNet library

const static String responseString = "R";

const static int PORT = 7; //port RJNet uses

//Don't forget to switch these around depending on which board you are programming!

// Enter a MAC address and IP address for your board below
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xE3 };
IPAddress ip(192, 168, 0, 173); //set the IP to find us at
EthernetServer server(PORT);
 
void setup() {
  Serial.begin(115200);
  
  // In case your RJ board wires the chip in an odd config,
  // otherwise, leave commented out
  // You can use Ethernet.init(pin) to configure the CS pin
  Ethernet.init(11);  // Most Arduino shields

  // initialize the ethernet device
  Ethernet.begin(mac, ip);

  //Ethernet.setSubnetMask(

  //check that ethernet equipment is plugged in
  while (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    delay(50);
  }
  while(Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
    delay(50);
  }

  // start listening for clients
  server.begin();

  Serial.print("Our address: ");
  Serial.println(Ethernet.localIP());

  
}

unsigned long lastPacketTime = 0;

void loop() {
  // wait for a new client:
  EthernetClient client = server.available();
  if (client) {
    String data = RJNet::readData(client);
    unsigned long newPacketTime = millis();
    
    if (data.length() != 0) {
      Serial.print(data); //show us what we read 
      Serial.print(" From client ");
      Serial.print(client.remoteIP());
      Serial.print(":");
      Serial.println(client.remotePort());
      
      Serial.println("Time between packets: " + String(newPacketTime - lastPacketTime));
      lastPacketTime = newPacketTime;
      
      //Reply to data
      RJNet::sendData(client, responseString);
      
    }
    else Serial.print(" Empty message recieved. ");
  }
}
