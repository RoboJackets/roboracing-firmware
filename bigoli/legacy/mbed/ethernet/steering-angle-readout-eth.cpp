#include "mbed.h"
#include "EthernetInterface.h"  // ethernet interface
#include "rtos.h"  // sleep

#include <string>
 
const unsigned char ECHO_SERVER_PORT = 7;
const std::string MBED_IP("192.168.2.2");
AnalogIn pot(p15);

int main (void) {
  EthernetInterface eth;
  eth.init(MBED_IP.c_str(), 0, 0);
  eth.connect(1000);

  TCPSocketServer server;
  server.bind(ECHO_SERVER_PORT);
  server.listen();

  while (true) {
    TCPSocketConnection client;
    server.accept(client);
    client.set_blocking(false, 1500); // Timeout after 1.5s

    while (true) {
      char angleStr[16];
      sprintf(angleStr, "%1.5f", pot.read());

      int n = client.send_all(angleStr, sizeof(angleStr));
      if (n <= 0)
        break;

      Thread::wait(50);
    }

    client.close();
  }
}
