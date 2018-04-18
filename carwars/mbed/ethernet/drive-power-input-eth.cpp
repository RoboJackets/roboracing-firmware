#include "mbed.h"
#include "EthernetInterface.h"  // ethernet interface
#include "rtos.h"  // sleep

#include <string>
#include <cstdlib>
 
const unsigned char ECHO_SERVER_PORT = 7;
const char* const MBED_IP = "192.168.2.2";

PwmOut driverPinA(p21);
PwmOut driverPinB(p22);


int main (void) {
  EthernetInterface eth;
  eth.init(MBED_IP, 0, 0);
  eth.connect(1000);

  TCPSocketServer server;
  server.bind(ECHO_SERVER_PORT);
  server.listen();

  while (true) {
    double speed = 0;

    TCPSocketConnection client;
    server.accept(client);
    client.set_blocking(false, 1500); // Timeout after 1.5s

    while (true) {
      char buffer[256];
      int n = client.receive(buffer, sizeof(buffer));
      if (n <= 0) 
        break;

      char* p = buffer;
      double speedTarget = strtod(p, &p);
      double steeringTarget = strtod(p, &p);

      // TODO read encoders and calculate speed here
      double speedMeasured = 0;

      sprintf(buffer, "%.3f %f ", speedMeasured, speedTarget);

      n = client.send_all(buffer, sizeof(buffer));
      if (n <= 0)
        break;
    }

    client.close();
  }
}
