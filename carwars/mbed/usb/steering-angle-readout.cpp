#include "mbed.h"
#include "rtos.h"
// #include "algorithm"
// #include "stdlib.h"

// PwmOut driverPin(p23);
Serial serial(USBTX, USBRX);
// DigitalOut led(LED1);
AnalogIn pot(p15);
// Timer timer;

int main() {
  Thread::wait(1500);

  while(true) {
      serial.printf("$pot=%.10f\r\n", pot.read());

      Thread::wait(50);
  }
}
