#include <mbed.h>

DigitalOut led1(LED1);
DigitalIn button1(BUTTON1);

int main() {

  // put your setup code here, to run once:
  int currentButtonState = button1.read();
  int lastButtonState;

  while(1) {
    lastButtonState = currentButtonState;
    currentButtonState = button1.read();
    if (currentButtonState == 0 && lastButtonState == 1) {
      led1 = !led1;
    }
    // put your main code here, to run repeatedly:
  }
}