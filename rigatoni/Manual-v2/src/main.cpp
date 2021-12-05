#include <mbed.h>
#include <Clock.h>
uint64_t time;
int32_t value;

InterruptIn button(PA_0);
DigitalOut led(LED2);
DigitalOut flash(PB_14);

int main()
{
    //button.rise(&start1);  // attach the address of the flip function to the rising edge
    //button.fall(&end1);
    while (1) {          // wait around, interrupts will interrupt this!
        flash = !flash;
        ThisThread::sleep_for(250);
        printf("Hello");
    }
}

void start1()
{
    time = clock_ms();
    led = true;

}

void end1()
{
  value = clock_ms - time;
  if(value < 0 ){
    value = 0;
  }
  led = false;
}