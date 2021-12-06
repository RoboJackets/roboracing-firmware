#include <mbed.h>
#include <inttypes.h>
#include "PWMReader.h"


int64_t value;

PWMReader ch1(PA_0);
DigitalOut flash(PB_14);

int main()
{
    while (1) {          // wait around, interrupts will interrupt this!
        flash = !flash;
        value = ch1.getValue();
        printf("%" PRIu64 "\n",value);
        ThisThread::sleep_for(250);
    }
}
