#include "mbed.h"
#include "rtos.h"

PwmOut driverPin(p21);
Serial serial(USBTX, USBRX);
DigitalOut led(LED1);

const int PERIOD = 16; //ms
const int pulseMin = 1250; //us
const int pulseMax = 1750;
const int pulseCenter = (pulseMin + pulseMax) / 2;

int main() {

    driverPin.period_ms(PERIOD);
    wait(1);
    
    driverPin.pulsewidth_us(pulseCenter);

    serial.getc();

    for(int x = pulseCenter; x >= pulseMin; x-=10) {
        driverPin.pulsewidth_us(x);
        wait_ms(30);
    }

    for(int i = 0; i < 4; i++) {
        for(int x = pulseMin; x <= pulseMax; x+=10) {
            driverPin.pulsewidth_us(x);
            wait_ms(30);
        }
        wait(1);
        for(int x = pulseMax; x >= pulseMin; x-=10) {
            driverPin.pulsewidth_us(x);
            wait_ms(30);
        }
        wait(1);
    }

    for(int x = pulseMin; x <= pulseCenter; x+=10) {
        driverPin.pulsewidth_us(x);
        wait_ms(30);
    }

    while(1) {
        Thread::wait(1000); //lower-power wait
    }
}
