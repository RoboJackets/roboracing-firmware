#include "mbed.h"
#include "rtos.h"

PwmOut driverPinA(p21);
PwmOut driverPinB(p22);
Serial serial(USBTX, USBRX);
DigitalOut led(LED1);

const int PERIOD = 1000; //us

int main() 
{
    driverPinA.period_us(PERIOD);
    driverPinB.period_us(PERIOD);
    
    while(true) 
    {
        serial.printf("a = low, b modulated\r\n");
        driverPinA.write(0.0f);
        for(float pwm = 0; pwm < 1; pwm += 0.01) {
            driverPinB.write(pwm);
            wait_ms(20);
        }
        for(float pwm = 1; pwm > 0; pwm -= 0.01) {
            driverPinB.write(pwm);
            wait_ms(20);
        }

        serial.printf("a modulated, b = low\r\n");
        driverPinB.write(0.0f);
        for(float pwm = 0; pwm < 1; pwm += 0.01) {
            driverPinA.write(pwm);
            wait_ms(20);
        }
        for(float pwm = 1; pwm > 0; pwm -= 0.01) {
            driverPinA.write(pwm);
            wait_ms(20);
        }
    }
}
