// ** DON'T INCLUDE <iostream>! THINGS WILL BREAK! **

#include <mbed.h>

#define RJ_ENABLE_ROBOT_CONSOLE

using namespace std;

DigitalOut myled(LED1);
/**
 * The entry point of the system where each submodule's thread is started.
 */
int main() {
while(1) {
        myled = 1;
        wait(0.2);
        myled = 0;
        wait(0.2);
    }
}
