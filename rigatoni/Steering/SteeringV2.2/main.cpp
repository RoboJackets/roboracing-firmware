#include "mbed.h"
#include "ODriveMbed.h"

BufferedSerial odrive_serial(D1, D0, 115200);  //tx, rx, baud

ODriveMbed odrive(odrive_serial);

void writeToOdrive(char to_send[]){
    odrive_serial.write(to_send, strlen(to_send));
}

void clearErrors(void){
    char command[] = "sc\n";
    writeToOdrive(command);
    ThisThread::sleep_for(100ms);
}

void requestState(AxisState requested_state){
    char to_send[] = "w axis0.requested_state _\n";
    //Now replace the _ with the ascii value of the number of the requested state
    uint16_t length_of_message = strlen(to_send);
    to_send[length_of_message - 2] = (char) requested_state + '0';
    writeToOdrive(to_send);
}

void closedLoopControl() {
    requestState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    ThisThread::sleep_for(3s);
}

void positionTestMove() {
    printf("Executing test move");
    writeToOdrive("q 0 -3\n");
    ThisThread::sleep_for(1s);
    writeToOdrive("q 0 -10 \n");
    ThisThread::sleep_for(1s);
}

// main() runs in its own thread in the OS
int main()
{
    ThisThread::sleep_for(500ms);
    clearErrors();
    printf("ODriveMbed calibrating\n");
    requestState(AXIS_STATE_FULL_CALIBRATION_SEQUENCE);
    ThisThread::sleep_for(30s);
    printf("Entering closed loop control");
    requestState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    ThisThread::sleep_for(100ms);

    while (true) {
        
    }
}

