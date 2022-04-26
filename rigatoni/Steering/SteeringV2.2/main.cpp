#include "mbed.h"
#include "ODriveMbed.h"
#include <cstring>

BufferedSerial odrive_serial(D1, D0, 115200);  //tx, rx, baud

//ODriveMbed odrive(odrive_serial);

/*Important Note:
DO NOT put any printf() statements as it messes the ODrive serial data being sent.
*/

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
    writeToOdrive("w axis0.controller.input_pos -3\n");
    ThisThread::sleep_for(30000ms);
    clearErrors();
    writeToOdrive("w axis0.controller.input_pos -10\n");
    ThisThread::sleep_for(30000ms);

}

// main() runs in its own thread in the OS
int main()
{
    ThisThread::sleep_for(500ms);
    odrive_serial.write("sc\n",strlen("sc\n"));
    ThisThread::sleep_for(5000ms);
    //printf("OdriveMbed Calibrating\n");
    odrive_serial.write("w axis0.requested_state 3\n", strlen("w axis0.requested_state 3\n"));
    //printf("Finished callibration\n");
    ThisThread::sleep_for(30000ms);
    odrive_serial.write("sc\n",strlen("sc\n"));
    ThisThread::sleep_for(5000ms);
    //printf("Entering closed loop control");
    odrive_serial.write("w axis0.requested_state 8\n", strlen("w axis0.requested_state 8\n"));
    ThisThread::sleep_for(30000ms);
    writeToOdrive("w axis0.controller.input_pos -3\n");
    ThisThread::sleep_for(3000ms);
    writeToOdrive("w axis0.controller.input_pos -10\n");
    /*printf("Entered main function\n");
    ThisThread::sleep_for(500ms);
    clearErrors();
    printf("ODriveMbed calibrating\n");
    requestState(AXIS_STATE_FULL_CALIBRATION_SEQUENCE);
    printf("Finished callibration\n");
    ThisThread::sleep_for(30000ms);
    clearErrors();
    printf("Entering closed loop control");
    requestState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    ThisThread::sleep_for(60000ms);
    clearErrors();
    /*positionTestMove();*/

    while (true) {
        
    }
}

