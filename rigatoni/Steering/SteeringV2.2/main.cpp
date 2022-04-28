#include "mbed.h"
#include "ODriveMbed.h"
#include <cstring>

BufferedSerial odrive_serial(D14, D15, 115200);  //tx, rx, baud

//ODriveMbed odrive(odrive_serial);

/*Important Note:
You cannot use D0, D1 for the serial port. Printf messes with those.
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
    char to_send[60];
    sprintf(to_send, "w axis0.requested_state %d\n", (int) requested_state);
    writeToOdrive(to_send);
}

void setTargetPosition(float targetPosition){
    char to_send[60];
    sprintf(to_send, "w axis0.controller.input_pos %f\n", targetPosition);
    writeToOdrive(to_send);
}

// main() runs in its own thread in the OS
int main()
{
    ThisThread::sleep_for(500ms);
    clearErrors();

    printf("OdriveMbed Calibrating\n");
    requestState(AXIS_STATE_FULL_CALIBRATION_SEQUENCE);
    ThisThread::sleep_for(35s);

    printf("Clearing error\n");
    clearErrors();

    printf("OdriveMbed Homing\n");
    requestState(AXIS_STATE_HOMING);
    ThisThread::sleep_for(12s);

    printf("Entering closed loop control\n");
    requestState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    ThisThread::sleep_for(500ms);

    //Test move
    setTargetPosition(10.5);
    ThisThread::sleep_for(1s);
    setTargetPosition(7);
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

