#include "mbed.h"
#include "OdriveMbedSteeringEnums.h"
#include "BufferedSerial.h"
#include <cstring>
#include "EthernetInterface.h"
#include "rjnet_mbed_udp.h"
#include "BrakeLUT.h"

//Possible network error types: https://os.mbed.com/docs/mbed-os/v6.15/mbed-os-api-doxy/group__netsocket.html#gac21eb8156cf9af198349069cdc7afeba
//UDPSocket reference: https://os.mbed.com/docs/mbed-os/v6.15/apis/udpsocket.html
//SocketAddress reference: https://os.mbed.com/docs/mbed-os/v6.15/apis/socketaddress.html
//EthernetInterface: https://os.mbed.com/docs/mbed-os/v6.15/apis/ethernet.html
//Threads and setting flags: https://os.mbed.com/docs/mbed-os/v6.15/apis/thread.html
//ThisThread and waiting for flags: https://os.mbed.com/docs/mbed-os/v6.15/apis/thisthread.html

//Built-in LEDs
const static bool LED_OFF = 0;
const static bool LED_ON = !LED_OFF;

DigitalOut yellow_led(LED2);
DigitalOut green_led(LED1);
DigitalOut red_led(LED3);

float motor_turns_target;  //How many turns from home to get the target braking force.

/*
Functions for the Odrive
You cannot use D0, D1 for the serial port. Printf messes with those.
*/
BufferedSerial odrive_serial(D14, D15, 115200);  //tx, rx, baud

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

// Now create the functions that use the braking lookup table

float motorTurnsFromHomeForForce(float brakingForce) {
    if(brakingForce <= BrakeLUT[0][1]) {
        //Force is too small
        return BrakeLUT[0][0];
    }

    for(unsigned int i = 1; i < BrakeLUTLength; i++) {
        float LUTBrakingForce = BrakeLUT[i][1];
        if(LUTBrakingForce >= brakingForce) {
            //entry i is > than target, i-1 is < target. Linear interpolate
            float LUTBrakingForcePrevEntry = BrakeLUT[i-1][1];
            return BrakeLUT[i-1][0] + (brakingForce - LUTBrakingForcePrevEntry) * (BrakeLUT[i][0] - BrakeLUT[i-1][0])/(LUTBrakingForce - LUTBrakingForcePrevEntry);
        }
    }
    //Ran off the end of the table.
    return BrakeLUT[BrakeLUTMaxIndex][0];
}

float brakingForceFromCurrentPos(float motorTurnsFromHome){
    if(motorTurnsFromHome <= BrakeLUT[0][0]) {
        return BrakeLUT[0][1];
    }

    for(unsigned int i = 1; i < BrakeLUTLength; i++) {
        float LUTBrakeMotorTurns = BrakeLUT[i][0];
        if(LUTBrakeMotorTurns >= motorTurnsFromHome) {
            //entry i is > than target, i-1 is < target. Linear interpolate
            float LUTBrakeMotorTurnsPrevEntry = BrakeLUT[i-1][0];
            return BrakeLUT[i-1][1] + (motorTurnsFromHome - LUTBrakeMotorTurnsPrevEntry) * (BrakeLUT[i][1] - BrakeLUT[i-1][1])/(LUTBrakeMotorTurns - LUTBrakeMotorTurnsPrevEntry);
        }
    }
    return BrakeLUT[BrakeLUTMaxIndex][1];
}

/*
Networking stuff.
We don't have a function to send messages since Braking doesn't send any messages.
Currently this implementation requires that the braking force be a float with a .0: 3.0 is OK, 3 is not.
*/

//Peter's laptop IP FOR TESTING ONLY. Not for the car.
const SocketAddress laptopIP("192.168.20.120");

//Function header
void process_single_message(const SocketAddress &, const char[], unsigned int);

//Create the networking class
RJNetMbed rjnet_udp(brakeIP, & process_single_message);

void process_single_message(const SocketAddress & senders_address, const char incoming_udp_message[], unsigned int num_bytes_in_message){
    //Parses a UDP message we just recieved. Places any received data in global variables so main can use it
    
    if(rjnet_udp.are_ip_addrs_equal(driveIP, senders_address) || rjnet_udp.are_ip_addrs_equal(laptopIP, senders_address)){
        //Parse braking force from message. Doing incoming_message + 2 ignores the first two characters
        float requested_braking_force;
        sscanf(incoming_udp_message + 2, "%f", &requested_braking_force);
        motor_turns_target = motorTurnsFromHomeForForce(requested_braking_force);
        printf("New braking force: %.2f Motor turns: %.2f\n", requested_braking_force, motor_turns_target);
    }
    else{
        //printf("Message from %s : %s \n", senders_address.get_ip_address(), incoming_udp_message);
    }
}

// main() runs in its own thread in the OS
int main()
{
    red_led = LED_OFF;
    yellow_led = LED_OFF;
    green_led = LED_OFF;
    motor_turns_target = motorTurnsFromHomeForForce(0);  //Initial target

    ThisThread::sleep_for(500ms);
    clearErrors();

    yellow_led = LED_ON;
    printf("OdriveMbed Calibrating\n");
    requestState(AXIS_STATE_FULL_CALIBRATION_SEQUENCE);
    ThisThread::sleep_for(35s);

    printf("Clearing error\n");
    clearErrors();

    red_led = LED_ON;
    printf("OdriveMbed Homing\n");
    requestState(AXIS_STATE_HOMING);
    ThisThread::sleep_for(8s);

    printf("Entering closed loop control\n");
    requestState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    ThisThread::sleep_for(500ms);

    //Set up Ethernet. This should probably be done in a parallel thread for efficiency
    green_led = LED_ON;
    rjnet_udp.start_network_and_listening_threads();

    red_led = LED_OFF;
    yellow_led = LED_OFF;

    while (true) {
        
        setTargetPosition(motor_turns_target);
        ThisThread::sleep_for(100ms);
    }
}
