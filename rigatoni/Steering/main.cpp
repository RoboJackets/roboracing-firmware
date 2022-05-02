#include "mbed.h"
#include "ODriveMbed.h"
#include <cstdio>
#include <cstring>
#include <rjnet_mbed_udp.h>
#include <string>
#include "EthernetInterface.h"
#include <algorithm>

/*Important Notes:
1. DO NOT use D1 and D0 for the UART pins as this impacts the ability
to use printf due to how MbedOS handles things internally.*/


BufferedSerial odrive_serial(D14, D15, 115200);  //tx, rx, baud

void process_single_message(const SocketAddress &, const char[], unsigned int);

RJNetMbed rjnet_udp(steeringIP, & process_single_message);

Thread udp_steering_sending_thread;

float desired_angle;

const SocketAddress laptopIP("192.168.20.12");

#define CENTER_POSITION 8.5
#define MIN_POSITION 0
#define MAX_POSITION 17

#define STEERING_CONVERSION_CONSTANT 34

void process_single_message(const SocketAddress & senders_address, const char incoming_udp_message[], unsigned int num_bytes_in_message){
    //Parses a UDP message we just recieved. Places any received data in global variables.
    //Do parsing later, just print for now.
    
    if(rjnet_udp.are_ip_addrs_equal(nucIP, senders_address)){
        //Parse speed from message. Doing incoming_message + 2 ignores the first two characters
        sscanf(incoming_udp_message + 2, "%f", &desired_angle);
        //Reply to NUC at once
        char outgoing_message [64];
        sprintf(outgoing_message, "Got angle = %f", desired_angle);
        rjnet_udp.send_single_message(outgoing_message, nucIP);
    }
    //printf("Message from %s : %s \n", senders_address.get_ip_address(), incoming_udp_message);
}

//Handles sending messages
void send_messages_udp_thread(){
    //This thread handles sending messages at a scheduled interval
    int i = 0;
    while (true){
        //Message sending loop. Sends scheduled messages every 100ms
        const unsigned int message_max_len = 32;
        char to_send[message_max_len];
        snprintf(to_send, message_max_len, "Scheduled message %d", i++);
        //These appear in different packets, received in the same millisecond.
        //So delimiting messages via packets seems to be working
        rjnet_udp.send_single_message(to_send, steeringIP);
        rjnet_udp.send_single_message(to_send, steeringIP);
        printf("Sent: %s\n", to_send);

        //Reset timer for scheduled messages
        ThisThread::sleep_for(RJNetMbed::TIME_BETWEEN_SENDS);
    }
}

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
    sprintf(to_send, "w axis0.controller.input_pos %f\n", CENTER_POSITION + targetPosition);
    writeToOdrive(to_send);
}

float calculatePhysicalInput(float desired_input) {
    desired_input = std::min(0.25, ((-0.25 < desired_input) ? desired_input:-0.25));
    return desired_input * STEERING_CONVERSION_CONSTANT;
}

// main() runs in its own thread in the OS
int main()
{
    char buffer[80] = {0};
    rjnet_udp.start_network_and_listening_threads();

    udp_steering_sending_thread.start(send_messages_udp_thread);

    ThisThread::sleep_for(500ms);
    clearErrors();
    printf("Initial Error Clearing\n");
    odrive_serial.write("sc\n", strlen("sc\n"));

    printf("OdriveMbed Calibrating\n");
    requestState(AXIS_STATE_FULL_CALIBRATION_SEQUENCE);
    ThisThread::sleep_for(35s);

    printf("Clearing error\n");
    odrive_serial.write("sc\n", strlen("sc\n"));

    printf("OdriveMbed Homing\n");
    requestState(AXIS_STATE_HOMING);
    ThisThread::sleep_for(12s);

    writeToOdrive("w axis0.encoder.set_linear_count(0)\n");
    ThisThread::sleep_for(500ms);    

    printf("Entered closed loop control\n");
    requestState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    ThisThread::sleep_for(500ms);

    printf("Moving to center position\n");
    ThisThread::sleep_for(3000ms);
    setTargetPosition(0);

    printf("Setup has finished.\n");


    while (true) {
        ThisThread::sleep_for(100ms);
        float input_position = calculatePhysicalInput(desired_angle);
        setTargetPosition(input_position);
        if (input_position != 0) {
            printf("UDP Steering Info Works, Data: %f\n",input_position);
        }
    }
}