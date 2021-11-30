#include <mbed.h>
#include "EthernetInterface.h"
#include "rjnet_mbed_udp.h"

//Possible network error types: https://os.mbed.com/docs/mbed-os/v6.15/mbed-os-api-doxy/group__netsocket.html#gac21eb8156cf9af198349069cdc7afeba
//UDPSocket reference: https://os.mbed.com/docs/mbed-os/v6.15/apis/udpsocket.html
//SocketAddress reference: https://os.mbed.com/docs/mbed-os/v6.15/apis/socketaddress.html
//EthernetInterface: https://os.mbed.com/docs/mbed-os/v6.15/apis/ethernet.html
//Threads and setting flags: https://os.mbed.com/docs/mbed-os/v6.15/apis/thread.html
//ThisThread and waiting for flags: https://os.mbed.com/docs/mbed-os/v6.15/apis/thisthread.html

//Function header
void process_single_message(const SocketAddress &, const char[], unsigned in);

//Create the networking class
RJNetMbed rjnet_udp(RJNetMbed::driveIP, & process_single_message);

void process_single_message(const SocketAddress & senders_address, const char incoming_udp_message[], unsigned int num_bytes_in_message){
    //Parses a UDP message we just recieved. Places any received data in global variables.
    //Do parsing later, just print for now.
    
    if(rjnet_udp.are_ip_addrs_equal(RJNetMbed::nucIP, senders_address)){
        //Parse speed from message. Doing incoming_message + 2 ignores the first two characters
        float new_speed;
        sscanf(incoming_udp_message + 2, "%f", &new_speed);
        //Reply to NUC at once
        char outgoing_message [64];
        sprintf(outgoing_message, "Got speed = %f", new_speed);
        rjnet_udp.send_single_message(outgoing_message, RJNetMbed::nucIP);
    }
}

//Deal with sending messages
void send_messages_udp_thread(){
    //This thread handles sending messages at a scheduled interval
    int i = 0;
    while (true){
        //Message sending loop. Sends scheduled messages every 100ms
        const unsigned int message_max_len = 32;
        char to_send[message_max_len];
        snprintf(to_send, message_max_len, "Scheduled message %d", i++);
        rjnet_udp.send_single_message(to_send, RJNetMbed::nucIP);

        //Reset timer for scheduled messages
        ThisThread::sleep_for(RJNetMbed::TIME_BETWEEN_SENDS);
    }
}

//Now make threads for the sending (receiving handled by class)
Thread udp_sending_thread;

int main() {

    rjnet_udp.start_network_and_listening_threads();
    udp_sending_thread.start(send_messages_udp_thread);

    while(1) {

        ThisThread::sleep_for(1s);

    }
}