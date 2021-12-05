#include <mbed.h>
#include "EthernetInterface.h"

const bool LED_OFF = 0;
const bool LED_ON = 1;

//Configuration
const uint16_t MAX_RJNET_MESSAGE_LEN_BYTES = 1500;
const uint16_t RJNET_DEFAULT_PORT = 2847;
const SocketAddress our_ip_addr("192.168.20.5");
const SocketAddress rjnet_netmask("255.255.255.0");     //Restrict to 192.168.20.*
const SocketAddress rjnet_default_gateway("0.0.0.0");   //Don't need a gateway since not communicating beyond the LAN.

const SocketAddress other_ip_addr("192.168.20.120", RJNET_DEFAULT_PORT);

const Kernel::Clock::duration_u32 TIME_BETWEEN_SENDS = 100ms;

//The default Ethernet interface:
EthernetInterface ethernet_port;

//Now make a UDP socket for next use
UDPSocket network_socket;


bool connect_if_no_network_connection(){
    //This function returns true if network connection OK.
    //Tries to start connection and returns false if network connection down.

    nsapi_connection_status_t eth_connection_status = ethernet_port.get_connection_status();
    if(eth_connection_status == NSAPI_STATUS_DISCONNECTED){
        //Not connected. Try to connect
        ethernet_port.connect();
        return false;
    }
    else if(eth_connection_status == NSAPI_STATUS_CONNECTING){
        //Not connected. Waiting for connection to start
        return false;
    }
    //Have local or global connectivity
    return true;
}

int main() {

    //Set up buffer for received message
    char udp_message_received[MAX_RJNET_MESSAGE_LEN_BYTES] = {0};

    //Set up buffer for sent message
    string udp_message_send = "The message";

    DigitalOut ethernet_led(LED1);
    ethernet_led = LED_OFF;

    //Sets static IP and implicitly disables DHCP
    ethernet_port.set_network(our_ip_addr, rjnet_netmask, rjnet_default_gateway);

    //Now try to connect. We don't have connect() block so that we can loop printing a message on failure
    ethernet_port.set_blocking(false);
    while(!connect_if_no_network_connection()){
        printf("Ethernet port trying to connect\n");
        ThisThread::sleep_for(10ms);
    }
    ethernet_led = LED_ON;
    printf("Got network connection. Our IP: %s\n", our_ip_addr.get_ip_address());

    nsapi_error_t opened_network_socket = network_socket.open(&ethernet_port);
    while(opened_network_socket != NSAPI_ERROR_OK){
        //Didn't open socket. Slow blink
        ethernet_led = ! ethernet_led;
        printf("Network connection open but opening socket failed with error: %d\n", opened_network_socket);
        ThisThread::sleep_for(1s);
        opened_network_socket = network_socket.open(&ethernet_port);
    }

    //Bind the socket to the RJNet port
    nsapi_error_t bound_network_socket = network_socket.bind(RJNET_DEFAULT_PORT);
    while(bound_network_socket != NSAPI_ERROR_OK){
        //Didn't bind to port. Slow blink
        ethernet_led = ! ethernet_led;
        printf("Socket created but binding to port failed with error: %d\n", bound_network_socket);
        ThisThread::sleep_for(500ms);
        bound_network_socket = network_socket.bind(RJNET_DEFAULT_PORT);
    }

    //Set timeout on new socket
    network_socket.set_timeout(50);

    //Start a timer for the last time we sent anything
    Timer last_send_timer;
    last_send_timer.start();
    //Start the main loop.

    while(1) {

        //You don't need a check for lost Ethernet connection. It auto-reconnects if needed

        //Check for new messages. Blocks for a limited time. If error code is NSAPI_ERROR_WOULD_BLOCK everything OK
        nsapi_size_or_error_t receive_bytes_read = network_socket.recv(udp_message_received, MAX_RJNET_MESSAGE_LEN_BYTES);
        if(receive_bytes_read > 0){
            //Got a message
            printf("Got message: %s\n", udp_message_received);
        }
        else if(receive_bytes_read == NSAPI_ERROR_WOULD_BLOCK){}
        else{
            printf("Network socket died\n");
        }

        //Send any messages needed
        if(std::chrono::duration_cast<std::chrono::milliseconds>(last_send_timer.elapsed_time()) > TIME_BETWEEN_SENDS){
            int num_bytes_to_send = udp_message_send.length();
            nsapi_size_or_error_t num_bytes_sent = network_socket.sendto(other_ip_addr, udp_message_send.c_str(), num_bytes_to_send);
            if(num_bytes_sent < 0){
                printf("Error when sending: %d\n", num_bytes_sent);
            }
            else if(num_bytes_sent < num_bytes_to_send){
                printf("Only sent partial message\n");
            }
            else{
                printf("Sent OK: %s\n", udp_message_send.c_str());
            }
            last_send_timer.reset();
        }

    }
}