#include <mbed.h>
#include "EthernetInterface.h"

const bool LED_OFF = 0;
const bool LED_ON = 1;

//Configuration
const Kernel::Clock::duration_u32 TIME_BETWEEN_SENDS = 100ms;
const uint16_t MAX_RJNET_MESSAGE_LEN_BYTES = 1500;
const uint16_t RJNET_DEFAULT_PORT = 2847;
const SocketAddress our_ip_addr("192.168.20.5");
const SocketAddress rjnet_netmask("255.255.255.0");     //Restrict to 192.168.20.*
const SocketAddress rjnet_default_gateway("0.0.0.0");   //Don't need a gateway since not communicating beyond the LAN.

const SocketAddress nucIP("192.168.20.120");

//Ethernet status LED
DigitalOut ethernet_led(LED1);

//The default Ethernet interface:
EthernetInterface ethernet_port;

//Now make a UDP socket to use the port
UDPSocket network_socket;

//Now make threads for the sending and receiving
Thread udp_sending_thread;
Thread udp_receiving_thread;

//Possible network error types: https://os.mbed.com/docs/mbed-os/v6.15/mbed-os-api-doxy/group__netsocket.html#gac21eb8156cf9af198349069cdc7afeba
//UDPSocket reference: https://os.mbed.com/docs/mbed-os/v6.15/apis/udpsocket.html
//SocketAddress reference: https://os.mbed.com/docs/mbed-os/v6.15/apis/socketaddress.html
//EthernetInterface: https://os.mbed.com/docs/mbed-os/v6.15/apis/ethernet.html
//Threads and setting flags: https://os.mbed.com/docs/mbed-os/v6.15/apis/thread.html
//ThisThread and waiting for flags: https://os.mbed.com/docs/mbed-os/v6.15/apis/thisthread.html

//Deal with receiving messages

bool are_ip_addrs_equal(const SocketAddress & address_a, const SocketAddress & address_b){
    //Returns true if the two SocketAddress objects have the same IP ADDRESS, ignoring port
    nsapi_addr_t a_ip = address_a.get_addr();
    nsapi_addr_t b_ip = address_b.get_addr();
    if(a_ip.version == NSAPI_UNSPEC || a_ip.version != b_ip.version){
        //Not same IP versions or both are unspecified. Either case is false
        return false;
    }

    //How many bytes in IP address?
    uint8_t num_bytes_in_addr = a_ip.version == NSAPI_IPv4 ? 4 : 16;

    for(unsigned int i = 0; i < num_bytes_in_addr; i++){
        if(a_ip.bytes[i] != b_ip.bytes[i]){
            return false;
        }
    }
    return true;
}

bool send_single_message(string message, SocketAddress send_to){
    //Send string message to the IP address send_to. Blocks until message send or socket error.
    send_to.set_port(RJNET_DEFAULT_PORT);  //Set right port

    int num_bytes_to_send = message.length();
    nsapi_size_or_error_t num_bytes_sent = network_socket.sendto(send_to, message.c_str(), num_bytes_to_send);

    //Print any errors
    if(num_bytes_sent < 0){
        printf("Error when sending: %d\n", num_bytes_sent);
    }
    else if(num_bytes_sent < num_bytes_to_send){
        printf("Only sent partial message\n");
    }
    
    return num_bytes_to_send == num_bytes_sent;
}

//Deal with sending messages
void send_messages_udp_thread(){
    //This thread handles sending messages at a scheduled interval
    int i = 0;
    while (true){
        //Main sending loop. Sends scheduled messages every 100ms

        const unsigned int message_max_len = 32;
        char to_send[message_max_len];
        snprintf(to_send, message_max_len, "Scheduled message %d", i++);
        send_single_message(to_send, nucIP);

        //Reset timer for scheduled messages
        ThisThread::sleep_for(TIME_BETWEEN_SENDS);
    }
}

void process_single_message(const SocketAddress & senders_address, const char incoming_udp_message[], unsigned int num_bytes_in_message){
    //Parses a UDP message we just recieved. Places any received data in global variables.
    //Do parsing later, just print for now.
    
    if(are_ip_addrs_equal(nucIP, senders_address)){
        //Parse speed from message. Doing incoming_message + 2 ignores the first two characters
        float new_speed;
        sscanf(incoming_udp_message + 2, "%f", &new_speed);
        //Reply to NUC at once
        char outgoing_message [64];
        sprintf(outgoing_message, "Got speed = %f", new_speed);
        send_single_message(outgoing_message, nucIP);
    }
}

//Thread for listening for incoming messages
void listen_for_new_messages(){
    //Set up buffer for received message
    char udp_message_received[MAX_RJNET_MESSAGE_LEN_BYTES] = {0};

    while (true){
        //This blocks indefinitely until a message is received
        SocketAddress senders_addr;
        nsapi_size_or_error_t receive_bytes_read = network_socket.recvfrom(&senders_addr, udp_message_received, MAX_RJNET_MESSAGE_LEN_BYTES);

        //Now process the message received
        if(receive_bytes_read >= 0){
            //Got a valid message
            //Get the sender's address
            
            printf("Message from %s : %s \n", senders_addr.get_ip_address(), udp_message_received);
            process_single_message(senders_addr, udp_message_received, receive_bytes_read);
        }
        else{
            //Possible network error types: https://os.mbed.com/docs/mbed-os/v6.15/mbed-os-api-doxy/group__netsocket.html#gac21eb8156cf9af198349069cdc7afeba
            printf("UDP receive error. Error code: %d\n", receive_bytes_read);
        }
    }
}

void start_network_and_networking_threads(){
    ethernet_led = LED_OFF;

    //Sets static IP and implicitly disables DHCP
    ethernet_port.set_network(our_ip_addr, rjnet_netmask, rjnet_default_gateway);
    ethernet_port.set_blocking(false);  //Don't block so we can print while waiting to connect
    ethernet_port.connect();

    //Wait for connection to come up
    nsapi_connection_status_t eth_connection_status = ethernet_port.get_connection_status();
    while(!(eth_connection_status == NSAPI_STATUS_LOCAL_UP || eth_connection_status == NSAPI_STATUS_GLOBAL_UP)){
        ethernet_led = ! ethernet_led;
        printf("Ethernet link not connected\n");
        ThisThread::sleep_for(50ms);
        eth_connection_status = ethernet_port.get_connection_status();
    }

    printf("Got ethernet link connection. Our IP: %s\n", our_ip_addr.get_ip_address());

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

    //Everything is good!
    ethernet_led = LED_ON;

    //Now start the networking threads
    udp_sending_thread.start(send_messages_udp_thread);
    udp_receiving_thread.start(listen_for_new_messages);

}

int main() {

    start_network_and_networking_threads();

    while(1) {

        ThisThread::sleep_for(1s);

    }
}