#include <cstdint>
#include <mbed.h>
#include "EthernetInterface.h"
#include "rjnet_mbed_udp.h"

//Initialize static members
EthernetInterface RJNetMbed::ethernet_port;
UDPSocket RJNetMbed::network_socket;
bool RJNetMbed::debug = true;

//Time between sends
constexpr Kernel::Clock::duration_u32 RJNetMbed::TIME_BETWEEN_SENDS = 100ms;

//Private Constants
const SocketAddress RJNetMbed::rjnet_netmask("255.255.255.0");
const SocketAddress RJNetMbed::rjnet_default_gateway("0.0.0.0");

//Functions

RJNetMbed::RJNetMbed(const SocketAddress & our_ip_addr, void (*process_one_message)(const SocketAddress &, const char[], unsigned int))
    : our_ip_address(our_ip_addr), process_single_message(process_one_message) {}

void RJNetMbed::start_network_and_listening_threads() {
    // if (debug) printf("Starting connection\n");
    //Sets static IP and implicitly disables DHCP
    ethernet_port.set_blocking(false);  //Don't block so we can print while waiting to connect
    ethernet_port.set_network(our_ip_address, rjnet_netmask, rjnet_default_gateway);
    ethernet_port.connect();

    //Wait for connection to come up
    nsapi_connection_status_t eth_connection_status = ethernet_port.get_connection_status();
    while(!(eth_connection_status == NSAPI_STATUS_LOCAL_UP || eth_connection_status == NSAPI_STATUS_GLOBAL_UP)) {
        if (debug) printf("Ethernet not connected. Our IP: %s Error code: %d\n", our_ip_address.get_ip_address(), eth_connection_status);
        ThisThread::sleep_for(100ms);
        eth_connection_status = ethernet_port.get_connection_status();
    }

    if (debug) printf("Got ethernet link connection. Our IP: %s\n", our_ip_address.get_ip_address());
    
    nsapi_error_t opened_network_socket = network_socket.open(&ethernet_port);
    while(opened_network_socket != NSAPI_ERROR_OK) {
        if (debug) printf("Network connection open but opening socket failed with error: %d\n", opened_network_socket);
        ThisThread::sleep_for(1s);
        opened_network_socket = network_socket.open(&ethernet_port);
    }

    //Bind the socket to the RJNet port
    nsapi_error_t bound_network_socket = network_socket.bind(RJNET_DEFAULT_PORT);
    while(bound_network_socket != NSAPI_ERROR_OK) {
        if (debug) printf("Socket created but binding to port failed with error: %d\n", bound_network_socket);
        ThisThread::sleep_for(500ms);
        bound_network_socket = network_socket.bind(RJNET_DEFAULT_PORT);
    }

    //Thread to listen for new messages
    udp_receiving_thread.start([this]() { this->listen_for_new_messages(); });
}

bool RJNetMbed::send_single_message(string message, SocketAddress send_to) {
    //Send string message to the IP address send_to. Blocks until message send or socket error.
    //Does not block or throw an error if the IP address is not connected to the network.
    send_to.set_port(RJNET_DEFAULT_PORT);  //Set right port

    int num_bytes_to_send = message.length();
    nsapi_size_or_error_t num_bytes_sent = network_socket.sendto(send_to, message.c_str(), num_bytes_to_send);

    //Print any errors
    if (num_bytes_sent < 0){
        if (debug) printf("Error when sending: %d\n", num_bytes_sent);
    }
    else if (num_bytes_sent < num_bytes_to_send) {
        if (debug) printf("Only sent partial message\n");
    }
    
    return num_bytes_to_send == num_bytes_sent;
}


bool RJNetMbed::are_ip_addrs_equal(const SocketAddress & address_a, const SocketAddress & address_b) {
    //Returns true if the two SocketAddress objects have the same IP ADDRESS, ignoring port
    nsapi_addr_t a_ip = address_a.get_addr();
    nsapi_addr_t b_ip = address_b.get_addr();
    if(a_ip.version == NSAPI_UNSPEC || a_ip.version != b_ip.version){
        //Not same IP versions or both are unspecified. Either case is false
        return false;
    }

    if (__builtin_expect(a_ip.version == NSAPI_IPv4, 1)) {
        int32_t *a_bytes = (int32_t *)&a_ip.bytes;
        int32_t *b_bytes = (int32_t *)&b_ip.bytes;
        return ~(*a_bytes ^ *b_bytes);
    } else {
        int64_t *a_bytes = (int64_t *)&a_ip.bytes;
        int64_t *b_bytes = (int64_t *)&b_ip.bytes;
        return ~((*a_bytes ^ *b_bytes) | (a_bytes[1] ^ a_bytes[1]));
    }
}

//Private functions


//Thread for listening for incoming messages. Calls process_single_message
//when message recieved.
void RJNetMbed::listen_for_new_messages() {
    //Set up buffer for received message
    char udp_message_received[RJNetMbed::MAX_RJNET_MESSAGE_LEN_BYTES] = {0};

    while (true) {         
        //This blocks indefinitely until a message is received
        SocketAddress senders_addr;
        nsapi_size_or_error_t receive_bytes_read = network_socket.recvfrom(&senders_addr, udp_message_received, MAX_RJNET_MESSAGE_LEN_BYTES);

        //Now process the message received
        if(receive_bytes_read >= 0) {
            //Got a valid message
            //Get the sender's address
            process_single_message(senders_addr, udp_message_received, receive_bytes_read);
        }
        else {
            //Possible network error types: https://os.mbed.com/docs/mbed-os/v6.15/mbed-os-api-doxy/group__netsocket.html#gac21eb8156cf9af198349069cdc7afeba
            if (debug) printf("UDP receive error. Error code: %d\n", receive_bytes_read);
        }
    }
}

void RJNetMbed::set_debug(bool debugMode) {
    debug = debugMode;
}