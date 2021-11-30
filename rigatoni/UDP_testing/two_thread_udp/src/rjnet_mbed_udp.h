#pragma once
#include <mbed.h>
#include "EthernetInterface.h"

//Possible network error types: https://os.mbed.com/docs/mbed-os/v6.15/mbed-os-api-doxy/group__netsocket.html#gac21eb8156cf9af198349069cdc7afeba
//UDPSocket reference: https://os.mbed.com/docs/mbed-os/v6.15/apis/udpsocket.html
//SocketAddress reference: https://os.mbed.com/docs/mbed-os/v6.15/apis/socketaddress.html
//EthernetInterface: https://os.mbed.com/docs/mbed-os/v6.15/apis/ethernet.html
//Threads and setting flags: https://os.mbed.com/docs/mbed-os/v6.15/apis/thread.html
//ThisThread and waiting for flags: https://os.mbed.com/docs/mbed-os/v6.15/apis/thisthread.html

class RJNetMbed {
    public:
        //Desired time between scheduled message sends
        static const Kernel::Clock::duration_u32 TIME_BETWEEN_SENDS = 100ms;
        //RJNet messages must be shorter than this:
        static const uint16_t MAX_RJNET_MESSAGE_LEN_BYTES = 1460;
        //The default port for RJNET messages
        static const uint16_t RJNET_DEFAULT_PORT = 2847;

        //Constructor. First arg is our IP address, next arg is the function to call whenever we get a message
        RJNetMbed(const SocketAddress &, void *(const SocketAddress &, const char[], unsigned int));

        //Checks IP addresses for equality ignoring port numbers
        static bool are_ip_addrs_equal(const SocketAddress &, const SocketAddress &);

    private:
        //Internal constants
        static const SocketAddress rjnet_netmask;     //Restrict to 192.168.20.*
        static const SocketAddress rjnet_default_gateway;   //Don't need a gateway since not communicating beyond the LAN.

        const SocketAddress our_ip_address;

        //This is a pointer to the function that gets called whenever we get a new messgae
        //The arguments are (Address of sender, the message as a character array, length of the message)
        void (*process_single_message)(const SocketAddress &, const char[], unsigned int);

        //The Ethernet interface
        EthernetInterface ethernet_port;

        //UDP socket that will be bound to the port
        UDPSocket network_socket;

        //Thread to listen for new messages
        Thread udp_receiving_thread;

        //Listens for new messages. Calls process_single_message when it gets a message
        void listen_for_new_messages();


}