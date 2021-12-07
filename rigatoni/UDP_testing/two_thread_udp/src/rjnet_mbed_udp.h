#pragma once
#include <mbed.h>
#include "EthernetInterface.h"

//Possible network error types: https://os.mbed.com/docs/mbed-os/v6.15/mbed-os-api-doxy/group__netsocket.html#gac21eb8156cf9af198349069cdc7afeba
//UDPSocket reference: https://os.mbed.com/docs/mbed-os/v6.15/apis/udpsocket.html
//SocketAddress reference: https://os.mbed.com/docs/mbed-os/v6.15/apis/socketaddress.html
//EthernetInterface: https://os.mbed.com/docs/mbed-os/v6.15/apis/ethernet.html
//Threads and setting flags: https://os.mbed.com/docs/mbed-os/v6.15/apis/thread.html
//ThisThread and waiting for flags: https://os.mbed.com/docs/mbed-os/v6.15/apis/thisthread.html

//The IPs of all the boards in the system
//These CANNOT be part of the class if they are passed to the constructor
//or your code will compile but the address will be uninitialized
const SocketAddress nucIP("192.168.20.2");
const SocketAddress estopIP("192.168.20.3");
const SocketAddress driveIP("192.168.20.4");
const SocketAddress steeringIP("192.168.20.5");
const SocketAddress manualIP("192.168.20.6");
const SocketAddress brakeIP("192.168.20.7");
//Send to this address to send a broadcast message
const SocketAddress broadcastIP("192.168.20.255");

class RJNetMbed {
    //Values of static const constants are in the .cpp file because c++ syntax
    public:

        static const SocketAddress rjnet_netmask;     //Set the netmask.
        static const SocketAddress rjnet_default_gateway;   //Can set gateway to 0.0.0.0 since not communicating beyond the LAN.

        //Desired time between scheduled message sends
        static const Kernel::Clock::duration_u32 TIME_BETWEEN_SENDS;
        //RJNet messages must be shorter than this:
        static const uint16_t MAX_RJNET_MESSAGE_LEN_BYTES = 1460;
        //The default port for RJNET messages
        static const uint16_t RJNET_DEFAULT_PORT = 8888;

        //Constructor. First arg is our IP address, next arg is the function to call whenever we get a message
        RJNetMbed(const SocketAddress &, void (*) (const SocketAddress &, const char[], unsigned int));

        //Call this ONCE. Sets up interfaces, binds socket, and starts listening thread
        //Blocks until Ethernet link connected
        void start_network_and_listening_threads();

        //Sends a UDP message to the target IP address. Make sure the message is not too long
        bool send_single_message(string, SocketAddress);

        //Checks IP addresses for equality ignoring port numbers
        static bool are_ip_addrs_equal(const SocketAddress &, const SocketAddress &);

    private:
        //Internal constants
        //Our IP address
        const SocketAddress our_ip_address;

        //Ethernet port and socket
        static EthernetInterface ethernet_port;
        static UDPSocket network_socket;

        //This is a pointer to the function that gets called whenever we get a new messgae
        //The arguments are (Address of sender, the message as a character array, length of the message)
        void (*process_single_message)(const SocketAddress &, const char[], unsigned int);

        //Thread to listen for new messages
        Thread udp_receiving_thread;

        //Listens for new messages. Calls process_single_message when it gets a message.
        //Has to be implemented as static with the argument as this object pointer because c++ is wierd
        static void listen_for_new_messages(RJNetMbed *);


};