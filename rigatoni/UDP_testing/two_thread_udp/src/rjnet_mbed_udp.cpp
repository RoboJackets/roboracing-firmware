#include "rjnet_mbed_udp.h"
#include <mbed.h>
#include "EthernetInterface.h"

//Private Constants
const SocketAddress RJNetMbed::rjnet_netmask("255.255.255.0");
const SocketAddress RJNetMbed::rjnet_default_gateway("0.0.0.0");