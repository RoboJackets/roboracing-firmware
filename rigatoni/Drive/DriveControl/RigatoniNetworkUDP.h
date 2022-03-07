#include <Ethernet.h>

//Header file for Ethernet for UDP communications

#define ETH_RETRANSMISSION_DELAY_MS 50  //Time before ARP times out if IP address has never existed in the network.

#define MIN_MESSAGE_SPACING 100 //Send messages at 10 Hz

#define BOARD_RESPONSE_TIMEOUT_MS 500 //If a board hasn't responded for 500ms we assume it has timed out and disconnected.


static const IPAddress nucIP(192, 168, 20, 2);
static const IPAddress estopIP(192, 168, 20, 3);
static const IPAddress driveIP(192, 168, 20, 4);
static const IPAddress steeringIP(192, 168, 20, 5);
static const IPAddress manualIP(192, 168, 20, 6);
static const IPAddress brakeIP(192, 168, 20, 7);
static const IPAddress broadcastIP(192, 168, 20, 255);

const static byte estopMAC[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x03};
const static byte driveMAC[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x04};
const static byte steeringMAC[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x05};
const static byte manualMAC[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x06};
const static byte brakeMAC[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x07};
