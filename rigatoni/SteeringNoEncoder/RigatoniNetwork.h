#include <Ethernet.h>

#define ETH_NUM_SENDS 2
#define ETH_RETRANSMISSION_DELAY_MS 50  //Time before TCP tries to resend the packet
#define ETH_TCP_INITIATION_DELAY 50     //How long we wait before .connected() or .connect() fails.

#define MIN_MESSAGE_SPACING 100 //Send messages at 10 Hz


static const IPAddress nucIP(192, 168, 0, 2);
static const IPAddress estopIP(192, 168, 0, 3);
static const IPAddress driveIP(192, 168, 0, 4);
static const IPAddress steeringIP(192, 168, 0, 5);
static const IPAddress manualIP(192, 168, 0, 6);
static const IPAddress brakeIP(192, 168, 0, 7);

const static byte estopMAC[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x03};
const static byte driveMAC[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x04};
const static byte steeringMAC[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x05};
const static byte manualMAC[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x06};
const static byte brakeMAC[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x07};

static const byte PORT = 7;

