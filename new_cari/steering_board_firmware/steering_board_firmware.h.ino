byte dirPin = 0;   // direction of stepper, HIGH: clockwise, LOW: counter clockwise
byte pulsePin = 1; // provides pwm signal to motor
float desiredAngle;
float currentAngle;
boolean toggle1 = 0;
boolean prevtoggle = 0;
const static int PORT = 7; // port RJnet uses

#define BAUDRATE        115200

/* SPI commands */
#define NOP       0x00   // no operation, encoder just sends data back
#define RESET     0x60
#define ZERO      0x70

/* Define special ascii characters */
#define NEWLINE         0x0A
#define TAB             0x09

/* SPI pins */
#define ENC             6     // Encoder select
#define SPI_MOSI        MOSI
#define SPI_MISO        MISO
#define SPI_SCLK        SCK

/* Ethernet */
// Enter a MAC address and IP address for your board below
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xE1 };
IPAddress ip(192, 168, 0, 171); //set the IP to find us at
EthernetServer server(PORT);

// Enter a IP address for other board below
IPAddress otherIP(192, 168, 0, 175); //set the IP to find us at
EthernetClient otherBoard;
