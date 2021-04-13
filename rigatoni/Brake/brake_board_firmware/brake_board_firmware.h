byte dirPin = 0;   // atmega pin 20, digital pin 0/RX; direction of stepper, HIGH: clockwise, LOW: counter clockwise
byte pulsePin = 1; // atmega pin 21, digital pin 1/TX; provides pwm signal to motor
byte commandInterruptPin = 2;
float desiredAngle;
float currentAngle;
bool toggle1 = 0;
bool prevtoggle = 0;
float maxInt = 2;
float minInt = 0;
unsigned long startTime = 0;

const int INT_ETH=2;  // atmega pin 19, digital pin 2/SDA
const int CS_ETH = 12;  // atmega pin 26, digital pin 12

const static String stopMsg = "D";
const static String limitedMsg = "L";
const static String goMsg = "G";

//Engage max braking if don't hear from estop in this long
const int estopTimeoutMS = 500;

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

//LED
#define LED A3