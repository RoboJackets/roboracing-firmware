byte dirPin = 0;   // atmega pin 20, digital pin 0/RX; direction of stepper, HIGH: clockwise, LOW: counter clockwise
byte pulsePin = 1; // atmega pin 21, digital pin 1/TX; provides pwm signal to motor
byte commandInterruptPin = 2;
<<<<<<< Updated upstream
float desiredAngle;
float currentAngle;
boolean toggle1 = 0;
boolean prevtoggle = 0;
=======
float maxAngle = PI/2;
float minAngle = -PI/2;
float desiredAngle = 0;
float currentAngle = 0;
bool toggle1 = false;
bool prevtoggle = false;
>>>>>>> Stashed changes
unsigned long startTime = 0;

const static int PORT = 7; // port RJnet uses
const int INT_ETH=2;  // atmega pin 19, digital pin 2/SDA

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

/* IP and MAC addresses */
#define NUC     2
#define MANUAL  6
