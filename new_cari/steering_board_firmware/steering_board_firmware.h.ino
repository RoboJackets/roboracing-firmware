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
