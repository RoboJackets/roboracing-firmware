/* SPI pins */
#define SPI_MOSI_PIN        MOSI
#define SPI_MISO_PIN        MISO
#define SPI_SCLK_PIN        SCK

static const byte DIR_PIN = 0;   // atmega pin 20, digital pin 0/RX; direction of stepper, HIGH: clockwise, LOW: counter clockwise
static const byte PULSE_PIN = 1; // atmega pin 21, digital pin 1/TX; provides pwm signal to motor
static const byte LIMIT_SWITCH_1_PIN = 7; // atmega pin 1, digital pin 7
static const byte LIMIT_SWITCH_2_PIN = 2; // atmega pin 19, digital pin 
static const byte ETH_CS_PIN = 12;  // atmega pin 26, digital pin 12
static const byte ETH_RST_PIN = 3;  // atmega pin 18, digital pin 3
static const byte ENC_CS_PIN = 6;
static const byte LED_PIN = A3;

#define BAUDRATE        115200

/* SPI commands */
#define NOP       0x00   // no operation, encoder just sends data back
#define RESET     0x60
#define ZERO      0x70

/* Define special ascii characters */
#define NEWLINE         0x0A
#define TAB             0x09



