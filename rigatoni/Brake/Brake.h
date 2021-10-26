static const byte DIR_PIN = 0;   // atmega pin 20, digital pin 0/RX; direction of stepper, HIGH: clockwise, LOW: counter clockwise
static const byte PULSE_PIN = 1; // atmega pin 21, digital pin 1/TX; provides pwm signal to motor
static const byte HOME_SWITCH_PIN = 7; // atmega pin 1, digital pin 7, ALM+
static const byte LIMIT_SWITCH_PIN = 2; // atmega pin 19, digital pin 2, EA+
static const byte ETH_CS_PIN = 12;  // atmega pin 26, digital pin 12
static const byte ETH_RST_PIN = 3;  // atmega pin 18, digital pin 3
static const byte ENC_CS_PIN = 6;
static const byte LED_PIN = A3;

//Engage max braking if don't hear from estop in this long
const int estopTimeoutMS = 500;

#define BAUDRATE        115200