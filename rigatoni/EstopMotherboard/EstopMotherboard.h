#define POWER_IN 6
#define SENSOR_1 8

 // NOTE: Steering and drive pins are switched from the schematic
#define STEERING_IN 1
#define DRIVE_IN 0
#define STEERING_EN 9 // Sensor 2 used to control steering, original pin is 9
#define DRIVE_EN 10

// Unused
//#define SAFE_RB 3 
//#define SENSOR_2 12 
//#define INT 7

#define STACK_G A1 //Br
#define STACK_Y A0 //Bs
#define STACK_R A2 //W

#define ETH_RST 13
#define ETH_CS_PIN 11

#define LED1 4

//States for state machine
#define GO 0
#define STATIONARY 1
#define RESET 2
#define ESTOP 3

//Easy names for light states
#define OFF 0
#define ON 1
#define BLINK 2

#define BLINK_PERIOD_MS 500
