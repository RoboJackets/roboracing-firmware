#define SAFE_RB 3 
#define START_RB 2 
#define STEERING_IN 0 
#define DRIVE_IN 1

#define STEERING_EN 10 
#define DRIVE_EN 9 
#define BRAKE_EN 6 

#define SENSOR_1 8 
#define SENSOR_2 12 

#define STACK_G A1 //Br
#define STACK_Y A0 //Bs
#define STACK_R A2 //W

#define INT 7
#define ETH_RST 13
#define ETH_CS_PIN 11

#define LED1 4

//States for state machine
#define GO 0
#define STOP 1
#define LIMITED 2

//Easy names for light states
#define OFF 0
#define ON 1
#define BLINK 2

#define BLINK_PERIOD_MS 500