const int RXLED=8;


const int INT_ETH=3;	// atmega pin 18, digital pin 3/SCL

const int ENCODER_A=2;	// atmega pin 19, digital pin 2/SDA
const int ENCODER_B=0;	// atmega pin 20, digital pin 0/RX

const int ETH_RST=21;	// atmega pin 21, digital pin 1/TX

const int REVERSE_LED=6;	// atmega pin 27, digital pin 6/PD7
const int USER_DEFINED_LED=8;	// atmega pin 28, digital pin 8/PB4

const int MOTOR_CONTROL=9;	// atmega pin 29, digital pin 9/PB5

const int BRAKE_EN=5;	// atmega pin 31, digital pin 5/PC6
const int BRAKE_PWM=13;	// atmega pin 32, digital pin 13/PC7

const int REVERSE_OUT=A1;	// atmega pin 37, analog input 1/PF6
const int FORWARD_OUT=A2;	// atmega pin 38, analog input 2/PF5

const int CURR_DATA=A5;	// atmega pin 41, analog input 5/PF0

void turnRXLEDOn(boolean on){
  if (on) digitalWrite(REVERSE_LED, HIGH);
  else digitalWrite(REVERSE_LED, LOW);
}

void turnReverseLEDOn(boolean on){
  if (on) digitalWrite(REVERSE_LED, HIGH);
  else digitalWrite(REVERSE_LED, LOW);
}

void turnUSERLEDOn(boolean on){
  if (on) digitalWrite(USER_DEFINED_LED, HIGH);
  else digitalWrite(USER_DEFINED_LED, LOW);
}
