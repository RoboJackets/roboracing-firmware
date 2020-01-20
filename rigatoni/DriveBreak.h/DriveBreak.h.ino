const int D_N=3;
const int D_P=4;

const int VUSB=7;
const int RXLED=8;

const int SCLK=9;
const int MOSI=10;
const int MISO=11;

const int MCU_RST=13;

const int XTAL2=16;
const int XTAL2=17;

const int INT_ETH=18;

const int ENCODER_A=19;
const int ENCODER_B=20;

const int ETH_RST=21;

const int REVERSE_LED=27;
const int USER_DEFINED_LED=28;

const int MOTOR_CONTROLLER_INPUT=29;

const int BRAKE_EN=31
const int BRAKE_PWM=32;

const int REVERSE_OUT=37;
const int FORWARD_OUT=38;

const int CURR_DATA=41;

void setup(){
  pinMode(REVERSE_LED, OUTPUT);
  pinMode(USER_DEFINED_LED, OUTPUT);
}

void turnReverseLEDON(boolean on){
  if (on) digitalWrite(REVERSE_LED, HIGH);
  else digitalWrite(REVERSE_LED, LOW);
}

void turnUSERLEDON(boolean on){
  if (on) digitalWrite(USER_DEFINED_LED, HIGH);
  else digitalWrite(USER_DEFINED_LED, LOW);
}
