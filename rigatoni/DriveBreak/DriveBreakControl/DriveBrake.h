const int RXLED=8;

const int MCU_RST=13;

const int INT_ETH=18;

const int ENCODER_A=19;
const int ENCODER_B=20;

const int ETH_RST=21;

const int REVERSE_LED=27;
const int USER_DEFINED_LED=28;

const int MOTOR_CONTROLLER_INPUT=29;

const int BRAKE_EN=31;
const int BRAKE_PWM=32;

const int REVERSE_OUT=37;
const int FORWARD_OUT=38;

const int CURR_DATA=41;

void setup(){
  pinMode(RXLED, OUTPUT);

  pinMode(INT_ETH, INPUT);

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);  

  pinMode(ETH_RST, OUTPUT);


  
  pinMode(REVERSE_LED, OUTPUT);
  pinMode(USER_DEFINED_LED, OUTPUT);

  pinMode(MOTOR_CONTROLLER_INPUT, OUTPUT);
  
  pinMode(BRAKE_EN, OUTPUT);
  pinMode(BRAKE_PWM, OUTPUT);

  pinMode(CURR_DATA, INPUT);
}

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
