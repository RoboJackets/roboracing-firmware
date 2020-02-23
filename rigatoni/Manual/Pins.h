// Microcontroller reset button
const int RST_MCU = 13;

// ISP
const int SS = 8;
const int SCK = 9;
const int MOSI = 10;
const int MISO = 11;
const int INTN = 21;

// RC receiver channels
// Wheel, Trigger, Switch
const int CH_1 = 18;
const int CH_2 = 19;
const int CH_3 = 20;

// Interrupt stuff
volatile int pwm_value1 = 0;
volatile int prev_time1 = 0;
volatile int pwm_value2 = 0;
volatile int prev_time2 = 0;
volatile int pwm_value3 = 0;
volatile int prev_time3 = 0;

// Is the receiver plugged in?
const int RC_IN = 32;

// Manual switch and foot pedal sensing
const int THROTTLE = 36;
const int SWITCH = 37;

// User-defined status LEDs
const int LED_1 = 38;
const int LED_2 = 39;

void setup(){
  pinMode(RST_MCU, INPUT);
  
  pinMode(RC_IN, INPUT);
  
  pinMode(THROTTLE, INPUT);
  pinMode(SWITCH, INPUT);
  
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);

  attachInterrupt(CH_1, rising1, RISING);
  attachInterrupt(CH_2, rising2, RISING);
  attachInterrupt(CH_3, rising3, RISING);
}

void setLED1(boolean on){
  if (on) digitalWrite(LED_1, HIGH);
  else digitalWrite(LED_1, LOW);
}

void setLED2(boolean on){
  if (on) digitalWrite(LED_2, HIGH);
  else digitalWrite(LED_2, LOW);
}

// Interrupt stuff
void rising1() {
  attachInterrupt(PIN_0, falling1, FALLING);
  prev_time1 = micros();
}
 
void falling1() {
  attachInterrupt(PIN_0, rising1, RISING);
  pwm_value1 = micros()-prev_time1;
}

void rising2() {
  attachInterrupt(PIN_2, falling2, FALLING);
  prev_time2 = micros();
}
 
void falling2() {
  attachInterrupt(PIN_2, rising2, RISING);
  pwm_value2 = micros()-prev_time2;
}

void rising3() {
  attachInterrupt(PIN_3, falling3, FALLING);
  prev_time3 = micros();
}
 
void falling3() {
  attachInterrupt(PIN_3, rising3, RISING);
  pwm_value3 = micros()-prev_time3;
}
