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
  
  pinMode(CH_1, INPUT);
  pinMode(CH_2, INPUT);
  pinMode(CH_3, INPUT);
  
  pinMode(RC_IN, INPUT);
  
  pinMode(THROTTLE, INPUT);
  pinMode(SWITCH, INPUT);
  
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
}

void setLED1(boolean on){
  if (on) digitalWrite(LED_1, HIGH);
  else digitalWrite(LED_1, LOW);
}

void setLED2(boolean on){
  if (on) digitalWrite(LED_2, HIGH);
  else digitalWrite(LED_2, LOW);
}
