#include "pins.h"

// Interrupt stuff
volatile unsigned long pwm_value_ch_1 = 0;
volatile unsigned long pwm_value_ch_2 = 0;
volatile unsigned long pwm_value_ch_3 = 0;

volatile unsigned long prev_time_ch_1 = 0;
volatile unsigned long prev_time_ch_2 = 0;
volatile unsigned long prev_time_ch_3 = 0;

#define ch_1_lower 1536
#define ch_1_upper 1852
#define ch_2_lower 1484
#define ch_2_upper 2004
#define ch_3_mid 1494

bool led_1_state = true;
bool led_2_state = false;

bool rc_present_state = false;
bool rc_prev_state = true;

bool manual_state = true;

float value_ch_1;
float value_ch_2;
bool value_ch_3;

float value_throttle;

void setup(){
    Serial.begin(115200);
    pin_assign();
   
}


void loop() {
    evaluate_manual_switch();
    if(!manual_state){
        rc_missing();
        if(rc_present_state){
            evaluate_ch_1();
            evaluate_ch_2();
            evaluate_ch_3();
            Serial.print("CH_1 ");
            Serial.println(value_ch_1);
            Serial.print(" CH_2 ");
            Serial.println(value_ch_2);
            Serial.print(" CH_3 ");
            Serial.println(value_ch_3);
            led_1_state = !led_1_state;
            led_2_state = !led_2_state;
            set_led_1(led_1_state);
            set_led_2(led_2_state);
        }
    }
    else{
      
    }
    delay(50);
}



void pin_assign(){
    pinMode(RC_IN, INPUT);
  
    pinMode(THROTTLE, INPUT);
    pinMode(SWITCH, INPUT);
  
    pinMode(LED_1, OUTPUT);
    pinMode(LED_2, OUTPUT);

    pinMode(CH_1, INPUT);
    pinMode(CH_2, INPUT);
    pinMode(CH_3, INPUT);

    attachInterrupt(digitalPinToInterrupt(CH_1), isr_rising_ch_1, RISING);
    attachInterrupt(digitalPinToInterrupt(CH_2), isr_rising_ch_2, RISING);
    attachInterrupt(digitalPinToInterrupt(CH_3), isr_rising_ch_3, RISING);
}

// Set status LEDs
void set_led_1(bool on){
    digitalWrite(LED_1, on);
}

void set_led_2(bool on){
    digitalWrite(LED_2, on);
}

void evaluate_manual_switch(){
    manual_state = digitalRead(SWITCH);
}

void rc_missing(){
    rc_present_state = digitalRead(RC_IN);
    if(!rc_present_state){
        Serial.println("RC Reciever Missing");
        pwm_value_ch_1 = 0;
        pwm_value_ch_2 = 0;
        pwm_value_ch_3 = 0;
        prev_time_ch_1 = 0;
        prev_time_ch_2 = 0;
        prev_time_ch_3 = 0;
        if(rc_prev_state){
            detachInterrupt(digitalPinToInterrupt(CH_1));
            detachInterrupt(digitalPinToInterrupt(CH_2));
            detachInterrupt(digitalPinToInterrupt(CH_3));
        }
    } else if (rc_present_state && !rc_prev_state){
        attachInterrupt(digitalPinToInterrupt(CH_1), isr_rising_ch_1, RISING);
        attachInterrupt(digitalPinToInterrupt(CH_2), isr_rising_ch_2, RISING);
        attachInterrupt(digitalPinToInterrupt(CH_3), isr_rising_ch_3, RISING);
    }
    rc_prev_state = rc_present_state;
}

// Interrupt stuff
void isr_rising_ch_1() {
  attachInterrupt(digitalPinToInterrupt(CH_1), isr_falling_ch_1, FALLING);
  prev_time_ch_1 = micros();
}
 
void isr_falling_ch_1() {
  attachInterrupt(digitalPinToInterrupt(CH_1), isr_rising_ch_1, RISING);
  pwm_value_ch_1 = micros()-prev_time_ch_1;
}

void isr_rising_ch_2() {
  attachInterrupt(digitalPinToInterrupt(CH_2), isr_falling_ch_2, FALLING);
  prev_time_ch_2 = micros();
}
 
void isr_falling_ch_2() {
  attachInterrupt(digitalPinToInterrupt(CH_2), isr_rising_ch_2, RISING);
  pwm_value_ch_2 = micros()-prev_time_ch_2;
}

void isr_rising_ch_3() {
  attachInterrupt(digitalPinToInterrupt(CH_3), isr_falling_ch_3, FALLING);
  prev_time_ch_3 = micros();
}
 
void isr_falling_ch_3() {
  attachInterrupt(digitalPinToInterrupt(CH_3), isr_rising_ch_3, RISING);
  pwm_value_ch_3 = micros()-prev_time_ch_3;
}

void evaluate_ch_1() {
  value_ch_1 = map(pwm_value_ch_1, ch_1_lower, ch_1_upper, 0, 100) / 100.0;
  value_ch_1 = value_ch_1 * (abs(value_ch_1) > 0.02); // dead zone of 2%, becomes zero if false
}

void evaluate_ch_2() {
  value_ch_2 = map(pwm_value_ch_2, ch_2_lower, ch_2_upper, 0, 100) / 100.0;
  value_ch_2 = value_ch_2 * (abs(value_ch_2) > 0.02); // dead zone of 2%, becomes zero if false
  if (value_ch_2 < 0) {
    value_ch_2 = value_ch_2 / 4; // quarter speed backwards
  }
}

void evaluate_ch_3() {
  value_ch_3 = pwm_value_ch_3 > ch_3_mid;
}

void evaluate_throttle() {
  value_throttle = map(analogRead(THROTTLE), 0, 1023, 0, 100) / 100.0;
}
