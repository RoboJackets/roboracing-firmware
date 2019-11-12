
// Notes:
// Need to connect VDC GND and dir- to common GND
// Need to have feedback connected to function
// 15.3 (gear ratio) * 1600 = pulses per revolution

byte dirPin = 3;
byte pulsePin = 6;
byte commandInterruptPin = 2;
float desiredAngle;
float currentAngle;

void setup() {
  Serial.begin(115200);
  pinMode(dirPin, OUTPUT);
  pinMode(pulsePin, OUTPUT);
  pinMode(commandInterruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(commandInterruptPin), commandInterrupt, FALLING);
  digitalWrite(dirPin, LOW); // HIGH: clockwise, LOW: counter clockwise
  timer1_counter = 34286;    // preload timer 
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  delayMicroseconds(30);
}
 
void loop() {
  currentAngle = encoder.read();    
  Serial.print(currentAngle);     // keep sending encoder data back to NUC
}

ISR(TIMER1_OVF_VECTOR){                  // timer interrupt to move stepper motor
  TCNT1 = timer1_counter;          // preload timer
  if (desiredAngle != currentAngle){
    digitalWrite(pulsePin, HIGH);
    delayMicroseconds(20);
    digitalWrite(pulsePin, LOW);
    delayMicroseconds(20);
  }
}

void commandInterrupt(){
  if(Serial.read() == '$') {
    desiredAngle = Serial.parseFloat(); 
    desiredAngle = constrain(desiredSteeringAngle, minSteeringAngle, maxSteeringAngle);
  }
  currentAngle = encoder.read();         // read current position from encoder
  if (desiredAngle < currentAngle){      // set dirPIN to CW or CCW
    digitalWrite(dirPin, LOW);
  } else {
    digitalWrite(dirPIN, HIGH);
  }
}
