byte dirPin = 3;
byte pulsePin = 6;
byte interruptPin = 2;

void setup() {
  pinMode(dirPin, OUTPUT);
  pinMode(pulsePin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), interrupt, FALLING);
  digitalWrite(dirPin, HIGH);
  delayMicroseconds(30);
}
 
void loop() {
  delay(10);
}

void interrupt(){
  digitalWrite(pulsePin, HIGH);
  delayMicroseconds(20);
  digitalWrite(pulsePin, LOW);
  delayMicroseconds(20);
}