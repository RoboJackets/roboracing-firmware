
// Notes:
// Need to connect VDC GND and dir- to common GND
// Need to have feedback connected to function
// 15.3 (gear ratio) * 1600 = pulses per revolution

byte dirPin = 3;
byte pulsePin = 6;
byte interruptPin = 2;

void setup() {
  Serial.begin(115200);
  pinMode(dirPin, OUTPUT);
  pinMode(pulsePin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), interrupt, FALLING);
  digitalWrite(dirPin, LOW); // HIGH: clockwise, LOW: counter clockwise
  delayMicroseconds(30);
   for(int i=0; i<24480; i++)
  {
    delay(.01);
    Serial.print("tick");
    digitalWrite(pulsePin, HIGH);
    delayMicroseconds(20);
    digitalWrite(pulsePin, LOW);
    delayMicroseconds(20);
  }
}
 
void loop() {

}


void interrupt(){
  digitalWrite(pulsePin, HIGH);
  delayMicroseconds(20);
  digitalWrite(pulsePin, LOW);
  delayMicroseconds(20);
}
