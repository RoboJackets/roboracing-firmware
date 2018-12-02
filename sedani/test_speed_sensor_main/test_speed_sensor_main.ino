const int speedSensor = 2;

float speed = 0.0;
volatile long pTime = micros();
volatile int interruptCount = 0;
int pInterruptCount = 0;
void setup() {
  // put your setup code here, to run once:
  pinMode(speedSensor, INPUT);
  attachInterrupt(digitalPinToInterrupt(speedSensor), calcSpeed, RISING);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  long cTime = micros();
  if(pTime != cTime){
    speed = 1000000*(interruptCount-pInterruptCount)/(cTime-pTime);
  }
  else if(interruptCount-pInterruptCount == 0 || pTime == cTime){
    speed = 0;
  }
  pTime = micros();
  Serial.print(interruptCount);
  Serial.print("     ");
  Serial.println(speed);
  pInterruptCount = interruptCount;
  delay(50);
}

void calcSpeed(){
  //pTime = micros();
  interruptCount ++;
}

