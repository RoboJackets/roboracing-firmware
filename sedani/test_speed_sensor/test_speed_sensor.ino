const int speedSensor = 2;

volatile float speed = 0.0;
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
  if(interruptCount-pInterruptCount == 0){
    speed = 0;
  }
  Serial.print(interruptCount);
  Serial.print("     ");
  Serial.println(speed);
  pInterruptCount = interruptCount;
  delay(100);
}

void calcSpeed(){
  if(pTime != micros()){
    speed = 1000000.0/(micros()-pTime);
  }
  else{
    speed = 0;
  }
  pTime = micros();
  interruptCount ++;
}

