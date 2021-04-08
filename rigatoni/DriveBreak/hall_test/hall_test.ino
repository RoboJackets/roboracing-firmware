const int encoder_A = 2;
bool isOn;

void setup() {
  pinMode(encoder_A, INPUT);
  Serial.begin(115200);
}
void loop() {
  isOn = digitalRead(encoder_A);
  Serial.println(isOn);
  // delay(200);
  /**
  if(isOn < 0) {
    Serial.println("1");
  }
  else
    Serial.println("0");
  **/
}
