#include "EstopMotherboardtest.h"

void setup() {
    pinMode(SENSOR_1, INPUT);
    pinMode(BRAKE_IN, INPUT);
    pinMode(STEERING_IN, INPUT);
    pinMode(DRIVE_IN, INPUT);
    pinMode(SAFE_RB, INPUT);
    pinMode(POWER_IN, INPUT);

    Serial.begin(115200);
    Serial.println("Initialized");
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(digitalRead(SENSOR_1));
  Serial.print(digitalRead(BRAKE_IN));
  Serial.print(digitalRead(STEERING_IN));
  Serial.print(digitalRead(DRIVE_IN));
  Serial.print(digitalRead(SAFE_RB));
  Serial.println(digitalRead(POWER_IN));
  delay(1000);
}
