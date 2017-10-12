/* IGVC electrical
const int encoderRightData1 = 3;
const int encoderRightData2 = 5;
const int encoderLeftData1 = 2;
const int encoderLeftData2 = 4;
const int rightDir = 7;
const int rightSpeed = 6;
const int rightDisable = 8;
const int leftDir = 10;
const int leftSpeed = 9;
const int leftDisable = 11;
*/

// Note: left/right motor output headers are mislabeled above
const int pinEscSpeed = 9;
const int pinEscDirection = 10;
const int pinEncoderA = 3;
const int pinEncoderB = 5;
const int pinLED = 13;

const float wheelCircumference = 0.246 * 3.14159; //meters
const float ticksPerRot = 8637; // TODO update
const float metersPerTick = wheelCircumference / ticksPerRot;
const float deadZone = 0.05;

float desiredSpeed = 0;
float actualSpeed = 0;
float lastDutyCycle = 0;
volatile int encoderTicks = 0;
int stickyEncoderTicks = 0;

unsigned long lastMessageMillis;
unsigned long lastDriveMillis;

const unsigned long speedUpdateMillis = 20;
float pidP = 0; //0.03;
float pidI = 0;
float pidD = 0; //0.5;
const int errorHistorySize = 20;
float errorHistory[errorHistorySize] = {0};
int errorHistoryIndex = 0;
float lastError = 0.0;


inline float clamp(float x, float xMin, float xMax) {
  return min(max(x, xMin), xMax);
}

void tickEncoder() {
  if(digitalRead(pinEncoderA) == digitalRead(pinEncoderB)) {
    encoderTicks++;
  } else {
    encoderTicks--;
  }
}

// find the current speed (m/s)
void measureCurrentSpeedBlocking() {
  encoderTicks = 0;
  delay(speedUpdateMillis);

  float ticksPerSec = (float)encoderTicks * 1000.0 / (float)speedUpdateMillis;
  stickyEncoderTicks = encoderTicks;
  actualSpeed = ticksPerSec * metersPerTick;
}

bool getMessage() {
  bool gotMessage = false;
  while(Serial.available()) {
    char first = Serial.read();
    if(first == '$') {
      desiredSpeed = -1.0 * Serial.parseFloat();
      pidP = Serial.parseFloat();
      pidI = Serial.parseFloat();
      pidD = Serial.parseFloat();
      gotMessage = true;
    }
  }
  return gotMessage;
}

bool sendMessage() {
  bool sentMessage = false;
  if(Serial.availableForWrite()) {
    String message = "$";
    message.concat(actualSpeed);
    message.concat(", ");
    message.concat(lastDutyCycle);
    Serial.println(message);
    sentMessage = true;
  }
  return sendMessage;
}


void drive() {
  float dt = millis() - lastDriveMillis;
  lastDriveMillis = millis();

  float error = desiredSpeed - actualSpeed;
  float dError = error - lastError;
  lastError = error;

  errorHistoryIndex = (errorHistoryIndex + 1) % errorHistorySize;
  errorHistory[errorHistoryIndex] = error;
  float errorTotal = 0;
  for(int i = 0; i < errorHistorySize; i++) {
    errorTotal += errorHistory[i];
  }

  float pidCorrection = (pidP * error) + (pidI * errorTotal) + (pidD * dError/dt);

  int direction;
  float dutyCycle = lastDutyCycle + pidCorrection;
  dutyCycle = clamp(dutyCycle, -1, 1);
  
  // account for estop error collection
  if(abs(actualSpeed) < 0.01 && abs(desiredSpeed) > 0) {
    dutyCycle = clamp(dutyCycle, -0.35, 0.35);
  }

  lastDutyCycle = dutyCycle;

  // dead zone
  if(abs(dutyCycle) < 0.15) {
    dutyCycle = 0;
  }

  float convertedDutyCycle;
  if(dutyCycle >= 0) {
    direction = HIGH;
    convertedDutyCycle = 1.0 - dutyCycle;
  } else { //negative speed
    direction = LOW;
    convertedDutyCycle = -1 * dutyCycle;
  }

  int pwm = (int)(255 * convertedDutyCycle);
  digitalWrite(pinEscDirection, direction);
  analogWrite(pinEscSpeed, pwm);
}

void setup() {
  Serial.begin(9600);

  pinMode(pinEscDirection, OUTPUT);
  pinMode(pinEscSpeed, OUTPUT);

  pinMode(pinEncoderA, INPUT);
  pinMode(pinEncoderB, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinEncoderA), tickEncoder, CHANGE);

  lastMessageMillis = lastDriveMillis = millis();
}

void loop() {
  if(getMessage() && sendMessage()) {
    lastMessageMillis = millis();
  }

  if(millis() - lastMessageMillis > 500) {
    // timeout
    desiredSpeed = 0.0;
  }

  measureCurrentSpeedBlocking();

  drive();
}
