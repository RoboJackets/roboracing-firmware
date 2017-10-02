const int pinEscSpeed = 9;
const int pinEscDirection = 10;
const int pinEncoder1 = 2;
const int pinEncoder2 = 4;

volatile int encoderTicks = 0;
const unsigned long speedUpdateMillis = 20;

const float wheelCircumference = 0.246 * 3.14159; //meters
const float ticksPerRot = 200.0; // TODO update
const float metersPerTick = wheelCircumference / ticksPerRot;

const float maxAcceleration = 0.1;

float desiredSpeed = 0;
float actualSpeed = 0;

unsigned long lastMessageMillis;
unsigned long lastDriveMillis;

const float pidP = 0.1;
const float pidI = 0.0;
const float pidD = 0.01;
const int errorHistorySize = 20;
float errorHistory[errorHistorySize] = {0};
int errorHistoryIndex = 0;
float lastError = 0.0;
float lastDutyCycle = 0.0;


inline float clamp(float x, float xMin, float xMax) {
  return min(max(x, xMin), xMax);
}

void tickEncoder() {
  if(digitalRead(pinEncoder1) == digitalRead(pinEncoder2)) {
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
  actualSpeed =  ticksPerSec * metersPerTick;
}

bool getMessage() {
  bool gotMessage = false;
  while(Serial.available()) {
    if(Serial.read() == '$') {
      desiredSpeed = -1.0 * Serial.parseFloat();
      gotMessage = true;
    }
  }
  return gotMessage;
}

bool sendMessage() {
  bool sentMessage = false;
  if(Serial.available()) {
    String message = "$";
    message.concat(actualSpeed);
    Serial.println(message);
    sentMessage = true;
  }
  return sentMessage;
}

float pidCorrection() {
  float error = actualSpeed - desiredSpeed;
  float dError = error - lastError;
  lastError = error;

  errorHistoryIndex = (errorHistoryIndex + 1) % errorHistorySize;
  errorHistory[errorHistoryIndex] = error;
  float errorTotal = 0;
  for(int i = 0; i < errorHistorySize; i++) {
    errorTotal += errorHistory[i];
  }

  return (pidP * error) + (pidI * errorTotal) + (pidD * dError);
}

void drive() {
  float dt = millis() - lastDriveMillis;
  float dV = maxAcceleration * dt;
  lastDriveMillis = millis();

  int direction;
  float clampedSpeed = clamp(desiredSpeed, actualSpeed-dV, actualSpeed+dV);
  float dutyCycle = clamp(clampedSpeed + pidCorrection(), -1.0, 1.0);
  if(dutyCycle >= 0) {
    direction = HIGH;
    dutyCycle = 1.0 - dutyCycle;
  } else { //negative speed
    direction = LOW;
    dutyCycle *= -1;
  }

  int pwm = (int)(255 * dutyCycle);
  digitalWrite(pinEscDirection, direction);
  analogWrite(pinEscSpeed, pwm);
}

void setup() {
  Serial.begin(9600);

  pinMode(pinEscDirection, OUTPUT);
  pinMode(pinEscSpeed, OUTPUT);

  pinMode(pinEncoder1, INPUT);
  pinMode(pinEncoder2, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinEncoder1), tickEncoder, CHANGE);

  lastMessageMillis = millis();
}

void loop() {
  if(getMessage() && sendMessage()) {
    drive();
    lastMessageMillis = millis();
  }

  if(millis() - lastMessageMillis > 500) {
    // timeout
    desiredSpeed = 0.0;
    drive();
  }

  measureCurrentSpeedBlocking();
}
