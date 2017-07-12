#include <Servo.h>

//forward method declarations
void motor(int val);
void steer(int val);
void update();
void limitDesiredSpeed(float& desiredSpeed);
void limitDesiredHeading(float& desiredHeading);
void updateHeading();
void updateSpeed();
void encoderCallback();
bool getMessage();
void measureCurrentSpeed();

//tracking distance traveled
const int encoderA = 2;
const int encoderB = 7;
static volatile int currentTicks = 0; //volatile data for manipulation in interrupt routines

//control limits
static const float maxSpeed = 430; // maximum velocity 
static const float minSpeed = -90;
static const float maxHeading = 0.35;
static const float minHeading = -0.35;

//PID Constants
static float   pid_p = 1;
static float   pid_i = 0.0;
static float   pid_d = 0.1;

static int trim = 0;

//Setpoint and error tracking
static volatile float currentSpeed = 0;
static float currentHeading = 0;
static float desiredHeading = 0;
static float desiredSpeed = 0;
static float currentMotorPWM = 0;

//variables used for "dt" parts of PID control
#define HISTORY_SIZE 100
static float         lastError = 0;
static unsigned long lastSpeedUpdateMicros = 0;
static unsigned long lastPIDUpdateMicros = 0;
static int           historyIndex = 0;

//stop conditions for the car
static int lastMessageTime;
static bool timeout = false; 

const int estopPin = 10;
bool estop = false;

//autonomous or human control
const int muxStatePin = A5;
bool muxState = false;

//Motor objects
Servo esc;
const int escPin = 9;
Servo steering;
const int steerPin = 3;

int led_state = HIGH;

void setup()
{
    pinMode(estop, INPUT);

    pinMode(escPin, OUTPUT);
    esc.attach(escPin);
    pinMode(steerPin, OUTPUT);
    steering.attach(steerPin);

    pinMode(encoderA, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderA), encoderCallback, CHANGE);
    pinMode(encoderB, INPUT);

    pinMode(muxState, INPUT);

    pinMode(13, OUTPUT);
      
    motor(0);
    steer(0);

    Serial.begin(115200);
    lastMessageTime = millis();

}

void loop()
{   
  digitalWrite(13, led_state = !led_state);
  muxState = !digitalRead(muxStatePin);
  estop = digitalRead(estopPin);
  if (estop) {
    motor(0);
    steer(0);
  } 
  //if we haven't received a message from the joule in a while, stop driving
  if (lastMessageTime + 500 < millis()) {
    timeout = true;
  }
  measureCurrentSpeed();
  update();
  delay(50);
}

void update()
{
  if(getMessage()) {
      limitDesiredSpeed(desiredSpeed);
      limitDesiredHeading(desiredHeading); 
  }
  updateHeading();
  updateSpeed();
  
}

void limitDesiredSpeed(float& desiredSpeed) {
  desiredSpeed = min(maxSpeed, max(desiredSpeed, minSpeed));
}

void limitDesiredHeading(float& desiredHeading) {
  desiredHeading = min(maxHeading, max(desiredHeading, minHeading));    
}

void updateHeading()
{
  if (!muxState || estop || timeout) {
    steer(0);  
    desiredHeading = 0;
    currentHeading = 0;
  }
  else if (currentHeading != desiredHeading) {
    currentHeading = desiredHeading;
    int pwm = ((currentHeading - minHeading) * 142.857) - 45;
    steer(pwm);
  }    
}

void updateSpeed()
{
  if (!muxState || estop || timeout) {
    motor(0);
    desiredSpeed = 0;
    currentSpeed = 0;
  } else {
    float error = desiredSpeed - currentSpeed;
    float dError = error - lastError;
    lastError = error;
    float deltaPWM = (pid_p * error) + (pid_d * dError);
    deltaPWM = max(min(deltaPWM, 2), -2);
    currentMotorPWM += deltaPWM;

	  motor(currentMotorPWM);
   }
}

void steer(int val)
{
  steering.write(val + trim + 90);
}

void motor(int val)
{
  esc.write(val + 90);
}

void encoderCallback()
{
  if (digitalRead(encoderA) == digitalRead(encoderB)) {
    currentTicks++;
  } else {
    currentTicks--;
  }
}

bool getMessage()
{
  bool gotMessage = false;
  while(Serial.available())
  {
    if(Serial.read() == '$')
    {
      timeout = false;
      lastMessageTime = millis();
      desiredSpeed = Serial.parseFloat();
      desiredHeading = Serial.parseFloat();
      pid_p = Serial.parseFloat();
      pid_i = Serial.parseFloat();
      pid_d = Serial.parseFloat();
      trim = Serial.parseInt();
      gotMessage = true;      
      Serial.println(currentSpeed);
      gotMessage = true;
      String message = "$";
      message.concat(currentSpeed);
      message.concat(",");
      message.concat(muxState);
      message.concat(",");
      message.concat(estop);
      Serial.println(message);
    }
  }
  return gotMessage;
}

void measureCurrentSpeed() 
{
    long time = micros();
    long deltaTime = (long)(time - lastSpeedUpdateMicros) / 50000;
    lastSpeedUpdateMicros = time;
    currentSpeed = currentTicks / deltaTime;
    currentTicks = 0;
}

