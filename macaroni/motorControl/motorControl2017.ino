#include <LiquidCrystal.h>
#include <Servo.h>

void motor(int val);
void steer(int val);
void update();
int limitDesiredSpeed(int desiredSpeed);
int limitDesiredHeading(int desiredHeading);
void updateHeading();
void updateSpeed();
void tick();
boolean getMessage();

static float currentSpeed = 0;
static float currentHeading = 0;
static float desiredHeading = 0;

static volatile int currentTicks = 0; //volatile data for manipulation in interrupt routines
static          int lastTicks = 0;
static int ticks_per_rotation = 200;
static int meters_per_rotation = 0.1107 * 3.14; 

//control limits
static const int maxSpeed = 30; // maximum velocity 
static const int minSpeed = -15;
static const int minSteer = -25;
static const int maxSteer = 25;

static const int maxHeading = 1;
static const int minHeading = 1;

const static int   pid_p = 0.1;
const static int   pid_i = 0.1;
const static int   pid_d = 0.1;

#define HISTORY_SIZE 100
static float         desiredSpeed = 0;
static float         errorSum = 0;
static float         errorHistory[HISTORY_SIZE] = {0};
static unsigned long lastSpeedUpdateMicros = 0;
static int           historyIndex = 0;

const int estopPin = A1;
boolean estop = false;
static boolean timeout = false;

Servo esc;
const int escMux = 10;
Servo steering;
const int steerMux = 11;

const int encoderA = 2;
const int encoderB = 4;

const int muxStatePin = A0;
boolean muxState = false;

volatile int tickData = 0;

static int lastMessageTime;
void setup()
{
    pinMode(estop, INPUT);

    pinMode(escMux, OUTPUT);
    pinMode(steerMux, OUTPUT);

    pinMode(encoderA, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderA), tick, CHANGE);
    pinMode(encoderB, INPUT);

    pinMode(muxState, INPUT);

    digitalWrite(escMux, 0);
    digitalWrite(steerMux, 0);

    Serial.begin(9600);
    lastMessageTime = millis();

}

void loop()
{   
    muxState = digitalRead(muxStatePin);
    estop = digitalRead(estopPin);
    if (estop) {
        digitalWrite(escMux, 0);
        digitalWrite(steerMux, 0);
    } 
    //if we haven't received a message from the joule in a while, stop driving
    if (lastMessageTime + 500 < millis()) {
      timeout = true;
    }
    update();
}

void update()
{
    if(getMessage()) {
        desiredSpeed = limitDesiredSpeed(desiredSpeed);
        desiredHeading = limitDesiredHeading(desiredHeading); 
    }
    updateHeading();
    updateSpeed();
}

int limitDesiredSpeed(int desiredSpeed) {
    return min(maxSpeed, max(desiredSpeed, minSpeed));
}

int limitDesiredHeading(int desiredHeading) {
    return min(maxHeading, max(desiredHeading, minHeading));    
}

void updateHeading()
{
    if (!muxState || estop) {
        steer(0);  
        desiredHeading = 0;
        currentHeading = 0;
    }
    else if (currentHeading != desiredHeading) {
        currentHeading = desiredHeading;
        steer(currentHeading); 
    }    
}

void updateSpeed()
{
  if (!muxState || estop || timeout) {
    motor(0);  
    errorSum = 0;
    errorHistory[HISTORY_SIZE] = {0};
    desiredSpeed = 0;
    currentSpeed = 0;
  } else {
	  float deltaMeters = (float)(currentTicks - lastTicks) / ticks_per_rotation * meters_per_rotation;
	  float deltaSeconds = (float)(micros() - lastSpeedUpdateMicros) / 1000000;
    currentSpeed = deltaMeters / deltaSeconds;
	  float currentError = desiredSpeed - (deltaMeters / deltaSeconds);
	  
	  // find derivative of error
	  float derivError = (float)(currentError - errorHistory[historyIndex]) 
				    / (micros() - lastSpeedUpdateMicros);

  	// update integral error
  	// TODO test assumption of even-enough spacing of the measurements thru time
  	// TODO test assumption that floating point inaccuracies won't add up too badly
  	historyIndex = (historyIndex+1) % HISTORY_SIZE;
  	errorSum -= errorHistory[historyIndex];
  	errorSum += currentError;
	  float integralError = errorSum / HISTORY_SIZE;
  	  
  	// combine PID terms
  	float targetMotorPwm = pid_p * currentError + pid_i * integralError + pid_d * derivError;
	  
  	// store previous state info
  	lastSpeedUpdateMicros = micros();
  	errorHistory[historyIndex] = currentError;
  	lastTicks = currentTicks;
	  
	  // update acutal PWM value. Slows down change rate to ESC to avoid errors
	  motor(targetMotorPwm);
   }
}

void steer(int val)
{
    steering.write(val + 90);
}

void motor(int val)
{
    esc.write(val + 90);
}

void tick()
{
    if (digitalRead(encoderA) == digitalRead(encoderB)) {
        currentTicks++;
    } else {
        currentTicks--;
    }
}

boolean getMessage()
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
      gotMessage = true;
      Serial.println('$' + currentSpeed + ',' + muxState + ',' + estop + '\n');
    }
  }
  return gotMessage;
}
