#include <LiquidCrystal.h>
#include <Servo.h>

void motor(int val);
void steer(int val);
void update();
int limitDesiredSpeed(int desiredSpeed);
int limitDesiredHeading(int desiredHeading);
void updateHeading();
void updateSpeed();

static int currentSpeed = 0;
static int desiredSpeed=0;
static int currentHeading = 0;
static int desiredHeading = 0;

//control limits
static const int maxSpeed = 30; // maximum velocity 
static const int minSpeed = -15;
static const int minSteer = -25;
static const int maxSteer = 25;

void setup()

}

void loop()
{

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

void limitDesiredSpeed(int desiredSpeed) {
    return min(maxSpeed, max(requestedSpeed, minSpeed));
}

void limitDesiredHeading(int desiredHeading) {
    return min(maxHeading, max(requestedHeading, minHeading));    
}

void updateHeading()
{
    
}
/*
    - encoder input -> PID -> writing to motor
    - encoder input -> back to Joule
    - read the mux state -> tell PID what do
    - read the E-Stop state -> tell PID what do

*/
