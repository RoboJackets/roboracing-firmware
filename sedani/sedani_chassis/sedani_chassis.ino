//#define AUDIO_ENABLE
//#define NEO_ENABLE

#ifdef AUDIO_ENABLE
#include "pitch.h"
#endif

#include "SpeedLUT.h"
#include <Servo.h>


// Function declaration
void setup();
void loop();
int escPwmFromMetersPerSecond(float);
int escPwmPID(float);
float metersPerSecondFromEscPwm(int);
float radiansFromServoPwm(int);
int steeringPwmFromRadians(float);
bool getMessage();
void setPID();
void sendFeedback(const float*, const int);
void playSong(int);
void executeStateMachine();
void runHold();
void runHoldDoNotTrack();
void runStateManual();
void runStateForward();
void runStateBraking();
void runStateStopped();
void runStateReverse();
void steer();
void drive(int);
void calculateSpeed();
void encoderInterrupt();

// Pins
//Encoder pins must be interrupt capable
const byte encoderPinA = 2;
const byte encoderPinB = 3; 
const byte escPin = 4;
const byte steerPin = 5;
const byte isManualPin = 6;
const byte rcSteerPin = 7;
const byte rcEscPin = 8;
const byte speakerOutputPin = 9;
const byte buttonEstopPin = 10;
const byte wirelessPinC = 14;
const byte wirelessPinB = 15;
const byte wirelessPinD = 16;

// Control Limits
const float maxSpeed = 3; // maximum velocity in m/s 
const float minSpeed = -1; // minimum velocity in m/s 
const int centerSpeedPwm = 1472;
const int maxSpeedPwm = SpeedLUT[SpeedLUTMaxIndex][0];

const float maxSteeringAngle = 0.463; // Radians
const float minSteeringAngle = -0.463; // Radians

const int maxSteeringPwm = 1773;
const int minSteeringPwm = 1347;
const int centerSteeringPwm = 1560;

// Control Variables
float currentSteeringAngle = 0;
float desiredSteeringAngle = 0;
float desiredSpeed = 0;
int currentEscPwm = 0;

// Timeout Variables
unsigned long lastMessageTime;
unsigned long timeoutDuration = 1000;
bool isTimedOut = true; 


// Wireless States
bool prevWirelessStateB = false;
bool prevWirelessStateC = false;
bool prevWirelessStateD = false;

// Encoders
volatile long currentEncoderPosition = 0;
long prevEncoderPosition = 0;
float measuredSpeed = 0.0;
unsigned long prevEncoderTime = 0;
unsigned long currEncoderTime = 0;
const float metersPerEncoderTick = 1.0/2408.7;

// PID Speed Control
float integral = 0.0;
float derivative = 0.0;
float prevError = 0.0;
bool pidTuning = false;
float kP = 50.0;
float kI = 0.0;
float kD = 0.0;

//For timing the main loop
const int millisPerSec = 1000;
const byte millisPerLoop = 25;
unsigned long loopStartTime = 0;

// Reverse
const int brakePwm = 1300;
const float minBrakingSpeed = 0.05;

unsigned int consecutiveZeroSpeed = 0;
const byte minConsecutiveZeroSpeed = 3;

unsigned int consecutiveStop = 0;
const byte minConsecutiveStop = 18;

const int reversePwm = 1350;
const int reverseHoldPwm = 1470;

bool reverseTag = false;
bool reverseRequired = true;

float recordBag = 0.0; // 0 is not recording, 1 is recording 

// Servo objects
Servo esc;
Servo steering;

// State machine possible states
enum ChassisState {
    STATE_DISABLED = 0,
    STATE_TIMEOUT = 1,
    STATE_MANUAL = 2,
    STATE_FORWARD = 3,
    STATE_FORWARD_BRAKING = 4,
    STATE_REVERSE = 5,
    STATE_REVERSE_COAST = 6,
    STATE_REVERSE_TRANSITION = 7,
    STATE_IDLE = 8
} 
currentState = STATE_DISABLED;

// Songs!
#ifdef AUDIO_ENABLE
const int songInformation0[2] = {NOTE_C4, NOTE_C4};
const int songInformation1[2] = {NOTE_C5, NOTE_C4};
const int songInformation2[2] = {NOTE_C4, NOTE_C5};
const int songInformation3[2] = {NOTE_C5, NOTE_C5};
const int songInformation4[2] = {NOTE_C4, NOTE_C6};
const int songInformation5[2] = {NOTE_C6, NOTE_C6};
const int songInformation6[2] = {NOTE_C6, NOTE_C5};
const int songInformation7[2] = {NOTE_C5, NOTE_C6};
const int songInformation8[2] = {NOTE_C4, NOTE_C6};
#endif

// NeoPixel
#ifdef NEO_ENABLE
#include <NeoPixelBrightnessBus.h> // instead of NeoPixelBus.h
const uint16_t PixelCount = 8; // this example assumes 4 pixels, making it smaller will cause a failure
const uint8_t PixelPin = A1;  // make sure to set this to the correct pin, ignored for Esp8266

#define colorSaturation 128
#define brightness 50
// three element pixels, in different order and speeds
NeoPixelBrightnessBus<NeoRgbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);
RgbColor red(colorSaturation, 0, 0);
RgbColor green(0, colorSaturation, 0);
RgbColor blue(0, 0, colorSaturation);
RgbColor white(colorSaturation);
RgbColor black(0);
#endif

void setup() {
    pinMode(wirelessPinB,INPUT);
    pinMode(wirelessPinC,INPUT);
    pinMode(wirelessPinD,INPUT);
    pinMode(buttonEstopPin, INPUT);
    pinMode(rcEscPin, INPUT);
    pinMode(rcSteerPin, INPUT);
    pinMode(isManualPin, INPUT);
    pinMode(speakerOutputPin, OUTPUT);
    pinMode(escPin, OUTPUT);
    pinMode(steerPin, OUTPUT);
    pinMode(encoderPinA, INPUT);
    pinMode(encoderPinB, INPUT);
    
    esc.attach(escPin);
    steering.attach(steerPin);

    steering.write(centerSteeringPwm);
    drive(centerSpeedPwm);
    
    attachInterrupt(digitalPinToInterrupt(encoderPinA),encoderInterrupt, RISING);
    lastMessageTime = millis();
    isTimedOut = true;
    
    Serial.begin(115200);

    #ifdef NEO_ENABLE
    strip.Begin();
    strip.Show();
    strip.SetBrightness(brightness);
    #endif
  
    playSong(3);
}

void loop() {
    loopStartTime = millis();      
    bool gotMessage = getMessage();
    
    if (pidTuning == true){
        setPID();
    }

    // If we haven't received a message from the NUC in a while, stop driving
    if(gotMessage) {
        isTimedOut = false;
        lastMessageTime = millis();
    } else if ((lastMessageTime + timeoutDuration) < millis()) {
        isTimedOut = true;
    }

    calculateSpeed();
    
    executeStateMachine();

    if(gotMessage) {
        float values[] = {currentState, measuredSpeed, currentSteeringAngle, recordBag};
        sendFeedback(values, sizeof(values)/sizeof(float));
    }

    while (loopStartTime + millisPerLoop > millis());

}

int escPwmFromMetersPerSecond(float velocity) {
    if(velocity <= 0) {
        return SpeedLUT[0][0];
    }
    float prevLUTVelocity = 0.0;
    for(byte i = 1; i < SpeedLUTLength; i++) {
        float LUTVelocity = SpeedLUT[i][1];
        if(fabs(velocity - LUTVelocity) > fabs(velocity - prevLUTVelocity)) {
            return SpeedLUT[i-1][0];
        }
        prevLUTVelocity = LUTVelocity;
    }
    return SpeedLUT[SpeedLUTMaxIndex][0];
}


int escPwmPID(float velocity) { 
    if(velocity <= 0) {
        integral = 0;
        return SpeedLUT[0][0];
    }
    else{
        const float dt = (float)millisPerLoop / millisPerSec;

        float error = velocity - measuredSpeed;

        // protect against runaway integral accumulation
        float trapezoidIntegral = (prevError + error) * 0.5 * dt;
        if (abs(kI * integral) < maxSpeedPwm - centerSpeedPwm || trapezoidIntegral < 0) {
            integral += trapezoidIntegral;
        }

        derivative = (error - prevError) / dt; //difference derivative

        int writePwmSigned = kP * error + kI * integral + kD * derivative + escPwmFromMetersPerSecond(velocity);
        int writePwm = constrain(writePwmSigned, 0, maxSpeedPwm); //control limits

        prevError = error;
        return writePwm;
    }
}

float metersPerSecondFromEscPwm(int escPwm) {
    int index = escPwm - centerSpeedPwm; 
    index = constrain(index, 0, SpeedLUTMaxIndex);
    return SpeedLUT[index][1];
}

float radiansFromServoPwm(int servoPwm) {
    float distanceFromCenter = (float)(servoPwm) - (float)(centerSteeringPwm);
    if(distanceFromCenter > 0) {
        float prop = distanceFromCenter / (maxSteeringPwm - centerSteeringPwm);
        return prop * maxSteeringAngle;
    } else {
        float prop = distanceFromCenter / (minSteeringPwm - centerSteeringPwm);
        return prop * minSteeringAngle;
    }
}

int steeringPwmFromRadians(float radiansToSteer) {
    if(radiansToSteer > 0) {
        float prop = radiansToSteer / maxSteeringAngle;
        return (prop * (maxSteeringPwm - centerSteeringPwm)) + centerSteeringPwm;
    } else {
        float prop = radiansToSteer / minSteeringAngle;
        return (prop * (minSteeringPwm - centerSteeringPwm)) + centerSteeringPwm;
    }
}

// Messaging

bool getMessage() {
    bool gotMessage = false;
    while(Serial.available()) {
        if(Serial.read() == '$') {
            gotMessage = true;
            desiredSpeed = Serial.parseFloat();
            desiredSteeringAngle = Serial.parseFloat();
            desiredSpeed = constrain(desiredSpeed, minSpeed, maxSpeed);
            desiredSteeringAngle = constrain(desiredSteeringAngle, minSteeringAngle, maxSteeringAngle);
        }
    }
    return gotMessage;
}

void setPID() {
    while(Serial.available()){
      if (Serial.read() == '#'){
        kP = Serial.parseFloat();
        kI = Serial.parseFloat();
        kD = Serial.parseFloat();
      }
    }
}

void sendFeedback(const float* feedbackValues, const int feedbackCount) {
    String message = "$";
    for (int i = 0; i < feedbackCount; i++) {
        message.concat(feedbackValues[i]);
        if(i < feedbackCount-1) {
            message.concat(",");
        }
    }
    Serial.println(message);
}

// Playing songs
void playSong(int number) {
    #ifdef AUDIO_ENABLE
    for (int thisNote = 0; thisNote < 2; thisNote++) {
        unsigned long noteDuration = 125;
        switch(number) {
            case 0:
                tone(speakerOutputPin, songInformation0[thisNote], noteDuration);
                break;
            case 1:
                tone(speakerOutputPin, songInformation1[thisNote], noteDuration);
                break;
            case 2:
                tone(speakerOutputPin, songInformation2[thisNote], noteDuration);
                break;
            case 3:
                tone(speakerOutputPin, songInformation3[thisNote], noteDuration);
                break;
            case 4:
                tone(speakerOutputPin, songInformation4[thisNote], noteDuration);
                break;
            case 5:
                tone(speakerOutputPin, songInformation5[thisNote], noteDuration);
                break;
            case 6:
                tone(speakerOutputPin, songInformation6[thisNote], noteDuration);
                break;
            case 7:
                tone(speakerOutputPin, songInformation7[thisNote], noteDuration);
                break;
            case 8:
                tone(speakerOutputPin, songInformation8[thisNote], noteDuration);
                break;
            default:
                break;
        }
        int pauseBetweenNotes = 162;
        delay(pauseBetweenNotes);
        noTone(speakerOutputPin);
    }
    #endif
}

// State Machine
void executeStateMachine(){
    // Wireless State buttons
    bool wirelessStateB = digitalRead(wirelessPinB);
    bool wirelessStateC = digitalRead(wirelessPinC);
    bool wirelessStateD = digitalRead(wirelessPinD);

    /*
    // If you want to add more states based on the wireless remote, use state transition checks like these
    if(wirelessStateB && !prevWirelessStateB){
        playSong(2);
    }

    if(wirelessStateC && !prevWirelessStateC){
        playSong(1);
    }

    if(wirelessStateD && !prevWirelessStateD){
        playSong(0);
    }*/
    
    prevWirelessStateB = wirelessStateB;
    prevWirelessStateC = wirelessStateC;
    prevWirelessStateD = wirelessStateD;
    
    // Manual Variable (true means human drives)
    // TODO Switch back after new board
    // bool isManual = digitalRead(isManualPin);
    bool isManual = !digitalRead(isManualPin);

    // E-Stop Variables (true means the car can't move)
    bool buttonEstopActive = !digitalRead(buttonEstopPin);
    bool wirelessEstopActive = !(wirelessStateB || wirelessStateC || wirelessStateD);
    bool isEstopped = wirelessEstopActive || buttonEstopActive;

    // Record a Bag File with E-Stop Remote button D
    recordBag = wirelessStateD ? 1.0 : 0.0;
    
    switch(currentState){

        ////////////////////////////////////////////////////////
        //               (0)    DISABLED STATE                //
        // Centers steering angle and sets speed command      //
        // to zero. Enter this state if the wireless remote   //
        // or emergency pushbutton are pressed.               //
        ////////////////////////////////////////////////////////
        case STATE_DISABLED:{         
            /*----------------------
                            LOGIC
            ----------------------*/

            runHold();

            /*----------------------
                        TRANSITIONS
            ----------------------*/
            // Transition to Manual State
            if (!isEstopped && isManual){
                currentState = STATE_MANUAL;
                playSong(2);
                break;
            }
            // Transition to Timeout State
            if (!isEstopped && !isManual && isTimedOut){
                currentState = STATE_TIMEOUT;
                playSong(1);
                break;
            }
            // Transition to Idle State (only transition if not moving)
            if (!isEstopped && !isManual && !isTimedOut &&
                    desiredSpeed == 0 && measuredSpeed == 0){
                currentState = STATE_IDLE;
                playSong(8);
                break;
            }
            // Transition to Forward State
            if (!isEstopped && !isManual && !isTimedOut && desiredSpeed > 0){
                currentState = STATE_FORWARD;
                playSong(3);
                break;
            }
            // Default Loop State
            currentState = STATE_DISABLED;
            break;
        }

        ////////////////////////////////////////////////////////
        //              (1) TIMEOUT STATE                     //
        //    Sets Speed and Steering Angle to Zero. If       // 
        //    communication has not occured since             //
        //    timeoutDuration (in ms) and in Auto, go to this //
        //    state.                                          //
        ////////////////////////////////////////////////////////
        case STATE_TIMEOUT:{
            /*----------------------
                            LOGIC
            ----------------------*/

            runHold();

            /*----------------------
                        TRANSITIONS
            ----------------------*/
            // Transition to Disabled State
            if (isEstopped){
                currentState = STATE_DISABLED;
                playSong(0);
                break;
            }
            // Transition to Manual State
            if (!isEstopped && isManual){
                currentState = STATE_MANUAL;
                playSong(2);
                break;
            }
            // Transition to Idle State
            if (!isEstopped && !isManual && !isTimedOut &&
                    desiredSpeed == 0 && measuredSpeed == 0){
                currentState = STATE_IDLE;
                playSong(8);
                break;
            }            
            // Transition to Forward State
            if (!isEstopped && !isManual && !isTimedOut && desiredSpeed > 0){
                currentState = STATE_FORWARD;
                playSong(3);
                break;
            }
            // Default Loop Case
            currentState = STATE_TIMEOUT;
            break;
        }

        ////////////////////////////////////////////////////////
        //          (2)    MANUAL STATE                       //
        ////////////////////////////////////////////////////////
        case STATE_MANUAL:{
            /*----------------------
                            LOGIC
            ----------------------*/

            runStateManual();            
                    
            /*----------------------
                        TRANSITIONS
            ----------------------*/
            // Transition to Disabled State
            if (isEstopped){
                currentState = STATE_DISABLED;
                playSong(0);
                break;
            }
            // Transition to Timeout State
            if (!isEstopped && !isManual && isTimedOut){
                currentState = STATE_TIMEOUT;
                playSong(1);
                break;
            }
            // Transition to Idle State
            if (!isEstopped && !isManual && !isTimedOut &&
                    desiredSpeed == 0 && measuredSpeed == 0){
                currentState = STATE_IDLE;
                playSong(8);
                break;
            }
            // Transition to Forward State
            if (!isEstopped && !isManual && !isTimedOut && desiredSpeed > 0){
                currentState = STATE_FORWARD;
                playSong(3);
                break;
            }
            // Default Loop Case
            currentState = STATE_MANUAL;
            break;
        }
             
        ////////////////////////////////////////////////////////
        //              (3) FORWARD STATE                     //
        //         Autonomous forward motion following        //
        //                 inputs from NUC                    //
        ////////////////////////////////////////////////////////
        case STATE_FORWARD:{                    
            /*----------------------
                            LOGIC
            ----------------------*/

            runStateForward();
            
            /*----------------------
                        TRANSITIONS
            ----------------------*/
            // Transition to Disabled State
            if (isEstopped){
                currentState = STATE_DISABLED;
                playSong(0);
                break;
            }
            // Transition to Manual State
            if (!isEstopped && isManual){
                currentState = STATE_MANUAL;
                playSong(2);
                break;
            }
            // Transition to Timeout State
            if (!isEstopped && !isManual && isTimedOut){
                currentState = STATE_TIMEOUT;
                playSong(1);
                break;
            }
            // Transition to Idle
            if (!isEstopped && !isManual && !isTimedOut && 
                    desiredSpeed == 0 && measuredSpeed == 0){
                currentState = STATE_IDLE;
                playSong(8);
                break;
            }
            // Transition to Forward Braking State
            if (!isEstopped && !isManual && !isTimedOut && 
                    desiredSpeed <= 0 && measuredSpeed > 0){
                currentState = STATE_FORWARD_BRAKING;
                consecutiveZeroSpeed = 0; // Reset zero speed cycle counter
                playSong(4);
                break;
            }
            // Default Loop Case
            currentState = STATE_FORWARD;
            break;
        }

        ////////////////////////////////////////////////////////
        //           (4) FORWARD BRAKING STATE                //
        //          If you are going forward and want to go   //
        //          to zero or negative speed                 //
        ////////////////////////////////////////////////////////
        case STATE_FORWARD_BRAKING:{            
            /*----------------------
                            LOGIC
            ----------------------*/

            runStateBraking();
            
            /*----------------------
                        TRANSITIONS
            ----------------------*/
            // Transition to Disabled State
            if (isEstopped){
                currentState = STATE_DISABLED;
                playSong(0);
                break;
            }
            // Transition to Manual State
            if (!isEstopped && isManual){
                currentState = STATE_MANUAL;
                playSong(2);
                break;
            }
            // Transition to Timeout State
            if (!isEstopped && !isManual && isTimedOut){
                currentState = STATE_TIMEOUT;
                playSong(1);
                break;
            }
            // Transition to Forward State
            if (!isEstopped && !isManual && !isTimedOut && desiredSpeed > 0){
                currentState = STATE_FORWARD;
                playSong(3);
                break;
            }
            // Transition to Reverse Transition State
            // if (!isEstopped && !isManual && !isTimedOut && 
            // 	     desiredSpeed < 0 && measuredSpeed == 0 && 
            // 	     consecutiveZeroSpeed > minConsecutiveZeroSpeed){
            //     currentState = STATE_REVERSE_TRANSITION;
            //     consecutiveStop = 0; // Reset stop cycle counter
            //     playSong(7);
            //     break;
            // }
            //Transition to Idle State
            if (!isEstopped && !isManual && !isTimedOut && desiredSpeed == 0 && measuredSpeed == 0){
                currentState = STATE_IDLE;
                playSong(8);
                break;
            }
            // Default Loop Case
            currentState = STATE_FORWARD_BRAKING;
            break;
        }
                 
        ////////////////////////////////////////////////////////
        //                (5) REVERSE STATE                   //
        //         When we are actively moving in reverse     //
        ////////////////////////////////////////////////////////
        // case STATE_REVERSE:{
        //     /*----------------------
        //                     LOGIC
        //     ----------------------*/ 

        //     runStateReverse();

        //     /*----------------------
        //                 TRANSITIONS
        //     ----------------------*/
        //     // Transition to Disabled State
        //     if (isEstopped){
        //         currentState = STATE_DISABLED;
        //         playSong(0);
        //         break;
        //     }
        //     // Transition to Manual State
        //     if (!isEstopped && isManual){
        //         currentState = STATE_MANUAL;
        //         playSong(2);
        //         break;
        //     }
        //     // Transition to Timeout State
        //     if (!isEstopped && !isManual && isTimedOut){
        //         currentState = STATE_TIMEOUT;
        //         playSong(1);
        //         break;
        //     }
        //     // Transition to Forward State
        //     if (!isEstopped && !isManual && !isTimedOut && desiredSpeed > 0){
        //         currentState = STATE_FORWARD;
        //         playSong(3);
        //         break;
        //     }
        //     // Transition to Reverse Coast State
        //     if (!isEstopped && !isManual && !isTimedOut && 
        //     		desiredSpeed == 0 && measuredSpeed < 0){
        //         currentState = STATE_REVERSE_COAST;
        //         playSong(5);
        //         break;
        //     }
        //     // Transition to Idle state
        //     if (!isEstopped && !isManual && !isTimedOut && 
        //             desiredSpeed == 0 && measuredSpeed == 0){
        //         currentState = STATE_IDLE;
        //         playSong(8);
        //         break;
        //     }
        //     // Default Loop Case
        //     currentState = STATE_REVERSE;
        //     break;                    
        // }

        ////////////////////////////////////////////////////////
        //           (6) REVERSE COAST STATE                  //
        //     If speed is negative, this state allows us     //
        //     to return to zero speed by coasting until      //
        //     we stop.  Braking isn't possible due to ESC.   //
        ////////////////////////////////////////////////////////
        // case STATE_REVERSE_COAST:{
        //     /*----------------------
        //                     LOGIC
        //     ----------------------*/             
        //     runHold();
        //     /*----------------------
        //                 TRANSITIONS
        //     ----------------------*/
        //     // Transition to Disabled State
        //     if (isEstopped){
        //         currentState = STATE_DISABLED;
        //         playSong(0);
        //         break;
        //     }
        //     // Transition to Manual State
        //     if (!isEstopped && isManual){
        //         currentState = STATE_MANUAL;
        //         playSong(2);
        //         break;
        //     }
        //     // Transition to Timeout State
        //     if (!isEstopped && !isManual && isTimedOut){
        //         currentState = STATE_TIMEOUT;
        //         playSong(1);
        //         break;
        //     }
        //     // Transition to Forward State
        //     if (!isEstopped && !isManual && !isTimedOut && desiredSpeed > 0){
        //         currentState = STATE_FORWARD;
        //         playSong(3);
        //         break;
        //     }
        //     // Transition to Reverse State
        //     if (!isEstopped && !isManual && !isTimedOut && 
        //             desiredSpeed < 0 && measuredSpeed <= 0){
        //         currentState = STATE_REVERSE;
        //         playSong(6);
        //         break;
        //     }
        //     // Transition to Idle
        //     if (!isEstopped && !isManual && !isTimedOut && 
        //                 desiredSpeed == 0 && measuredSpeed == 0){
        //         currentState = STATE_IDLE;
        //         playSong(8);
        //         break;
        //     }
        //     // Default Loop Case
        //     currentState = STATE_REVERSE_COAST;
        //     break;
        // }

        ////////////////////////////////////////////////////////
        //           (7) REVERSE TRANSITION STATE             //
        //         State between braking and reversing where  //
        //         the robot must remain stationary for       //
        //         a short period of time to keep the ESC     //
        //         happy.                                     //
        ////////////////////////////////////////////////////////
        // case STATE_REVERSE_TRANSITION:{
        //     /*----------------------
        //                     LOGIC
        //     ----------------------*/             
        //     runStateStopped();
                    
        //     /*----------------------
        //                 TRANSITIONS
        //     ----------------------*/
        //     // Transition to Disabled State
        //     if (isEstopped){
        //         currentState = STATE_DISABLED;
        //         playSong(0);
        //         break;
        //     }
        //     // Transition to Manual State
        //     if (!isEstopped && isManual){
        //         currentState = STATE_MANUAL;
        //         playSong(2);
        //         break;
        //     }
        //     // Transition to Timeout State
        //     if (!isEstopped && !isManual && isTimedOut){
        //         currentState = STATE_TIMEOUT;
        //         playSong(1);
        //         break;
        //     }
        //     //Transition to Idle State
        //     if (!isEstopped && !isManual && !isTimedOut && 
        //     		desiredSpeed == 0 && measuredSpeed == 0){
        //         currentState = STATE_IDLE;
        //         playSong(8);
        //         break;
        //     }
        //     // Transition to Forward State
        //     if (!isEstopped && !isManual && !isTimedOut && desiredSpeed > 0){
        //         currentState = STATE_FORWARD;
        //         playSong(3);
        //         break;
        //     }
        //     // Transition to Reverse State (must wait for a minimum number of stop cycles before going to reverse)
        //     if (!isEstopped && !isManual && !isTimedOut && 
        //     		desiredSpeed < 0 && consecutiveStop > minConsecutiveStop){
        //         currentState = STATE_REVERSE;
        //         playSong(6);
        //         break;
        //     }
        //     // Default Loop Case
        //     currentState = STATE_REVERSE_TRANSITION;
        //     break;
        // }                    
         
        ////////////////////////////////////////////////////////
        //             (8) IDLE STATE                         //
        //          When the robot is at zero speed.          //
        //          External influence means we are not       //
        //          guaranteed to be at zero speed.           //
        //          Will hold the last commanded steering.    //
        ////////////////////////////////////////////////////////
        case STATE_IDLE:{
            /*----------------------
                            LOGIC
            ----------------------*/
            runHold();
                    
            /*----------------------
                        TRANSITIONS
            ----------------------*/
            // Transition to Disabled State
            if (isEstopped){
                currentState = STATE_DISABLED;
                playSong(0);
                break;
            }
            // Transition to Manual State
            if (!isEstopped && isManual){
                currentState = STATE_MANUAL;
                playSong(2);
                break;
            }
            // Transition to Timeout State
            if (!isEstopped && !isManual && isTimedOut){
                currentState = STATE_TIMEOUT;
                playSong(1);
                break;
            }
            // Transition to Forward State
            if (!isEstopped && !isManual && !isTimedOut && desiredSpeed > 0){
                currentState = STATE_FORWARD;
                playSong(3);
                break;
            }
            // Transition to Forward Braking
            // if (!isEstopped && !isManual && !isTimedOut && desiredSpeed < 0 && measuredSpeed >= 0){
            //     currentState = STATE_FORWARD_BRAKING;
            //     consecutiveZeroSpeed = 0; // Reset zero speed cycle counter
            //     playSong(4);
            //     break;
            // }
            //Default Loop Case
            currentState = STATE_IDLE;
            break;
        }
    }
}

// State Functions

void runHold(){
    steer();
    drive(centerSpeedPwm);
}

void runHoldDoNotTrack(){
    steering.write(centerSteeringPwm);
    esc.write(centerSpeedPwm);
}

void runStateManual() {
    int currentReadSteerPwm = pulseIn(rcSteerPin,HIGH);
    int currentReadEscPwm = pulseIn(rcEscPin,HIGH);
    currentEscPwm = currentReadEscPwm;
    if(currentEscPwm > centerSpeedPwm){
        reverseRequired = true;
    }
    currentSteeringAngle = radiansFromServoPwm(currentReadSteerPwm);
    // Speed measurement is handled by calculate speed
    runHoldDoNotTrack();
}

void runStateForward() {
    drive(escPwmPID(desiredSpeed));
    steer();
}

void runStateBraking() {
    if(measuredSpeed > minBrakingSpeed){
        consecutiveZeroSpeed = 0;
        drive(brakePwm);
    }
    else{
        consecutiveZeroSpeed++;
    }
    steer();
}

void runStateStopped() {
    drive(reverseHoldPwm);
    steer();
    consecutiveStop++;
}

void runStateReverse() {
    drive(reversePwm);
    steer();
}

// Steering
void steer(){
    currentSteeringAngle = desiredSteeringAngle;
    int newSteerPwm = steeringPwmFromRadians(currentSteeringAngle);
    steering.writeMicroseconds(newSteerPwm);
}

// Driving
void drive(int pwm){
    currentEscPwm = pwm;
    if(pwm > centerSpeedPwm){
        reverseRequired = true;
    }
    esc.writeMicroseconds(pwm);
}

// Speed calculation
void calculateSpeed(){
    currEncoderTime = millis();    
    if(currEncoderTime-prevEncoderTime != 0){
        measuredSpeed = -(currentEncoderPosition - prevEncoderPosition)*metersPerEncoderTick/(currEncoderTime-prevEncoderTime)*millisPerSec;
        prevEncoderTime = currEncoderTime;
        prevEncoderPosition = currentEncoderPosition;
    }
}

void encoderInterrupt(){
    if(digitalRead(encoderPinB)){
        currentEncoderPosition--;
    }
    else{
        currentEncoderPosition++;
    }
}