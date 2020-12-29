#include <avr/wdt.h>
#include "DriveBrake.h"
#include "RJNet.h"
#include <Ethernet.h>

#define TIMER_MIN_RESOLUTION_US 8  //Timer minimum resolution of 8 us

#define NUM_MAGNETS 24
#define WHEEL_DIA 0.27305 //meters
const static unsigned long US_PER_SEC = 1000000;
#define PI_M 3.141592653589793
#define SQRT_2 1.4142135623730951

#define ETH_NUM_SENDS 2
#define ETH_RETRANSMISSION_DELAY_MS 50  //Time before TCP tries to resend the packet
#define ETH_TCP_INITIATION_DELAY 50 //How long we wait before .connected() or .connect() fails.

#define REPLY_TIMEOUT_MS 4000 //We have to receive a reply from Estop and Brake within this many MS of our message or we are timed out. NOT TCP timeout. Have to 
#define MIN_MESSAGE_SPACING 100  //Send messages to Brake and request state from e-stop at 10 Hz

/*******FUNCTION HEADERS*****/
byte MotorPwmFromVoltage(float);

/****************ETHERNET****************/
// See https://docs.google.com/document/d/10klaJG9QIRAsYD0eMPjk0ImYSaIPZpM_lFxHCxdVRNs/edit#
////

// Enter a MAC address and IP address for your board below
const static byte driveMAC[] = {0x11, 0xAD, 0xBA, 0xEF, 0xFE, 0xEE};
const static int PORT = 7;  //port RJNet uses

const static IPAddress driveIP(192, 168, 0, 4); //set the IP to find us at
EthernetServer server(PORT);

// Our two clients: 
const static IPAddress brakeIP(192, 168, 0, 5); //set the IP of the brake
EthernetClient brakeBoard;  // client 

const static IPAddress estopIP(192, 168, 0, 3); //set the IP of the estop
EthernetClient estopBoard;  // client

//Manual board, which is not a client:
const static IPAddress manualIP(192, 168, 0, 144); //set the IP of the estop

/****************Messages from clients****************/
//Universal acknowledge message
const static String ackMsg = "R";

//Possible messages from e-stop
const static String estopStopMsg = "D";
const static String estopLimitedMsg = "L";
const static String estopGoMsg = "G";

//Speed request message
const static String speedRequestMsg = "S?";

//Manual RC speed command header
const static String manualStringHeader = "v=";

//Timestamps of replies from boards in ms
unsigned long lastEstopReply = 0;
unsigned long lastBrakeReply = 0;
unsigned long lastManualCommand = 0;
//Timestamps of our last messages to boards in ms
unsigned long estopRequestSent = 0;
unsigned long brakeCommandSent = 0;

//TCP Connection status to brake and estop
bool estopConnected = false;
bool brakeConnected = false;

/*
State Machine

We are disabled if:
 - Estop says we are
 - timed out waiting for message from estop, brake, or manual
 
Forward state if commanded velocity >=0. Backward if velocity < 0.
*/

// State machine possible states
enum ChassisState {
    STATE_DISABLED = 0,
    STATE_TIMEOUT = 1,
    STATE_FORWARD = 2,
    STATE_REVERSE = 3
}
currentState = STATE_DISABLED;
bool motorEnabled = false;

////
// ENCODER
// See motor_and_brake_independent_controllers.ipynb
// Encoder measures position and we want speed, so we created an estimator
////

unsigned long lastPIDTimeUS = 0; // the time used to run the loop
volatile unsigned long currEncoderCount = 0;
unsigned long prevEncoderCount = 0;
unsigned long lastSpeedReadTimeUs = 0;

float currentSpeed = 0;
float currentPosition = 0;
float motorCurrent = 0;

////
// MOTOR CONTROL
// See motor_and_brake_independent_controllers.ipynb
// We construct two independent feedforward controllers - one for the brakes and one for the motor - and run them simultaneously
// Motor controller is in volts, brake controller is in Newtons
////


// Control Limits and parameters
#define USE_FEEDFORWARD_CONTROL true        //If false, will not use feedforward control
const float maxVelocity = 10.0;             //maximum velocity in m/s
const float controller_decel_limit = 6;     //maximum deceleration in m/s^2
const float controller_accel_limit = 4.5;    //max accel in m/s^2
const float velocity_filter_bandwidth = 5;

const float batteryVoltage = 48;
const byte maxSpeedPwm = 255;
const byte zeroSpeedPwm = 0; // 100% PWM, 0% Voltage

//Motor feedforward and PI parameters
const float k_m_inv_r_to_u = 2.8451506121016807;
const float k_m_inv_r_dot_to_u = 1.93359375;
const float k_m_inv_r_to_x = 1.0;

const float k_1m = 4.908560325397578;   //P gain
const float k_2m = 7.773046874998515;   //I gain

//Brake feedforward and PI parameters
const float k_b_inv_r_to_u = -10.0;
const float k_b_inv_r_dot_to_u = -150.0;
const float k_b_inv_r_to_x = 1.0;

const float k_1b = -591.5;   //P gain
const float k_2b = -603;   //I gain

const float maxBrakingForce = 600.0;    //In Newtons

//Global variables
float error_integral = 0;
float softwareDesiredVelocity = 0;      //What Software is commanding us
float trapezoidal_target_velocity = 0;  //trapezoidal interpolation of Software's commands
float filtered_target_vel = 0;          //Lowpass filtered version of trapezoidal interpolated velocities
float filtered_target_accel = 0;

float desired_braking_force = 0;

struct FloatPair{   //Replacement for std::pair only good for floats
    float first;
    float second;
};


void setup(){
    // Ethernet Pins
	pinMode(ETH_INT_PIN, INPUT);
    pinMode(ETH_RST_PIN, OUTPUT);
    pinMode(ETH_CS_PIN, OUTPUT);

	// Encoder Pins
	pinMode(ENCODER_A_PIN, INPUT);
  
    // LED Pins
	pinMode(REVERSE_LED_PIN, OUTPUT);

    // Motor Pins
	pinMode(MOTOR_CNTRL_PIN, OUTPUT);
    pinMode(FORWARD_OUT_PIN, OUTPUT);
    pinMode(REVERSE_OUT_PIN, OUTPUT);
    writeReversingContactorForward(true); //Start out with reversing contactor going forwards.

    // Current Sensing Pins
	pinMode(CURR_DATA_PIN, INPUT);
    
    Serial.begin(115200);

	//********** Ethernet Initialization *************//
    resetEthernet();
    
	Ethernet.init(ETH_CS_PIN); 	// CS pin from eth header
	Ethernet.begin(driveMAC, driveIP); 	// initialize the ethernet device
    
	while (Ethernet.hardwareStatus() == EthernetNoHardware) {
		Serial.println("Ethernet shield was not found.");
		delay(50);
	}
	while(Ethernet.linkStatus() == LinkOFF) {
		Serial.println("Ethernet cable is not connected.");	// do something with this
		delay(50);	 	// TURN down delay to check/startup faster
	}
    
    Ethernet.setRetransmissionCount(ETH_NUM_SENDS); //Set number resends before failure
    Ethernet.setRetransmissionTimeout(ETH_RETRANSMISSION_DELAY_MS);  //Set timeout delay before failure
    
    server.begin();	// launches OUR server
    Serial.println("Server started");
    
    //Make sure we are connected to clients before proceeding.
    estopBoard.setConnectionTimeout(ETH_TCP_INITIATION_DELAY);
    brakeBoard.setConnectionTimeout(ETH_TCP_INITIATION_DELAY);
    
    while(!estopConnected || !brakeConnected){
        if(!estopConnected){
            Serial.println("Connecting to Estop");
            estopConnected = estopBoard.connect(estopIP, PORT) > 0;
        }
        if(!brakeConnected){
            Serial.println("Connecting to Brake");
            brakeConnected = brakeBoard.connect(brakeIP, PORT) > 0;
        }
    }

    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), HallEncoderInterrupt, FALLING);
    
    // WATCHDOG TIMER
    wdt_reset();
    wdt_enable(WDTO_500MS);
}

const static int printDelayMs = 1000;
unsigned long lastPrintTime = 0;

void loop() {
    wdt_reset();
    
	// Read new messages from WizNet and make necessary replies
    readAllNewMessages();
	
    //Calculate current speed
    motorCurrent = GetMotorCurrent();
    currentSpeed = CalcCurrentSpeed();
    
    if (millis() > lastPrintTime + printDelayMs){
        Serial.print("Current speed: ");
        Serial.print(currentSpeed);
        Serial.print(" Motor Current: ");
        Serial.println(motorCurrent);
        
        lastPrintTime = millis();
    }

    executeStateMachine();
    
    //Now make requests to estop and brake
    sendToBrakeEstop();
}

////
// ETHERNET FUNCTIONS
////

bool parseEstopMessage(const String msg){
    /*
    Parse a message from the estop board and determine if the drive motor is enabled or disabled.
    If state is GO, motor enabled; in all other cases, motor is disabled.
    */
    return estopGoMsg.equals(msg);
}

float parseSpeedMessage(const String speedMessage){
    //takes in message from manual and converts it to a float desired speed.
    return speedMessage.substring(2).toFloat();
}

void handleSingleClientMessage(EthernetClient otherBoard){
    /*
    Handles a single client with new data. We make no initial distinction about who the otherBoard is - it could be a server connection OR 
    a reply from Estop or Brake where WE have the EthernetClient object
    
    If it is a request for the current speed, reply with the speed.
    Else, see if it is Estop telling us the state
    or Manual telling us the velocity
    or Brake acknowledging a command
    */
    String data = RJNet::readData(otherBoard);  //RJNet handles getting complete message
    
    IPAddress otherIP = otherBoard.remoteIP();
        
    if(data.length() != 0){  //Got valid data - string will be empty if message not valid
        otherBoard.setConnectionTimeout(ETH_TCP_INITIATION_DELAY);   //Set connection delay so we don't hang
        
        if(speedRequestMsg.equals(data)){  //Somebody's asking for our speed
            //Send back our current speed
            RJNet::sendData(otherBoard, "v=" + String(currentSpeed) + " I=" + String(motorCurrent));
            Serial.print("Speed request from");
            Serial.println(otherIP);
        }
        else if(estopIP == otherIP){
            //Message from Estop board
            motorEnabled = parseEstopMessage(data);
            lastEstopReply = millis();
            Serial.print("Estop: motor enabled? ");
            Serial.println(motorEnabled);
        }
        else if(brakeIP == otherIP){
            //Message from brake board, should be acknoweldge
            if(ackMsg.equals(data)){
                lastBrakeReply = millis();
            }
            else{
                Serial.print("Invalid message from brake: ");
                Serial.println(data);
            }
        }
        else if(manualIP == otherIP){
            //Message from manual board, should be speed command
            if(data.length() > 2){
                if(manualStringHeader.equals(data.substring(0,2))){
                    softwareDesiredVelocity = parseSpeedMessage(data);
                    lastManualCommand = millis();
                    
                    //Acknowledge receipt of message
                    RJNet::sendData(otherBoard, ackMsg);
                    
                    Serial.print("New target speed: ");
                    Serial.println(softwareDesiredVelocity);
                }
                else{
                    Serial.print("Wrong prefix from manual: ");
                    Serial.println(data);
                }
            }
            else{
                Serial.print("Too short message from manual: ");
                Serial.println(data);
            }
        }
    }
    else{
        Serial.print("Empty/invalid message received from ");
        Serial.println(otherIP);
    }
}

void readAllNewMessages(){
    /*
    Checks the server, brake, and Estop for new messages and deals with them
    */
    EthernetClient client = server.available();  //if there is a new message form client create client object, otherwise new client object null
    while(client){
        handleSingleClientMessage(client);
        client = server.available();  //Go to next message
    }
    
    while(brakeBoard.available()){
        handleSingleClientMessage(brakeBoard);
    }
    
    while(estopBoard.available()){
        handleSingleClientMessage(estopBoard);
    }
}

void sendToBrakeEstop(void){
    /*
    Checks our connection to the brake and estop and attempts to reconnect if we are disconnected.
    If we are connected and sufficient time has elapsed both from our last request and their last reply, send new request to estop/brake
    */
    brakeConnected = brakeBoard.connected();
    if(!brakeConnected){
        //Lost TCP connection with the brake board. Takes 10 seconds for TCP connection to fail after disconnect.
        //Takes a very long time to time out
        brakeBoard.connect(brakeIP, PORT);
        Serial.println("Lost connection with brakes");
    }
    else{
        if(millis() > lastBrakeReply + MIN_MESSAGE_SPACING && millis() > brakeCommandSent + MIN_MESSAGE_SPACING){
            RJNet::sendData(brakeBoard, "B=" + String(desired_braking_force));
            brakeCommandSent = millis();
        }
    }
    
    estopConnected = estopBoard.connected();
    if(!estopConnected){
        //Lost TCP connection with the estop board
        estopBoard.connect(estopIP, PORT);
        Serial.println("Lost connection with estop");
    }
    else{
        if(millis() > lastEstopReply + MIN_MESSAGE_SPACING && millis() > estopRequestSent + MIN_MESSAGE_SPACING){
            RJNet::sendData(estopBoard, "S?");
            estopRequestSent = millis();
        }
    }
}

void resetEthernet(void){
    //Resets the Ethernet shield
    digitalWrite(ETH_RST_PIN, LOW);
    delay(1);
    digitalWrite(ETH_RST_PIN, HIGH);
    delay(501);
}

////
// STATE MACHINE FUNCTIONS
////

void executeStateMachine(){ 
    //Check what state we are in at the moment.
    //We are disabled if the motor is disabled, or any of our clients has timed out 
    unsigned long currTime = millis();
    if(!motorEnabled){
        currentState = STATE_DISABLED;
    }
    else if((!brakeConnected || currTime > lastBrakeReply + REPLY_TIMEOUT_MS) || (!estopConnected || currTime > lastEstopReply + REPLY_TIMEOUT_MS) || currTime > lastManualCommand + REPLY_TIMEOUT_MS){
        currentState = STATE_TIMEOUT;
    }
    else{
        if(softwareDesiredVelocity < 0){
            currentState = STATE_REVERSE;
        }
        else{
            currentState = STATE_FORWARD;
        }
    }
    //Now execute that state
	switch(currentState) { 
		case STATE_DISABLED:{
			runStateDisabled();
			break;
		}
        case STATE_TIMEOUT:{
			runStateTimeout();
			break;
		}
		case STATE_FORWARD:{
			runStateForward();
			break;
		}
		case STATE_REVERSE:{
			runStateReverse();
			break;
		}
	}
}

void runStateDisabled(){
    /*
    Motor off, brakes engaged. Don't care what reversing contactor is doing.
    */
    Serial.println("Motor disabled");
    writeMotorOff();
    desired_braking_force = maxBrakingForce;
}
void runStateTimeout(){
    /*
    Motor off, brakes disengaged. Don't care what reversing contactor is doing.
    */
    unsigned long currTime = millis();
    
    if (!brakeConnected){
        Serial.println("Lost brake TCP connection");
    }
    else if(currTime > lastBrakeReply + REPLY_TIMEOUT_MS){
        Serial.println("Brake timed out");
    }
    
    if (!estopConnected){
        Serial.println("Lost estop TCP connection");
    }
    else if(currTime > lastEstopReply + REPLY_TIMEOUT_MS){
        Serial.println("Estop timed out");
    }
    
    if (currTime > lastManualCommand + REPLY_TIMEOUT_MS){
        Serial.println("Manual timed out");
    }
    
    writeMotorOff();
    desired_braking_force = 0;
}

void runStateForward(){
    /*
    Going forward in normal operation.
    */
    writeReversingContactorForward(true);
    
    //TODO: controls math here
    MotorPwmFromVoltage(12.0);
    desired_braking_force = 0;
}

void runStateReverse(){
    /*
    Going reversed in normal operation.
    */
    writeReversingContactorForward(false);
    
    //TODO: controls math here
    MotorPwmFromVoltage(12.0);
    desired_braking_force = 0;
}


void writeReversingContactorForward(bool forward){
    /*
    Controls the reversing contactor. True means go forward, False means go backwards
    */
    digitalWrite(FORWARD_OUT_PIN, forward);
    digitalWrite(REVERSE_OUT_PIN, !forward);
}


////
// ENCODER FUNCTIONS
////

void HallEncoderInterrupt(){
    currEncoderCount++;
}

float CalcCurrentSpeed() {
    unsigned long currTime = micros();
    unsigned long timeDelta = max(currTime - lastSpeedReadTimeUs, TIMER_MIN_RESOLUTION_US);
    
    float val = (((float) currEncoderCount - prevEncoderCount)/timeDelta) * US_PER_SEC * PI_M * WHEEL_DIA/NUM_MAGNETS;
    prevEncoderCount = currEncoderCount;
    lastSpeedReadTimeUs = currTime;
    return val;
}

////
// SPEED ESTIMATOR FUNCTIONS
// See ipython notebook
////
void estimate_vel(float delta_t, float encoder_pos, float motor_current, float brake_force){
    //All arguments are in SI units
    //This updates the global velocity estimate, currentSpeed
    //Use the Forward Euler method

    float new_est_pos = est_pos + delta_t * (est_vel - L_pos*(est_pos - encoder_pos))
    float new_est_vel = est_vel + delta_t * (-d/m*est_vel + Gr*Kt/(m*rw)*motor_current - 1/m*brake_force - L_vel*(est_pos - encoder_pos))
    currentPosition = new_est_pos;
    currentSpeed = new_est_vel;
}


////
// MOTOR CONTROLLER FUNCTIONS
////

void writeMotorOff(void){
    //Disables the motor by writing correct voltage
    analogWrite(MOTOR_CNTRL_PIN, zeroSpeedPwm);
}

byte MotorPwmFromVoltage(float voltage){
    if(voltage > batteryVoltage){
        return maxSpeedPwm;
    }
    else if(voltage < 0){
        return 0;
    }
    return (byte) maxSpeedPwm*voltage/batteryVoltage;
}

//Generate the target velocity and acceleration

float gen_trapezoidal_vel(float timestep, float last_trapezoidal_vel, float software_cmd_vel){
    //Limit our maximum acceleration.
    float trapezoidal_target_vel = min(max(software_cmd_vel, last_trapezoidal_vel - timestep * controller_decel_limit), last_trapezoidal_vel + timestep * controller_accel_limit);
    
    //Cap our speed
    trapezoidal_target_vel = max(trapezoidal_target_vel, 0);
    return trapezoidal_target_vel;
}

const float butterworth_coeff_accel = -SQRT_2*velocity_filter_bandwidth;
const float butterworth_coeff_vel = velocity_filter_bandwidth*velocity_filter_bandwidth;

void filter_target_vel_accel(float timestep, float trap_target_vel){
    //Applies a butterworth lowpass filter to the trapezoidal velocities, using the forward Euler approximation
    //Updates filtered_target_vel, filtered_target_accel in place
    
    float new_filtered_target_vel = filtered_target_vel + timestep*filtered_target_accel;
    float new_filtered_target_accel = filtered_target_accel + timestep*(butterworth_coeff_accel*filtered_target_accel + butterworth_coeff_vel*(trap_target_vel - filtered_target_vel));
   
    filtered_target_vel = new_filtered_target_vel;
    filtered_target_accel = new_filtered_target_accel;
}

//Feedforward reference commands for motor + brake

FloatPair gen_motor_feedforward_reference(float target_vel, float target_accel){
    //Returns (voltage reference in volts, velocity reference in m/s) for motor controller
    if(!USE_FEEDFORWARD_CONTROL){
        return make_float_pair(0.0, 0.0);
    }
    //Generate the feedforward reference commands
    float voltage_ref = k_m_inv_r_to_u * target_vel + k_m_inv_r_dot_to_u * target_accel;
    float vel_ref_m = k_m_inv_r_to_x * target_vel;
    
    return make_float_pair(voltage_ref, vel_ref_m);
}

FloatPair gen_brake_feedforward_reference(float target_vel, float target_accel){
    //Returns (force reference in N, velocity reference in m/s) for brake controller
    if(!USE_FEEDFORWARD_CONTROL){
        return make_float_pair(0.0, 0.0);
    }
    //Generate the feedforward reference commands
    float brake_force_ref = k_b_inv_r_to_u * target_vel + k_b_inv_r_dot_to_u * target_accel;
    float vel_ref_b = k_b_inv_r_to_x * target_vel;
    
    return make_float_pair(brake_force_ref, vel_ref_b);
}

//PI control functions for motor and brake
float gen_motor_PI_control_voltage(float voltage_ref, float vel_ref_m, float current_vel, float err_integral){
    return voltage_ref - k_1m * (current_vel - vel_ref_m) - k_2m * err_integral;
}

float gen_brake_PI_control_voltage(float brake_force_ref, float vel_ref_b, float current_vel, float err_integral){
    return brake_force_ref - k_1b * (current_vel - vel_ref_b) - k_2b * err_integral;
}

FloatPair gen_control_voltage_brake_force(float delta_t, float est_vel, float software_cmd_vel){
    //The main controller function.
    //Returns (motor voltage, braking force). All arguments are in SI units (seconds, m/s)
    
    //Get target velocity + accel
    trapezoidal_target_velocity = gen_trapezoidal_vel(delta_t, trapezoidal_target_velocity, software_cmd_vel);
    
    //Now filter it
    filter_target_vel_accel(delta_t, trapezoidal_target_velocity);
    
    //For the motor
    //Get feedforward reference values
    FloatPair motor_voltage_velocity_ref = gen_motor_feedforward_reference(filtered_target_vel, filtered_target_accel);
    float voltage_ref = motor_voltage_velocity_ref.first;
    float vel_ref_m = motor_voltage_velocity_ref.second;
    
    //Get voltage command
    float voltage_command = gen_motor_PI_control_voltage(voltage_ref, vel_ref_m, est_vel, error_integral);
    
    //For the brakes
    //Get feedforward reference values
    FloatPair brake_force_velocity_ref = gen_brake_feedforward_reference(filtered_target_vel, filtered_target_accel);
    float brake_force_ref = brake_force_velocity_ref.first;
    float vel_ref_b = brake_force_velocity_ref.second;
    
    //Get brake force command
    float brake_force_command = max(gen_brake_PI_control_voltage(brake_force_ref, vel_ref_b, est_vel, error_integral), 0);
    
    //Update error integral
    error_integral += delta_t * (est_vel - filtered_target_vel);
    
    return make_float_pair(voltage_command, brake_force_command);
}

////
// CURRENT SENSE FUNCTIONS
////

const static float currentSensorMidpoint = 500.0;
const static int adcResolution = 1024;
const static float adcMaxVoltage = 5.0;
const static float currentSensorAmpsPerVolt = 200.0;
const static float ampsPerBit = currentSensorAmpsPerVolt*adcMaxVoltage/adcResolution;

float GetMotorCurrent(){
    unsigned int rawCurrentBits = analogRead(CURR_DATA_PIN);
    return (ampsPerBit*rawCurrentBits) - currentSensorMidpoint; // 200A/V * current voltage -> gives A
}

//Pair-making function
FloatPair make_float_pair(float first, float second){
    struct FloatPair retVal = {first, second};
    return retVal;
}