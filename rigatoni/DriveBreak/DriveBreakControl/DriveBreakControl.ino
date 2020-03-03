#include "DriveBrake.h"
#include "RJNet.h"
const char RJNet::startMarker = '$';
const char RJNet::endMarker = ';';


volatile long encoder0Pos=0;
float speed = 0;	// 0 - 255

void setup(){
  pinMode(RXLED, OUTPUT);

  pinMode(INT_ETH, INPUT);

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);  

  pinMode(ETH_RST, OUTPUT);


  
  pinMode(REVERSE_LED, OUTPUT);
  pinMode(USER_DEFINED_LED, OUTPUT);

  pinMode(MOTOR_CONTROL, OUTPUT);
  
  pinMode(BRAKE_EN, OUTPUT);
  pinMode(BRAKE_PWM, OUTPUT);

  pinMode(CURR_DATA, INPUT);

  Serial.begin(115200);
}


void loop() {
  // put your main code here, to run repeatedly:
	motorTest();
	Serial.println(speed);
	getSpeedMessage();
	
}

void motorTest() {
	analogWrite(MOTOR_CONTROLLER_INPUT, speed);
	
}

void getSpeedMessage() {
    while(Serial.available()){
      if (Serial.read() == '#'){
        speed = Serial.parseFloat();
      }
    }
}

double accelerate(double a){
  
}

double brake(double targetSpeed){
  
}

double getSpeed(){  
  digitalWrite(ENCODER_A, HIGH);
  digitalWrite(ENCODER_B, HIGH);
  //encoder counts 64 every revolution
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), doEncoder, RISING); // interrupts encoderpin in 19
  int newposition = encoder0Pos;
  int newtime = (0.001*millis());
  int counts = (newposition-oldposition)/(newtime-oldtime);
  

}

void doEncoder(){
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
}


void setMotorPWM(double voltage){
 byte PWM = ((0.556 - sqrt(0.556*0.556 - 0.00512*(62.1-voltage)))/0.00256); //see "rigatoni PWM to motor power" in drive for equation
 analogWrite(MOTOR_CONTROLLER_INPUT, PWM);
}

int getCurrent(){
  int counts = analogRead(CURR_DATA);
  return (200*(5/1024)*counts) - 500; // 200A/V * 5V/count -> gives A
}

void executeStateMachine(){ 
	switch(currentState) { 
		case STATE_DISABLED:{
			
		}
		case STATE_TIMEOUT:{
			
		}
		case STATE_FORWARD:{
			
		}
		case STATE_FORWARD_BRAKE:{
			
		}
		case STATE_REVERSE:{
			
		}
		case STATE_REVERSE_BRAKE:{
			
		}
		case STATE_IDLE:{
			// create state transition conditions	
		}
	}
}

void RJNet() {}

/*
 *  Receives data from client you are connected to in a safe manner.
 *  @Note: not named read() to limit confusion with Arduino Stream library interface
 *
 *  client: Serial, ethernet client or server, other communication method
 */
String RJNet::readData(Stream &client) {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char rc;
    const uint8_t numChars = 128; //buffer size
    char receivedChars[numChars]; //input buffer
    boolean newData = false;

    while (client.available() > 0 && newData == false) {
        rc = client.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }

    if (newData) {
      return receivedChars;
    } else {
      return "";
    }
}

/*
 * Sends info to client (Serial, Ethernet, etc output)
 * @Note: not named print() to limit confusion with Arduino Stream library interface
 * client: Serial, ethernet client or server, other communication method
 * msg: Message to send
 */
void RJNet::sendData(Stream &client, String msg) {
    client.print(addMarkersToMessage(msg));
}

/*
 * Formats String message to have start and end markers.
 * msg: Message to send
 */
String RJNet::addMarkersToMessage(String msg) {
    //add markers
    if (msg.charAt(0) != startMarker) {
        msg = startMarker + msg;
    }
    if (msg.charAt(msg.length() - 1) != endMarker) {
        msg = msg + endMarker;
    }

    return msg;
}
