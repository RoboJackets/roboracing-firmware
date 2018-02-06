#define NUM_SENSORS 6
#define BAUD_RATE 9600
//multiplexer pins
#define S0 8
#define S1 7
#define S2 6
#define S3 5

#define SPEED_OF_SOUND 2941.2 //microseconds per meter!
//#TODO: write a version of ping that does not use delay. Increment through two at a time, need to write your own "pulseIn"

int trigPin = 9; //trigger pin (to send out ultrasonic)
int echoPin = 10; //echo pin (to listen to return)
double distances[NUM_SENSORS];

/*
 * Firmware for sensor array with HC-SR04 ultrasonic sensors
 * Simply define the proper pins and # of sensors
 * Uses 2 Muxes: 1 of triggers and 1 for echos
 * 
 * @Note: https://www.bananarobotics.com/shop/HC-SR04-Ultrasonic-Distance-Sensor
 * Use the above as a reference for some code and reasoning
 */


void switchMux(int channel) {
  //Channels 0 - 15 on mux
  digitalWrite(S0, bitRead(channel, 0));
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
  digitalWrite(S3, bitRead(channel, 3));

  delayMicroseconds(1); //Max switching time is 1 microsecond

  }


double ping(int trigPin, int echoPin) {
  //outputs distance in METERS

  long duration;
  double distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2); //May be unnecessary #TODO
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);

  distance = ((double)duration / 2.0) / SPEED_OF_SOUND; //Distance = (Time x SpeedOfSound) / 2. The "2" is in the formula because the sound has to travel back and forth
  return distance;

}

void testSensor(int sensorNum) {
  //useful to debug a single ultrasonic sensor
  switchMux(sensorNum);
  Serial.println(ping(trigPin, echoPin));
  delay(200);
}

void setup() {
  Serial.begin(BAUD_RATE);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  switchMux(0);

}

void loop() {

  for (int i = 0; i < NUM_SENSORS; i++) {
    switchMux(i);
    distances[i] = ping(trigPin, echoPin);
    delay(10); //#TODO: This value needs to be played with based on location
    //delay(5); //neccessary because echo from other sensor can interfere if lower
  }

  //output distances to serial
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(distances[i]);
    if (i != NUM_SENSORS - 1) {
      Serial.print(",");
    }
  }
  Serial.println();

  delay(50);//delay(100); //#TODO 100 sounds safe but lower could be tested
  
}
