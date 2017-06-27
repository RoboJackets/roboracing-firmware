const int red_led = 5;
const int yellow_led = 4;
const int green_led = 3;

const int relay = 9;

#define sayHi "Hi!"
#define recieveHello "Hello."

int interval;
int timeout = 50;

char packet;
String incoming;

boolean con = false;

void setup() {
  Serial.begin(9600);

  pinMode(red_led, OUTPUT);
  pinMode(yellow_led, OUTPUT);
  pinMode(green_led, OUTPUT);

  digitalWrite(relay, LOW);

  digitalWrite(red_led, LOW);
  digitalWrite(yellow_led, LOW);
  digitalWrite(green_led, LOW);

  interval = millis();

  while (!Serial.available()) {}
  if (Serial.available()) {
    incoming = Serial.readString();
  }
  if (incoming.equals(recieveHello)) {
    con = true;
    Serial.print("Hi!");
  }
}

void loop() {
  if (Serial.available()) {
    packet = Serial.read();
  }
  if (packet == 'G') {
    digitalWrite(relay, HIGH);
    Serial.print('G');
  }
  else if (packet == 'E') {
    digitalWrite(relay, LOW);
    Serial.print('E');
  }
}
