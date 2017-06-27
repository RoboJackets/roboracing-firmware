const int red_led = 5;
const int yellow_led = 4;
const int green_led = 3;

const int red_button = 7;
const int yellow_button = 8;
const int green_button = 9;

const int active_button = 8;

#define sayHello "Hello."
#define recieveHi "Hi!"

int interval;
int timeout = 50;

char packet;

void setup() {
  Serial.begin(9600);

  pinMode(red_led, OUTPUT);
  pinMode(yellow_led, OUTPUT);
  pinMode(green_led, OUTPUT);

  pinMode(red_button, INPUT);
  pinMode(yellow_button, INPUT);
  pinMode(green_button, INPUT);

  digitalWrite(red_led, LOW);
  digitalWrite(yellow_led, LOW);
  digitalWrite(green_led, LOW);

  digitalWrite(active_button, LOW);

  Serial.print("Hello.");

  int start = millis();
  int initTimeout = 200;
  String incoming = "";
  while (!Serial.available() && millis() - start < initTimeout) {}
  if (Serial.available()) {
    incoming = Serial.readString();
  }
  if (incoming.equals(recieveHi)) {
    Serial.println("We're friends!.");
    digitalWrite(yellow_led, HIGH);
  }
  else {
    Serial.println("We can't find eachother... Plase reset.");
    digitalWrite(red_led, HIGH);
    digitalWrite(yellow_led, HIGH);
    digitalWrite(green_led, HIGH);
  }

  interval = millis();
}

void loop() {
  if (interval >= timeout) {
    Serial.print('E');
  }
  
  if (Serial.available()) {
    packet = Serial.read();
    interval = millis();
  }
  if (packet == 'G') {
    digitalWrite(green_led, HIGH);
    Serial.print('G');
  }
  else if (packet == 'E') {
    digitalWrite(red_led, HIGH);
  }

  if (digitalRead(red_button)) {
    Serial.print('E');
  }
  else if (digitalRead(green_button)) {
    Serial.print('G');
  }
  else if (digitalRead(yellow_button)) {
    // what do we want this to do?
  }
}
