/*
const int leftDir = 10;
const int leftSpeed = 9;
const int rightDir = 7;
const int rightSpeed = 6;
*/

const int pin_esc_speed = 9;
const int pin_esc_direction = 10;

float SPEED = 0;

bool getMessage() {
    bool gotMessage = false;
    while(Serial.available()) {
        if(Serial.read() == '$') {
            SPEED = Serial.parseFloat();
            gotMessage = true;
        }
    }
    return gotMessage;
}

void drive() {
    int direction;
    float pwm = SPEED;
    if(SPEED >= 0) {
        direction = HIGH;
        pwm = 1.0 - pwm;
    } else {
        direction = LOW;
        pwm *= -1;
    }

    int analogInt = (int) (255 * pwm);

    digitalWrite(pin_esc_direction, direction);
    analogWrite(pin_esc_speed, analogInt);
}

void setup() {
    Serial.begin(9600);
    pinMode(pin_esc_direction, OUTPUT);
    pinMode(pin_esc_speed, OUTPUT);
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
    drive();
}

void loop() {
    if(getMessage()) {
        drive();
        digitalWrite(13, HIGH);
    } else {
        digitalWrite(13, LOW);
    }
    delay(50);
}
