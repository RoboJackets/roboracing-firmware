
const int pin_esc_speed = 1;
const int pin_esc_direction = 2;

int SPEED = 0;

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
    if(SPEED >= 0) {
        direction = HIGH;
    } else {
        direction = LOW;
        SPEED = -SPEED;
    }

    float pwm = SPEED * 0.1;

    //note: circuitry flips the signal
    int analogInt = (int) (255 * (1.0 - pwm));

    digitalWrite(pin_esc_direction, direction);
    analogWrite(pin_esc_speed, analogInt);
}

void setup() {
    pinMode(pin_esc_direction, OUTPUT);
    pinMode(pin_esc_speed, OUTPUT);
    drive();
}

void loop() {
    if(getMessage()) {
        drive();
    }
}
