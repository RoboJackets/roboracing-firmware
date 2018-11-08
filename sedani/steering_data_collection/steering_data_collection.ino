#include <Servo.h>
#include "pitch.h"

const int wirelessD0Pin = 16;
bool wirelessD0State = false;
bool wirelessD0Buffer = false;

const int wirelessD1Pin = 14;
bool wirelessD1State = false;
bool wirelessD1Buffer = false;

const int wirelessD2Pin = 15;
bool wirelessD2State = false;
bool wirelessD2Buffer = false;

const int speakerOutputPin = 9;

int songInformation0[2] = {NOTE_C4, NOTE_C4};
int songInformation1[2] = {NOTE_C5, NOTE_C4};
int songInformation2[2] = {NOTE_C4, NOTE_C5};

Servo steering;
const int steerOutputPin = 5;
int currentSteerPWM = 1500;

String message = "";



void setup() {
  pinMode(wirelessD0Pin,INPUT);
  pinMode(wirelessD1Pin,INPUT);
  pinMode(wirelessD2Pin,INPUT);
  pinMode(speakerOutputPin, OUTPUT);
  pinMode(steerOutputPin, OUTPUT);
  steering.attach(steerOutputPin);
  steerDrive(0);
}

void loop() {  
  wirelessD0State = digitalRead(wirelessD0Pin);
  wirelessD1State = digitalRead(wirelessD1Pin);
  wirelessD2State = digitalRead(wirelessD2Pin);
 
  if(wirelessD0State != wirelessD0Buffer && wirelessD0State){
    message.concat("Current Output PWM: ");
    message.concat(currentSteerPWM);
    Serial.println(message);
    message = "";
    playSong(0);
  }
 
  if(wirelessD1State != wirelessD1Buffer && wirelessD1State){
    currentSteerPWM -= 100;
    playSong(1);
  }
  
  if(wirelessD2State != wirelessD2Buffer && wirelessD2State){
    currentSteerPWM += 100;
    playSong(2);
  }

  steerDrive(currentSteerPWM);
  wirelessD0Buffer = wirelessD0State;
  wirelessD1Buffer = wirelessD1State;
  wirelessD2Buffer = wirelessD2State;
  delay(100);
 
}

void steerDrive(int val)
{
  steering.write(val);
}


void playSong(int number){
    for (int thisNote = 0; thisNote < 2; thisNote++) {
      int noteDuration = 125;
      if(number == 0){
        tone(speakerOutputPin, songInformation0[thisNote], noteDuration);
      } else if(number == 1){
        tone(speakerOutputPin, songInformation1[thisNote], noteDuration);
      } else{
        tone(speakerOutputPin, songInformation2[thisNote], noteDuration);
      }
      int pauseBetweenNotes = 162;
      delay(pauseBetweenNotes);
      noTone(speakerOutputPin);
    }
}
