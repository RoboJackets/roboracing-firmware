#include "pitch.h"

const int estopStatePin = 10;
bool estopState = false;

const int wirelessD0Pin = 16;
bool wirelessD0State = false;
bool wirelessD0Buffer = false;

const int wirelessD1Pin = 14;
bool wirelessD1State = false;
bool wirelessD1Buffer = false;

const int wirelessD2Pin = 15;
bool wirelessD2State = false;
bool wirelessD2Buffer = false;

const int speakerPin = 9;

int songInformation1[2][4] = 
  {
    {
      NOTE_F4, NOTE_FS4, NOTE_G4, NOTE_GS4
    },
    {
      8, 8, 8, 8
    }
 };
int songInformation2[2][6] = 
  {
    {
      NOTE_F4, NOTE_C4, NOTE_F4, NOTE_C4, NOTE_F4, NOTE_C4
    },
    {
      4, 4, 4, 4, 4, 4
    }
 };
 
int songInformation3[2][9] = 
  {
    {
      NOTE_A4, NOTE_AS4, 0, NOTE_AS4, 0, NOTE_FS3, NOTE_G3, 0, NOTE_G3
    },
    {
      8, 8, 4, 8, 8, 8, 8, 4, 8
    }
};


void setup() {
  // put your setup code here, to run once:
  pinMode(estopStatePin,INPUT);
  pinMode(wirelessD0Pin,INPUT);
  pinMode(wirelessD1Pin,INPUT);
  pinMode(wirelessD2Pin,INPUT);
  pinMode(speakerPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

 String message = "";
 
 estopState = digitalRead(estopStatePin);
 wirelessD0State = digitalRead(wirelessD0Pin);
 wirelessD1State = digitalRead(wirelessD1Pin);
 wirelessD2State = digitalRead(wirelessD2Pin);
 
 
 message.concat(estopState);
 message.concat(",");
 message.concat(wirelessD0State);
 message.concat(",");
 message.concat(wirelessD1State);
 message.concat(",");
 message.concat(wirelessD2State);
 Serial.println(message);
 
 if(wirelessD0State != wirelessD0Buffer && wirelessD0State){
  playSong(0);
 }
 if(wirelessD1State != wirelessD1Buffer && wirelessD1State){
  playSong(1);
 }
 if(wirelessD2State != wirelessD2Buffer && wirelessD2State){
  playSong(2);
 }

 wirelessD0Buffer = wirelessD0State;
 wirelessD1Buffer = wirelessD1State;
 wirelessD2Buffer = wirelessD2State;
 delay(100);
 
}

void playSong(int number){
    if(number == 0){
      for (int thisNote = 0; thisNote < 4; thisNote++) {
  
      // to calculate the note duration, take one second divided by the note type.
      //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
      int noteDuration = 1000 / songInformation1[1][thisNote];
      tone(speakerPin, songInformation1[0][thisNote], noteDuration);
  
      // to distinguish the notes, set a minimum time between them.
      // the note's duration + 30% seems to work well:
      int pauseBetweenNotes = noteDuration * 1.30;
      delay(pauseBetweenNotes);
      // stop the tone playing:
      noTone(speakerPin);
    }
  }else if(number == 1){
    for (int thisNote = 0; thisNote < 6; thisNote++) {
  
      // to calculate the note duration, take one second divided by the note type.
      //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
      int noteDuration = 1000 / songInformation2[1][thisNote];
      tone(speakerPin, songInformation2[0][thisNote], noteDuration);
  
      // to distinguish the notes, set a minimum time between them.
      // the note's duration + 30% seems to work well:
      int pauseBetweenNotes = noteDuration * 1.30;
      delay(pauseBetweenNotes);
      // stop the tone playing:
      noTone(speakerPin);
    }
  }else{
    for (int thisNote = 0; thisNote < 9; thisNote++) {
  
      // to calculate the note duration, take one second divided by the note type.
      //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
      int noteDuration = 1000 / songInformation3[1][thisNote];
      tone(speakerPin, songInformation3[0][thisNote], noteDuration);
  
      // to distinguish the notes, set a minimum time between them.
      // the note's duration + 30% seems to work well:
      int pauseBetweenNotes = noteDuration * 1.30;
      delay(pauseBetweenNotes);
      // stop the tone playing:
      noTone(speakerPin);
    }
  }
  


}

