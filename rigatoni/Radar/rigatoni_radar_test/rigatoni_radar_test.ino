/*
   OmniPreSense Sensor reader for arduino.
   The readings from the OmniPreSense radar are read by this sketch
   Some light-duty processing is performed ont he input
   The output is sent out the Arduino back to a host connected by USB
   The listening program can be the arduino Serial Monitor OR any program that can read USB ports, such as
   TeraTerm or Putty (or on Linux, minicom or screen)
  The circuit for an Arduino Uno or equivalent:
   RX is digital pin 10 (connect to TX of the OmniPreSense sensor)
   TX is digital pin 11 (connect to RX of the OmniPreSense sensor)
  This is based on the "Software serial multple serial test" found at the Arduino web site.
  created back in the mists of time
  modified 25 May 2012
  by Tom Igoe
  based on Mikal Hart's example
  This example code is in the public domain.
*/
#include <SoftwareSerial.h>

// this line lets us call the Serial object by a more descriptive name, usb_serial

// this declares that we have an OmniPreSense board hooked up to Arduino pin 10 & 11
SoftwareSerial ops_serial(10, 11); // RX, TX

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(19200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  //Serial.println("Setting up connection to sensor at 19200 baud!");

  // set the data rate for the SoftwareSerial port
  Serial1.begin(19200);
  Serial1.write("\r\nOU\r\n"); // insist on getting the unit before the value
}
float distance = 0.0;
String distanceUnit = "";
float velocity = 0.0;
String velocityUnit = "";
float reading = 0.0;
String inString = "";    // string to hold speed input from serial line
void loop() { // run over and over
  // pass along anything from teh USB port to the OPS board connected via pin 10&11
  if (Serial.available()) {
    char c = Serial.read();
    //debug: let me see what I typed
    //Serial.write(c);
    Serial1.write(c);
  }

  if (Serial1.available() > 0)
  {
    int inChar = Serial1.read();

    // if it's the end of a line....
    if (inChar != '\n' && inChar != '\r')
    {
      // As long as the incoming byte is not a newline,
      // convert the incoming byte to a char and add it to the string
      inString += (char)inChar;
    }
    else {
      // if you get a line with a reading, process it
      if (inString.length() > 1 && inString.charAt(0) != '{') {  // but ignore empty lines and JSON
        char *comma_pos = inString.indexOf(',');
        String units = "";
        if (comma_pos < 1) { // no comma found, so lets assume it's not a OPS243C and that it's a velocity
          reading = inString.toFloat();
          units = "";
        } else {
          // split the line into unit and reading.  Unit string is always surrounded by ""
          units = inString.substring(0, comma_pos);
          // now lets remove the quotes around the units
          if (units.charAt(0) == '"') {
            units = units.substring(1, units.length() - 1);
            inString.remove(0, comma_pos + 1); // now delete the units value and the comma
            reading = inString.toFloat();
          }
        }

        // Aggressively round close to zero
        if (reading < 0.1 && reading > -0.1)
          reading = 0.0;

        if (reading != 0) {
          if (units.equals("m") || units.equals("cm") || units.equals("in") || units.equals("ft") || units.equals("yd")) {

            distance = reading;
            distanceUnit = units;
            Serial.print("Distance: ");
            Serial.print(distance);
            Serial.print(",");
            //Serial.print(distanceUnit);
            Serial.print("Velocity: ");
            Serial.print(velocity);
            //Serial.print(" ");
            //Serial.print(velocityUnit);
            Serial.println();
            velocity = 0;

          } else { // again, if no unit, assume it's a velocity
            velocity = reading;
            velocityUnit = units;
          }


        }
      }

      // clear the string for new input:
      inString = "";
    }
  }
}
