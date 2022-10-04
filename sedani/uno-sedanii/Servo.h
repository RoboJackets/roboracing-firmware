/*
*	 This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/


#ifndef MBED_SERVO_H
#define MBED_SERVO_H

#include <mbed.h>

/**
 * The following parameters are servo specific but should work with most servos. They can be changed without modifing this header using the following code
 *      #undef SERVO_REFRESH_RATE_US
 *      #define SERVO_REFRESH_RATE_US [servo refresh rate in us]
 *      #undef SERVO_MIN_PULSE
 *      #define SERVO_MIN_PULSE [pulse width for -90 degrees in us]
 *      #undef SERVO_MAX_PULSE
 *      #define SERVO_MAX_PULSE [pulse width for +90 degrees in us]
 */

#define SERVO_REFRESH_RATE_US 20000 // 50Hz
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE  2400


#define SERVO_MIN_DEGREE  -90
#define SERVO_MAX_DEGREE  90

#define SERVO_POS_CENTER  0
#define SERVO_POS_MIN  SERVO_MIN_DEGREE
#define SERVO_POS_MAX  SERVO_MAX_DEGREE

/**
 * Driver for a servo motor
 */
class Servo {
    public:
        /**
         * Constructs new servo object for servo connected to pin
         * @param pin the pin the servo is connected to
         */
        Servo(PinName pin);

        /**
         * Enables the servo and moves it to the initial position
         * @param startPos the initial position of the servo in degrees
         */ 
        void attach(int startPos);

        /**
         * Disables the servo
         */ 
        void detach();

        /**
         * Sets the position of the servo
         * @param pos the position of the servo in degrees
         */
        void write(int pos);

        /**
         * Gets the current position of the servo
         * @return the current servo position in degrees
         */
        int read();

    private:
        DigitalOut pulsePin;
        Ticker refreshTicker;
        Timeout pulseTimer;
        uint16_t currentPulse;

        void startPulse();
        void endPulse();
};

#endif
