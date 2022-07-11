#include "mbed.h"
#include "NeoStripRGBW.h"
#include <cmath>

#define PI 3.14159265

const float width = 0.185;          // the width of the car in meters (X coords)
const float length = 0.255;         // the length of the car in meters (Y coords)
const float originToEnd = 0.04;     // distance from orign to back of the car in meters
const float neoPerUnit = 60;        // the number of neopixels per meter
const float maxMag = 181.019;       // the max distance away from the origin the object can be in meters
const float maxVelocity = 10;       // the max velocity away the robot can travel in meters/second
const int topLeftIndex = 0;         // the index of the top left individual Pixel
const int botLeftIndex = 15;        // the index of the bottom left individual Pixel
const int botRightIndex = 27;       // the index of the bottom right individual Pixel
const int topRightIndex = 44;       // the index of the top right individual Pixel
const int N = 56;                   // the number of total NeoPixels
const char * gradientChar = 'rg';	// the selected gradient colors
const float phi = atan2((length - originToEnd),width / 2);
const float psi = atan2(-originToEnd,width / 2);

DigitalIn b1(p25); // Direction button
DigitalIn b2(p26); // Point button
DigitalOut myLed1(LED1);
DigitalOut myLed2(LED2);

AnalogIn pot(p20);
AnalogIn photo(p19);

NeoStripRGBW strip(p8, N);
Serial pc(USBTX, USBRX); // tx, rx

void neoDirectionAndVelocity(float,float);
void neoPointDetection(signed char *,int);
void setPixelGradient(NeoStripRGBW &, int, int, float, char*, bool);

int floatToIntWithRounding(float);


/**
*  Direction of NeoPixels, sides, and geometry:
*
*			   b  4  a
*            ___________
*           |\    |    /|
*         a | \phi|   / | b
* LED   |   |  \  |  /  |
* Index | 1 |   \ | /   | 3 
*       V   |    \ /    |
*           | - - . - - | 
*         b |     |  `  | a
*            ----------`  \__> psi   
*			   a  2  b      
*               ---->
*
*		Coordinate System:
*            +x |
*               |
*         +y ------- -y
*               |
*            -x |
*
*
*
*/

int main() {    
    b1.mode(PullDown);
    b2.mode(PullDown);
    bool b1o = b1;      // old copy of button 1 to poll for changes
    bool b2o = b2;      // old copy of button 2 to poll for changes

    bool neoDirection = true;
    
    strip.setBrightness(bright);
    float potSample;
    float photoSample;
    signed char arr[] = {0, -4};


    while(1) {
        
        // button 1 changes the pattern, only do stuff when its state has changed
        if (b1 != b1o) {
            neoDirection = true;
            b1o = b1;
        }
        
        if (b2 != b2o) {
            neoDirection = false;
            b2o = b2;
        }
        strip.clear();

        if (neoDirection) {
            potSample = pot;
            photoSample = photo;
            neoDirectionAndVelocity(potSample,photoSample);
            myLed1 = true;
            myLed2 = false;
        }
        else{
            potSample = pot;
            photoSample = photo;
            
            neoPointDetection(arr, 2);
            myLed1 = false;
            myLed2 = true;
        
        }
        
        strip.write();
        wait_ms(50);
    }
}

//TODO Fix documentation
/** 
*   Given an arbitrary array of points stringed together in 
*   a 'xyxyxy' format, where every odd character is a X value and 
*   every even character is a Y value, the position is superimposed on 
*   the NeoPixel array in line with the detected point.
*
*   The brightness of each point is set by the distance from the origin each point exists.
*
*   @param pointArr The array of detected points with X and Y values in ranges of -128 to 127.
*   @param size The size of the pointArr.
*/

void neoDirectionAndVelocity(float directionTheta, float velocityValue){
    int index = thetaToIndex(directionTheta);
    if (index > -1) {
        float magnitude = velocityValue / maxVelocity;
        setPixelGradient(strip, index, magnitude, gradientChar, true);
        strip.write(); 
    }
    //TODO Add error message
}

/** 
*   Given an arbitrary array of points stringed together in 
*   a 'xyxyxy' format, where every odd character is a X value and 
*   every even character is a Y value, the position is superimposed on 
*   the NeoPixel array in line with the detected point.
*
*   The brightness of each point is set by the distance from the origin each point exists.
*
*   @param pointArr The array of detected points with X and Y values in ranges of -128 to 127.
*   @param size The size of the pointArr.
*/

void neoPointDetection(signed char* pointArr, int size){
    if (size % 2 == 0) {
        int numberOfPoints = (int)(size / 2);
        strip.clear();
        for (int i = 0; i < numberOfPoints; i++) {
            signed char xPos = pointArr[(i * 2)];
            signed char yPos = pointArr[(i * 2) + 1];
            float theta = atan2(yPos, xPos);
            float magnitude = sqrt((pow(xPos, 2) + pow(yPos, 2)));
            int index = thetaToIndex(theta); 
            
            if (index > -1) {
                float percentMagnitude = magnitude / maxMag;
                setPixelGradient(strip, index, percentMagnitude, gradientChar, false);
            }
        }
        strip.write();
    }
}

/** 
*   Given a theta (with 0 rads straight forward, positive clockwise, and negative counter-clockwise),
*   an index is calculated on the NeoPixels according the given size constraints 
*
*   @param theta The given direction in radians
*   @return The index on the NeoPixels.
*/

int thetaToIndex(float theta){
    int index = -1;
    if (theta >= 0 && theta >= phi && theta < (PI-psi)) {
        // Side 1
        int originXIndex = botLeftIndex - floatToIntWithRounding(neoPerUnit*originToEnd);
        float fractionX = abs(tan(theta - (PI / 2)));

        if (theta < PI/2) {
            // Side 1a
            // Above the origin in X direction
            index = topLeftIndex + floatToIntWithRounding(fractionX*(originXIndex-topLeftIndex));
        } else {
            // Side 1b
            // Below the origin in X direction
            index = originXIndex + floatToIntWithRounding(fractionX*(botLeftIndex-originXIndex));
        }

    } else if ((theta >= 0 && theta >= (PI - psi)) || (theta < 0 && theta < (-(PI - psi)))) {
        // Side 2
        int originYIndex = floatToIntWithRounding((botRightIndex - botLeftIndex) / 2 + botLeftIndex);
        float fractionY = abs(tan(PI - theta));

        if (theta > 0) {
            // Side 2a
            // To the left of the origin in the Y direction
            index = originYIndex - floatToIntWithRounding(fractionY * (originYIndex - botLeftIndex));
        } else {
            // Side 2b
            // To the right of the origin in the Y direction
            index = originYIndex + floatToIntWithRounding(fractionY * (botRightIndex - originYIndex));
        }
    } else if (theta < 0 && theta < -phi && theta >= (-PI / 2) - psi) {
        // Side 3
        int originXIndex = botRightIndex + floatToIntWithRounding(neoPerUnit * originToEnd);
        float fractionX = abs(tan(theta + (PI / 2)));
        
        if (theta < -PI / 2) {
            // Side 3a
            // Below the origin in the X direction
            index = originXIndex - floatToIntWithRounding(fractionX * (originXIndex - botRightIndex));
        }
        else {
            // Side 3b
            // Above the origin in X direction
            index = originXIndex + floatToIntWithRounding(fractionX * (topRightIndex - originXIndex));
        } // TODO Check for invalid theta
    } else {
        // Side 4
        int originYIndex = floatToIntWithRounding((N - topRightIndex) / 2 + topRightIndex);
        float fractionY = abs(tan(theta));

        if (theta < 0) {
            // Side 4a
            // To the right of the origin in the Y direction
            index = originYIndex - floatToIntWithRounding(fractionY * (originYIndex - topRightIndex));
        } else{
            // Side 4b
            // To the left of the origin in the Y direction
            index = originYIndex + floatToIntWithRounding(fractionY * (N - originYIndex));
        }
    }

    return index;
}

//TODO Documentation

/** 
*   Given an arbitrary array of points stringed together in 
*   a 'xyxyxy' format, where every odd character is a X value and 
*   every even character is a Y value, the position is superimposed on 
*   the NeoPixel array in line with the detected point.
*
*   The brightness of each point is set by the distance from the origin each point exists.
*
*   @param pointArr The array of detected points with X and Y values in ranges of -128 to 127.
*   @param size The size of the pointArr.
*/

void setPixelGradient(NeoStripRGBW &strip, int index, int width, float gradient, char * transitionType, bool reset){
    if (gradient >= 0 && gradient <= 1) {
        if (reset) {
            strip.clear();
        }

        float minusGradient = 1 - gradient;
        NeoColorRGBW createdColor;

        createdColor.red = (uint8_t)(0);
        createdColor.green = (uint8_t)(0);
        createdColor.blue = (uint8_t)(0);
        createdColor.white = (uint8_t)(0);

        if (transitionType[0] == 'r') {
            createdColor.red = (uint8_t)(gradient * 255);
        } else if (transitionType[0] == 'g') {
            createdColor.green = (uint8_t)(gradient * 255);
        } else if (transitionType[0] == 'b') {
            createdColor.blue = (uint8_t)(gradient * 255);
        } else if (transitionType[0] == 'w') {
            createdColor.white = (uint8_t)(gradient * 255);
        } else {
            printf("Error: Invalid Initial Color Combo\n");
        }

        if (transitionType[1] == 'r') {
            createdColor.red = (uint8_t)(minusGradient * 255);
        } else if (transitionType[1] == 'g') {
            createdColor.green = (uint8_t)(minusGradient * 255);
        } else if (transitionType[1] == 'b') {
            createdColor.blue = (uint8_t)(minusGradient * 255);
        } else if (transitionType[1] == 'w') {
            createdColor.white = (uint8_t)(minusGradient * 255);
        } else {
            printf("Error: Invalid Secondary Color Combo\n");
        }

        for (int i = 0; i < width; i++){
            if (index - i >= 0) {
                strip.setPixel(index - i, createdColor); 
            }

            if (index + i < N) {
                strip.setPixel(index + i, createdColor); 
            }
        }

    } else {
        printf("Error: Invalid Gradient\n");
    }
    
}


//TODO Fix 
/** 
*   Given an arbitrary array of points stringed together in 
*   a 'xyxyxy' format, where every odd character is a X value and 
*   every even character is a Y value, the position is superimposed on 
*   the NeoPixel array in line with the detected point.
*
*   The brightness of each point is set by the distance from the origin each point exists.
*
*   @param pointArr The array of detected points with X and Y values in ranges of -128 to 127.
*   @param size The size of the pointArr.
*/

int floatToIntWithRounding(float input){
    return (int)(input + 0.5);
}
