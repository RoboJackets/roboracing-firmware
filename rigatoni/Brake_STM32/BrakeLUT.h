#pragma once
/**
This file contains a lookup table with {brake turns from home, braking force} pairs.
Needs to be calibrated; currently I made this up.
Note that this assumes home is 1 turn away from the limit switch.
*/
const unsigned int BrakeLUTLength = 3;
const unsigned int BrakeLUTMaxIndex = BrakeLUTLength - 1;
const float BrakeLUT[BrakeLUTLength][2] = {
  {0.0, -0.000001},  //just slightly negative so function is monotonic
  {6.0, 0.0},
  {20, 300}
};

const float MAX_STEPPER_POSITION_FROM_HOME = BrakeLUT[BrakeLUTMaxIndex][0];
const float MAX_COMMAND_FORCE = BrakeLUT[BrakeLUTMaxIndex][1];
