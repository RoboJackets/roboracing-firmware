/**
This file contains a lookup table with {stepper steps from home, braking force} pairs.
Needs to be calibrated; currently I made this up
*/
const byte BrakeLUTLength = 3;
const byte BrakeLUTMaxIndex = BrakeLUTLength - 1;
const double BrakeLUT[BrakeLUTLength][2] = {
  {100, 0.0},
  {300, 50},
  {1000, 800}
};

const int MAX_STEPPER_POSITION_FROM_HOME = (int) BrakeLUT[BrakeLUTMaxIndex][0];
const int MAX_COMMAND_FORCE = (int) BrakeLUT[BrakeLUTMaxIndex][1];
