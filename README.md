# RoboRacing - Firmware [![CircleCI](https://circleci.com/gh/RoboJackets/roboracing-firmware.svg?style=shield)](https://circleci.com/gh/RoboJackets/roboracing-firmware)

This repository contains RoboRacing's firmware for all of its platforms. Firmware includes any code that runs on the cars' microcontrollers and sensors. We currently use MBED and Arduino based microcontrollers and communicates to a client ROS node over USB Serial or Ethernet TCP. For all of our ROS code, please see our [RoboRacing-Software Github repository](https://github.com/RoboJackets/roboracing-software)

To use code in this repo, navigate to your desired car's folder, and code should be ready to load onto the microcontroller. If the code requires communcation with the ROS network, there is a matching ROS node in the RoboRacing-Software repo. To load firmware onto the Arduino, simply use the Arduino IDE to upload code. To load firmware onto the MBED, the mbed-run python script is designed to run the offline compiler for MBED.

#TODO: better instructions for the MBED offline compiler

### Our Platforms: ###
* **Bigoli:** (*active*)
    * *bigoli_chassis*: MBED based shield that uses Ethernet TCP and controls both steering and drive.
    * *legacy*: Folder containing unused Arduino and Mbed code kept for reference as the platform develops.
* **Sedani:** (*active*)
    * *sedani_chassis*: Arduino Pro-Micro based shield that uses Serial and controls both steering and drive.
* **Macaroni:** (*retired*)
    * *motorControl*: Arduino based shield that uses Serial communication and controls steering and drive.
    * *ultrasonic\_sensor\_array*: Arduino based ultrasonic sensor array that used as a "poor man's lidar" to detect obstacles


* **Other folders:**
    * *test*: Contains files for general testing such as pin outputs, servos and ethernet communications that are needed often.
    * *mbed-libs*: Contains libraries used by the offline mbed compiler
