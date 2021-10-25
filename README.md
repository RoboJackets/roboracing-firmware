# RoboRacing - Firmware

This repository contains RoboRacing's firmware for all of its platforms. Firmware includes any code that runs on the cars' microcontrollers and sensors. We currently use MBED and Arduino based microcontrollers and communicates to a client ROS node over USB Serial or Ethernet TCP. For all of our ROS code, please see our [RoboRacing-Software Github repository](https://github.com/RoboJackets/roboracing-software)


### Our Platforms: ###

* **Rigatoni** (*active*)
* **Sedani:** (*active*)
    * *sedani_chassis*: Arduino Pro-Micro based shield that uses Serial and controls both steering and drive.
* **Macaroni:** (*retired*)
    * *motorControl*: Arduino based shield that uses Serial communication and controls steering and drive.
    * *ultrasonic\_sensor\_array*: Arduino based ultrasonic sensor array that used as a "poor man's lidar" to detect obstacles
* **Bigoli:** (*retired*)
    * *bigoli_chassis*: MBED based shield that uses Ethernet TCP and controls both steering and drive.
    * *legacy*: Folder containing unused Arduino and Mbed code kept for reference as the platform develops.

