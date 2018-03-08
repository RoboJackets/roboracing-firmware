

#include <SparkFunMPU9250-DMP.h> // Include SparkFun MPU-9250-DMP library
//#include <Wire.h> // Depending on your Arduino version, you may need to include Wire.h
#define SerialPort SerialUSB

MPU9250_DMP imu; // Create an instance of the MPU9250_DMP class


// initialize varibales to hold sensor data in respective units
float accelX;
float accelY;
float accelZ;
float gyroX;
float gyroY;
float gyroZ;
float magX;
float magY;
float magZ;

//initialize variables to hold quaternion values
float q0;
float q1;
float q2;
float q3;

// used for orientation calculation
const signed char orientationMatrix[9] = {
  1, 0, 0,
  0, 1, 0,
  0, 0, 1
};
unsigned char lastOrient = 0;
String orientation; 

void setup() {

	SerialPort.begin(115200)

// verfiies communication with and initialize MPU-9250 to default values
	if (imu.begin() != INV_SUCCESS){
    	while (1){
        // Failed to initialize MPU-9250
			SerialPort.println("Unable to communicate with MPU-9250");
      		SerialPort.println("Check connections, and try again.");
      		SerialPort.println();
      		delay(5000);
		}
	}

// Enable 6 axis quaternion calculation; enable gyroscope recalibration after 8 secs of no motion 
	imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT |
				 DMP_FEATURE_GYRO_CAL |
				 DMP_FEATURE_ANDROID_ORIENT,
				 10);

// enable sensors: gyroscope, accelerometer, and compass
	imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

// set sensor ranges and rates
 	imu.setGyroFSR(2000); 			// (+/- 250, 500, 1000, or 2000) dps
  	imu.setAccelFSR(2); 			// (+/- 2, 4, 8, or 16) g
  	imu.setLPF(5); 					// digital low-pass filter (188, 98, 42, 20, 10, 5) Hz
  	imu.setSampleRate(10); 			// (4 - 1000) Hz
  	imu.setCompassSampleRate(10); 	// (1-100) Hz

	imu.dmpSetOrientation(orientationMatrix);
}

void loop() {

// to compute acceleration, gyroscope and magnetometer readings 
	if ( imu.dataReady() ){
    	imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    	setSensorData();
  	}


// to use  MPU-9250’s digital motion processor (DMP) for 
// quaternion calculation to estimare roll, pitch, and yaw
	if ( imu.fifoAvailable() ){

		if ( imu.dmpUpdateFifo() == INV_SUCCESS){
			imu.computeEulerAngles();
			setQuatData();
			setOrientation();
		}
	}

  	printIMUData();
}

void setSensorData(void){

// convert raw sensor readings to respective units
   accelX = imu.calcAccel(imu.ax);
   accelY = imu.calcAccel(imu.ay);
   accelZ = imu.calcAccel(imu.az);

   gyroX = imu.calcGyro(imu.gx);
   gyroY = imu.calcGyro(imu.gy);
   gyroZ = imu.calcGyro(imu.gz);

   magX = imu.calcMag(imu.mx);
   magY = imu.calcMag(imu.my);
   magZ = imu.calcMag(imu.mz);
}

void setQuatData(){
// covert quat values to float (-1 to 1)
	q0 = imu.calcQuat(imu.qw);
	q1 = imu.calcQuat(imu.qx);
	q2 = imu.calcQuat(imu.qy);
	q3 = imu.calcQuat(imu.qz);
}

void setOrientation(void){

	unsigned char orient = imu.dmpGetOrientation();
	if (orient != lastOrient){

		switch (orient){
			case ORIENT_PORTRAIT:
				orientation = "Portrait";
				break;
			case ORIENT_LANDSCAPE:
				orientation = "Landscape";
				break;

			case ORIENT_REVERSE_PORTRAIT:
				orientation = "Portrait (Reverse)";
				break;
			case ORIENT_REVERSE_LANDSCAPE:
				orientation = "Landscape (Reverse)";
				break;
      	}
      	lastOrient = orient;
    }

}


void printIMUData(void){  
  
	SerialPort.println("Time: " + String(imu.time) + " ms");
	SerialPort.println("Accel: " + String(accelX) + ", " + 
		String(accelY) + ", " + String(accelZ) + " g");
	SerialPort.println("Gyro: " + String(gyroX) + ", " +
		String(gyroY) + ", " + String(gyroZ) + " dps");
	SerialPort.println("Mag: " + String(magX) + ", " +
		String(magY) + ", " + String(magZ) + " uT");
	SerialPort.println();
	SerialPort.println();


	SerialPort.println("Quaternion Values: " + String(q0, 4) + ", " +
	                String(q1, 4) + ", " + String(q2, 4) + 
	                ", " + String(q3, 4));
	SerialPort.println("Roll: " + String(imu.roll));
	SerialPort.println("Pitch: " + String(imu.pitch));
	SerialPort.println("Yaw: " + String(imu.yaw));
 	SerialPort.println();
	SerialPort.println();

	SerialPort.println("Orientation: " + orientation);
}