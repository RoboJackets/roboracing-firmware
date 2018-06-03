
#include <MPU9250_RegisterMap.h>
#include <SparkFunMPU9250-DMP.h> 
//#include <Wire.h> // Depending on your Arduino version, you may need to include Wire.h
#define SerialPort SerialUSB

MPU9250_DMP imu; 

float accelX;
float accelY;
float accelZ;
float gyroX;
float gyroY;
float gyroZ;
float magX;
float magY;
float magZ;

float q0;
float q1;
float q2;
float q3;


void setup() {

  SerialPort.begin(115200);

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
         DMP_FEATURE_GYRO_CAL,
         10);

// enable sensors: gyroscope, accelerometer, and compass
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

// set sensor ranges and rates
  imu.setGyroFSR(2000);       // (+/- 250, 500, 1000, or 2000) dps
    imu.setAccelFSR(2);       // (+/- 2, 4, 8, or 16) g
    imu.setLPF(5);          // digital low-pass filter (188, 98, 42, 20, 10, 5) Hz
    imu.setSampleRate(10);      // (4 - 1000) Hz
    imu.setCompassSampleRate(10);   // (1-100) Hz
}

void loop() {

// to compute acceleration, gyroscope and magnetometer readings 
  if ( imu.dataReady() ){
      imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
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


// to use  MPU-9250’s digital motion processor (DMP) for 
// quaternion calculation to estimare roll, pitch, and yaw
  if ( imu.fifoAvailable() ){
    if ( imu.dmpUpdateFifo() == INV_SUCCESS){
      imu.computeEulerAngles();
      q0 = imu.calcQuat(imu.qw);
      q1 = imu.calcQuat(imu.qx);
      q2 = imu.calcQuat(imu.qy);
      q3 = imu.calcQuat(imu.qz);
    }
  }
    printIMUData();
}

void printIMUData(void){  
  String accel = String(String(accelX) + ", " + String(accelY) + ", " + String(accelZ));
  String gyro = String(String(gyroX) + ", " + String(gyroY) + ", " + String(gyroZ));
  String quat = String( String(q0, 4) + ", " + String(q1, 4) + ", " + String(q2, 4) +  ", " + String(q3, 4));
  String mag = String(String(magX) + ", " + String(magY) + ", " + String(magZ));
  String axes = String(String(imu.roll) + ", " + String(imu.pitch) + ", " + String(imu.yaw));

  SerialPort.println("ax," + accel); 
  SerialPort.println("gx," + gyro);
  SerialPort.println("q0," + quat);
  SerialPort.println("mx," + mag);
  SerialPort.println("axes," + axes);
}
