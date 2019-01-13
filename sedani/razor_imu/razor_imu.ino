
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


float roll;
float pitch;
float yaw;


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
  imu.dmpBegin(  DMP_FEATURE_6X_LP_QUAT |
                 DMP_FEATURE_GYRO_CAL|
                 DMP_FEATURE_SEND_CAL_GYRO, 
                 10);

// enable sensors: gyroscope, accelerometer, and compass
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

// set sensor ranges and rates
    imu.setGyroFSR(2000);           // (+/- 250, 500, 1000, or 2000) dps
    imu.setAccelFSR(2);             // (+/- 2, 4, 8, or 16) g
    imu.setLPF(10);                 // digital low-pass filter (188, 98, 42, 20, 10, 5) Hz
    imu.setSampleRate(10);          // (4 - 1000) Hz
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

  SerialPort.print("ax,");
  SerialPort.print(accelX);
  SerialPort.print(",");
  SerialPort.print(accelY);
  SerialPort.print(",");
  SerialPort.println(accelZ);
 
  SerialPort.print("gx,");
  SerialPort.print(gyroX);
  SerialPort.print(",");
  SerialPort.print(gyroY);
  SerialPort.print(",");
  SerialPort.println(gyroZ);

  SerialPort.print("q0,");
  SerialPort.print(q0, 4);
  SerialPort.print(",");
  SerialPort.print(q1, 4);
  SerialPort.print(",");
  SerialPort.print(q2, 4);
  SerialPort.print(",");
  SerialPort.println(q3, 4);

  SerialPort.print("mx,");
  SerialPort.print(magX);
  SerialPort.print(",");
  SerialPort.print(magY);
  SerialPort.print(",");
  SerialPort.println(magZ);

  toEulerianAngle(q0, q1, q2, q3, roll, pitch, yaw);
  
  SerialPort.print("axes,");
  SerialPort.print(roll,4);
  SerialPort.print(",");
  SerialPort.print(pitch, 4);
  SerialPort.print(",");
  SerialPort.println(yaw, 4);

}


//https://forum.sparkfun.com/viewtopic.php?f=14&t=45516&hilit=9dof+razor+M0
static void toEulerianAngle(float w, float x, float y, float z, float& roll, float& pitch, float& yaw){
  double ysqr = y * y;

  double t0 = +2.0 * (w * x + y * z);
  double t1 = +1.0 - 2.0 * (x * x + ysqr);
  roll = atan2(t0, t1);

  double t2 = +2.0 * (w * y - z * x);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  pitch = asin(t2);
  
  double t3 = +2.0 * (w * z + x * y);
  double t4 = +1.0 - 2.0 * (ysqr + z * z);  
  yaw = atan2(t3, t4);
}




