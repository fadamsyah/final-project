#include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);

unsigned long ros_time = 0; // Milisecond
int ros_period = 20; // 50 Hz

void setup() {
  // serial to display data
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("BEGIN");

  IMU.begin(); // initializes communication with the MPU-9250
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G); // setting the accelerometer full scale range to +/-2G
  IMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS); // setting the gyroscope full scale range to +/-2000 deg/s
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_184HZ); // setting DLPF bandwidth to 184 Hz
  IMU.setSrd(0); // setting SRD to 0 for a 1 kHz accelerometer & gyroscope update rate and 100 Hz magnetometer update rate

  // We want the RAW DATA !
  IMU.setGyroBiasX_rads(0.0);
  IMU.setGyroBiasY_rads(0.0);
  IMU.setGyroBiasZ_rads(0.0);
  IMU.setAccelCalX(0.0, 1.0);
  IMU.setAccelCalY(0.0, 1.0);
  IMU.setAccelCalZ(0.0, 1.0);
  IMU.setMagCalX(0.0, 1.0);
  IMU.setMagCalY(0.0, 1.0);
  IMU.setMagCalZ(0.0, 1.0);
}

void loop() {
  
  if ((millis() - ros_time) >= ros_period) {
    IMU.readSensor();

    // display the data
    Serial.print(IMU.getAccelX_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelY_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelZ_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroX_rads(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroY_rads(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroZ_rads(), 6);
    Serial.print("\t");
    Serial.print(IMU.getMagX_uT(), 6);
    Serial.print("\t");
    Serial.print(IMU.getMagY_uT(), 6);
    Serial.print("\t");
    Serial.print(IMU.getMagZ_uT(), 6);
    Serial.print("\t");
    Serial.println(IMU.getTemperature_C(), 6);

    ros_time = ros_time + ros_period;
  }
}
