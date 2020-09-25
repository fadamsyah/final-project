#include "MPU9250.h"
#include <ros.h>
#include <pkg_ta/IMU_9_DOF.h>
#define BAUD 500000

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);

unsigned long ros_time = 0; // Milisecond
int ros_period = 20; // 50 Hz
// int ros_period = 5; // 200 Hz

ros::NodeHandle nh;
pkg_ta::IMU_9_DOF msg;
ros::Publisher pub("mpu9250_raw_data", &msg);


void setup() {
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

  nh.getHardware()-> setBaud(BAUD);
  nh.initNode();
  nh.advertise(pub);
  msg.header.frame_id = "/imu_raw_data";
}

void loop() {
  if ((millis() - ros_time) >= ros_period){
    msg.header.stamp = nh.now();
    IMU.readSensor();

    msg.accelerometer.x = IMU.getAccelX_mss();
    msg.accelerometer.y = IMU.getAccelY_mss();
    msg.accelerometer.z = IMU.getAccelZ_mss();
    msg.gyroscope.x = IMU.getGyroX_rads();
    msg.gyroscope.y = IMU.getGyroY_rads();
    msg.gyroscope.z = IMU.getGyroZ_rads();
    msg.magnetometer.x = IMU.getMagX_uT();
    msg.magnetometer.y = IMU.getMagY_uT();
    msg.magnetometer.z = IMU.getMagZ_uT();

    pub.publish( &msg);
    ros_time = ros_time + ros_period;
  }
  nh.spinOnce();
}
