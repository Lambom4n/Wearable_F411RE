#ifndef QUATERNION_H
#define QUATERNION_H
extern "C" {
#include "IMU_9250.h"
}
#include "MadgwickAHRS.h"

// Declare the Madgwick instance as extern
extern Madgwick madgwick;

// Function declarations
void quaternion_init();
void quaternion_update();
float quaternion_get_roll();
float quaternion_get_pitch();
float quaternion_get_yaw();
float quaternion_get_roll_radians();
float quaternion_get_pitch_radians();
float quaternion_get_yaw_radians();
void quaternion_update_imu(float gx, float gy, float gz, float ax, float ay, float az);
float quaternion_get_roll_imu();
float quaternion_get_pitch_imu();
float quaternion_get_yaw_imu();
float quaternion_get_roll_imu_radians();
float quaternion_get_pitch_imu_radians();
float quaternion_get_yaw_imu_radians();
void quaternion_print_data();  // Function to print the quaternion-filtered data
void set_yaw_offset();  // Function to set the yaw offset
#endif















