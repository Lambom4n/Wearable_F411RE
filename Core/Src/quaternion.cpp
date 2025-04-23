// quaternion.cpp
//  ******************************************************************************
#include "quaternion.h"
#include "custom_printf.h" // Include the custom printf header
#include "queue.h"

extern QueueHandle_t receive_queue;

// Define the Madgwick instance
Madgwick madgwick;

// Global variables to store IMU data
float gx, gy, gz; // Gyroscope data in degrees/s
float ax, ay, az; // Accelerometer data in g
float mx, my, mz; // Magnetometer data in uT
float magnetic_declination = 0.19f; // Magnetic declination in radians (example value, adjust as needed)    
// In quaternion.h/cpp, add a global variable
float yaw_offset = 0.0f;

// Create a function to set the reference yaw
void set_yaw_offset() {
    // Capture the current yaw as the offset
    yaw_offset = madgwick.getYawRadians(); // Use radians to avoid 0-360 wrap issues
}
// Initialize the Madgwick filter
void quaternion_init()
{
    madgwick.begin(1.0f); // 1 Hz sample frequency
}

// Update the Madgwick filter with IMU data
void quaternion_update()
{
    // Read IMU data
    IMU_Read_Gyro(&gx, &gy, &gz);
    IMU_Read_Accel(&ax, &ay, &az);
    IMU_Read_Mag(&mx, &my, &mz);

    // Update the Madgwick filter with the IMU data
    madgwick.update(gx, gy, gz, ax, ay, az, mx, my, mz);
}

// Get roll angle in degrees
float quaternion_get_roll()
{
    return madgwick.getRoll();
}

// Get pitch angle in degrees
float quaternion_get_pitch()
{
    return madgwick.getPitch();
}

// Get yaw angle in degrees
// float quaternion_get_yaw()
// {
//     float yaw = madgwick.getYaw();
//     yaw -= magnetic_declination;
//     // Normalize to 0-360
//     if (yaw < 0) yaw += 360.0f;
//     return yaw;
// }

float quaternion_get_yaw() {
    float yaw = madgwick.getYawRadians(); // Get raw yaw in radians
    yaw -= magnetic_declination;          // Apply declination
    yaw -= yaw_offset;                    // Subtract calibration offset

    // Normalize to 0-360 degrees
    yaw = fmodf(yaw, 2 * M_PI);          // Wrap to [-2π, 2π]
    if (yaw < 0) yaw += 2 * M_PI;        // Convert to [0, 2π]
    return yaw * (180.0f / M_PI);        // Convert to degrees
}

// Get roll angle in radians
float quaternion_get_roll_radians()
{
    return madgwick.getRollRadians();
}

// Get pitch angle in radians
float quaternion_get_pitch_radians()
{
    return madgwick.getPitchRadians();
}

// Get yaw angle in radians
float quaternion_get_yaw_radians()
{
    return madgwick.getYawRadians();
}

// Update the Madgwick filter with IMU data (without magnetometer)
void quaternion_update_imu(float gx, float gy, float gz, float ax, float ay, float az)
{
    madgwick.updateIMU(gx, gy, gz, ax, ay, az);
}

// Get roll angle in degrees (IMU only)
float quaternion_get_roll_imu()
{
    return madgwick.getRoll();
}

// Get pitch angle in degrees (IMU only)
float quaternion_get_pitch_imu()
{
    return madgwick.getPitch();
}

// Get yaw angle in degrees (IMU only)
float quaternion_get_yaw_imu()
{
    return madgwick.getYaw();
}

// Get roll angle in radians (IMU only)
float quaternion_get_roll_imu_radians()
{
    return madgwick.getRollRadians();
}

// Get pitch angle in radians (IMU only)
float quaternion_get_pitch_imu_radians()
{
    return madgwick.getPitchRadians();
}

// Get yaw angle in radians (IMU only)
float quaternion_get_yaw_imu_radians()
{
    return madgwick.getYawRadians();
}

// Print the quaternion-filtered orientation data
void quaternion_print_data()
{
    // Get the filtered orientation angles
    float roll = quaternion_get_roll();
    float pitch = quaternion_get_pitch();
    float yaw = quaternion_get_yaw();

    // Print the orientation data
    custom_printf("Print data\n");
    custom_printf("Orientation: Roll=%.2f, Pitch=%.2f, Yaw=%.2f (degrees)\n", roll, pitch, yaw);
    vTaskDelay(pdMS_TO_TICKS(100));
    // Print the raw IMU data for comparison
    custom_printf("Raw IMU: Gx=%.2f, Gy=%.2f, Gz=%.2f (deg/s), Ax=%.2f, Ay=%.2f, Az=%.2f (g)\n",
                  gx, gy, gz, ax, ay, az);
    vTaskDelay(pdMS_TO_TICKS(100));
    // Print the magnetometer data
    custom_printf("Magnetometer: Mx=%.2f, My=%.2f, Mz=%.2f (uT)\n\n", mx, my, mz);
    custom_printf("End data\n\n");
    vTaskDelay(pdMS_TO_TICKS(3000));
}


void quaternion_update_task(void *pvParameters)
{
    while (1)
    {   
        quaternion_update();
        custom_printf("Update quaternion\n");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay to allow for IMU data update
    }
}