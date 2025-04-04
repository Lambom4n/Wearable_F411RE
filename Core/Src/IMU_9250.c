#include "IMU_9250.h"

#define PI 3.1415
#define EXPECTED_Z_RAW 8192

extern I2C_HandleTypeDef hi2c2;

float accel_division_factor = 9.81 / 8192.0f;
float gyro_division_factor = PI / (65.5 * 180.0f);

int16_t accel_offset[3] = {0, 0, 0};
int16_t gyro_offset[3] = {0, 0, 0};

float mag_bias[3] = {0, 0, 0};
float mag_scale[3] = {1, 1, 1};
float avg_scale = 1;

float mag_sensitivity[3] = {0, 0, 0};
float mag_division_factor = 0.15f;

void IMU_init()
{
    uint8_t whoAmI;
    uint8_t settings;
    HAL_StatusTypeDef status;

    // Check device ID (should be 0x71 for MPU9250)
    status = HAL_I2C_Mem_Read(&hi2c2, MPU9250_ADDRESS << 1, WHO_AM_I, 1, &whoAmI, 1, HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        custom_printf("Error reading WHO_AM_I register\n");
    }

    // Wake up MPU-9250
    settings = 0x01; // Use PLL with X-axis gyro reference
    status = HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS << 1, PWR_MGMT_1, 1, &settings, 1, HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        custom_printf("Error waking up MPU-9250\n");
        return;
    }
    HAL_Delay(10); // Wait for stabilization

    // Configure gyroscope
    settings = 0x08; // Range: ±500 degrees/sec and F_CHOICE_B = 0b00
    HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS << 1, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &settings, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    // Configure accelerometer
    settings = 0x08; // Range: ±4g
    HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS << 1, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &settings, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    settings = 0x08;
    HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS << 1, ACCEL_CONFIG_2, I2C_MEMADD_SIZE_8BIT, &settings, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    // Configure low pass filter
    settings = 0x04; // Bandwidth: 20Hz
    HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS << 1, CONFIG, I2C_MEMADD_SIZE_8BIT, &settings, 1, 1000);
    HAL_Delay(10);

    // Configure magnetometer
    // First enable I2C bypass to access AK8963
    settings = 0x02;
    HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS << 1, INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &settings, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    // set fuse ROM access
    settings = 0x0F;
    HAL_I2C_Mem_Write(&hi2c2, AK8963_ADDRESS << 1, AK8963_CNTL1, I2C_MEMADD_SIZE_8BIT, &settings, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    // Read magnetometer sensitivity
    uint8_t mag_sensitivity_data[3];
    settings = 0x0F;
    status = HAL_I2C_Mem_Read(&hi2c2, AK8963_ADDRESS << 1, 0x10, I2C_MEMADD_SIZE_8BIT, mag_sensitivity_data, 3, HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        custom_printf("Error reading magnetometer sensitivity\n");
    }

    for (int i = 0; i < 3; i++)
    {
        mag_sensitivity[i] = ((mag_sensitivity_data[i] - 128) * 0.5 / 128) + 1;
    }
    HAL_Delay(10);
    // Set magnetometer to continuous measurement mode
    // // Reset first
    // settings = 0x01;
    // HAL_I2C_Mem_Write(&hi2c2, AK8963_ADDRESS << 1, AK8963_CNTL2, I2C_MEMADD_SIZE_8BIT, &settings, 1, HAL_MAX_DELAY);
    // HAL_Delay(10); // Wait for reset
    // Then set mode
    settings = 0x12; // 0x12 = 16-bit resolution + continuous mode 1 (8Hz)
    status = HAL_I2C_Mem_Write(&hi2c2, AK8963_ADDRESS << 1, AK8963_CNTL1, I2C_MEMADD_SIZE_8BIT, &settings, 1, HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        custom_printf("Error setting magnetometer mode\n");
    }
    HAL_Delay(10);
}

void IMU_Calibrate_Accel_Gyro(void)
{
    int16_t ax = 0, ay = 0, az = 0;
    int16_t gx = 0, gy = 0, gz = 0;
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int32_t count = 1000;
    custom_printf("Calibrating accelerometer and gyroscope...\n");

    // Collect 1000 samples
    for (int i = 0; i < count; i++)
    {
        IMU_Read_Accel_Raw(&ax, &ay, &az);
        HAL_Delay(1);
        IMU_Read_Gyro_Raw(&gx, &gy, &gz);
        sum_ax += ax;
        sum_ay += ay;
        sum_az += az;
        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;
        HAL_Delay(1);
    }

    // Calculate offsets
    accel_offset[0] = (int16_t)(sum_ax / count);
    accel_offset[1] = (int16_t)(sum_ay / count);
    accel_offset[2] = (int16_t)((sum_az / count) - EXPECTED_Z_RAW);
    gyro_offset[0] = (int16_t)(sum_gx / count);
    gyro_offset[1] = (int16_t)(sum_gy / count);
    gyro_offset[2] = (int16_t)(sum_gz / count);

    custom_printf("sum_ax: %d, sum_ay: %d, sum_az: %d\n", sum_ax, sum_ay, sum_az);
    custom_printf("sum_gx: %d, sum_gy: %d, sum_gz: %d\n", sum_gx, sum_gy, sum_gz);

    custom_printf("Accel offset: %d, %d, %d\n", accel_offset[0], accel_offset[1], accel_offset[2]);
    custom_printf("Gyro offset: %d, %d, %d\n", gyro_offset[0], gyro_offset[1], gyro_offset[2]);
}

void IMU_Calibrate_Mag(void)
{
    int16_t mx, my, mz;
    int16_t mag_max[3] = {-32767, -32767, -32767};
    int16_t mag_min[3] = {32767, 32767, 32767};
    custom_printf("Calibrating magnetometer...\n");
    uint32_t start_time = HAL_GetTick();
    while (HAL_GetTick() - start_time < 15000)
    {
        IMU_Read_Mag_Raw(&mx, &my, &mz);
        if (mx < mag_min[0])
            mag_min[0] = mx;
        if (mx > mag_max[0])
            mag_max[0] = mx;
        if (my < mag_min[1])
            mag_min[1] = my;
        if (my > mag_max[1])
            mag_max[1] = my;
        if (mz < mag_min[2])
            mag_min[2] = mz;
        if (mz > mag_max[2])
            mag_max[2] = mz;
        HAL_Delay(10);
    }

    mag_bias[0] = (mag_max[0] + mag_min[0]) / 2.0f;
    mag_bias[1] = (mag_max[1] + mag_min[1]) / 2.0f;
    mag_bias[2] = (mag_max[2] + mag_min[2]) / 2.0f;
    
    mag_scale[0] = (float)(mag_max[0] - mag_min[0]) / 2.0f;
    mag_scale[1] = (float)(mag_max[1] - mag_min[1]) / 2.0f;
    mag_scale[2] = (float)(mag_max[2] - mag_min[2]) / 2.0f;
    for (int i = 0; i < 3; i++) {
        if (mag_scale[i] < 0.01f) {
            // Handle invalid calibration (e.g., sensor not moved)
            // Set default scale to avoid division by zero
            mag_scale[i] = 1.0f; // Or trigger an error
        }
    }
    avg_scale = (float)(mag_scale[0] + mag_scale[1] + mag_scale[2]) / 3.0f;
    custom_printf("Mag bias: %f, %f, %f\n", mag_bias[0], mag_bias[1], mag_bias[2]);
    custom_printf("Mag scale: %f, %f, %f\n", mag_scale[0], mag_scale[1], mag_scale[2]);
    custom_printf("Avg scale: %f\n", avg_scale);
}

uint8_t IMU_Read_Reg(uint8_t reg)
{
    uint8_t data;
    HAL_I2C_Mem_Read(&hi2c2, MPU9250_ADDRESS << 1, reg, 1, &data, 1, HAL_MAX_DELAY);
    return data;
}

void IMU_Write_Reg(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS << 1, reg, 1, &data, 1, HAL_MAX_DELAY);
}

void IMU_Read_Accel_Raw(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t data[6] = {0};
    HAL_I2C_Mem_Read(
        &hi2c2,               // I²C handle
        MPU9250_ADDRESS << 1, // Device address (shifted for HAL)
        ACCEL_XOUT_H,         // Starting register (0x3B)
        I2C_MEMADD_SIZE_8BIT, // Critical: Use 8-bit addressing
        data,                 // Buffer to store data
        6,                    // Read 6 bytes
        HAL_MAX_DELAY         // Timeout
    );
    *ax = (int16_t)(data[0] << 8 | data[1]);
    *ay = (int16_t)(data[2] << 8 | data[3]);
    *az = (int16_t)(data[4] << 8 | data[5]);
}

void IMU_Read_Accel(float *ax, float *ay, float *az)
{
    int16_t raw_ax, raw_ay, raw_az;
    IMU_Read_Accel_Raw(&raw_ax, &raw_ay, &raw_az);
    // Apply offset
    raw_ax -= accel_offset[0];
    raw_ay -= accel_offset[1];
    raw_az -= accel_offset[2];
    // Convert to m/s^2
    *ax = (float)raw_ax * accel_division_factor;
    *ay = (float)raw_ay * accel_division_factor;
    *az = (float)raw_az * accel_division_factor;
    custom_printf("ax: %f, ay: %f, az: %f\n", *ax, *ay, *az);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void IMU_Read_Gyro_Raw(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t data[6] = {0};
    HAL_I2C_Mem_Read(
        &hi2c2,               // I²C handle
        MPU9250_ADDRESS << 1, // Device address (shifted for HAL)
        GYRO_XOUT_H,         // Starting register (0x3B)
        I2C_MEMADD_SIZE_8BIT, // Critical: Use 8-bit addressing
        data,                 // Buffer to store data
        6,                    // Read 6 bytes
        HAL_MAX_DELAY         // Timeout
    );
    *gx = (int16_t)(data[0] << 8 | data[1]);
    *gy = (int16_t)(data[2] << 8 | data[3]);
    *gz = (int16_t)(data[4] << 8 | data[5]);
}

void IMU_Read_Gyro(float *gx, float *gy, float *gz)
{
    int16_t raw_gx, raw_gy, raw_gz;
    IMU_Read_Gyro_Raw(&raw_gx, &raw_gy, &raw_gz);
    // Apply offset
    raw_gx -= gyro_offset[0];
    raw_gy -= gyro_offset[1];
    raw_gz -= gyro_offset[2];
    // Convert to rad/s
    *gx = (float)raw_gx * gyro_division_factor;
    *gy = (float)raw_gy * gyro_division_factor;
    *gz = (float)raw_gz * gyro_division_factor;
    custom_printf("gx: %f, gy: %f, gz: %f\n", *gx, *gy, *gz);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void IMU_Read_Mag_Raw(int16_t *mx, int16_t *my, int16_t *mz)
{
    uint8_t data[6];
        HAL_I2C_Mem_Read(
        &hi2c2,               // I²C handle
        AK8963_ADDRESS << 1, // Device address (shifted for HAL)
        AK8963_HXL,         // Starting register (0x03)
        I2C_MEMADD_SIZE_8BIT, // Critical: Use 8-bit addressing
        data,                 // Buffer to store data
        6,                    // Read 6 bytes
        HAL_MAX_DELAY         // Timeout
    );
    *mx = (int16_t)(data[1] << 8 | data[0]) * mag_sensitivity[0];
    *my = (int16_t)(data[3] << 8 | data[2]) * mag_sensitivity[1];
    *mz = (int16_t)(data[5] << 8 | data[4]) * mag_sensitivity[2];
    uint8_t settings = 0x12;
    HAL_I2C_Mem_Write(&hi2c2, AK8963_ADDRESS << 1, AK8963_CNTL1, I2C_MEMADD_SIZE_8BIT, &settings, 1, HAL_MAX_DELAY);
}

void IMU_Read_Mag(float *mx, float *my, float *mz)
{
    int16_t raw_mx, raw_my, raw_mz;
    IMU_Read_Mag_Raw(&raw_mx, &raw_my, &raw_mz);
    // Apply offset
    float cal_mx = ((float)raw_mx - mag_bias[0]) * (avg_scale / mag_scale[0]);
    float cal_my = ((float)raw_my - mag_bias[1]) * (avg_scale / mag_scale[1]);
    float cal_mz = ((float)raw_mz - mag_bias[2]) * (avg_scale / mag_scale[2]);
    // Convert to μT
    *mx = cal_mx * mag_division_factor;
    *my = cal_my * mag_division_factor;
    *mz = cal_mz * mag_division_factor;
    custom_printf("mx: %f, my: %f, mz: %f\n", *mx, *my, *mz);
}

void IMU_Task(void *param)
{
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    while (1)
    {
        IMU_Read_Accel(&ax, &ay, &az);
        IMU_Read_Gyro(&gx, &gy, &gz);
        IMU_Read_Mag(&mx, &my, &mz);
        custom_printf("--------------------------------\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}