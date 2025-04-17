#ifndef IMU_9250_H
#define IMU_9250_H

#include "stm32f4xx_hal.h"
#include "custom_printf.h"
#include "FreeRTOS.h"
#include "task.h"

// MPU9250 I2C Address
#define MPU9250_ADDRESS    0x68
#define AK8963_ADDRESS     0x0C

// MPU9250 Registers
#define WHO_AM_I          0x75
#define PWR_MGMT_1        0x6B
#define ACCEL_CONFIG      0x1C
#define ACCEL_CONFIG_2    0x1D
#define GYRO_CONFIG       0x1B
#define INT_PIN_CFG       0x37
#define CONFIG            0x1A
// #define SMPLRT_DIV 0x19 // Sample Rate Divider Register

// AK8963 Registers
#define AK8963_CNTL1      0x0A
#define AK8963_CNTL2      0x0B
#define AK8963_HXL        0x03
#define AK8963_HXH        0x04
#define AK8963_HYL        0x05
#define AK8963_HYH        0x06
#define AK8963_HZL        0x07
#define AK8963_HZH        0x08 
// #define AK8963_ST1   0x02 

// MPU9250 Accel/Gyro Data Registers
#define ACCEL_XOUT_H      (0x3B)

#define GYRO_XOUT_H       (0x43)



/*IMU Initialization*/
void IMU_init(void);
uint8_t IMU_Read_Reg(uint8_t reg);  
void IMU_Write_Reg(uint8_t reg, uint8_t data);  

/*Calibration*/
void IMU_Calibrate_Accel_Gyro(void);  // Compute accel/gyro biases
void IMU_Calibrate_Mag(void);  // Compute mag biases

/*Accelerometer*/
void IMU_Read_Accel_Raw(int16_t *ax, int16_t *ay, int16_t *az);  
void IMU_Read_Accel(float *ax, float *ay, float *az);  // Scaled to g

/*Gyroscope*/
void IMU_Read_Gyro_Raw(int16_t *gx, int16_t *gy, int16_t *gz);  
void IMU_Read_Gyro(float *gx, float *gy, float *gz);  // Scaled to °/s

/*Magnetometer*/
void IMU_Read_Mag_Raw(int16_t *mx, int16_t *my, int16_t *mz);  
void IMU_Read_Mag(float *mx, float *my, float *mz);  // Scaled to μT

/*Task*/
void IMU_Task(void* param);

#endif
