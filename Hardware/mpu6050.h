#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>

typedef struct
{
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t temp_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temperature_c;
} MPU6050_Data_t;

extern volatile MPU6050_Data_t mpu6050_data;
extern volatile uint8_t mpu6050_online;
extern volatile uint16_t mpu6050_update_count;
extern volatile uint8_t mpu6050_who_am_i;
extern volatile uint8_t mpu6050_gyro_calibrated;
extern volatile float mpu6050_gyro_x_bias;
extern volatile float mpu6050_gyro_y_bias;
extern volatile float mpu6050_gyro_z_bias;

uint8_t MPU6050_Init(void);
uint8_t MPU6050_Update(void);
uint8_t MPU6050_ReadID(void);
uint8_t MPU6050_CalibrateGyro(void);

#endif
