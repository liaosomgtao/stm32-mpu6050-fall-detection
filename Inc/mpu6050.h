/**
  ******************************************************************************
  * @file    mpu6050.h
  * @brief   MPU6050 六轴传感器驱动头文件（STM32 HAL，I2C）
  ******************************************************************************
  */
#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f1xx_hal.h"

/* MPU6050 器件地址：AD0 接地时为 0x68（HAL 使用 8 位地址，需左移 1 位） */
#define MPU6050_ADDR           (0x68 << 1)

/* 常用寄存器地址 */
#define MPU6050_REG_SELF_TEST_X   0x0D
#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_TEMP_OUT_H    0x41
#define MPU6050_REG_GYRO_XOUT_H   0x43
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_WHO_AM_I      0x75

/* 量程配置（GYRO_CONFIG / ACCEL_CONFIG） */
#define MPU6050_GYRO_FS_250       0x00   /* ±250 dps，灵敏度 131 LSB/(°/s) */
#define MPU6050_GYRO_FS_500       0x08   /* ±500 dps */
#define MPU6050_GYRO_FS_1000      0x10   /* ±1000 dps */
#define MPU6050_GYRO_FS_2000      0x18   /* ±2000 dps */

#define MPU6050_ACCEL_FS_2G       0x00   /* ±2g，灵敏度 16384 LSB/g */
#define MPU6050_ACCEL_FS_4G       0x08   /* ±4g，8192 LSB/g */
#define MPU6050_ACCEL_FS_8G       0x10   /* ±8g，4096 LSB/g */
#define MPU6050_ACCEL_FS_16G      0x18   /* ±16g，2048 LSB/g */

/* 数据灵敏度换算系数 */
#define MPU6050_ACCEL_LSB_PER_G   16384.0f
#define MPU6050_GYRO_LSB_PER_DPS  131.0f

/* 解析后的传感器数据结构 */
typedef struct {
    int16_t ax, ay, az;        /* 加速度原始值 */
    int16_t gx, gy, gz;        /* 角速度原始值 */
    float   accel_mag;         /* 合加速度（单位 g） */
    float   roll;              /* 横滚角（单位 °） */
    float   pitch;             /* 俯仰角（单位 °） */
} MPU6050_Data_t;

/* 接口函数 */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_ReadAccel(I2C_HandleTypeDef *hi2c, int16_t *ax, int16_t *ay, int16_t *az);
HAL_StatusTypeDef MPU6050_ReadGyro(I2C_HandleTypeDef *hi2c, int16_t *gx, int16_t *gy, int16_t *gz);
HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data);

#endif /* __MPU6050_H */
