/**
  ******************************************************************************
  * @file    mpu6050.c
  * @brief   MPU6050 六轴传感器驱动（STM32 HAL，I2C）
  *
  *   - I2C 寄存器读写封装
  *   - 器件初始化（唤醒、量程、低通滤波）
  *   - 加速度 / 角速度原始数据读取
  ******************************************************************************
  */
#include "mpu6050.h"

/* I2C 操作超时时间（ms） */
#define MPU6050_TIMEOUT  100U

/**
  * @brief  写单个寄存器
  */
static HAL_StatusTypeDef MPU6050_WriteReg(I2C_HandleTypeDef *hi2c,
                                          uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, MPU6050_TIMEOUT);
}

/**
  * @brief  连续读多个寄存器
  */
static HAL_StatusTypeDef MPU6050_ReadRegs(I2C_HandleTypeDef *hi2c,
                                          uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len, MPU6050_TIMEOUT);
}

/**
  * @brief  初始化 MPU6050
  * @note   复位 -> 唤醒 -> 校验 WHO_AM_I -> 配置采样率/带宽/量程
  */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t who_am_i = 0;

    HAL_Delay(100);

    /* 器件复位 */
    MPU6050_WriteReg(hi2c, MPU6050_REG_PWR_MGMT_1, 0x80);
    HAL_Delay(100);

    /* 退出休眠，时钟源选择内部 8MHz 振荡器 */
    MPU6050_WriteReg(hi2c, MPU6050_REG_PWR_MGMT_1, 0x00);
    HAL_Delay(10);

    /* 校验设备 ID */
    MPU6050_ReadRegs(hi2c, MPU6050_REG_WHO_AM_I, &who_am_i, 1);
    if (who_am_i != 0x68) {
        return HAL_ERROR;
    }

    /* 采样率分频：1kHz / (1 + 7) = 125Hz */
    MPU6050_WriteReg(hi2c, MPU6050_REG_SMPLRT_DIV, 0x07);

    /* 数字低通滤波 DLPF=6（带宽约 5Hz），抑制高频噪声 */
    MPU6050_WriteReg(hi2c, MPU6050_REG_CONFIG, 0x06);

    /* 陀螺仪量程 ±250 dps */
    MPU6050_WriteReg(hi2c, MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_FS_250);

    /* 加速度计量程 ±2g */
    MPU6050_WriteReg(hi2c, MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS_2G);

    return HAL_OK;
}

/**
  * @brief  读取加速度原始值
  */
HAL_StatusTypeDef MPU6050_ReadAccel(I2C_HandleTypeDef *hi2c,
                                    int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];

    if (MPU6050_ReadRegs(hi2c, MPU6050_REG_ACCEL_XOUT_H, buf, 6) != HAL_OK) {
        return HAL_ERROR;
    }

    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);

    return HAL_OK;
}

/**
  * @brief  读取角速度原始值
  */
HAL_StatusTypeDef MPU6050_ReadGyro(I2C_HandleTypeDef *hi2c,
                                   int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];

    if (MPU6050_ReadRegs(hi2c, MPU6050_REG_GYRO_XOUT_H, buf, 6) != HAL_OK) {
        return HAL_ERROR;
    }

    *gx = (int16_t)((buf[0] << 8) | buf[1]);
    *gy = (int16_t)((buf[2] << 8) | buf[3]);
    *gz = (int16_t)((buf[4] << 8) | buf[5]);

    return HAL_OK;
}

/**
  * @brief  读取加速度并解析为合加速度与姿态角
  * @note   姿态角由加速度计解算：
  *         roll  = atan2(ay, az)
  *         pitch = atan2(-ax, sqrt(ay^2 + az^2))
  */
HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data)
{
    int16_t ax, ay, az;
    float fax, fay, faz;

    if (MPU6050_ReadAccel(hi2c, &ax, &ay, &az) != HAL_OK) {
        return HAL_ERROR;
    }

    data->ax = ax;
    data->ay = ay;
    data->az = az;

    /* 换算成重力加速度 g */
    fax = (float)ax / MPU6050_ACCEL_LSB_PER_G;
    fay = (float)ay / MPU6050_ACCEL_LSB_PER_G;
    faz = (float)az / MPU6050_ACCEL_LSB_PER_G;

    /* 合加速度 */
    data->accel_mag = sqrtf(fax * fax + fay * fay + faz * faz);

    /* 姿态角（角度制） */
    data->roll  = atan2f(fay, faz) * 57.29578f;
    data->pitch = atan2f(-fax, sqrtf(fay * fay + faz * faz)) * 57.29578f;

    return HAL_OK;
}
