/**
  ******************************************************************************
  * @file    main.c
  * @brief   基于 STM32F103 的 MPU6050 跌倒检测主程序
  *
  *   流程：读取 MPU6050 -> 一阶低通滤波 -> 合加速度与姿态角解算 ->
  *         跌倒状态机判断 -> UART 输出结果
  *
  *   硬件：I2C1(PB6/PB7) 接 MPU6050；USART1(PA9/PA10) 输出，115200。
  *   注意：本文件包含 HAL 初始化函数，默认假设 HSE=8MHz、主频 72MHz，
  *         若板子晶振不同请在 CubeMX 中重新生成时钟配置。
  ******************************************************************************
  */
#include "stm32f1xx_hal.h"
#include "mpu6050.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* 外设句柄 */
I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart1;

/* 一阶低通滤波系数（0~1，越小越平滑） */
#define FILTER_ALPHA        0.2f

/* 跌倒检测阈值（可调） */
#define MAG_THRESHOLD_LOW   0.5f    /* 合加速度低于该值视为自由落体 */
#define MAG_THRESHOLD_HIGH  3.0f    /* 合加速度高于该值视为撞击 */
#define MAG_STATIC_WINDOW   0.15f   /* 静止时合加速度允许偏离 1g 的范围 */
#define ANGLE_FALL          45.0f   /* 倾角超过该值视为躺倒 */
#define ANGLE_STATIC        20.0f   /* 倾角小于该值视为直立 */

/* 跌倒状态 */
typedef enum {
    STATE_STATIC = 0,   /* 静止 */
    STATE_ACTIVE = 1,   /* 正常活动 */
    STATE_FALL   = 2    /* 跌倒 */
} FallState_t;

/* 函数声明 */
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void Error_Handler(void);

int main(void)
{
    MPU6050_Data_t data;
    float mag_f = 1.0f, roll_f = 0.0f, pitch_f = 0.0f;
    float angle, mag;
    FallState_t state;

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();

    if (MPU6050_Init(&hi2c1) != HAL_OK) {
        /* 设备自检失败：循环发送错误提示 */
        const char *err = "MPU6050 init failed\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t *)err, strlen(err), 100);
        while (1) { }
    }

    while (1) {
        /* 1. 读取并解算合加速度、姿态角 */
        if (MPU6050_ReadAll(&hi2c1, &data) != HAL_OK) {
            continue;
        }
        mag   = data.accel_mag;
        angle = fabsf(data.roll) > fabsf(data.pitch) ? fabsf(data.roll) : fabsf(data.pitch);

        /* 2. 一阶低通滤波降噪 */
        mag_f   = FILTER_ALPHA * mag   + (1.0f - FILTER_ALPHA) * mag_f;
        roll_f  = FILTER_ALPHA * data.roll  + (1.0f - FILTER_ALPHA) * roll_f;
        pitch_f = FILTER_ALPHA * data.pitch + (1.0f - FILTER_ALPHA) * pitch_f;

        /* 3. 跌倒状态机判断 */
        if (mag_f < MAG_THRESHOLD_LOW || mag_f > MAG_THRESHOLD_HIGH) {
            state = STATE_FALL;                 /* 自由落体或撞击 */
        } else if (angle > ANGLE_FALL) {
            state = STATE_FALL;                 /* 明显倾斜（躺倒） */
        } else if (fabsf(mag_f - 1.0f) < MAG_STATIC_WINDOW && angle < ANGLE_STATIC) {
            state = STATE_STATIC;               /* 接近 1g 且直立 */
        } else {
            state = STATE_ACTIVE;
        }

        /* 4. 串口输出：roll/pitch 保留 1 位小数，mag 保留 2 位（避免 %f，兼容 nano 库） */
        {
            char buf[96];
            int r10 = (int)(roll_f * 10.0f);
            int p10 = (int)(pitch_f * 10.0f);
            int m100 = (int)(mag_f * 100.0f);
            int len = snprintf(buf, sizeof(buf),
                               "roll=%d.%d pitch=%d.%d mag=%d.%02d state=%d\r\n",
                               r10 / 10, (r10 < 0 ? -r10 : r10) % 10,
                               p10 / 10, (p10 < 0 ? -p10 : p10) % 10,
                               m100 / 100, m100 % 100, (int)state);
            HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 100);
        }

        HAL_Delay(20);   /* 约 50Hz 输出 */
    }
}

/* ==================== HAL 初始化（可由 CubeMX 生成替换） ==================== */

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* HSE 8MHz -> PLL x9 -> 72MHz */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* USART1：PA9(TX)/PA10(RX)，复用推挽 */
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hi2c->Instance == I2C1) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_I2C1_CLK_ENABLE();

        /* I2C1：PB6(SCL)/PB7(SDA)，开漏复用，上拉 */
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART1_CLK_ENABLE();
    }
}

static void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
