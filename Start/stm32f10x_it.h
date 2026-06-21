#ifndef __STM32F10X_IT_H
#define __STM32F10X_IT_H

#include "stm32f10x.h"
#include "mpu6050.h"

// 定义传感器数据结构体（与 main.c 中一致）
typedef struct
{
    MPU6050_Angle angle;
    MPU6050_Gyro gyro;
} SensorData_t;

void EXTI4_IRQHandler(void);

#endif
