#ifndef __MPU6050_H
#define __MPU6050_H


#include "stm32f10x.h"                  // Device header
#include "i2c.h"

/*
飞控里需要解算的数据是ax,ay,az,gx,gy,gz
*/

#define MPU6050_ADDR 0x68
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_PWR_MGMT_2 0x6C
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_ACCEL_XOUT_L    0x3C
#define MPU6050_ACCEL_YOUT_H    0x3D
#define MPU6050_ACCEL_YOUT_L    0x3E
#define MPU6050_ACCEL_ZOUT_H    0x3F
#define MPU6050_ACCEL_ZOUT_L    0x40

#define MPU6050_TEMP_OUT_H      0x41
#define MPU6050_TEMP_OUT_L      0x42

#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_GYRO_XOUT_L     0x44
#define MPU6050_GYRO_YOUT_H     0x45
#define MPU6050_GYRO_YOUT_L     0x46
#define MPU6050_GYRO_ZOUT_H     0x47
#define MPU6050_GYRO_ZOUT_L     0x48

//原始量结构体定义
typedef struct
{
	uint16_t ACCEL_X;
	uint16_t ACCEL_Y;
	uint16_t ACCEL_Z;
	
	uint16_t GYRO_X;
	uint16_t GYRO_Y;
	uint16_t GYRO_Z;
	
} MPU6050_RawData;

//转换量结构体定义
typedef struct
{
	uint16_t ACCEL_X;
	uint16_t ACCEL_Y;
	uint16_t ACCEL_Z;
	
	uint16_t GYRO_X;
	uint16_t GYRO_Y;
	uint16_t GYRO_Z;
} MPU6050_Data;

uint8_t MPU6050_Init();
uint8_t MPU6050_GetRawData(MPU6050_RawData *rawdata);
void MPU6050_GetData(MPU6050_RawData *rawdata, MPU6050_Data *data);

#endif
