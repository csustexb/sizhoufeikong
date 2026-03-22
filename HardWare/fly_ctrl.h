#ifndef __FLY_CTRL_H
#define __FLY_CTRL_H

#include "stm32f10x.h"                  // Device header

/*
油门决定上升
横滚角是通过增加一侧电机转速，减小另一侧电机转速
俯仰角则是通过增加前侧电机和减小后侧电机
偏航则是通过控制对角的电机
*/

typedef struct
{
	float throttle;//油门
	float target_roll;//横滚角
	float target_pitch;//俯仰角
	float target_yaw_rate;//偏航角速度。只能用角速度，不能用角度，因为没有磁力计（没有参考，都是相对计算的）
}Fly_target;

//再定义一个电机转速参数
typedef struct
{
	float m1;
	float m2;
	float m3;
	float m4;
}Motor_Out;

void Fly_Init(void);
uint8_t Fly_Control_Update(float dt);//更新新的工作参数
void Fly_Control_SetTarget(float throttle, float target_roll, float target_pitch, float target_yaw_rate);
uint8_t Fly_Control_GetMotorOut(Motor_Out *motor);//获取电机转速参数

#endif