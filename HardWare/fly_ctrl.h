#ifndef FLY_CTRL_H
#define FLY_CTRL_H

#include "stm32f10x.h"

/*
Throttle controls lift.
Roll:  differential thrust on left/right motors.
Pitch: differential thrust on front/rear motors.
Yaw:   differential thrust on diagonal motor pairs.
*/

/* Safety: max safe roll/pitch angle before auto-throttle-cut */
#define MAX_SAFE_ANGLE  60.0f

typedef struct
{
	float throttle;
	float target_roll;
	float target_pitch;
	float target_yaw_rate;  /* Angular rate only (no compass reference) */
} Fly_target;

typedef struct
{
	float m1;
	float m2;
	float m3;
	float m4;
} Motor_Out;

void Fly_Init(void);
uint8_t Fly_Control_Update(float dt);
void Fly_Control_SetTarget(float throttle, float target_roll, float target_pitch, float target_yaw_rate);
uint8_t Fly_Control_GetMotorOut(Motor_Out *motor);
void Fly_Control_GetTarget(Fly_target *target);

/* PID access interface for CLI tuning */
#include "pid.h"
PID_t* Fly_GetRollAnglePID(void);
PID_t* Fly_GetPitchAnglePID(void);
PID_t* Fly_GetRollRatePID(void);
PID_t* Fly_GetPitchRatePID(void);
PID_t* Fly_GetYawRatePID(void);

#endif