#ifndef __PID_H
#define __PID_H

#include "stm32f10x.h"                  // Device header

typedef struct
{
	float kp;
	float ki;
	float kd;
	
	float integral;
	float last_error;
	
	float integral_limit;
	float output_limit;
}PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float i_limit, float outlimit);
float PID_Output(PID_t *pid, float target, float measure, float dt);
void PID_Reset(PID_t *pid);

#endif
