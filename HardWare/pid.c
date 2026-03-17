#include "pid.h"

static float PID_limit(float x, float limit)
{
	if (x > limit) return limit;
	if (x < -limit) return -limit;
	return x;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd, float i_limit, float outlimit)
{
	pid->ki = ki;
	pid->kp = kp;
	pid->kd = kd;
	pid->integral_limit = i_limit;
	pid->output_limit = outlimit;
}

float PID_Output(PID_t *pid, float target, float measure, float dt)
{
	float output;
	float error;
	float derivative;
	
	error = target - measure;
	pid->integral = dt * error;
	pid->integral = PID_limit(pid->integral, pid->integral_limit);
	
	derivative = (error - pid->last_error) / dt;
	
	output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
	output = PID_limit(output, pid->output_limit);
	
	return output;
}

void PID_Reset(PID_t *pid)
{
	pid->integral = 0.0f;
	pid->last_error = 0.0f;
}