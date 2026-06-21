#include "fly_ctrl.h"
#include "mpu6050.h"
#include "pid.h"
#include "pwm.h"

/*
PID algorithm: internal variables preferred for encapsulation
Centralizes all variable calculations within the library
Reduces logic complexity in application code

PID requires outer-loop and inner-loop parameters
*/

//Outer loop (angle): each axis uses PID_t
static PID_t pid_roll_angle;
static PID_t pid_pitch_angle;

//Inner loop parameters
static PID_t pid_roll_rate;
static PID_t pid_pitch_rate;
static PID_t pid_yaw_rate;
//Target values
static Fly_target fly_target = {0};
static Motor_Out motor_out = {0};

static float Limit(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

void Fly_Init(void)
{
	PID_Init(&pid_roll_angle,  4.0f, 0.0f, 0.8f,  50.0f, 200.0f);
	PID_Init(&pid_pitch_angle, 4.0f, 0.0f, 0.8f,  50.0f, 200.0f);

	/* Inner loop (rate): Ki enabled for steady-state error elimination
	   Kp reduced from 0.8→0.6, i_limit tightened 100→50 to prevent windup */
	PID_Init(&pid_roll_rate,   0.6f, 0.12f, 0.06f, 50.0f, 300.0f);  // Ki: 0→0.12
	PID_Init(&pid_pitch_rate,  0.6f, 0.12f, 0.06f, 50.0f, 300.0f);
	PID_Init(&pid_yaw_rate,    1.0f, 0.08f, 0.00f, 50.0f, 300.0f);  // Ki: 0→0.08
}
uint8_t Fly_Control_Update(float dt)
{
	MPU6050_Angle angle;
	MPU6050_Gyro gyro;

	float target_gx, target_gy;
	float roll_out, pitch_out, yaw_out;

	if (dt <= 0) return 0;
	/* Attitude updated by MPU6050 interrupt at 250Hz, read latest */
	MPU6050_GetAngle(&angle);
	MPU6050_GetGyro(&gyro);

	/* S3: Angle limit protection — cut throttle if exceeding safe angle */
	if (angle.Roll > MAX_SAFE_ANGLE || angle.Roll < -MAX_SAFE_ANGLE
	 || angle.Pitch > MAX_SAFE_ANGLE || angle.Pitch < -MAX_SAFE_ANGLE)
	{
		fly_target.throttle = 0.0f;
	}

	/* Update outer loop (angle to rate) */
	/*
	PID: angle controlled via angular rate, rate via torque
	Thus we need to compute the required torque
	*/
	target_gx = PID_Output(&pid_roll_angle,fly_target.target_roll,angle.Roll,dt);
	target_gy = PID_Output(&pid_pitch_angle,fly_target.target_pitch,angle.Pitch,dt);
	//Update inner loop (rate to motor), gyro = angular velocity
	roll_out = PID_Output(&pid_roll_rate,target_gx,gyro.GYRO_X,dt);
	pitch_out = PID_Output(&pid_pitch_rate,target_gy,gyro.GYRO_Y,dt);
	yaw_out = PID_Output(&pid_yaw_rate,fly_target.target_yaw_rate,gyro.GYRO_Z,dt);

	//Motor mixing (X-config quadcopter)
	/*
		M1 				M2



		M3				M4
	*/
	motor_out.m1 = fly_target.throttle - pitch_out - roll_out - yaw_out;
	motor_out.m2 = fly_target.throttle - pitch_out + roll_out + yaw_out;
	motor_out.m3 = fly_target.throttle + pitch_out + roll_out - yaw_out;
	motor_out.m4 = fly_target.throttle + pitch_out - roll_out + yaw_out;
	//Apply output limits
	motor_out.m1 = Limit(motor_out.m1, 1000.0f, 2000.0f);
	motor_out.m2 = Limit(motor_out.m2, 1000.0f, 2000.0f);
	motor_out.m3 = Limit(motor_out.m3, 1000.0f, 2000.0f);
	motor_out.m4 = Limit(motor_out.m4, 1000.0f, 2000.0f);

	PWM_SetCompare1((uint16_t)motor_out.m1);
	PWM_SetCompare2((uint16_t)motor_out.m2);
	PWM_SetCompare3((uint16_t)motor_out.m3);
	PWM_SetCompare4((uint16_t)motor_out.m4);

	return 1;
}
PID_t* Fly_GetRollAnglePID(void)  { return &pid_roll_angle; }
PID_t* Fly_GetPitchAnglePID(void) { return &pid_pitch_angle; }
PID_t* Fly_GetRollRatePID(void)   { return &pid_roll_rate; }
PID_t* Fly_GetPitchRatePID(void)  { return &pid_pitch_rate; }
PID_t* Fly_GetYawRatePID(void)    { return &pid_yaw_rate; }

//Set quadcopter target values
void Fly_Control_SetTarget(float throttle, float target_roll, float target_pitch, float target_yaw_rate)
{
	fly_target.throttle = throttle;
	fly_target.target_roll = target_roll;
	fly_target.target_pitch = target_pitch;
	fly_target.target_yaw_rate = target_yaw_rate;
}
//Get latest motor output
uint8_t Fly_Control_GetMotorOut(Motor_Out *motor)
{
	if (motor == 0) return 0;
	*motor = motor_out;
	return 1;
}

//Get current flight targets (for logging & monitoring)
void Fly_Control_GetTarget(Fly_target *target)
{
	if (target == 0) return;
	*target = fly_target;
}





