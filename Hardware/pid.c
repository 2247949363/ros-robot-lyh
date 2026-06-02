#include "stm32f10x.h"                  // Device header
#include "pid.h"


void PID_Init(PID *pid,float p, float i, float d, float maxI, float maxOut)
{
	pid-> kp = p;
	pid-> ki = i;
	pid-> kd = d;
	pid-> max_Integral = maxI;
	pid-> maxOutput = maxOut;
	pid-> error = 0;
	pid-> last_error = 0;
	pid-> integral = 0;
	pid-> output = 0;
}

void PID_Reset(PID *pid)
{
	pid-> error = 0;
	pid-> last_error = 0;
	pid-> integral = 0;
	pid-> output = 0;
}
void PID_Calc(PID *pid, float Reference ,float feedback)
{
	float pout;

	pid-> error = Reference - feedback;
	pout = pid-> error;
	pid-> integral += pid-> error;
	float dout = (pid-> last_error - pid-> error);
	
	if (pid-> integral > pid-> max_Integral)
		pid-> integral = pid-> max_Integral;
	else if (pid-> integral < - pid-> max_Integral)
		pid-> integral = - pid-> max_Integral;				//积分限幅
	
	pid-> output = pid->kp * pout + pid-> ki*pid-> integral+pid-> kd* dout;
	
	if (pid-> output > pid-> maxOutput)
		pid-> output = pid-> maxOutput;
	else if (pid-> output < - pid-> maxOutput)
		pid-> output = - pid-> maxOutput;
	
	pid-> last_error = pid-> error;
}



//方向控制——角度和偏移
#define u1_max  000

float p_jiao =0.0,d_jiao =0.0;
float u1 = 0;

//int PD_jiao(float expect,float error)
//{
//	volatile static float error_current,error_last;
//	float ek,ek1;
//	error_current = error - expect;
//	ek = error_current;
//	ek1 =error_current - error_last;
//	u1=p_jiao * ek+d_jiao * ek1;
//	error_last = error_current;
//	
//	if(u1 > u1_max)
//		u1 = u1_max;
//	if(u1 < -u1_max)
//		u1 = -u1_max;
//	
//	return (int)u1;
//}

//#define u2_max  000

//float p_pian =0.0,d_pian =0.0;
//float u2 = 0;

//int PD_pian(float expect,float error)
//{
//	volatile static float error_current,error_last;
//	float ek,ek1;
//	error_current = error - expect;
//	ek = error_current;
//	ek1 =error_current - error_last;
//	u2=p_pian * ek+d_pian * ek1;
//	error_last = error_current;
//	
//	if(u2 > u2_max)
//		u2 = u2_max;
//	if(u2 < -u2_max)
//		u2 = -u2_max;
//	
//	return (int)u2;
//}

