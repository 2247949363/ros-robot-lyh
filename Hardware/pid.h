#ifndef __PID_H
#define __PID_H

typedef struct
{
	float kp,ki,kd;
	float error,last_error;
	float integral, max_Integral;		
	float output, maxOutput;			
}PID;


void PID_Init(PID *pid,float p, float i, float d, float maxI, float maxOut);
void PID_Reset(PID *pid);

void PID_Calc(PID *pid, float Reference ,float feedback);

int PD_jiao(float expect,float error);
int PD_pian(float expect,float error);

#endif
