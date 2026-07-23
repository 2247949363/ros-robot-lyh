#ifndef __PID_H
#define __PID_H

typedef struct
{
    float kp;
    float ki;
    float kd;
    float error;
    float last_error;
    float integral;
    float max_Integral;
    float output;
    float maxOutput;
    float integral_separation;
    float error_deadzone;
} PID;

void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut);
void PID_SetProtection(PID *pid, float integralSeparation, float errorDeadzone);
void PID_Reset(PID *pid);

/* dt is seconds. The result is also stored in pid->output. */
float PID_Calc(PID *pid, float reference, float feedback, float dt);

#endif
