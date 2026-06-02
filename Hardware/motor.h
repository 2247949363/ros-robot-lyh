#ifndef __MOTOR_H
#define __MOTOR_H

void motor_Init(void);
void hou_Positive(void);
void hou_Negative(void);
void left_Positive(void);
void left_Negative(void);
void right_Positive(void);
void right_Negative(void);

void motor_Set1(int PWM);
void motor_Set2(int PWM);
void motor_Set3(int PWM);

void Speed_Target(float Vx_dipan ,float Vy_dipan, float W);
void Speed_cal(float v1_actual ,float v2_actual, float v3_actual);
	
#endif
