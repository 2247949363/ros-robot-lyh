#ifndef __MOTOR_H
#define __MOTOR_H

#define MOTOR_PWM_COMMAND_MAX       1000
#define MOTOR_PWM_OUTPUT_LIMIT      600
#define MOTOR_PWM_MIN_EFFECTIVE     50
#define MOTOR_WHEEL_SPEED_LIMIT_MMPS 500.0f
#define CHASSIS_CENTER_TO_WHEEL_MM  117.0f

void motor_Init(void);
void motor_Set1(int pwm);
void motor_Set2(int pwm);
void motor_Set3(int pwm);
void motor_StopAll(void);

void Speed_Target(float vxMmps, float vyMmps, float wzRadps);
void Speed_cal(float v1Actual, float v2Actual, float v3Actual);

#endif
