#include "stm32f10x.h"                  // Device header
#include "pwm.h"
#include "encoder.h"
#include "math.h"
#include "timer.h"




float angle_to_radian = 0.01745f;
#define L 117		//mm

extern volatile float Vy_dipan, Vx_dipan ,W;
float v1_jisuan,v2_jisuan,v3_jisuan;
float PWM1,PWM2,PWM3;

volatile float Vx_cal,Vy_cal,W_cal;


void motor_Init()
{
	PWM_Init(1000 , 36);
	TIM2_encoder_Init();
	TIM3_encoder_Init();
	TIM4_encoder_Init();


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitTypeDef GPIO_InitStructure1;
	GPIO_InitStructure1.GPIO_Mode =GPIO_Mode_Out_PP;
	GPIO_InitStructure1.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure1);
	
	
	GPIO_ResetBits(GPIOB,GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
	GPIO_ResetBits(GPIOC,GPIO_Pin_10 | GPIO_Pin_11);
}

void hou_Positive()						//后轮正转
{
	GPIO_SetBits(GPIOB,GPIO_Pin_13);
	GPIO_ResetBits(GPIOB,GPIO_Pin_12);
}

void hou_Negative()						//后轮反转
{
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
	GPIO_ResetBits(GPIOB,GPIO_Pin_13);
}


void left_Positive()						//右轮正转
{
	GPIO_ResetBits(GPIOC,GPIO_Pin_11);
	GPIO_SetBits(GPIOC,GPIO_Pin_10);
}

void left_Negative()						//右轮反转
{
	GPIO_ResetBits(GPIOC,GPIO_Pin_10);
	GPIO_SetBits(GPIOC,GPIO_Pin_11);
}


void right_Positive()						//h轮正转
{
	GPIO_SetBits(GPIOB,GPIO_Pin_15);
	GPIO_ResetBits(GPIOB,GPIO_Pin_14);
}

void right_Negative()						//右轮反转
{
	GPIO_SetBits(GPIOB,GPIO_Pin_14);
	GPIO_ResetBits(GPIOB,GPIO_Pin_15);
}



void motor_Set1(int PWM)
{
	if(PWM>1000)
	{
		PWM=1000;
	}
	else if (PWM<-1000)
	{
		PWM = -1000;
	}
	
	
	if(PWM > 0)			//正转
	{
		right_Positive();
		PWM_SetCompare1(PWM);
	}
	else
	{
		right_Negative();
		PWM_SetCompare1(-PWM);
	}
}

void motor_Set2(int PWM)
{
	if(PWM>1000)
	{
		PWM=1000;
	}
	else if (PWM<-1000)
	{
		PWM = -1000;
	}
	
	
	if(PWM > 0)			//正转
	{
		left_Positive();
		PWM_SetCompare2(PWM);
	}
	else
	{
		left_Negative();
		PWM_SetCompare2(-PWM);
	}
}


void motor_Set3(int PWM)
{
	if(PWM>1000)
	{
		PWM=1000;
	}
	else if (PWM<-1000)
	{
		PWM = -1000;
	}
	
	
	if(PWM > 0)			//正转
	{
		hou_Positive();
		PWM_SetCompare3(PWM);
	}
	else
	{
		hou_Negative();
		PWM_SetCompare3(-PWM);
	}
}

void Speed_Target(float Vx_dipan ,float Vy_dipan, float W)  
{
	v1_jisuan = -cos(60 * angle_to_radian) *(-Vy_dipan) + sin(60 * angle_to_radian) *Vx_dipan + W*L;
	v2_jisuan = -cos(60 * angle_to_radian) * ( -Vy_dipan) - sin(60 * angle_to_radian) *Vx_dipan + W*L;
	v3_jisuan =  ( -Vy_dipan) + W*L;
}

void Speed_cal(float v1_actual ,float v2_actual, float v3_actual) 
{
	Vx_cal = (v1_actual - v2_actual) * sqrt(3)/3;
	Vy_cal = v1_actual/3 + v2_actual/3 - v3_actual*2/3;
	W_cal = (v1_actual/3 + v2_actual/3 + v3_actual/3)/L;
}







