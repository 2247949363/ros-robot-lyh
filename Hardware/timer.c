#include "stm32f10x.h"                  // Device header
#include "encoder.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "my_robot_usart.h"
#include "pid.h"
#include "motor.h"

#define PI 3.1416
#define R 0.029 //单位m


extern int num;

extern volatile float speed_actual1, speed_actual2, speed_actual3;
extern volatile float speed_capture1,speed_capture2,speed_capture3;

int count; 
extern volatile int16_t second;

extern float Pitch,Roll,Yaw;								//俯仰角默认跟中值一样，翻滚角，偏航角

extern short testSend1 ;
extern short testSend2 ;
extern short testSend3 ;
extern short testSend4 ;
extern unsigned char testSend5 ;

extern PID mypid1,mypid2,mypid3;
extern float v1_jisuan,v2_jisuan,v3_jisuan;
extern int KeyNum1,KeyNum2,KeyNum3;

extern volatile int flag_stop;

extern volatile float Vy_dipan , Vx_dipan  ,W;

#if defined(STM32F10X_HD) || defined(STM32F10X_XL)

void Timer_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 7200 - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 -1;  //72M / 7200 /1 = 10000hz 
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStructure); 
	
	TIM_ClearITPendingBit(TIM8,TIM_IT_Update);	
	
	TIM_ITConfig(TIM8,TIM_IT_Update,ENABLE);
	
	TIM_Cmd(TIM8,ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStruture;
	NVIC_InitStruture.NVIC_IRQChannel = TIM8_UP_IRQn;  //TIM5_IRQn
	NVIC_InitStruture.NVIC_IRQChannelCmd =ENABLE;
	NVIC_InitStruture.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStruture.NVIC_IRQChannelSubPriority = 0;
	
	NVIC_Init(&NVIC_InitStruture);
	

}

void TIM8_UP_IRQHandler(void)							//1、2、3号轮的速度闭环
{
	if(TIM_GetITStatus(TIM8,TIM_IT_Update) != RESET)		//0.1ms
	{
		num ++;
		count++;
		TIM_ClearITPendingBit(TIM8,TIM_IT_Update);
		if(num == 50)
		{
		speed_capture1 = TIM2_Encoder_Get();			//读取电机速度
		speed_capture2 = TIM3_Encoder_Get();
		speed_capture3 = TIM4_Encoder_Get();
			
		speed_actual1 = speed_capture1*1000.0/(num/10000.0)/(13*74.8)*(2.0*PI*R);  //单位：mm/s
		speed_actual2 = speed_capture2*1000.0/(num/10000.0)/(13*74.8)*(2.0*PI*R);  //单位：mm/s
		speed_actual3 = speed_capture3*1000.0/(num/10000.0)/(13*74.8)*(2.0*PI*R);  //单位：mm/s
			
			
		
		Speed_cal(speed_actual1 , speed_actual2,  speed_actual3);
		
	
//		Speed_Target( Vx_dipan,Vy_dipan, W);		//获得一次目标值
//			
//		PID_Calc(&mypid1 , v1_jisuan , speed_actual1);	//pid运算
//		KeyNum1 = mypid1.output;
//		PID_Calc(&mypid2 , v2_jisuan , speed_actual2);	//pid运算
//		KeyNum2 = mypid2.output;
//		PID_Calc(&mypid3 , v3_jisuan , speed_actual3);	//pid运算
//		KeyNum3 = mypid3.output;

//		if(flag_stop==0)
//		{
//			motor_Set1(KeyNum1);
//			motor_Set2(KeyNum2);
//			motor_Set3(KeyNum3);
//		}


			
//		MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw);				//读取姿态信息(其中偏航角有飘移是正常现象)
		
		num=0;
		}
		if(count == 40000)
		{
			
			if(second<=3) second++;

			count=0;
		}
		
	}	
}

#else

void Timer_Init(void)
{
	/*
	 * TIM8 is not available on STM32F103RB/medium-density devices.
	 * The RTOS version samples encoders in ControlTask every 5 ms, so this
	 * legacy TIM8 speed-sampling timer is intentionally disabled.
	 */
}

#endif
/*
void TIM6_IRQHander(void)
{
	if(TIM_GetITStatus(TIM6,TIM_IT_Update) == SET)
	{
		
		TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
	}
	
}

*/



