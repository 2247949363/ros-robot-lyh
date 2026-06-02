#include "stm32f10x.h"                  // Device header



void CountSensor_Init(void)  //初始化函数
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource14);	
	//配置AFIO的数据选择器，选择想要中断的引脚（此处是PB14引脚）
	//PB14号口的第14个中断线路
	
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line14;		//指定要配置的中断线
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		//外部中断或者事件中断
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;		//边沿触发方式
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		//指定中断分组
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;		//指定要启用或禁用的中断通道
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		//指定抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			//指定相应优先级
	NVIC_Init(&NVIC_InitStructure);
}

void EXTI15_10_IRQHandler(void)		//中断函数
{
	if(EXTI_GetITStatus(EXTI_Line14) == SET)    //中断标志位的判断,此处判断exit14的中断标志位是否为1
	{
	
		EXTI_ClearITPendingBit(EXTI_Line14);		//清楚中断标志位，要不然会一直中断
	}
	
}
