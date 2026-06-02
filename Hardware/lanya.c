#include "stm32f10x.h"                  // Device header


	


void USART2_Config(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    										
    GPIO_InitTypeDef GPIO_InitStructure;
    
	//TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);
   
    //RX	  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);      

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 19200; 					// 设置波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; 	// 8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1; 			// 停止位
    USART_InitStructure.USART_Parity = USART_Parity_No; 		// 奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 		// 硬件流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; 			// 支持接收和发送模式
    USART_Init(USART2, &USART_InitStructure);
    
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    USART_Cmd(USART2, ENABLE);
}

u8 lanya_js[6];
u8 lanya_i=0;
volatile int flag =0;
volatile int f=0;
volatile int x=0;
volatile int w=0;
volatile int flag_stop = 0;

extern float p_jiao,d_jiao,p_pian,d_pian;
extern volatile float Vy_dipan, Vx_dipan ,W;



void USART2_IRQHandler(void)  
{  
     if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)      
      { 
		  
			USART_ClearITPendingBit(USART2, USART_IT_RXNE);   
            lanya_js[lanya_i++]=USART_ReceiveData(USART2);
		    if(lanya_js[0]!='a') lanya_i=0;
		    if(lanya_i==5)
			{
				lanya_i=0;
				
			
						
				/*       phone         */
				
							if(lanya_js[4]=='b')//速度取值
								{
								f=100*(lanya_js[1]-48)+10*(lanya_js[2]-48)+(lanya_js[3]-48);
								USART_SendData(USART2, 'k');
								flag=1;
								 }
							if(lanya_js[4]=='c')//速度增加
								{
								f+=100*(lanya_js[1]-48)+10*(lanya_js[2]-48)+(lanya_js[3]-48);
								USART_SendData(USART2, 'k');
									flag=1;
								 }
							if(lanya_js[4]=='d')//速度减少
								{
								f-=100*(lanya_js[1]-48)+10*(lanya_js[2]-48)+(lanya_js[3]-48);
								USART_SendData(USART2, 'k');
									flag=1;
								 }							
							if(lanya_js[4]=='m')//x速度增加
								{
								x+=100*(lanya_js[1]-48)+10*(lanya_js[2]-48)+(lanya_js[3]-48);
								USART_SendData(USART2, 'k');
									flag=1;
								 }
							if(lanya_js[4]=='n')//x速度减少
								{
								x-=100*(lanya_js[1]-48)+10*(lanya_js[2]-48)+(lanya_js[3]-48);
								USART_SendData(USART2, 'k');
									flag=1;
								}
							if(lanya_js[4]=='o')//w速度增加
								{
								w+=100*(lanya_js[1]-48)+10*(lanya_js[2]-48)+(lanya_js[3]-48);
								USART_SendData(USART2, 'k');
									flag=1;
								 }
							if(lanya_js[4]=='p')//w速度减少
								{
								w-=100*(lanya_js[1]-48)+10*(lanya_js[2]-48)+(lanya_js[3]-48);
								USART_SendData(USART2, 'k');
									flag=1;
								}
							if(lanya_js[4] == 'z')		//停车
								{
									flag_stop = 1;
									USART_SendData(USART2,'k');
									flag=1;
								}
							if(lanya_js[4] == 'y')		//重新运动
								{
									flag_stop = 0;
									USART_SendData(USART2,'k');
									flag=1;
								}
					 
			}
		  
		  }
		  

	  }


/*--------------------------------蓝牙接收协议-----------------------------------
//----------------0a 00 00 00 0x----------------------
//数据头0a + 数据值 + 数据尾0x（通过改变x的值，可实现Vx、Vy、W等不同参数的修改）
--------------------------------------------------------------------------*/



