#include "my_robot_usart.h"
#include "USART3.h"         //包含printf

/*--------------------------------发送协议-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//数据头55aa + 数据字节数size + 数据（利用共用体） + 校验crc8 + 数据尾0d0a
//数据中预留了一个字节的控制位，其他的可以自行扩展，更改size和数据
--------------------------------------------------------------------------*/

/*--------------------------------接收协议-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//数据头55aa + 数据字节数size + 数据（利用共用体） + 校验crc8 + 数据尾0d0a
//数据中预留了一个字节的控制位，其他的可以自行扩展，更改size和数据
--------------------------------------------------------------------------*/


/**************************************************************************
通信的发送函数和接收函数必须的一些常量、变量、共用体对象
**************************************************************************/

//数据接收暂存区
unsigned char  receiveBuff[16] = {0};         
//通信协议常量
const unsigned char header[2]  = {0x55, 0xaa};
const unsigned char ender[2]   = {0x0d, 0x0a};

//发送数据（左轮速、右轮速、角度）共用体（-32767 - +32768）
union sendData
{
	short d;
	unsigned char data[2];
}x_vel_send,y_vel_send,w_vel_send,angle_send;

//左右轮速控制速度共用体
union receiveData
{
	short d;
	unsigned char data[2];
}x_VelSet,y_VelSet,w_VelSet;

/**************************************************************************
函数功能：通过串口中断服务函数，获取上位机发送的左右轮控制速度、预留控制标志位，分别存入参数中
入口参数：左轮轮速控制地址、右轮轮速控制地址、预留控制标志位
返回  值：1表示收到完整有效帧，0表示未完成或校验失败
**************************************************************************/
int usartParseOneByte(unsigned char USART_Receiver,int *p_x_SpeedSet,int *p_y_SpeedSet,int *p_w_SpeedSet,unsigned char *p_crtlFlag)
{
	static unsigned char checkSum             = 0;
	static unsigned char USARTBufferIndex     = 0;
	static short j=0,k=0;
	static unsigned char USARTReceiverFront   = 0;
	static unsigned char Start_Flag           = START;
	static short dataLength                   = 0;

	if(Start_Flag == START)
	{
		if(USART_Receiver == 0xaa)
		{
			if(USARTReceiverFront == 0x55)
			{
				Start_Flag = !START;
				receiveBuff[0]=header[0];
				receiveBuff[1]=header[1];
				USARTBufferIndex = 0;
				checkSum = 0x00;
			}
		}
		else
		{
			USARTReceiverFront = USART_Receiver;
		}
	}
	else
	{
		switch(USARTBufferIndex)
		{
			case 0:
				receiveBuff[2] = USART_Receiver;
				dataLength     = receiveBuff[2];
				if(dataLength < 7 || dataLength > 10)
				{
					USARTBufferIndex   = 0;
					USARTReceiverFront = 0;
					Start_Flag         = START;
					checkSum           = 0;
					dataLength         = 0;
					j = 0;
					k = 0;
					return 0;
				}
				USARTBufferIndex++;
				break;
			case 1:
				receiveBuff[j + 3] = USART_Receiver;
				j++;
				if(j >= dataLength)
				{
					j = 0;
					USARTBufferIndex++;
				}
				break;
			case 2:
				receiveBuff[3 + dataLength] = USART_Receiver;
				checkSum = getCrc8(receiveBuff, 3 + dataLength);
				if (checkSum != receiveBuff[3 + dataLength])
				{
					USARTBufferIndex   = 0;
					USARTReceiverFront = 0;
					Start_Flag         = START;
					checkSum           = 0;
					dataLength         = 0;
					j = 0;
					k = 0;
					return 0;
				}
				USARTBufferIndex++;
				break;
			case 3:
				if(k==0)
				{
					if(USART_Receiver != ender[0])
					{
						USARTBufferIndex   = 0;
						USARTReceiverFront = 0;
						Start_Flag         = START;
						checkSum           = 0;
						dataLength         = 0;
						j = 0;
						k = 0;
						return 0;
					}
					k++;
				}
				else if (k==1)
				{
					if(USART_Receiver == ender[1])
					{
						for(k = 0; k < 2; k++)
						{
							x_VelSet.data[k] = receiveBuff[k + 3];
							y_VelSet.data[k] = receiveBuff[k + 5];
							w_VelSet.data[k] = receiveBuff[k + 7];
						}

						*p_x_SpeedSet  =  (int)x_VelSet.d;
						*p_y_SpeedSet  =  (int)y_VelSet.d;
						*p_w_SpeedSet  =  (int)w_VelSet.d;
						*p_crtlFlag = receiveBuff[9];

						USARTBufferIndex   = 0;
						USARTReceiverFront = 0;
						Start_Flag         = START;
						checkSum           = 0;
						dataLength         = 0;
						j = 0;
						k = 0;
						return 1;
					}
					USARTBufferIndex   = 0;
					USARTReceiverFront = 0;
					Start_Flag         = START;
					checkSum           = 0;
					dataLength         = 0;
					j = 0;
					k = 0;
				}
				break;
			 default:break;
		}
	}
	return 0;
}

int usartReceiveOneData(int *p_x_SpeedSet,int *p_y_SpeedSet,int *p_w_SpeedSet,unsigned char *p_crtlFlag)
{
	unsigned char USART_Receiver = (unsigned char)USART_ReceiveData(USART3);
	return usartParseOneByte(USART_Receiver, p_x_SpeedSet, p_y_SpeedSet, p_w_SpeedSet, p_crtlFlag);
}
/**************************************************************************
函数功能：将左右轮速和角度数据、控制信号进行打包，通过串口发送给Linux
入口参数：实时左轮轮速、实时右轮轮速、实时角度、控制信号（如果没有角度也可以不发）
返回  值：1表示DMA发送已启动，0表示DMA忙或参数错误
**************************************************************************/
unsigned char usartSendData(short x_vel, short y_vel,short w_vel,short angle,unsigned char ctrlFlag)
{
	// 协议数据缓存数组
	unsigned char buf[15] = {0};
	int i, length = 0;

	// 计算左右轮期望速度
	x_vel_send.d  = x_vel;
	y_vel_send.d = y_vel;
	w_vel_send.d = w_vel;
	angle_send.d    = angle;
	
	// 设置消息头
	for(i = 0; i < 2; i++)
		buf[i] = header[i];                      // buf[0] buf[1] 
	
	// 设置机器人左右轮速度、角度
	length = 9;
	buf[2] = length;                             // buf[2]
	for(i = 0; i < 2; i++)
	{
		buf[i + 3] = x_vel_send.data[i];         // buf[3] buf[4]
		buf[i + 5] = y_vel_send.data[i];        // buf[5] buf[6]
		buf[i + 7] = w_vel_send.data[i];        // buf[7] buf[8]
		buf[i + 9] = angle_send.data[i];           // buf[9] buf[10]
	}
	// 预留控制指令
	buf[3 + length - 1] = ctrlFlag;              // buf[11]
	
	// 设置校验值、消息尾
	buf[3 + length] = getCrc8(buf, 3 + length);  // buf[12]
	buf[3 + length + 1] = ender[0];              // buf[13]
	buf[3 + length + 2] = ender[1];              // buf[14]
	
	//发送字符串数据
	return USART_Send_String(buf,sizeof(buf));
}
/**************************************************************************
函数功能：发送指定大小的字符数组，被usartSendData函数调用
入口参数：数组地址、数组大小
返回  值：1表示DMA发送已启动，0表示DMA忙或参数错误
**************************************************************************/
unsigned char USART_Send_String(u8 *p,u16 sendSize)
{
    return USART3_SendBytesDMA((const uint8_t *)p, sendSize);
}
/**************************************************************************
函数功能：计算八位循环冗余校验，被usartSendData和usartReceiveOneData函数调用
入口参数：数组地址、数组大小
返回  值：无
**************************************************************************/
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
	unsigned char crc;
	unsigned char i;
	crc = 0;
	while(len--)
	{
		crc ^= *ptr++;
		for(i = 0; i < 8; i++)
		{
			if(crc&0x01)
                crc=(crc>>1)^0x8C;
			else 
                crc >>= 1;
		}
	}
	return crc;
}
/**********************************END***************************************/







/*--------------------------------发送协议-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//数据头55aa + 数据字节数size + 数据（利用共用体） + 校验crc8 + 数据尾0d0a
--------------------------------------------------------------------------*/

/*--------------------------------接收协议-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//数据头55aa + 数据字节数size + 数据（利用共用体） + 校验crc8 + 数据尾0d0a
--------------------------------------------------------------------------*/







