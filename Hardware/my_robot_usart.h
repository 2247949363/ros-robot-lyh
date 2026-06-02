#ifndef __MYROBOTUSART__
#define __MYROBOTUSART__
#include <sys.h>	

#define START   0X11

//从linux接收并解析数据到参数地址中，返回1表示收到完整有效帧
extern int usartParseOneByte(unsigned char data,int *p_x_SpeedSet,int *p_y_SpeedSet,int *p_w_SpeedSet,unsigned char *p_crtlFlag);
extern int usartReceiveOneData(int *p_x_SpeedSet,int *p_y_SpeedSet,int *p_w_SpeedSet,unsigned char *p_crtlFlag);   
//封装数据，调用USART1_Send_String将数据发送给linux
extern unsigned char usartSendData(short x_vel, short y_vel,short w_vel,short angle,unsigned char ctrlFlag); 
//发送指定字符数组的函数
unsigned char USART_Send_String(unsigned char *p,unsigned short sendSize);     
//计算八位循环冗余校验，得到校验值，一定程度上验证数据的正确性
unsigned char getCrc8(unsigned char *ptr, unsigned short len); 

#endif
