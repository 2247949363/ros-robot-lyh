#ifndef __MPU6050_I2C_H
#define __MPU6050_I2C_H

#include "sys.h"

/*
 * PC0/PC1 are unused by the existing motor, encoder, OLED and USART wiring.
 * This preserves PB3/PB4 for SWD/JTAG. Most MPU6050 modules already include
 * pull-ups; otherwise add external 4.7 kOhm pull-ups to 3.3 V.
 */
#define MPU6050_IIC_GPIO       GPIOC
#define MPU6050_IIC_SCL_Pin    GPIO_Pin_0
#define MPU6050_IIC_SDA_Pin    GPIO_Pin_1

#define MPU6050_IIC_SCL        PCout(0)
#define MPU6050_IIC_SDA        PCout(1)
#define MPU6050_IIC_SDA_IN     PCin(1)
#define MPU6050_IIC_DELAY()    MPU6050_IIC_DelayUs(1U)

void MPU6050_IIC_DelayUs(u32 microseconds);
void MPU6050_IIC_IO_Init(void);
void MPU6050_IIC_SDA_IO_OUT(void);
void MPU6050_IIC_SDA_IO_IN(void);
void MPU6050_IIC_Start(void);
void MPU6050_IIC_Stop(void);
u8 MPU6050_IIC_Send_Byte(u8 txd);
u8 MPU6050_IIC_Read_Byte(u8 ack);
u8 MPU6050_IIC_Read_Ack(void);
void MPU6050_IIC_Send_Ack(u8 ack);

#endif
