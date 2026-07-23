#include "MPU6050_I2C.h"

#define DWT_CTRL_REG            (*(volatile u32 *)0xE0001000UL)
#define DWT_CYCCNT_REG          (*(volatile u32 *)0xE0001004UL)
#define DWT_CTRL_CYCCNTENA_BIT  (1UL << 0)

void MPU6050_IIC_DelayUs(u32 microseconds)
{
    u32 startCycle = DWT_CYCCNT_REG;
    u32 requiredCycles = (SystemCoreClock / 1000000U) * microseconds;

    while ((u32)(DWT_CYCCNT_REG - startCycle) < requiredCycles)
    {
    }
}

void MPU6050_IIC_IO_Init(void)
{
    u8 i;

    /* A private cycle counter delay must not reconfigure FreeRTOS SysTick. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT_CYCCNT_REG = 0U;
    DWT_CTRL_REG |= DWT_CTRL_CYCCNTENA_BIT;

    /* Open-drain outputs are required for a valid I2C bus. */
    My_GPIO_Init(MPU6050_IIC_GPIO,
                 MPU6050_IIC_SCL_Pin | MPU6050_IIC_SDA_Pin,
                 GPIO_KL_OUT,
                 GPIO_P_NO,
                 GPIO_50MHz);

    MPU6050_IIC_SDA = 1;
    MPU6050_IIC_SCL = 1;

    /* Recover a slave that was reset in the middle of a transfer. */
    for (i = 0U; i < 9U; i++)
    {
        MPU6050_IIC_SCL = 0;
        MPU6050_IIC_DELAY();
        MPU6050_IIC_SCL = 1;
        MPU6050_IIC_DELAY();
    }
    MPU6050_IIC_Stop();
}

void MPU6050_IIC_SDA_IO_OUT(void)
{
    My_GPIO_Init(MPU6050_IIC_GPIO,
                 MPU6050_IIC_SDA_Pin,
                 GPIO_KL_OUT,
                 GPIO_P_NO,
                 GPIO_50MHz);
}

void MPU6050_IIC_SDA_IO_IN(void)
{
    My_GPIO_Init(MPU6050_IIC_GPIO,
                 MPU6050_IIC_SDA_Pin,
                 GPIO_FK_IN,
                 GPIO_P_UP,
                 GPIO_50MHz);
}

void MPU6050_IIC_Start(void)
{
    MPU6050_IIC_SDA_IO_OUT();
    MPU6050_IIC_SDA = 1;
    MPU6050_IIC_SCL = 1;
    MPU6050_IIC_DELAY();
    MPU6050_IIC_SDA = 0;
    MPU6050_IIC_DELAY();
    MPU6050_IIC_SCL = 0;
}

void MPU6050_IIC_Stop(void)
{
    MPU6050_IIC_SDA_IO_OUT();
    MPU6050_IIC_SCL = 0;
    MPU6050_IIC_SDA = 0;
    MPU6050_IIC_DELAY();
    MPU6050_IIC_SCL = 1;
    MPU6050_IIC_DELAY();
    MPU6050_IIC_SDA = 1;
    MPU6050_IIC_DELAY();
}

u8 MPU6050_IIC_Read_Ack(void)
{
    u16 timeout = 0U;

    MPU6050_IIC_SDA_IO_IN();
    MPU6050_IIC_DELAY();
    MPU6050_IIC_SCL = 1;
    MPU6050_IIC_DELAY();

    while (MPU6050_IIC_SDA_IN != 0U)
    {
        timeout++;
        if (timeout > 250U)
        {
            MPU6050_IIC_Stop();
            return 1U;
        }
        MPU6050_IIC_DELAY();
    }

    MPU6050_IIC_SCL = 0;
    return 0U;
}

void MPU6050_IIC_Send_Ack(u8 ack)
{
    MPU6050_IIC_SDA_IO_OUT();
    MPU6050_IIC_SCL = 0;
    MPU6050_IIC_SDA = (ack != 0U) ? 1U : 0U;
    MPU6050_IIC_DELAY();
    MPU6050_IIC_SCL = 1;
    MPU6050_IIC_DELAY();
    MPU6050_IIC_SCL = 0;
}

u8 MPU6050_IIC_Send_Byte(u8 txd)
{
    u8 bit;

    MPU6050_IIC_SDA_IO_OUT();
    MPU6050_IIC_SCL = 0;

    for (bit = 0U; bit < 8U; bit++)
    {
        MPU6050_IIC_SDA = ((txd & 0x80U) != 0U) ? 1U : 0U;
        txd <<= 1;
        MPU6050_IIC_DELAY();
        MPU6050_IIC_SCL = 1;
        MPU6050_IIC_DELAY();
        MPU6050_IIC_SCL = 0;
        MPU6050_IIC_DELAY();
    }

    return MPU6050_IIC_Read_Ack();
}

u8 MPU6050_IIC_Read_Byte(u8 ack)
{
    u8 bit;
    u8 value = 0U;

    MPU6050_IIC_SDA_IO_IN();

    for (bit = 0U; bit < 8U; bit++)
    {
        MPU6050_IIC_SCL = 0;
        MPU6050_IIC_DELAY();
        MPU6050_IIC_SCL = 1;
        value <<= 1;
        if (MPU6050_IIC_SDA_IN != 0U)
        {
            value++;
        }
        MPU6050_IIC_DELAY();
    }

    MPU6050_IIC_SCL = 0;
    MPU6050_IIC_DELAY();
    MPU6050_IIC_Send_Ack(ack);
    return value;
}
