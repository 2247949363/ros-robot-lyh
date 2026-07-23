#include "mpu6050.h"
#include "delay.h"

u8 mpu6050_write(u8 addr, u8 reg, u8 len, u8 *buf)
{
    u8 i;
    u8 busAddress = (u8)(addr << 1);

    if ((buf == 0) || (len == 0U)) return 1U;

    MPU6050_IIC_Start();
    if (MPU6050_IIC_Send_Byte(busAddress) != 0U) return 1U;
    if (MPU6050_IIC_Send_Byte(reg) != 0U) return 1U;

    for (i = 0U; i < len; i++)
    {
        if (MPU6050_IIC_Send_Byte(buf[i]) != 0U) return 1U;
    }

    MPU6050_IIC_Stop();
    return 0U;
}

u8 mpu6050_read(u8 addr, u8 reg, u8 len, u8 *buf)
{
    u8 i;
    u8 busAddress = (u8)(addr << 1);

    if ((buf == 0) || (len == 0U)) return 1U;

    MPU6050_IIC_Start();
    if (MPU6050_IIC_Send_Byte(busAddress) != 0U) return 1U;
    if (MPU6050_IIC_Send_Byte(reg) != 0U) return 1U;

    MPU6050_IIC_Start();
    if (MPU6050_IIC_Send_Byte((u8)(busAddress + 1U)) != 0U) return 1U;

    for (i = 0U; i < (u8)(len - 1U); i++)
    {
        buf[i] = MPU6050_IIC_Read_Byte(0U);
    }
    buf[len - 1U] = MPU6050_IIC_Read_Byte(1U);
    MPU6050_IIC_Stop();
    return 0U;
}

void mpu6050_write_reg(u8 reg, u8 dat)
{
    (void)mpu6050_write(MPU_ADDR, reg, 1U, &dat);
}

u8 mpu6050_read_reg(u8 reg)
{
    u8 dat = 0xFFU;
    (void)mpu6050_read(MPU_ADDR, reg, 1U, &dat);
    return dat;
}

u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
    u8 value = (u8)(fsr << 3);
    return mpu6050_write(MPU_ADDR, GYRO_CONFIG, 1U, &value);
}

u8 MPU_Set_Accel_Fsr(u8 fsr)
{
    u8 value = (u8)(fsr << 3);
    return mpu6050_write(MPU_ADDR, ACCEL_CONFIG, 1U, &value);
}

u8 MPU_Set_LPF(u16 lpf)
{
    u8 data;

    if (lpf >= 188U) data = 1U;
    else if (lpf >= 98U) data = 2U;
    else if (lpf >= 42U) data = 3U;
    else if (lpf >= 20U) data = 4U;
    else if (lpf >= 10U) data = 5U;
    else data = 6U;

    return mpu6050_write(MPU_ADDR, MPU_CFG_REG, 1U, &data);
}

u8 MPU_Set_Rate(u16 rate)
{
    u8 divider;

    if (rate > 1000U) rate = 1000U;
    if (rate < 4U) rate = 4U;
    divider = (u8)(1000U / rate - 1U);

    if (mpu6050_write(MPU_ADDR, MPU_SAMPLE_RATE_REG, 1U, &divider) != 0U)
    {
        return 1U;
    }
    return MPU_Set_LPF((u16)(rate / 2U));
}

u8 MPU6050_Init(void)
{
    u8 value;

    MPU6050_IIC_IO_Init();

    value = 0x80U;
    if (mpu6050_write(MPU_ADDR, PWR_MGMT_1, 1U, &value) != 0U) return 1U;
    delay_ms(100U);

    value = 0x00U;
    if (mpu6050_write(MPU_ADDR, PWR_MGMT_1, 1U, &value) != 0U) return 1U;

    /* Match the reference lower controller: +/-500 dps, +/-4 g, 250 Hz. */
    if (MPU_Set_Gyro_Fsr(1U) != 0U) return 1U;
    if (MPU_Set_Accel_Fsr(1U) != 0U) return 1U;
    if (MPU_Set_Rate(250U) != 0U) return 1U;

    value = 0x00U;
    if (mpu6050_write(MPU_ADDR, MPU_INT_EN_REG, 1U, &value) != 0U) return 1U;
    if (mpu6050_write(MPU_ADDR, MPU_USER_CTRL_REG, 1U, &value) != 0U) return 1U;
    if (mpu6050_write(MPU_ADDR, MPU_FIFO_EN_REG, 1U, &value) != 0U) return 1U;
    if (mpu6050_write(MPU_ADDR, MPU_INTBP_CFG_REG, 1U, &value) != 0U) return 1U;

    if (mpu6050_read_reg(MPU_DEVICE_ID_REG) != MPU_ADDR) return 1U;

    value = 0x01U;
    if (mpu6050_write(MPU_ADDR, PWR_MGMT_1, 1U, &value) != 0U) return 1U;
    value = 0x00U;
    if (mpu6050_write(MPU_ADDR, PWR_MGMT_2, 1U, &value) != 0U) return 1U;
    return 0U;
}

short MPU_Get_Temperature(void)
{
    u8 buf[2];
    short raw;

    if (mpu6050_read(MPU_ADDR, TEMP_OUT_H, 2U, buf) != 0U) return 0;
    raw = (short)(((u16)buf[0] << 8) | buf[1]);
    return (short)(3653L + ((long)raw * 100L) / 340L);
}

u8 MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
    u8 buf[6];
    u8 result = mpu6050_read(MPU_ADDR, GYRO_XOUT_H, 6U, buf);

    if (result == 0U)
    {
        *gx = (short)(((u16)buf[0] << 8) | buf[1]);
        *gy = (short)(((u16)buf[2] << 8) | buf[3]);
        *gz = (short)(((u16)buf[4] << 8) | buf[5]);
    }
    return result;
}

u8 MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
    u8 buf[6];
    u8 result = mpu6050_read(MPU_ADDR, ACCEL_XOUT_H, 6U, buf);

    if (result == 0U)
    {
        *ax = (short)(((u16)buf[0] << 8) | buf[1]);
        *ay = (short)(((u16)buf[2] << 8) | buf[3]);
        *az = (short)(((u16)buf[4] << 8) | buf[5]);
    }
    return result;
}

u8 MPU_Get_Raw6Axis(short *ax,
                    short *ay,
                    short *az,
                    short *temperatureRaw,
                    short *gx,
                    short *gy,
                    short *gz)
{
    u8 buf[14];
    u8 result = mpu6050_read(MPU_ADDR, ACCEL_XOUT_H, 14U, buf);

    if (result == 0U)
    {
        *ax = (short)(((u16)buf[0] << 8) | buf[1]);
        *ay = (short)(((u16)buf[2] << 8) | buf[3]);
        *az = (short)(((u16)buf[4] << 8) | buf[5]);
        *temperatureRaw = (short)(((u16)buf[6] << 8) | buf[7]);
        *gx = (short)(((u16)buf[8] << 8) | buf[9]);
        *gy = (short)(((u16)buf[10] << 8) | buf[11]);
        *gz = (short)(((u16)buf[12] << 8) | buf[13]);
    }
    return result;
}
