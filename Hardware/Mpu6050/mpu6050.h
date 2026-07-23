#ifndef _MPU6050_H_
#define _MPU6050_H_

#include "MPU6050_I2C.h"
#include "sys.h"

#define MPU_SAMPLE_RATE_REG     0x19
#define MPU_CFG_REG             0x1A
#define GYRO_CONFIG             0x1B
#define ACCEL_CONFIG            0x1C
#define MPU_FIFO_EN_REG         0x23
#define MPU_I2CMST_STA_REG      0x36
#define MPU_INTBP_CFG_REG       0x37
#define MPU_INT_EN_REG          0x38
#define MPU_INT_STA_REG         0x3A
#define ACCEL_XOUT_H            0x3B
#define ACCEL_XOUT_L            0x3C
#define ACCEL_YOUT_H            0x3D
#define ACCEL_YOUT_L            0x3E
#define ACCEL_ZOUT_H            0x3F
#define ACCEL_ZOUT_L            0x40
#define TEMP_OUT_H              0x41
#define TEMP_OUT_L              0x42
#define GYRO_XOUT_H             0x43
#define GYRO_XOUT_L             0x44
#define GYRO_YOUT_H             0x45
#define GYRO_YOUT_L             0x46
#define GYRO_ZOUT_H             0x47
#define GYRO_ZOUT_L             0x48
#define MPU_USER_CTRL_REG       0x6A
#define PWR_MGMT_1              0x6B
#define PWR_MGMT_2              0x6C
#define MPU_DEVICE_ID_REG       0x75
#define MPU_ADDR                0x68

u8 mpu6050_write(u8 addr, u8 reg, u8 len, u8 *buf);
u8 mpu6050_read(u8 addr, u8 reg, u8 len, u8 *buf);
void mpu6050_write_reg(u8 reg, u8 dat);
u8 mpu6050_read_reg(u8 reg);

/* Returns 0 when WHO_AM_I and configuration are valid. */
u8 MPU6050_Init(void);
u8 MPU_Set_Gyro_Fsr(u8 fsr);
u8 MPU_Set_Accel_Fsr(u8 fsr);
u8 MPU_Set_LPF(u16 lpf);
u8 MPU_Set_Rate(u16 rate);

short MPU_Get_Temperature(void);
u8 MPU_Get_Gyroscope(short *gx, short *gy, short *gz);
u8 MPU_Get_Accelerometer(short *ax, short *ay, short *az);

/* One 14-byte burst keeps accel, temperature and gyro from the same sample. */
u8 MPU_Get_Raw6Axis(short *ax,
                    short *ay,
                    short *az,
                    short *temperatureRaw,
                    short *gx,
                    short *gy,
                    short *gz);

#endif
