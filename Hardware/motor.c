#include "motor.h"

#include "encoder.h"
#include "pwm.h"
#include "stm32f10x.h"

#define SIN_60_F            0.8660254f
#define ONE_OVER_SQRT3_F    0.5773503f
#define ONE_THIRD_F         0.3333333f

float v1_jisuan;
float v2_jisuan;
float v3_jisuan;
float PWM1;
float PWM2;
float PWM3;

volatile float Vx_cal;
volatile float Vy_cal;
volatile float W_cal;

static int ClampMotorCommand(int pwm)
{
    if (pwm > MOTOR_PWM_OUTPUT_LIMIT) pwm = MOTOR_PWM_OUTPUT_LIMIT;
    if (pwm < -MOTOR_PWM_OUTPUT_LIMIT) pwm = -MOTOR_PWM_OUTPUT_LIMIT;

    if ((pwm > 0) && (pwm < MOTOR_PWM_MIN_EFFECTIVE))
    {
        pwm = MOTOR_PWM_MIN_EFFECTIVE;
    }
    else if ((pwm < 0) && (pwm > -MOTOR_PWM_MIN_EFFECTIVE))
    {
        pwm = -MOTOR_PWM_MIN_EFFECTIVE;
    }

    return pwm;
}

static float AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

void motor_Init(void)
{
    GPIO_InitTypeDef gpioInit;

    /* 72 MHz / (7199 + 1) = 10 kHz, with a normalized 0..1000 command. */
    PWM_Init(7199U, 0U);

    TIM2_encoder_Init();
    TIM3_encoder_Init();
    TIM4_encoder_Init();
    Encoder_ResetSpeedFilter();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    gpioInit.GPIO_Mode = GPIO_Mode_Out_PP;
    gpioInit.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpioInit);

    gpioInit.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_Init(GPIOC, &gpioInit);

    motor_StopAll();
}

void motor_Set1(int pwm)
{
    pwm = ClampMotorCommand(pwm);

    if (pwm > 0)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_15);
        GPIO_ResetBits(GPIOB, GPIO_Pin_14);
        PWM_SetCompare1((u16)pwm);
    }
    else if (pwm < 0)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_14);
        GPIO_ResetBits(GPIOB, GPIO_Pin_15);
        PWM_SetCompare1((u16)(-pwm));
    }
    else
    {
        PWM_SetCompare1(0U);
        GPIO_ResetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15);
    }
}

void motor_Set2(int pwm)
{
    pwm = ClampMotorCommand(pwm);

    if (pwm > 0)
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_11);
        GPIO_SetBits(GPIOC, GPIO_Pin_10);
        PWM_SetCompare2((u16)pwm);
    }
    else if (pwm < 0)
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_10);
        GPIO_SetBits(GPIOC, GPIO_Pin_11);
        PWM_SetCompare2((u16)(-pwm));
    }
    else
    {
        PWM_SetCompare2(0U);
        GPIO_ResetBits(GPIOC, GPIO_Pin_10 | GPIO_Pin_11);
    }
}

void motor_Set3(int pwm)
{
    pwm = ClampMotorCommand(pwm);

    if (pwm > 0)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_13);
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
        PWM_SetCompare3((u16)pwm);
    }
    else if (pwm < 0)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
        GPIO_ResetBits(GPIOB, GPIO_Pin_13);
        PWM_SetCompare3((u16)(-pwm));
    }
    else
    {
        PWM_SetCompare3(0U);
        GPIO_ResetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13);
    }
}

void motor_StopAll(void)
{
    motor_Set1(0);
    motor_Set2(0);
    motor_Set3(0);
}

void Speed_Target(float vxMmps, float vyMmps, float wzRadps)
{
    float maxMagnitude;
    float scale;

    /* Three-wheel omni kinematics; keep the user's chassis geometry. */
    v1_jisuan = 0.5f * vyMmps + SIN_60_F * vxMmps + wzRadps * CHASSIS_CENTER_TO_WHEEL_MM;
    v2_jisuan = 0.5f * vyMmps - SIN_60_F * vxMmps + wzRadps * CHASSIS_CENTER_TO_WHEEL_MM;
    v3_jisuan = -vyMmps + wzRadps * CHASSIS_CENTER_TO_WHEEL_MM;

    /* Scale all wheels together so the commanded chassis direction is preserved. */
    maxMagnitude = AbsFloat(v1_jisuan);
    if (AbsFloat(v2_jisuan) > maxMagnitude) maxMagnitude = AbsFloat(v2_jisuan);
    if (AbsFloat(v3_jisuan) > maxMagnitude) maxMagnitude = AbsFloat(v3_jisuan);

    if (maxMagnitude > MOTOR_WHEEL_SPEED_LIMIT_MMPS)
    {
        scale = MOTOR_WHEEL_SPEED_LIMIT_MMPS / maxMagnitude;
        v1_jisuan *= scale;
        v2_jisuan *= scale;
        v3_jisuan *= scale;
    }
}

void Speed_cal(float v1Actual, float v2Actual, float v3Actual)
{
    Vx_cal = (v1Actual - v2Actual) * ONE_OVER_SQRT3_F;
    Vy_cal = (v1Actual + v2Actual - 2.0f * v3Actual) * ONE_THIRD_F;
    W_cal = (v1Actual + v2Actual + v3Actual) *
            (ONE_THIRD_F / CHASSIS_CENTER_TO_WHEEL_MM);
}
