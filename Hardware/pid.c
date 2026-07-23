#include "pid.h"

static float AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float ClampFloat(float value, float minimum, float maximum)
{
    if (value > maximum) return maximum;
    if (value < minimum) return minimum;
    return value;
}

void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->max_Integral = maxI;
    pid->maxOutput = maxOut;
    pid->integral_separation = 50.0f;
    pid->error_deadzone = 3.0f;
    PID_Reset(pid);
}

void PID_SetProtection(PID *pid, float integralSeparation, float errorDeadzone)
{
    pid->integral_separation = integralSeparation;
    pid->error_deadzone = errorDeadzone;
}

void PID_Reset(PID *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

float PID_Calc(PID *pid, float reference, float feedback, float dt)
{
    float previousIntegral;
    float derivative;
    float unclampedOutput;
    unsigned char integratedThisCycle = 0U;

    if (dt <= 0.0f)
    {
        pid->output = 0.0f;
        return pid->output;
    }

    pid->error = reference - feedback;

    /* Avoid hunting around zero and discard any stale integral bias. */
    if (AbsFloat(pid->error) <= pid->error_deadzone)
    {
        pid->integral = 0.0f;
        pid->last_error = pid->error;
        pid->output = 0.0f;
        return pid->output;
    }

    previousIntegral = pid->integral;

    /* Only integrate near the setpoint; large errors are handled by P + feedforward. */
    if (AbsFloat(pid->error) <= pid->integral_separation)
    {
        pid->integral += pid->error * dt;
        pid->integral = ClampFloat(pid->integral,
                                   -pid->max_Integral,
                                   pid->max_Integral);
        integratedThisCycle = 1U;
    }
    else
    {
        pid->integral = 0.0f;
    }

    derivative = (pid->error - pid->last_error) / dt;
    unclampedOutput = pid->kp * pid->error +
                      pid->ki * pid->integral +
                      pid->kd * derivative;

    pid->output = ClampFloat(unclampedOutput, -pid->maxOutput, pid->maxOutput);

    /* Conditional integration: do not accumulate further into saturation. */
    if ((integratedThisCycle != 0U) && (pid->output != unclampedOutput))
    {
        pid->integral = previousIntegral;
    }

    pid->last_error = pid->error;
    return pid->output;
}
