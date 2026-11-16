/*********************************************************************************************************************
 * Name:pid.c
 *      ��̬����
 * Created on:
 * Author:
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <math.h>
#include <stdlib.h>
#include "pid.h"
#include "norch.h"
#include "small_driver_uart_control.h"

void PID_Init(PIDController *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0;
    pid->derivative = 0.0;
    pid->last_error = 0.0;
    pid->output = 0.0;
    pid->error = 0.0;
}

float PID_Update(PIDController *pid, float set_value, float real_value,float max_integral)
{

    pid->error = set_value - real_value;

    pid->integral += pid->error;

    if (pid->integral > max_integral)
     {
       pid->integral = max_integral;
     }
     else if (pid->integral < -max_integral)
     {
         pid->integral = -max_integral;
     }

    pid->derivative = (pid->error - pid->last_error);

    pid->last_error = pid->error;

    pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * pid->derivative;

    return pid->output;
}
float PID_Update_1(PIDController *pid, float set_value, float real_value,float max_integral,float ether)
{

    pid->error = set_value - real_value;

    pid->integral += pid->error;

    if (pid->integral > max_integral)
     {
       pid->integral = max_integral;
     }
     else if (pid->integral < -max_integral)
     {
         pid->integral = -max_integral;
     }

    pid->derivative = (pid->error - pid->last_error);

    pid->last_error = pid->error;

    pid->output = pid->kp * pid->error*ether + pid->ki * pid->integral*ether + pid->kd * pid->derivative*ether;

    return pid->output;
}
void FeedforwardPID_Init(FeedforwardPID *pid, float kp, float ki, float kd, float kff, float kffd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kff = kff;
    pid->kffd = kffd;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->last_error = 0.0f;
    pid->last_feedforward_value = 0.0f;
    pid->output = 0.0f;
    pid->error = 0.0f;
    pid->feedforward = 0.0f;
    pid->feedforward_d = 0.0f;
}
float FeedforwardPID_Update(FeedforwardPID *pid, float setpoint, float measured_value,
                            float feedforward_value, float max_integral)
{
    pid->error = setpoint - measured_value;

    pid->integral += pid->error;
    if (pid->integral > max_integral)       pid->integral = max_integral;
    else if (pid->integral < -max_integral) pid->integral = -max_integral;

    pid->derivative = pid->error - pid->last_error;
    pid->last_error = pid->error;

    // 直接前馈：目标值 × Kff（稳态映射）
    pid->feedforward = pid->kff * feedforward_value;

    // 微分前馈：目标值变化率 × Kffd（动态补偿）
    pid->feedforward_d = pid->kffd * (feedforward_value - pid->last_feedforward_value);
    pid->last_feedforward_value = feedforward_value;

    // 输出 = PID反馈 + 直接前馈 + 微分前馈（只算一次）
    pid->output = pid->kp * pid->error
                + pid->ki * pid->integral
                + pid->kd * pid->derivative
                + pid->feedforward
                + pid->feedforward_d;

    return pid->output;
}
