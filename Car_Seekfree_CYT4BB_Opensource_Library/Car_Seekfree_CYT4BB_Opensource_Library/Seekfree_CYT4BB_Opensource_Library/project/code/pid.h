/*********************************************************************************************************************
 * Name:pid.h
 *  
 * Created on:
 * Author:
********************************************************************************************************************/

#ifndef _PID_H_
#define _PID_H_

typedef struct {
    float kp;      
    float ki;       
    float kd;      
    float integral; 
    float derivative;
    float last_error; 
    float last_tager;
    float output;  
    float error;
} PIDController;

typedef struct {
    float kp, ki, kd, kff;              // PID 系数 + 前馈系数
    float kffd;                          // 微分前馈系数（目标值变化率 → 输出）
    float integral, derivative, last_error;
    float last_feedforward_value;        // 上一次前馈目标值（用于算微分前馈）
    float output, error;
    float feedforward;                   // 直接前馈分量（调试用）
    float feedforward_d;                 // 微分前馈分量（调试用）
} FeedforwardPID;

// 普通 PID
void  PID_Init(PIDController *pid, float kp, float ki, float kd);
float PID_Update_1(PIDController *pid, float set_value, float real_value,float max_integral,float ether);
float PID_Update(PIDController *pid, float setpoint, float measured_value, float max_integral);

// 前馈式 PID（直接前馈 + 微分前馈）
void  FeedforwardPID_Init(FeedforwardPID *pid, float kp, float ki, float kd, float kff, float kffd);
float FeedforwardPID_Update(FeedforwardPID *pid, float setpoint, float measured_value,
                            float feedforward_value, float max_integral);

#endif /* CODE_ATTITUDE_CONTROL_H_ */
