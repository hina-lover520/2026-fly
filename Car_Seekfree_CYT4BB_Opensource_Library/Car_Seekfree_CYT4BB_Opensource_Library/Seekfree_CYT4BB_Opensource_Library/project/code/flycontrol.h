/*********************************************************************************************************************
 * Name:flycontrol.h
 *  
 * Created on:
 * Author:Lsl
********************************************************************************************************************/

#ifndef _FLY_H_
#define _FLY_H_

extern PIDController hight;
extern PIDController Roll;
extern FeedforwardPID Roll_v;
extern PIDController Yaw;
extern FeedforwardPID Yaw_v;
extern PIDController Pitch;
extern FeedforwardPID Pitch_v;
extern PIDController XX;
extern PIDController YY;
extern PIDController PMW_X;
extern PIDController PMW_Y;
extern FeedforwardPID hight_v;
extern PIDController Car_x;
extern PIDController Car_y;
extern int my_flag;
extern float want_win_x;
extern float want_win_y;
extern int32 hhh;
extern float hhhh;
extern float yyyy;
extern float yy_v;
extern float rrrr;
extern float rr_v;
extern float pppp;
extern float pp_v;
extern float offest_x;
extern float offest_y;
extern float pmw_xx;
extern float pmw_yy;
extern float target_xx;
extern float target_yy;
extern float highter;//�߶�
extern int highter_1;
extern float Lhighter;
extern float Vhight;
extern float PPXX;
extern float PPYY;
extern  int magic_count;
extern int m_ppyy;
extern int m_ppxx;
extern int PPYY_S;
extern int PPXX_S;
extern float lo_roll;
extern float lo_pitch;
extern float lo_yaw;
extern float lo_hight;
extern int32 yyy;
extern int left_2;
extern int jiangluo;
extern int jiangluo;
extern int hv_count;
extern int deng_coumt;
extern uint8 buf[104];

extern uint32 M1;
extern uint32 M2;
extern uint32 M3;
extern uint32 M4;

extern int32 rrr;

extern int32 ppp;


extern  float  ufx;

void PID_UPdate (void);
void pid_Init_all(float hight_p,float hight_i,float hight_d,float Roll_p,float Roll_i,float Roll_d,float Yaw_p,float Yaw_i,float Yaw_d,float Pitch_p,float Pitch_i,float Pitch_d,float XX_p,
                  float XX_i,float XX_d,float YY_p,float YY_i,float YY_d,float PMW_X_p,float PMW_X_i,float PMW_X_d,float PMW_Y_p,float PMW_Y_i,float PMW_Y_d);
float get_Thight(float hh);
void fly_star (void);
void PID_UPdate_1 (void);
void PID_UPdate_2(void);
#endif
