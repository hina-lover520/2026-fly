/*********************************************************************************************************************
 * Name:flycontrol.c
 *      飞控
 * Created on:
 * Author:  田深深深深深深
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <math.h>
#include <stdlib.h>
#include "pid.h"
#include "ztjx.h"
#include "matrix.h"
#include "kaerman.h"
#include "quaternion.h"
#include "norch.h"
#include "small_driver_uart_control.h"
#include "flycontrol.h"

#define MAX_DUTY                (30)                    // 最大 测试 占空比 30%
#define MIN_DUTY                (10)                    // 最小 测试 占空比 10%


#define PMW_CHANNEL_1           (TCPWM_CH19_P08_0)      // pwm输出通道1                                 
#define PMW_CHANNEL_2           (TCPWM_CH20_P08_1)      // pwm输出通道2                             
#define PMW_CHANNEL_3           (TCPWM_CH21_P08_2)      // pwm输出通道3                                                                   
#define PMW_CHANNEL_4           (TCPWM_CH22_P08_3)      // pwm输出通道4


PIDController hight;
PIDController hight_v;
PIDController Roll;
PIDController Roll_v;
PIDController Yaw;
PIDController Yaw_v;
PIDController Pitch;
PIDController Pitch_v;
PIDController XX;
PIDController YY;
PIDController PMW_X;
PIDController PMW_Y;
PIDController Car_x;
PIDController Car_y;
int my_flag=0;
int32  hhh=0;
float hhhh=0;
float yyyy=0;
float yy_v=0;
float rrrr=0;
float rr_v=0;
float pppp=0;
float pp_v=0;
float offest_x=0;
float offest_y=0;
float pmw_xx=0;
float pmw_yy=0;
float want_win_x=0;
float want_win_y=0;
float target_xx=0;
float target_yy=0;
float highter=0;//高度
int highter_1=0;
float Lhighter=0;
float Vhight;
float PPXX=0;
float PPYY=0;
int m_ppyy=0;
int m_ppxx=0;
int PPYY_S=0;
int PPXX_S=0;
static int32 star_count=15000;

uint32 M1=0;
uint32 M2=0;
uint32 M3=0;
uint32 M4=0;

float lo_roll=0;
float lo_pitch=0;
float lo_yaw=0;
float lo_hight=0;

float ufx =0;
int left_2=1;
int jiangluo=0;
int deng_coumt=0;
int magic_count = 0;
int jiangluo_count=0;
int hv_count=0;
int32 yyy = 0;

int32 rrr = 0;

int32 ppp = 0;

uint8 buf[104];
void pid_Init_all(float hight_p,float hight_i,float hight_d,float Roll_p,float Roll_i,float Roll_d,float Yaw_p,float Yaw_i,float Yaw_d,float Pitch_p,float Pitch_i,float Pitch_d,float XX_p,
                  float XX_i,float XX_d,float YY_p,float YY_i,float YY_d,float PMW_X_p,float PMW_X_i,float PMW_X_d,float PMW_Y_p,float PMW_Y_i,float PMW_Y_d)
{
  PID_Init(&hight,hight_p,hight_i,hight_d);
  PID_Init(&Roll,Roll_p,Roll_i,Roll_d);
  PID_Init(&Yaw,Yaw_p,Yaw_i,Yaw_d);
  PID_Init(&Pitch,Pitch_p,Pitch_i,Pitch_d);


  /*PID_Init(&Roll_v,0.6,0.0,0.42);          // 0.9,0.0,0.8
  PID_Init(&Pitch_v,0.62,0.0,0.47);        //  0.9,0.0,0.8
  PID_Init(&Yaw_v,3.0,0.0,0.35);         // 0 0 0    //3.0 0.0 0.35
  */
  
  PID_Init(&Roll_v,0.9,0.0,0.9);          // 0.9,0.0,0.8
  PID_Init(&Pitch_v,0.9,0.0,0.9);        //  0.9,0.0,0.8
  PID_Init(&Yaw_v,3.0,0.0,0.35);         // 0 0 0    //3.0 0.0 0.35
  //PID_Init(&hight_v,150.5,0.0,15.0);       //12.0   0.0  0.15
  //PID_Init(&hight_v,2.5,0.0,0.25);//150
  
  PID_Init(&XX,XX_p,XX_i,XX_d);
  PID_Init(&YY,YY_p,YY_i,YY_d);
  PID_Init(&PMW_X,PMW_X_p,PMW_X_i,PMW_X_d);
  PID_Init(&PMW_Y,PMW_Y_p,PMW_Y_i,PMW_Y_d);
  PID_Init(&Car_x,0.0,0.0,0.0);
  PID_Init(&Car_y,0.0,0.0,0.0);

  pwm_init(PMW_CHANNEL_1, 400, 0);                    // 初始化pwm引脚功能  默认400hz  0%占空比
  pwm_init(PMW_CHANNEL_2, 400, 0);
  pwm_init(PMW_CHANNEL_3, 400, 0);
  pwm_init(PMW_CHANNEL_4, 400, 0);

}

void PID_UPdate_2(void)///////////信标灯
{
    float bb=pmw_xx;
  float cc=pmw_yy;
  if(bb>4.5)
  {
    bb=4.5;
  }
  if(bb<-4.5)
  {
    bb=-4.5;
  }
  if(cc>4.5)
  {
    cc=4.5;
  }
  if(cc<-4.5)
  {
    cc=-4.5;
  }
      //printf("%f,%f\n",pmw_xx,pmw_yy);
  
    yyyy= -PID_Update(&Yaw,0.0+lo_yaw,s_angle.agz,100);//电机要的 s_angle.agz+lo_yaw
    rrrr= -PID_Update(&Roll,0.0+lo_roll+bb,s_angle.agx+6.75f,100); //s_angle.agx-2.8+lo_roll
    pppp= PID_Update(&Pitch,0.0+lo_pitch+cc,s_angle.agy+1.58f-0.1f,100);//s_angle.agy+1.5+lo_pitch
    //printf("%f,%f,%f\n",rrrr,pppp,yyyy);
    //printf("%d\n",dl1b_distance_mm);
deng_coumt++;
if(deng_coumt>=1){
  deng_coumt=0;
  if(my_flag>=1&&my_flag<=7)
  {
    want_win_x = PID_Update(&XX,94.0,target_xx,100);
    want_win_y = PID_Update(&YY,60.0,target_yy,100);
    //printf("%f,%f\n",want_win_x,want_win_y);
  }
}
}

void PID_UPdate_1 (void)///////////////////光流
{
    
    //pmw_xx=PID_Update(&PMW_X,0+target_xx,PPXX+0.5f,100);//光流PPXX+0.25f
    //pmw_yy=PID_Update(&PMW_Y,0+target_yy,PPYY+0.5f,100);   // mubiaozhi  //shijizhi
    //printf("%f,%f\n",lo_roll,lo_pitch);
    pmw_xx=PID_Update(&PMW_X,0+want_win_y,PPXX+0.25,50);//光流PPXX+0.25f
    pmw_yy=PID_Update(&PMW_Y,0+want_win_x,PPYY+0.5,50);   // mubiaozhi  //shijizhi
    //printf("%f,%f\n",PPXX+0.25f,PPYY+0.6f);
}

float get_Thight(float hh)
{
    float roll  = (s_angle.agx + 5.1f) ;
    float pitch = (s_angle.agy + 0.08f - 0.1f) ;     
    
    return (float)hh * cosf(roll) * cosf(pitch);      //高度
}
void PID_UPdate (void)/////////////////////高度
{   
  if(highter<1500)
    {
      highter_1=(int)highter;
      hhhh= PID_Update(&hight,900.0+lo_hight,dl1b_distance_mm,200.0);  //700
      //printf("hhhh:%f\n",hhhh);
    }//TOF 
    if(highter>=1500)
    {
    highter=1500;
    hhhh= PID_Update(&hight,900.0,1500,100.0);
    }
    //printf("%d\n",highter_1);
    //printf("hhhh:%f\n",hhhh);
     if (hhhh>=250)  hhhh=250;
     if (hhhh<=-250) hhhh=-250;
}

void fly_star (void)
{   
   magic_count++;
   jiangluo_count++;
   if(magic_count>=2){
     magic_count=0;
     //printf("%f,%f,%f\n",rrrr,pppp,yyyy);
     //printf("%f,%f,%d\n",rrrr,imu660rb_gyro_x_l,rrr);
    float rv = PID_Update(&Roll_v,0+rrrr,imu660rb_gyro_x_l-1.0,100);//imu660rc_gyro_x_2
    float pv = PID_Update(&Pitch_v,0+pppp,imu660rb_gyro_y_l+5.0,100);
    float yv = PID_Update(&Yaw_v,0+yyyy,imu660rb_gyro_z_l+4.0,100);
    
    //float hv = PID_Update(&hight_v,0+hhhh,Vhight,100);
    
    yyy = (int32)-yv;

    rrr = (int32)-rv;

    ppp = (int32)pv;
    //printf("%d\n",dl1b_distance_mm);

    //printf("%f\n",hv);
    //hhh = (int32)hv;
    hhh = (int32)hhhh;
   }
    /*int32 hhh1 = -hhh;
    int32 hhh2 = -hhh1;*/
    
    //printf("%d,%d,%d\n",yyy,rrr,ppp);
    
   // printf("%d,\n",yyy);
   //}
   //printf("%d,%d,%d,%d\n",rrr-1,ppp,yyy,hhh);
   if(jiangluo_count>=200){
   if(left_2==0){
   jiangluo+=1;
   }
   }
    if (star_count>0)
    {
    star_count-=2;
    }
    //printf("hhh:%d\n",hhh);
    int m11 = 5800+hhh+(rrr-1)+ppp-yyy-jiangluo-star_count;
    int m22 = 5800+hhh+(rrr-1)-ppp+yyy-jiangluo-star_count;
    int m33 = 5800+hhh-(rrr-1)+ppp+yyy-jiangluo-star_count;
    int m44 = 5800+hhh-(rrr-1)-ppp-yyy-jiangluo-star_count;
    /*int m11 = 5580+hhh+(rrr-10)+(ppp-30)-yyy-jiangluo-star_count;
    int m22 = 5580+hhh+(rrr-10)*1.05-(ppp-30)+yyy-jiangluo-star_count;
    int m33 = 5580+hhh-(rrr-10)*0.95+(ppp-30)+yyy-jiangluo-star_count;
    int m44 = 5580+hhh-(rrr-10)-(ppp-30)-yyy-jiangluo-star_count;*/
    //printf("%d,%d\n",rrr,ppp-2);
    if(m11<=0)
    {
    m11=0;
    }
    if(m22<=0)
    {
    m22=0;
    }
    if(m33<=0)
    {
    m33=0;
    }
    if(m44<=0)
    {
    m44=0;
    }
    M1=m11;      //4750
    M2=m22;
    M3=m33;     //yvan ban
    M4=m44;
   
    /*uint32 M1 = 5300+hhh+rrr+ppp -yyy;
    uint32 M2 = 5300+hhh+rrr-ppp +yyy; 
    uint32 M3 = 5300+hhh-rrr+ppp +yyy;
    uint32 M4 = 5300+hhh-rrr-ppp -yyy;*/
   

    // printf("ppp:%d\n",ppp);
    // printf("rrr:%d\n",rrr);
    if(M1>7500) M1=7500;
    if(M2>7500) M2=7500;
    if(M3>7500) M3=7500;
    if(M4>7500) M4=7500;

   
    /*if(M1<4600) M1=4600;
    if(M2<4600) M2=4600;
    if(M3<4600) M3=4600;
    if(M4<4600) M4=4600;*/
    
    //printf("%d,%d,%d,%d\n",M1,M2,M3,M4);
    small_driver_set_duty(M1,M2,M3,M4);
    //printf("%d,%d ,%d,%d\n" ,M1,M2,M3,M4 );
    //pwm_set_duty(PMW_CHANNEL_1,M1);//+rrr +ppp -yyy                      // 输出占空比数值  输出值 = 占空比设置值 + 1ms固定占空比
    //pwm_set_duty(PMW_CHANNEL_2,M2);//+rrr -ppp +yyy
    //pwm_set_duty(PMW_CHANNEL_3,M3);//-rrr +ppp +yyy
    //pwm_set_duty(PMW_CHANNEL_4,M4);//-rrr -ppp-yyy
}
