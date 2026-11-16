/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cm7_1_isr
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-9      pudding            first version
* 2024-5-14     pudding            新增12个pit周期中断 增加部分注释说明
* 2025-2-4      pudding            优化串口中断逻辑，防止意外干扰导致的卡死问题，优化串口波特率计算逻辑
* 2025-2-4      pudding            新增两个串口接口
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "zf_device_upflow302.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "pid.h"
#include "ztjx.h"
#include "matrix.h"
#include "flycontrol.h"
#include "kaerman.h"
#include "quaternion.h"
#include "norch.h"
#include "small_driver_uart_control.h"
#include "small_driver_uart_control_2.h"
#include "zf_device_lora3a22.h"
#define ACC_SCALE  4098.0f
#define PIT_NUM                 (PIT_CH0 )                              // 使用的周期中断编号
#define PIT_period              (10 )                                   // 中断周期
int want_count=0;
int lzx;
// **************************** PIT中断函数 ****************************
void pit0_ch0_isr()                     // 定时器通道 0 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH0);//2
    
    //EKF_UpData();
    //imurc_get_values();
    //imurb_get_values();
    //Rc_Get();
    //highter = Get_Hight(&tof , dl1b_distance_mm);
    //printf("%d\n",highter_1);
    //ycl();
    //xacc =imu_data_acc_x;
    //yacc =imu_data_acc_y;
    //PPXX = kaermanb_Updata(&guangliu_x,pmw3901_delta_x,yacc);
    //PPYY =kaermanb_Updata(&guangliu_y,pmw3901_delta_y,xacc);
   // printf("%d,%f\n",pmw3901_delta_y,xacc);
    imu660rb_get_acc();
    imu660rb_get_gyro();
    imurb_get_values();
    imu660rb_gyro_x_l=kaermanb_Updata_1(&gyro_x,imu660rb_gyro_x_l);
    imu660rb_gyro_y_l=kaermanb_Updata_1(&gyro_y,imu660rb_gyro_y_l);
    my_quaternion(&sc,&s_angle,&s_shu);
    //sprintf(buf,"%f",imu660rb_gyro_x_l);    
    //printf("%f,%f,%f\n",imu660rb_gyro_x_l-1.0,imu660rb_gyro_y_l+4.0,imu660rb_gyro_y_l+4.0);

}

void pit0_ch1_isr()                     // 定时器通道 1 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH1);//1
    //printf("%f,%f,%f\n",s_angle.agx+5.45f,s_angle.agy+1.58f-0.1+0.2f,s_angle.agz);
    //printf("%d\n",1);//hhh
    //printf("%f\n",lo_roll)
   // printf("1");
    
    fly_star();
    
    //sprintf(buf,"%f",s_angle.agz);
    //lzx=wifi_spi_send_buffer(buf, sizeof(buf));

    //no+=0.025;
} 

void pit0_ch2_isr()                     // 定时器通道 2 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH2);//10
    
    //imurb_get_values();
    xacc =imu_data_acc_x*750;
    yacc =imu_data_acc_y*750;
    //ufx = upflow302_receive.upflow302_x *0.25;
    //printf("%f,%f\n",yacc,xacc);
    //printf("%d,%d\n",pmw3901_delta_x,pmw3901_delta_y);
    PPXX = kaermanb_Updata(&guangliu_x,-pmw3901_delta_y*0.25,(yacc+57)*1.25);
    PPYY = kaermanb_Updata(&guangliu_y,-pmw3901_delta_x*0.25,(xacc-2)*1.25);
    m_ppxx = (int)PPXX;
    m_ppyy = (int)PPYY;
    //PPYY = kaermanb_Updata(&guangliu_x,-upflow302_receive.upflow302_x,xacc+8);
    //PPXX = kaermanb_Updata(&guangliu_y,-upflow302_receive.upflow302_y,yacc+63);
    PPXX_S += m_ppxx;
    PPYY_S += m_ppyy;
    
    highter = kaermanb_Updata_1(&my_hight,dl1b_distance_mm);
        //printf("%f\n",Lhighter-highter);
    Vhight=kaermanb_Updata_1(&hv,(Lhighter-highter));
    //printf("%f,%f,%f\n",Lhighter-highter,Lhighter,highter);
    Lhighter=highter;


    //float Thight = get_Thight(highter);

    //printf("PWM3901:%f,%f\n",PPXX+0.25,pmw3901_delta_x*0.75);
    //printf("%f,%f\n",PPXX,PPYY);
    //printf("%f\n",highter);
    //printf("%f,%f,%d,%d\n",xacc+8,yacc+63,upflow302_receive.upflow302_x, upflow302_receive.upflow302_y);
    //printf("%f,%d\n",xacc+8,upflow302_receive.upflow302_x);
    //printf("%f,%f\n",xacc+8,ufx);
    //printf("%f,%d\n",yacc+63,upflow302_receive.upflow302_y);
    PID_UPdate_1 ();
}

void pit0_ch10_isr()                    // 定时器通道 10 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH10);//10
    //want_count++;
    /*if(want_count>=3)
    {
      want_count=0;
      PID_UPdate_2();
    }*/
    dl1b_get_distance();
    PID_UPdate ();
    //want_count++;
    //printf("%d\n",dl1b_distance_mm);
}

void pit0_ch11_isr()                    // 定时器通道 11 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH11);//30
    pmw3901_get_motion();
    PID_UPdate_2();
}

void pit0_ch12_isr()                    // 定时器通道 12 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH12);
    lora3a22_response_time++;
    
    if (lora3a22_response_time > 500 / PIT_period)   //500ms 没有接受倒数据判断位发送端异常
    {
        lora3a22_state_flag = 0;                     //遥控器状态位清零
        
        lora3a22_response_time = 0;
    }
}

void pit0_ch13_isr()                    // 定时器通道 13 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH13);
          //if(!data_length_1)
          //{
          //    printf("\r\n send success.");
          //}
          //else
         // {
              //printf("\r\n %d bytes data send failed.", data_length_1);
         // }

}

void pit0_ch14_isr()                    // 定时器通道 14 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH14);
    
}

void pit0_ch15_isr()                    // 定时器通道 15 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH15);
    
}

void pit0_ch16_isr()                    // 定时器通道 16 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH16);
    
}

void pit0_ch17_isr()                    // 定时器通道 17 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH17);
    
}

void pit0_ch18_isr()                    // 定时器通道 18 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH18);
    
}

void pit0_ch19_isr()                    // 定时器通道 19 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH19);
    
}

void pit0_ch20_isr()                    // 定时器通道 20 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH20);
    
}

void pit0_ch21_isr()                    // 定时器通道 21 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH21);
    tsl1401_collect_pit_handler();
}
// **************************** PIT中断函数 ****************************

void lora_communication_receive(void);
void uart_rx_interrupt_handler (void);

// **************************** 串口中断函数 ****************************
// 串口0默认作为调试串口
void uart0_isr (void)
{
    if(uart_isr_mask(UART_0))            // 串口0接收中断
    {
        
#if DEBUG_UART_USE_INTERRUPT             // 如果开启 debug 串口中断
        debug_interrupr_handler();       // 调用 debug 串口接收处理函数 数据会被 debug 环形缓冲区读取
#endif                                   // 如果修改了 DEBUG_UART_INDEX 那这段代码需要放到对应的串口中断去
      
    }
    else                                 // 串口0发送中断
    {           
        
        
        
    }
}

void uart1_isr (void)
{
    if(uart_isr_mask(UART_1))            // 串口1接收中断
    {
        
        wireless_module_uart_handler();
        //uart_receiver_callback();
        //lora_communication_receive();
    }
    else                                // 串口1发送中断
    {
      
        
        
    }
}

void uart2_isr (void)
{
    if(uart_isr_mask(UART_2))            // 串口2接收中断
    {
        
        //gnss_uart_callback();
        //upflow302_receive_callback();
    }
    else                                // 串口2发送中断
    {
        
        
       
    }
}

void uart3_isr (void)
{
    if(uart_isr_mask(UART_3))            // 串口3接收中断
    {
        
        
        
    }
    else                                // 串口3发送中断
    {
      
        
        
    }
}

void uart4_isr (void)
{
    if(uart_isr_mask(UART_4))            // 串口4接收中断
    {

        uart_receiver_handler();                                                                // 串口接收机回调函数
       
    }
    else                                // 串口4发送中断
    {
      
        
        
    }
}

void uart5_isr (void)
{
    if(uart_isr_mask(UART_5))            // 串口5接收中断
    {
        
        
       
    }
    else                                // 串口5发送中断
    {
      
        
        
    }
}

void uart6_isr (void)
{
    if(uart_isr_mask(UART_6))            // 串口6接收中断
    {

        
       
    }
    else                                // 串口6发送中断
    {
      
        
        
    }
}
// **************************** 串口中断函数 ****************************

// **************************** 外部中断函数 ****************************
void gpio_0_exti_isr()                  // 外部 GPIO_0 中断服务函数     
{
    
  
  
}

void gpio_1_exti_isr()                  // 外部 GPIO_1 中断服务函数     
{
    if(exti_flag_get(P01_0))		// 示例P1_0端口外部中断判断
    {

      
      
            
    }
    if(exti_flag_get(P01_1))
    {

            
            
    }
}

void gpio_2_exti_isr()                  // 外部 GPIO_2 中断服务函数     
{
    if(exti_flag_get(P02_0))
    {
            
            
    }
    if(exti_flag_get(P02_4))
    {
            
            
    }

}

void gpio_3_exti_isr()                  // 外部 GPIO_3 中断服务函数     
{



}

void gpio_4_exti_isr()                  // 外部 GPIO_4 中断服务函数     
{



}

void gpio_5_exti_isr()                  // 外部 GPIO_5 中断服务函数     
{



}


void gpio_6_exti_isr()                  // 外部 GPIO_6 中断服务函数     
{
    if(exti_flag_get(P06_7))
    {

        imu660rc_callback();
    }
}

void gpio_7_exti_isr()                  // 外部 GPIO_7 中断服务函数     
{



}

void gpio_8_exti_isr()                  // 外部 GPIO_8 中断服务函数     
{



}

void gpio_9_exti_isr()                  // 外部 GPIO_9 中断服务函数     
{



}

void gpio_10_exti_isr()                  // 外部 GPIO_10 中断服务函数     
{



}

void gpio_11_exti_isr()                  // 外部 GPIO_11 中断服务函数     
{



}

void gpio_12_exti_isr()                  // 外部 GPIO_12 中断服务函数     
{



}

void gpio_13_exti_isr()                  // 外部 GPIO_13 中断服务函数     
{



}

void gpio_14_exti_isr()                  // 外部 GPIO_14 中断服务函数     
{



}

void gpio_15_exti_isr()                  // 外部 GPIO_15 中断服务函数     
{



}

void gpio_16_exti_isr()                  // 外部 GPIO_16 中断服务函数     
{



}

void gpio_17_exti_isr()                  // 外部 GPIO_17 中断服务函数     
{



}

void gpio_18_exti_isr()                  // 外部 GPIO_18 中断服务函数     
{



}

void gpio_19_exti_isr()                  // 外部 GPIO_19 中断服务函数     
{



}

void gpio_20_exti_isr()                  // 外部 GPIO_20 中断服务函数     
{



}

void gpio_21_exti_isr()                  // 外部 GPIO_21 中断服务函数     
{



}

void gpio_22_exti_isr()                  // 外部 GPIO_22 中断服务函数     
{



}

void gpio_23_exti_isr()                  // 外部 GPIO_23 中断服务函数     
{



}
// **************************** 串口中断函数 ****************************