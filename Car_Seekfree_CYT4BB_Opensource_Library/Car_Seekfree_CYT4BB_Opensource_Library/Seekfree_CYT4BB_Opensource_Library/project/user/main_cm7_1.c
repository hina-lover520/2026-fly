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
* 文件名称          main_cm7_1
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
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
#include "zf_device_upflow302.h"
#include "zf_device_lora3a22.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************
#pragma location = 0x28001000
__no_init int m7_1_data_1[3];

#define PIT_NUM                 (PIT_CH12)                              // 使用的周期中断编号
#define PIT_period              (10 )                                   // 中断周期uint8 lora_get_data[4];                 // 接收数据缓冲数组      

#define WIFI_SSID_TEST          "REDMI Turbo 4 Pro"
#define WIFI_PASSWORD_TEST      "88888888"                      // 如果需要连接的WIFI 没有密码 替换为 NULL
#define TCP_TARGET_IP           "10.44.10.182"                 // 连接目标的 IP
#define TCP_TARGET_PORT         "8086"                          // 连接目标的端口
#define WIFI__LOCAL_PORT        "6666"                          // 本机的端口 0：随机  可设置范围2048-65535  默认 6666

#define BUZZER_PIN              (P19_4)                                         // 定义主板蜂鸣器引脚
uint32 data_length;
uint8 wifi_spi_get_data_buffer[]="vucaguvu";
uint32 data_length_1;
static int len =0;
uint8 by_another[5];

int main(void)
{
  
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    //debug_info_init()                  // 调试串口信息初始化
    debug_init();// Q噪声矩阵1，Q噪声矩阵2,R噪声矩阵1,R噪声矩阵2，P矩阵，中断时间
    printf("%d\n",1);
   /*while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
    {
        printf("\r\n connect wifi failed. \r\n");
        system_delay_ms(100);                                                   // 初始化失败 等待 100ms
    }*/
printf("%d\n",1);
/*if(0 == WIFI_SPI_AUTO_CONNECT)                                              // 如果没有开启自动连接 就需要手动连接目标 IP
    {
        while(wifi_spi_socket_connect(                                          // 向指定目标 IP 的端口建立 TCP 连接
            "TCP",                                                              // 指定使用TCP方式通讯
            TCP_TARGET_IP,                                                      // 指定远端的IP地址，填写上位机的IP地址
            TCP_TARGET_PORT,                                                    // 指定远端的端口号，填写上位机的端口号，通常上位机默认是8080
            WIFI__LOCAL_PORT))                                                  // 指定本机的端口号
        {
            // 如果一直建立失败 考虑一下是不是没有接硬件复位
            printf("\r\n Connect TCP Servers error, try again.");
            system_delay_ms(100);                                               // 建立连接失败 等待 100ms
        }
    }*/
    //uart_init(UART_1, 9600, UART1_TX_P04_1, UART1_RX_P04_0);             // 初始化串口
    //uart_rx_interrupt(UART_1, 1);                                           // 开启 UART_INDEX 的接收中断
    //lora3a22_init();
    //upflow302_receive_init();                                                   // 初始化 UP-FLOW-302 光流模块 硬件功能
    kaerman_b_Init(&guangliu_x,6.0f,2.0f,2.0f,6.0f,5.5f,0.01f);//Q噪声矩阵，R噪声矩阵1,R噪声矩阵2，P矩阵，中断时间
    kaerman_b_Init(&guangliu_y,8.0f,3.0f,3.0f,8.0f,5.5f,0.01f);
    
    kaerman_b_Init(&hv,1.0f,1.0f,8.0f,8.0f,10000.0f,0.01f);  
    kaerman_b_Init(&gyro_x,2.0f,2.0f,80.0f,80.0f,0.002f,0.002f);
    kaerman_b_Init(&gyro_y,1.0f,1.0f,80.0f,80.0f,0.002f,0.002f);
    
    //kaerman_b_Init(&guangliu_x,3.0f,3.0f,80.0f,20.0f,1.5f,0.01);//Q噪声矩阵，R噪声矩阵1,R噪声矩阵2，P矩阵，中断时间
    //kaerman_b_Init(&guangliu_y,3.0f,3.0f,80.0f,20.0f,1.5f,0.01f);

    kaerman_b_Init(&my_hight,2.0f,2.0f,6.0f,6.0f,10000.0f,0.01f);
    
    quaternion_Init(&sc,&s_angle,&s_shu,1.2,0.01,0.002f);//pi,角度，四元数，kp,kd,中断时间,2.0,0.001,0.05f
                                        //初2.2 ，0.005 ，0.03
    imu660rb_init();
    //imu660rc_init(IMU660RC_QUARTERNION_120HZ);
    //imu_calibration();
    //Notch_Init_all();
    //filter_init(&tof);
    pmw3901_init();
    dl1b_init();
    small_driver_uart_init();
    small_driver_uart_init_2();
    gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);                        // 初始化 BUZZER_PIN 输出 默认低电平 推挽输出模式
    
    
printf("%d\n",1);
    //EKF_Init();
    pit_ms_init(PIT_CH0, 2);
    pit_enable(PIT_CH0); 
    system_delay_ms(1970);
    gpio_toggle_level(BUZZER_PIN);
    system_delay_ms(500);
    gpio_set_level(BUZZER_PIN, GPIO_LOW);
    system_delay_ms(500);
    gpio_toggle_level(BUZZER_PIN);
    system_delay_ms(500);
    gpio_set_level(BUZZER_PIN, GPIO_LOW);
    system_delay_ms(500);
    gpio_toggle_level(BUZZER_PIN);
    system_delay_ms(500);
    gpio_set_level(BUZZER_PIN, GPIO_LOW);
    system_delay_ms(500);
    gpio_toggle_level(BUZZER_PIN);
    system_delay_ms(500);
    gpio_set_level(BUZZER_PIN, GPIO_LOW);
    system_delay_ms(500);
    
    pit_ms_init(PIT_CH1,1);
    pit_enable(PIT_CH1);
    pit_ms_init(PIT_CH2,10);
    pit_enable(PIT_CH2);
    pit_ms_init(PIT_CH10,10);
    pit_enable(PIT_CH10);
    pit_ms_init(PIT_CH11,30);
    pit_enable(PIT_CH11);
    pit_ms_init(PIT_CH13,1);
    pit_enable(PIT_CH13);
    pit_ms_init(PIT_NUM, PIT_period);
    //          15.5,0.0,2.0        3.5,0.0,2.5       15.4，0.0，1.0         0.115,0.0,0.03                                            //32.0  0.10 ,3.68      32.0  0.10  3.68
               //hight;          Roll;（我俯仰）         Yaw;                    Pitch;(横滚）       XX;               YY;               PMW_X;             PMW_Y ;
   pid_Init_all(15.5,0.0,0.0,     3.5,0.0,2.5,        2.0,0.0,0.2,         7.0,0.0,2.5,    0.0,0.0,0.0,      0.0,0.0,0.0,     0.0,0.0,0.0,       0.0,0.0,0.0);
   //pid_Init_all(8.0,3.0,0.0,     7.0,0.0,3.0,         7.0,0.0,1.1,           7.00,0.0,3.0,    0.0,0.0,0.0,      0.0,0.0,0.0,     0.0,0.0,0.0,       0.0,0.0,0.0);
   
    // 此处编写用户代码 例如外设初始化代码等
    //printf("%d\n",1);
    // 此处编写用户代码 例如外设初始化代码等
    
    while(true)
    {
        SCB_CleanInvalidateDCache_by_Addr(&m7_1_data_1,sizeof(m7_1_data_1));
 
        //printf("%f,%f,%f\n",imu_data_acc_x,imu_data_acc_y,imu_data_acc_z);
        //printf("TOF:%d\n",dl1b_distance_mm);
        /*printf("state:%d, delta x:%d, delta y:%d\r\n", upflow302_state_flag,   
                                                   upflow302_receive.upflow302_x, 
                                                   upflow302_receive.upflow302_y);   // 打印光流数据*/
        //data_length = wifi_spi_send_buffer(buf, sizeof(buf));

        target_xx=m7_1_data_1[0];
        target_yy=m7_1_data_1[1];
        my_flag=m7_1_data_1[2];
        //sprintf(buf,"%f",s_angle.agx);
        //data_length_1 = wifi_spi_send_buffer(buf, sizeof(buf));
        if (lora3a22_state_flag == 1)
        {
            if (lora3a22_finsh_flag == 1)
            {
              
                /*lora3a22 帧头
                printf ("head = %d\r\n",lora3a22_uart_transfer.head);

                //lora3a22 和校验
                printf ("sum_check = %d\r\n",lora3a22_uart_transfer.sum_check);

                //左边摇杆左右值
                printf ("joystick[0] = %d\r\n",lora3a22_uart_transfer.joystick[0]);
                //左边摇杆上下值
                printf ("joystick[1] = %d\r\n",lora3a22_uart_transfer.joystick[1]);
                //右边摇杆左右值
                printf ("joystick[2] = %d\r\n",lora3a22_uart_transfer.joystick[2]);
                //右边摇杆上下值
                printf ("joystick[3] = %d\r\n",lora3a22_uart_transfer.joystick[3]);

                //左边摇杆按键
                printf ("key0 = %d\r\n",lora3a22_uart_transfer.key[0]);
                //右边摇杆按键
                printf ("key1 = %d\r\n",lora3a22_uart_transfer.key[1]);
                // 左边侧向按键
                printf ("key2 = %d\r\n",lora3a22_uart_transfer.key[2]);
                // 右边侧向按键
                printf ("key3 = %d\r\n",lora3a22_uart_transfer.key[3]);

                // 左边拨码开关_1
                printf ("switch_key[0] = %d\r\n",lora3a22_uart_transfer.switch_key[0]);
                // 左边拨码开关_2
                printf ("switch_key[1] = %d\r\n",lora3a22_uart_transfer.switch_key[1]);
                // 右边拨码开关_1
                printf ("switch_key[2] = %d\r\n",lora3a22_uart_transfer.switch_key[2]);
                // 右边拨码开关_2
                printf ("switch_key[3] = %d\r\n",lora3a22_uart_transfer.switch_key[3]);*/
              if(lora3a22_uart_transfer.switch_key[1]==0){
              lo_roll=lora3a22_uart_transfer.joystick[1]*0.72;
              lo_pitch=lora3a22_uart_transfer.joystick[0]*0.72;
              lo_yaw=lora3a22_uart_transfer.joystick[2]*0.4;
              lo_hight=lora3a22_uart_transfer.joystick[3]*0.5;
              left_2=lora3a22_uart_transfer.switch_key[0];
              //printf("%f,%f\n",lo_roll,lo_pitch);
              }
               lora3a22_finsh_flag = 0;
            }

        }
        else
        {
           // printf("lora3a22 connection fail \r\n");
        }
        //此处编写需要循环执行的代码
        system_delay_ms(30);
}
}
    // **************************** 代码区域 ****************************
void uart_rx_interrupt_handler (void)//记得在对应串口中断调用一下
{
  if(len>4)len=0;
  uart_query_byte(UART_1, &by_another[0+len]);
  len++;
}