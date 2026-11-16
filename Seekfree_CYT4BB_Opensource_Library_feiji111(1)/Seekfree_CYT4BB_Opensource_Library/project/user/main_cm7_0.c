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
* 文件名称          main_cm7_0
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
#include "find.h"
#include "globals.h"
#include "image.h"
unsigned char yvzhi=0;
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************
#pragma location = 0x28001000
int m7_1_data_1[3];

uint8 to_car[5];

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等

    mt9v03x_init();
        tft180_init();
    //uart_init(UART_1,115200,UART2_TX_P10_1,UART2_RX_P10_0);//
    //uart_tx_interrupt(UART_2,1);
    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        // 此处编写需要循环执行的代码
       if(mt9v03x_finish_flag)
      {
        mt9v03x_finish_flag=0;
        //Get_mt9v03x_image_BandW(1);//用显示数据操作
        yvzhi=110;//大津法求阈值  GetOSTU (mt9v03x_image)
        lq_sobel(mt9v03x_image,mt9v03x_image_BandW,yvzhi);//边缘检测
        //erosion_operation(mt9v03x_image_BandW,3,yvzhi);//腐蚀
        //erosion_operation(mt9v03x_image_BandW,3,yvzhi);
        //erosion_operation(mt9v03x_image_BandW,3,yvzhi);
        //erosion_operation(mt9v03x_image_BandW,3,yvzhi);
        //PENGZHANG_operation(mt9v03x_image_BandW,3,yvzhi);//膨胀
        //PENGZHANG_operation(mt9v03x_image_BandW,3,yvzhi);
        //erosion_operation(mt9v03x_image_BandW,2,1);
        PENGZHANG_operation(mt9v03x_image_BandW,3,yvzhi);
        PENGZHANG_operation(mt9v03x_image_BandW,3,yvzhi);
        //PENGZHANG_operation(mt9v03x_image_BandW,3,yvzhi);
        //PENGZHANG_operation(mt9v03x_image_BandW,3,yvzhi);
        //PENGZHANG_operation(mt9v03x_image_BandW,3,yvzhi);
        //PENGZHANG_operation(mt9v03x_image_BandW,3,yvzhi);
        //PENGZHANG_operation(mt9v03x_image_BandW,3,yvzhi);
        //PENGZHANG_operation(mt9v03x_image_BandW,3,yvzhi);
        //PENGZHANG_operation(mt9v03x_image_BandW,3,yvzhi);
        erosion_operation(mt9v03x_image_BandW,3,1);
        erosion_operation(mt9v03x_image_BandW,3,1);
        erosion_operation(mt9v03x_image_BandW,3,1);
        erosion_operation(mt9v03x_image_BandW,3,1);
        erosion_operation(mt9v03x_image_BandW,3,1);
        erosion_operation(mt9v03x_image_BandW,5,1);
        erosion_operation(mt9v03x_image_BandW,5,1);
        erosion_operation(mt9v03x_image_BandW,4,1);
        //erosion_operation(mt9v03x_image_BandW);
        //PENGZHANG_operation(mt9v03x_image_BandW,3,yvzhi);
        erosion_operation(mt9v03x_image_BandW,3,1);
        erosion_operation(mt9v03x_image_BandW,3,1);
        erosion_operation(mt9v03x_image_BandW,4,1);
        erosion_operation(mt9v03x_image_BandW,4,1);
        //PENGZHANG_operation(mt9v03x_image_BandW,3,yvzhi);
        //PENGZHANG_operation(mt9v03x_image_BandW,3,yvzhi);
        //PENGZHANG_operation(mt9v03x_image_BandW,3,yvzhi);
        Set_image_twovalues(yvzhi);
        //erosion_operation(mt9v03x_image_BandW,2,1);
        Set_image_twovalues(yvzhi);
        //tft180_show_gray_image(0, 0, display_image[0], MT9V03X_W, MT9V03X_H, 128, 81,0);//显示二值化图像
        tft180_show_gray_image (0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, 128, 80,0);//显示图像
        
                  //环境复杂时使用,三个必须同时使用
        ConnectedComponentLabeling((unsigned char *)mt9v03x_image_BandW ,labelBuffer);   //连通域计算
        tft180_show_string(0,81, "yvzhi:");
        tft180_show_int(64,81, yvzhi, 3); 
        tft180_show_int(64,100,labelcount,3);
        
        quanzhong_light_panduan();//通过连通域得到的图像进行判断
        int wei=find_max();
        tft180_show_int(64,120,xx,3);//xx
        tft180_show_int(64,140,yy,3);
        tft180_show_int(0,100,wei,3);//areamax
        tft180_show_int(0,120,center_x,3);
        tft180_show_int(0,140,center_y,3);
        //printf("%d\n",center_x);
        tft180_draw_line(1,yy,127,yy,RGB565_RED);//yy
        tft180_draw_line(xx,1,xx,80,RGB565_BLUE);//xx
        //quanzhong_light();//计算目标位置信息
                    // 发送数据给核心M7_1
      }
      m7_1_data_1[0]=center_x;
      m7_1_data_1[1]=center_y;
      m7_1_data_1[2]=labelcount;
      SCB_CleanInvalidateDCache_by_Addr(&m7_1_data_1,sizeof(m7_1_data_1));
      to_car[0]=0xAA;
      to_car[1]=m7_1_data_1[0];
      to_car[2]=m7_1_data_1[1];
      to_car[3]=m7_1_data_1[2];
      to_car[4]=0x55;
      uart_write_buffer(UART_2,&to_car[0],5);
      //system_delay_ms(10);
        //system_delay_ms(20);
        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************
void uart_tx_interrupt_handler (void)
{
  //uart_write_buffer(UART_2,&to_car[0],5);
}

/*
#include "zf_common_headfile.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// *************************** 例程硬件连接说明 ***************************
// 使用逐飞科技 CMSIS-DAP 调试下载器连接
//      直接将下载器正确连接在核心板的调试下载接口即可
// 使用 USB-TTL 模块连接
//      模块管脚            单片机管脚
//      USB-TTL-RX          查看 zf_common_debug.h 文件中 DEBUG_UART_TX_PIN 宏定义的引脚 默认 P14_0
//      USB-TTL-TX          查看 zf_common_debug.h 文件中 DEBUG_UART_RX_PIN 宏定义的引脚 默认 P14_1
//      USB-TTL-GND         核心板电源地 GND
//      USB-TTL-3V3         核心板 3V3 电源

// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程，单独使用核心板与调试下载器或者 USB-TTL 模块，在断电情况下完成连接
// 
// 2.将调试下载器或者 USB-TTL 模块连接电脑，完成上电
// 
// 3.电脑上使用串口助手打开对应的串口，串口波特率为 zf_common_debug.h 文件中 DEBUG_UART_BAUDRATE 宏定义 默认 115200，核心板按下复位按键
// 
// 4.可以在串口助手上看到如下串口信息：
//      receive M7_0 data:0
//      receive M7_0 data:1
//      receive M7_1 data:1
//      receive M7_0 data:2
//      receive M7_0 data:3
//      receive M7_1 data:2
// 
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

// **************************** 代码区域 ****************************
uint32 send_data_test = 0;
// 定义数据接收回调函数 如果另外一个核心发送信息 此核心会触发中断并且可以在回调函数读取数据
void my_ipc_callback(uint32 receive_data)
{
    printf("receive M7_1 data:%d\r\n", receive_data);        // 将接收到的数据打印到串口     
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等
    SCB_DisableDCache(); // 关闭DCache
    
    ipc_communicate_init(IPC_PORT_1, my_ipc_callback);          // 初始化IPC模块 选择端口1 填写中断回调函数
    
    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        // 此处编写需要循环执行的代码

        ipc_send_data(send_data_test ++);               // 发送数据给核心M7_1
        system_delay_ms(500);
        
        // 此处编写需要循环执行的代码
    }
}
*/