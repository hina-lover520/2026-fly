#include "small_driver_uart_control_2.h"

small_device_value_struct_2 motor_value_2;      // 电调通讯数据结构体


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷电机 串口接收回调函数
// 参数说明     void
// 返回参数     void
// 使用示例     uart_control_callback_2(1000, -1000);
// 备注信息     串口接收到电调的速度数据 该函数需要在对应的串口接收中断中调用
//-------------------------------------------------------------------------------------------------------------------
void uart_control_callback_2(void)
{
    uint8 receive_data;                                                                     // 接收临时变量

    if(uart_query_byte(SMALL_DRIVER_UART_2, &receive_data))                                 // 接收串口数据
    {
        if(receive_data == 0xA5 && motor_value_2.receive_data_buffer[0] != 0xA5)            // 判断是否收到帧头 或者 当前接收数据是否是正确的帧头
        {
            motor_value_2.receive_data_count = 0;                                           // 未收到帧头或者未正确收到帧头则重新计数
        }

        motor_value_2.receive_data_buffer[motor_value_2.receive_data_count ++] = receive_data;  // 存储串口数据

        if(motor_value_2.receive_data_count >= 11)                                          // 判断是否接收到指定数量的数据
        {
            if(motor_value_2.receive_data_buffer[0] == 0xA5)                                // 判断帧头是否正确
            {
                motor_value_2.sum_check_data = 0;                                           // 清空校验位

                for(int i = 0; i < 10; i ++)
                {
                    motor_value_2.sum_check_data += motor_value_2.receive_data_buffer[i];   // 重新计算校验位
                }

                if(motor_value_2.sum_check_data == motor_value_2.receive_data_buffer[10])   // 校验结果准确
                {
                    for(int i = 0; i < 4; i ++)
                    {
                        motor_value_2.receive_speed_data[i]   = (((int)motor_value_2.receive_data_buffer[i * 2 + 2] << 8) | (int)motor_value_2.receive_data_buffer[i * 2 + 3]);
                    }

                    motor_value_2.receive_data_count = 0;                                   // 清空接收计数器数值

                    memset(motor_value_2.receive_data_buffer, 0, 11);                       // 清空接收缓冲区
                }
                else
                {
                    motor_value_2.receive_data_count = 0;                                   // 清空接收计数器数值

                    memset(motor_value_2.receive_data_buffer, 0, 11);                       // 清空接收缓冲区
                }
            }
            else
            {
                motor_value_2.receive_data_count = 0;                                       // 清空接收计数器数值

                memset(motor_value_2.receive_data_buffer, 0, 11);                           // 清空接收缓冲区
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷电机 设置占空比
// 参数说明     left_duty       左侧占空比  范围 -10000 ~ 10000  负值为反转
// 参数说明     right_duty      右侧占空比  范围 -10000 ~ 10000  负值为反转
// 返回参数     void
// 使用示例     small_driver_set_duty_2(1000, -1000, 1000, -1000);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_set_duty_2(int16 motor_duty_1, int16 motor_duty_2, int16 motor_duty_3, int16 motor_duty_4)
{
    motor_value_2.send_data_buffer[0] = 0xA5;                                         // 帧头

    motor_value_2.send_data_buffer[1] = 0X01;                                         // 设置功能字

    motor_value_2.send_data_buffer[2] = (uint8)((motor_duty_1 & 0xFF00) >> 8);        // 电机1占空比 高八位

    motor_value_2.send_data_buffer[3] = (uint8)(motor_duty_1 & 0x00FF);               // 电机1占空比 低八位

    motor_value_2.send_data_buffer[4] = (uint8)((motor_duty_2 & 0xFF00) >> 8);        // 电机2占空比 高八位

    motor_value_2.send_data_buffer[5] = (uint8)(motor_duty_2 & 0x00FF);               // 电机2占空比 低八位

    motor_value_2.send_data_buffer[6] = (uint8)((motor_duty_3 & 0xFF00) >> 8);        // 电机3占空比 高八位

    motor_value_2.send_data_buffer[7] = (uint8)(motor_duty_3 & 0x00FF);               // 电机3占空比 低八位

    motor_value_2.send_data_buffer[8] = (uint8)((motor_duty_4 & 0xFF00) >> 8);        // 电机4占空比 高八位

    motor_value_2.send_data_buffer[9] = (uint8)(motor_duty_4 & 0x00FF);               // 电机4占空比 低八位

    motor_value_2.send_data_buffer[10] = 0;                                           // 清空校验位

    for(int i = 0; i < 10; i ++)
    {
        motor_value_2.send_data_buffer[10] += motor_value_2.send_data_buffer[i];      // 计算校验位
    }

    uart_write_buffer(SMALL_DRIVER_UART_2, motor_value_2.send_data_buffer, 11);       // 发送设置占空比的数据字节包
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷电机 获取速度信息
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_get_speed_2();
// 备注信息     需要发送一次 之后电调会返回速度信息(默认10ms)
//-------------------------------------------------------------------------------------------------------------------
void small_driver_get_speed_2(void)
{
    motor_value_2.send_data_buffer[0] = 0xA5;                                         // 帧头

    motor_value_2.send_data_buffer[1] = 0X02;                                         // 设置功能字

    motor_value_2.send_data_buffer[2] = 0x00;                                         // 占空比清零

    motor_value_2.send_data_buffer[3] = 0x00;                                         // 占空比清零

    motor_value_2.send_data_buffer[4] = 0x00;                                         // 占空比清零

    motor_value_2.send_data_buffer[5] = 0x00;                                         // 占空比清零

    motor_value_2.send_data_buffer[6] = 0x00;                                         // 占空比清零

    motor_value_2.send_data_buffer[7] = 0x00;                                         // 占空比清零

    motor_value_2.send_data_buffer[8] = 0x00;                                         // 占空比清零

    motor_value_2.send_data_buffer[9] = 0x00;                                         // 占空比清零

    motor_value_2.send_data_buffer[10] = 0xA7;                                        // 固定校验位

    uart_write_buffer(SMALL_DRIVER_UART_2, motor_value_2.send_data_buffer, 11);       // 发送获取转速数据的字节包
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷电机 结构体初始化
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_init_2();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_init_2(void)
{
    memset(motor_value_2.send_data_buffer, 0, 11);                             // 清空发送缓冲区

    memset(motor_value_2.receive_data_buffer, 0, 11);                          // 清空接收缓冲区

    memset(motor_value_2.receive_speed_data, 0, 8);                            // 清空速度缓冲区

    motor_value_2.receive_data_count          = 0;

    motor_value_2.sum_check_data              = 0;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷电机 串口通讯初始化
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_uart_init_2();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_uart_init_2(void)
{
    uart_init(SMALL_DRIVER_UART_2, SMALL_DRIVER_BAUDRATE_2, SMALL_DRIVER_RX_2, SMALL_DRIVER_TX_2);      // 串口初始化

    uart_rx_interrupt(SMALL_DRIVER_UART_2, 1);                                                          // 使能串口接收中断

    small_driver_init_2();                                                                              // 结构体变量初始化

    small_driver_set_duty_2(0, 0, 0, 0);                                                                // 输出0占空比

    small_driver_get_speed_2();                                                                         // 获取实时速度数据
}
