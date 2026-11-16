#ifndef SMALL_DRIVER_UART_CONTROL_2_H_
#define SMALL_DRIVER_UART_CONTROL_2_H_

#include "zf_common_headfile.h"


#define SMALL_DRIVER_UART_2                       (UART_3        )

#define SMALL_DRIVER_BAUDRATE_2                   (460800        )

#define SMALL_DRIVER_RX_2                         (UART3_TX_P13_1)

#define SMALL_DRIVER_TX_2                         (UART3_RX_P13_0)

typedef struct
{
    uint8 send_data_buffer[11];                 // 发送缓冲区

    uint8 receive_data_buffer[11];              // 接收缓冲区

    uint8 receive_data_count;                   // 接收计数

    uint8 sum_check_data;                       // 校验位

    int16 receive_speed_data[4];                // 接收到的电机速度数据

}small_device_value_struct_2;

extern small_device_value_struct_2 motor_value_2;



void uart_control_callback_2(void);                                   // 无刷电机 串口接收回调函数

void small_driver_set_duty_2(int16 motor_duty_1, int16 motor_duty_2, int16 motor_duty_3, int16 motor_duty_4);      // 无刷电机 设置占空比

void small_driver_get_speed_2(void);                                  // 无刷电机 获取速度信息

void small_driver_uart_init_2(void);                                  // 无刷电机 串口通讯初始化


#endif
