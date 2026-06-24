
#ifndef _seekfree_assistant_interface_h_
#define _seekfree_assistant_interface_h_



// 数据发送设备枚举
typedef enum
{
    SEEKFREE_ASSISTANT_DEBUG_UART,      // 调试串口    使用的串口由DEBUG_UART_INDEX宏定义指定
    SEEKFREE_ASSISTANT_WIRELESS_UART,   // 无线转串口
    SEEKFREE_ASSISTANT_CH9141,          // CH9141蓝牙
    SEEKFREE_ASSISTANT_WIFI_UART,       // WIFI转串口
    SEEKFREE_ASSISTANT_WIFI_SPI,        // 高速WIFI SPI
    SEEKFREE_ASSISTANT_BLE6A20,        // 高速WIFI SPI
    SEEKFREE_ASSISTANT_CUSTOM,          // 自定义通讯方式 需要自行seekfree_assistant_transfer与seekfree_assistant_receive函数
}seekfree_assistant_transfer_device_enum;


void seekfree_assistant_interface_init(uint32 (*send_func) (const uint8 *, uint32 ), uint32 (*recv_func)  (uint8 *, uint32 ));

#endif
