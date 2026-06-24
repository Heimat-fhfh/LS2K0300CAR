
#include "zf_common_typedef.hpp"
#include "zf_common_fifo.hpp"
// #include "zf_common_debug.h"
// #include "zf_driver_uart.h"
// #include "zf_device_wireless_uart.h"
// #include "zf_device_bluetooth_ch9141.h"
// #include "zf_device_wifi_uart.h"
// #include "zf_device_wifi_spi.h"
// #include "zf_device_ble6a20.h"
#include "seekfree_assistant.hpp"

#include "seekfree_assistant_interface.hpp"


extern seekfree_assistant_transfer_callback_function   seekfree_assistant_transfer_callback;    // 数据发送函数指针
extern seekfree_assistant_receive_callback_function    seekfree_assistant_receive_callback;     // 数据接收函数指针


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐飞助手发送函数
// 参数说明     *buff           需要发送的数据地址
// 参数说明     length          需要发送的长度
// 返回参数     uint32          剩余未发送数据长度
// 使用示例
//-------------------------------------------------------------------------------------------------------------------
ZF_WEAK uint32 seekfree_assistant_transfer (const uint8 *buff, uint32 length)
{
    
    // 当选择自定义通讯方式时 需要自行完成数据发送功能
    return length;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐飞助手接收数据函数
// 参数说明     *buff           需要接收的数据地址
// 参数说明     length          要接收的数据最大长度
// 返回参数     uint32          接收到的数据长度
// 使用示例
//-------------------------------------------------------------------------------------------------------------------
ZF_WEAK uint32 seekfree_assistant_receive (uint8 *buff, uint32 length)
{
    // 当选择自定义通讯方式时 需要自行完成数据接收功能
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐飞助手接口 初始化
// 参数说明
// 返回参数     void
// 使用示例     seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI); 使用高速WIFI SPI模块进行数据收发
// 备注         需要自行调用设备的初始化，例如使用无线转串口进行数据的收发，则需要自行调用无线转串口的初始化，然后再调用seekfree_assistant_interface_init完成逐飞助手的接口初始化
//-------------------------------------------------------------------------------------------------------------------
ZF_WEAK void seekfree_assistant_interface_init(uint32 (*send_func) (const uint8 *, uint32 ), uint32 (*recv_func)  (uint8 *, uint32 ))
{
    seekfree_assistant_transfer_callback = send_func;
    seekfree_assistant_receive_callback = recv_func;
}


