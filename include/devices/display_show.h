#pragma once 

#include "vision/AAAdefine.h"
#include "common/main.hpp"
#include "devices/zf_device_ips200_fb.h"
#include "common/zf_common_font.h"
#include "common/zf_common_function.h"
#include <string>

/**
 * @brief 获取本机IP地址
 * @return 返回IP地址字符串，格式为"192.168.X.XXX"
 * @note 如果获取失败，返回空字符串
 */
std::string get_local_ip_address();

/**
 * @brief 显示IP地址
 */
void display_ip_address(uint16 x, uint16 y);

/**
 * @brief my_zf 巡线信息 IPS200 显示
 * 显示 80x60 二值化图像 + 边界中线标注 + 下方文字参数（偏差/Road_type/OFFLine等）
 */
void displayMyZFOnIPS200();
