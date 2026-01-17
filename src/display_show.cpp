#include "display_show.h"
#include <sys/socket.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

/**
 * @brief 将OpenCV的Mat图像显示到IPS200屏幕上
 * @param img 输入的图像（支持BGR或灰度格式）
 * @note 图像将被自动缩放以适应屏幕分辨率（假设屏幕分辨率为IPS200_WIDTH x IPS200_HEIGHT）
 */
void displayMatOnIPS200(const cv::Mat& img) {
    
    // 检查输入图像是否有效
    if (img.empty()) {
        return;
    }
    
    // 转换图像颜色空间（BGR -> RGB565）并缩放尺寸
    cv::Mat resizedImg;
    cv::resize(img, resizedImg, cv::Size(240, 180));
    
    // 根据输入图像类型处理
    for (uint16_t y = 0; y < 180; ++y) {
        for (uint16_t x = 0; x < 240; ++x) {
            cv::Vec3b pixel = resizedImg.at<cv::Vec3b>(y, x);
            
            // 将BGR转换为RGB565 (5-6-5格式)
            uint16_t color = ((pixel[2] >> 3) << 11) |  // R
                             ((pixel[1] >> 2) << 5)  |  // G
                             (pixel[0] >> 3);           // B
            
            // 绘制像素点
            ips200_draw_point(x, y, color);
        }
    }
}

void display_data(int y,const char dat[],int data,int num)
{
    ips200_show_string(0,16*y,dat);
    ips200_show_int(8*(strlen(dat)),16*y,int32(data),num);
}

void display_dataf(int y,const char dat[],float data,int num1,int num2)
{
    ips200_show_string(0,16*y,dat);
    ips200_show_float(8*(strlen(dat)),16*y,data,num1,num2);
}

std::string get_local_ip_address() {
    struct ifaddrs *ifaddr, *ifa;
    int family;
    char host[NI_MAXHOST];
    std::string ip_address = "";

    if (getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        return ip_address;
    }

    // 遍历所有网络接口
    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == NULL)
            continue;

        family = ifa->ifa_addr->sa_family;

        // 只关注IPv4地址
        if (family == AF_INET) {
            // 跳过回环接口
            if (strcmp(ifa->ifa_name, "lo") == 0)
                continue;

            // 获取IP地址
            if (getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in),
                            host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST) == 0) {
                // 检查是否是192.168.x.x格式的地址
                if (strncmp(host, "192.168.", 8) == 0) {
                    ip_address = host;
                    break;
                }
                // 如果没有找到192.168.x.x，使用第一个非回环IPv4地址
                if (ip_address.empty()) {
                    ip_address = host;
                }
            }
        }
    }

    freeifaddrs(ifaddr);
    return ip_address;
}

void display_ip_address(uint16 x, uint16 y) {
    std::string ip = get_local_ip_address();
    if (ip.empty()) {
        ip = "No IP";
    }
    
    // 构建显示字符串
    std::string display_str = "IP:" + ip;
    
    // 在屏幕上显示
    ips200_show_string(x, y, display_str.c_str());
}
