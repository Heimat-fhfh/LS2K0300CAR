#include "devices/display_show.h"
#include "vision/image_my_zf.h"
#include <sys/socket.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>

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

    cv::Mat bgrImg;
    if (img.channels() == 3) {
        bgrImg = img;
    } else if (img.channels() == 1) {
        cv::cvtColor(img, bgrImg, cv::COLOR_GRAY2BGR);
    } else if (img.channels() == 4) {
        cv::cvtColor(img, bgrImg, cv::COLOR_BGRA2BGR);
    } else {
        return;
    }

    // 转换图像颜色空间（BGR -> BGR565）并缩放尺寸
    cv::Mat resizedImg;
    cv::resize(bgrImg, resizedImg, cv::Size(240, 180));

    cv::Mat img565;
    cv::cvtColor(resizedImg, img565, cv::COLOR_BGR2BGR565);

    if (!img565.isContinuous()) {
        img565 = img565.clone();
    }

    ips200_show_rgb565_image(0, 0, reinterpret_cast<const uint16*>(img565.data), 240, 180);
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

#define SCALE_PX 3
#define IMG_OFFSET_X 0
#define IMG_OFFSET_Y 0

void displayMyZFOnIPS200() {
    uint8_t off_line = ImageStatus.OFFLine;

    // draw 80x60 binary image at 3x scale
    for (int i = 0; i < 60; i++) {
        for (int j = 0; j < 80; j++) {
            uint16_t color = Pixle[i][j] ? RGB565_WHITE : RGB565_BLACK;
            for (int dy = 0; dy < SCALE_PX; dy++) {
                for (int dx = 0; dx < SCALE_PX; dx++) {
                    ips200_draw_point(IMG_OFFSET_X + j * SCALE_PX + dx,
                                      IMG_OFFSET_Y + i * SCALE_PX + dy, color);
                }
            }
        }
    }

    // draw left border (red), right border (green), center (black) - 2x2 blocks
    for (int i = off_line; i < 60; i++) {
        int base_y = IMG_OFFSET_Y + i * SCALE_PX;
        for (int dy = 0; dy < 2; dy++) {
            int y = base_y + dy;
            if (y < 0 || y >= 320) continue;

            int left_x = IMG_OFFSET_X + ImageDeal[i].LeftBorder * SCALE_PX;
            for (int dx = 0; dx < 2; dx++) {
                int lx = left_x + dx;
                if (lx >= 0 && lx < 240) ips200_draw_point(lx, y, RGB565_RED);
            }

            int right_x = IMG_OFFSET_X + ImageDeal[i].RightBorder * SCALE_PX;
            for (int dx = 0; dx < 2; dx++) {
                int rx = right_x + dx;
                if (rx >= 0 && rx < 240) ips200_draw_point(rx, y, RGB565_GREEN);
            }

            int center_x = IMG_OFFSET_X + ImageDeal[i].Center * SCALE_PX;
            for (int dx = 0; dx < 2; dx++) {
                int cx = center_x + dx;
                if (cx >= 0 && cx < 240) ips200_draw_point(cx, y, RGB565_BLACK);
            }
        }
    }

    // draw enlarged forward point (5x5 cross) at TowPoint_True row
    {
        int tp = ImageStatus.TowPoint_True;
        if (tp >= off_line && tp < 60) {
            int fx = IMG_OFFSET_X + ImageDeal[tp].Center * SCALE_PX + 1;
            int fy = IMG_OFFSET_Y + tp * SCALE_PX + 1;
            for (int dy = -2; dy <= 2; dy++) {
                for (int dx = -2; dx <= 2; dx++) {
                    if (abs(dx) == abs(dy) || dx == 0 || dy == 0) {
                        int px = fx + dx, py = fy + dy;
                        if (px >= 0 && px < 240 && py >= 0 && py < 320)
                            ips200_draw_point(px, py, RGB565_BLUE);
                    }
                }
            }
        }
    }

    // text info below image (y >= 180)
    char buf[64];
    int text_y = IMG_OFFSET_Y + 60 * SCALE_PX + 2;  // 182

    float norm_dev = (float)((int)ImageStatus.Det_True - 40) / 40.0f;
    const char* road_names[] = {"Normol","Straight","Cross","Ramp","LCirque","RCirque","ForkIn","ForkOut","BarnOut","BarnIn","CrossT"};
    int road_idx = (int)ImageStatus.Road_type;
    if (road_idx < 0 || road_idx > 10) road_idx = 0;

    snprintf(buf, sizeof(buf), "Det:%d NDev:%.2f", ImageStatus.Det_True, norm_dev);
    ips200_show_string(0, text_y, buf); text_y += 16;

    snprintf(buf, sizeof(buf), "Road:%s", road_names[road_idx]);
    ips200_show_string(0, text_y, buf); text_y += 16;

    snprintf(buf, sizeof(buf), "Ring:%d Flag:%d", ImageFlag.image_element_rings, ImageFlag.image_element_rings_flag);
    ips200_show_string(0, text_y, buf); text_y += 16;

    snprintf(buf, sizeof(buf), "OFF:%d  L_L:%d R_L:%d W_L:%d",
             ImageStatus.OFFLine, ImageStatus.Left_Line, ImageStatus.Right_Line, ImageStatus.WhiteLine);
    ips200_show_string(0, text_y, buf);
}
