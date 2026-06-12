/*********************************************************************************************************************
* WiFi连接管理程序
* 功能：扫描附近WiFi，按键选择连接，自动获取IP
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <algorithm>

// 按键定义
#define KEY_0       "/dev/zf_driver_gpio_key_0"
#define KEY_1       "/dev/zf_driver_gpio_key_1"
#define KEY_2       "/dev/zf_driver_gpio_key_2"
#define KEY_3       "/dev/zf_driver_gpio_key_3"

// 屏幕尺寸
#define SCREEN_WIDTH    240
#define SCREEN_HEIGHT   320

#define RGB565_LIGHTBLUE    0x7BEF

void system_delay_ms(int ms) {
    usleep(ms * 1000);
}

// WiFi信息结构体
struct WiFiInfo {
    std::string ssid;
    std::string bssid;
    int signal_level;      // 信号强度 (dBm)
    std::string security;   // 加密方式
    bool has_password;      // 是否有密码
};

// 预存密码结构体
struct StoredPassword {
    std::string ssid;
    std::string password;
    bool is_open;           // 是否为开放网络
};

// 预存密码列表
std::vector<StoredPassword> stored_passwords = {
    {"MyHomeWiFi", "12345678", false},
    {"Office_Guest", "guest123", false},
    {"Public_WiFi", "", true},
    {"CMCC", "", true},
    {"ChinaNet", "", true},
    {"TP-LINK_1234", "admin123", false}
};

// 全局变量
std::vector<WiFiInfo> wifi_list;
int selected_index = 0;
int scroll_offset = 0;
bool connected = false;
std::string connected_ssid = "";
std::string ip_address = "";
bool show_password_selection = false;
int password_selected_index = -1;
std::vector<StoredPassword> matched_passwords;

// 函数声明
void scan_wifi_networks(void);
void display_wifi_list(void);
void display_connection_status(void);
void display_password_selection(void);
bool connect_to_wifi(const std::string& ssid, const std::string& password);
std::string get_ip_address(void);
void execute_command(const char* cmd, char* result, size_t result_size);
std::vector<StoredPassword> find_matched_passwords(const std::string& ssid);
int get_signal_strength_bars(int signal_dbm);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行shell命令并获取输出
//-------------------------------------------------------------------------------------------------------------------
void execute_command(const char* cmd, char* result, size_t result_size) {
    FILE* pipe = popen(cmd, "r");
    if (!pipe) {
        if (result) result[0] = '\0';
        return;
    }
    
    if (result) {
        fgets(result, result_size, pipe);
    }
    
    pclose(pipe);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     扫描WiFi网络
//-------------------------------------------------------------------------------------------------------------------
void scan_wifi_networks(void) {
    wifi_list.clear();
    
    // 执行iw扫描命令
    FILE* pipe = popen("iw dev wlan0 scan 2>/dev/null", "r");
    if (!pipe) {
        printf("Failed to scan WiFi networks\n");
        return;
    }
    
    char buffer[256];
    WiFiInfo current_wifi;
    bool in_bss = false;
    
    while (fgets(buffer, sizeof(buffer), pipe) != NULL) {
        std::string line(buffer);
        
        // 检测新的BSS开始
        if (line.find("BSS ") == 0) {
            if (in_bss && !current_wifi.ssid.empty()) {
                wifi_list.push_back(current_wifi);
            }
            
            // 解析BSSID
            size_t start = line.find("BSS ") + 4;
            size_t end = line.find("(", start);
            if (end != std::string::npos) {
                current_wifi.bssid = line.substr(start, end - start);
            }
            
            current_wifi = WiFiInfo();
            current_wifi.has_password = true;  // 默认有密码
            in_bss = true;
        }
        
        // 解析SSID
        if (line.find("SSID:") != std::string::npos) {
            size_t pos = line.find("SSID:") + 6;
            current_wifi.ssid = line.substr(pos);
            // 去除末尾的换行符
            if (!current_wifi.ssid.empty() && current_wifi.ssid.back() == '\n') {
                current_wifi.ssid.pop_back();
            }
        }
        
        // 解析信号强度
        if (line.find("signal:") != std::string::npos) {
            sscanf(buffer, " signal: %d dBm", &current_wifi.signal_level);
        }
        
        // 检测加密方式
        if (line.find("RSN:") != std::string::npos || 
            line.find("WPA:") != std::string::npos) {
            current_wifi.security = "WPA";
        }
        
        // 检测开放网络
        if (line.find("Authentication suites:") != std::string::npos) {
            if (line.find("802.1X") == std::string::npos) {
                current_wifi.has_password = false;
                current_wifi.security = "OPEN";
            }
        }
    }
    
    // 添加最后一个WiFi
    if (in_bss && !current_wifi.ssid.empty()) {
        wifi_list.push_back(current_wifi);
    }
    
    pclose(pipe);
    
    // 按信号强度排序（从强到弱）
    std::sort(wifi_list.begin(), wifi_list.end(), 
        [](const WiFiInfo& a, const WiFiInfo& b) {
            return a.signal_level > b.signal_level;
        });
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取信号强度显示条
//-------------------------------------------------------------------------------------------------------------------
int get_signal_strength_bars(int signal_dbm) {
    if (signal_dbm >= -50) return 4;
    if (signal_dbm >= -60) return 3;
    if (signal_dbm >= -70) return 2;
    if (signal_dbm >= -80) return 1;
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查找匹配的预存密码
//-------------------------------------------------------------------------------------------------------------------
std::vector<StoredPassword> find_matched_passwords(const std::string& ssid) {
    std::vector<StoredPassword> matches;
    
    for (const auto& sp : stored_passwords) {
        // 完全匹配或部分匹配（包含关系）
        if (ssid.find(sp.ssid) != std::string::npos || 
            sp.ssid.find(ssid) != std::string::npos) {
            matches.push_back(sp);
        }
    }
    
    // 如果没有完全匹配，尝试模糊匹配（去除特殊字符后比较）
    if (matches.empty()) {
        std::string clean_ssid;
        for (char c : ssid) {
            if (isalnum(c)) clean_ssid += c;
        }
        
        for (const auto& sp : stored_passwords) {
            std::string clean_stored;
            for (char c : sp.ssid) {
                if (isalnum(c)) clean_stored += c;
            }
            
            if (clean_ssid.find(clean_stored) != std::string::npos ||
                clean_stored.find(clean_ssid) != std::string::npos) {
                matches.push_back(sp);
            }
        }
    }
    
    return matches;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     连接到WiFi网络
//-------------------------------------------------------------------------------------------------------------------
bool connect_to_wifi(const std::string& ssid, const std::string& password) {
    char cmd[512];
    char result[256];
    
    // 先断开现有连接
    execute_command("sudo killall wpa_supplicant dhclient 2>/dev/null", NULL, 0);
    
    // 创建wpa_supplicant配置文件
    FILE* conf = fopen("/tmp/wifi.conf", "w");
    if (!conf) return false;
    
    if (password.empty()) {
        // 开放网络
        fprintf(conf, "network={\n"
                      "    ssid=\"%s\"\n"
                      "    key_mgmt=NONE\n"
                      "}\n", ssid.c_str());
    } else {
        // 加密网络
        fprintf(conf, "network={\n"
                      "    ssid=\"%s\"\n"
                      "    psk=\"%s\"\n"
                      "}\n", ssid.c_str(), password.c_str());
    }
    fclose(conf);
    
    // 启动wpa_supplicant
    snprintf(cmd, sizeof(cmd), 
        "sudo wpa_supplicant -B -i wlan0 -c /tmp/wifi.conf 2>&1");
    execute_command(cmd, result, sizeof(result));
    
    // 等待连接
    sleep(3);
    
    // 获取IP地址
    execute_command("sudo dhclient wlan0 2>&1", result, sizeof(result));
    sleep(2);
    
    // 检查是否成功获取IP
    ip_address = get_ip_address();
    return !ip_address.empty() && ip_address != "0.0.0.0";
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取IP地址
//-------------------------------------------------------------------------------------------------------------------
std::string get_ip_address(void) {
    char buffer[128];
    execute_command("ip -4 addr show wlan0 | grep inet | awk '{print $2}' | cut -d/ -f1", 
                    buffer, sizeof(buffer));
    
    std::string ip(buffer);
    if (!ip.empty() && ip.back() == '\n') {
        ip.pop_back();
    }
    return ip;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示WiFi列表
//-------------------------------------------------------------------------------------------------------------------
void display_wifi_list(void) {
    ips200_clear();
    
    // 显示标题
    ips200_show_string(10, 5, "Available WiFi Networks");
    ips200_draw_line(0, 20, SCREEN_WIDTH - 1, 20, RGB565_BLACK);
    
    int y_pos = 30;
    int display_count = 0;
    const int max_display = 10;  // 最多显示10个
    
    for (size_t i = scroll_offset; i < wifi_list.size() && display_count < max_display; i++) {
        // 高亮当前选中的项
        if ((int)i == selected_index) {
            // 画选中背景
            for (int x = 0; x < SCREEN_WIDTH; x++) {
                for (int y = y_pos - 2; y < y_pos + 20; y++) {
                    ips200_draw_point(x, y, RGB565_LIGHTBLUE);
                }
            }
        }
        
        // 显示信号强度图标
        int bars = get_signal_strength_bars(wifi_list[i].signal_level);
        for (int b = 0; b < 4; b++) {
            uint16 color = (b < bars) ? RGB565_GREEN : RGB565_GRAY;
            ips200_draw_line(5 + b*5, y_pos + 15 - b*3, 
                           5 + b*5, y_pos + 15, color);
        }
        
        // 显示SSID和加密状态
        char display_str[64];
        if (wifi_list[i].has_password) {
            snprintf(display_str, sizeof(display_str), "%s [%s]", 
                     wifi_list[i].ssid.c_str(), wifi_list[i].security.c_str());
        } else {
            snprintf(display_str, sizeof(display_str), "%s [OPEN]", 
                     wifi_list[i].ssid.c_str());
        }
        
        // 截断过长的SSID
        if (strlen(display_str) > 20) {
            display_str[20] = '\0';
            strcat(display_str, "...");
        }
        
        ips200_show_string(30, y_pos, display_str);
        
        // 显示信号强度数值
        char signal_str[16];
        snprintf(signal_str, sizeof(signal_str), "%d dBm", wifi_list[i].signal_level);
        ips200_show_string(180, y_pos, signal_str);
        
        y_pos += 20;
        display_count++;
    }
    
    // 显示提示信息
    ips200_draw_line(0, SCREEN_HEIGHT - 30, SCREEN_WIDTH - 1, 
                     SCREEN_HEIGHT - 30, RGB565_BLACK);
    ips200_show_string(10, SCREEN_HEIGHT - 25, 
                      "KEY0:Up  KEY1:Down  KEY2:Select  KEY3:Scan");
    
    // 如果已连接，显示连接状态
    if (connected) {
        ips200_show_string(10, SCREEN_HEIGHT - 15, 
                          ("Connected to: " + connected_ssid).c_str());
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示密码选择界面
//-------------------------------------------------------------------------------------------------------------------
void display_password_selection(void) {
    ips200_clear();
    
    ips200_show_string(10, 5, "Select Password:");
    ips200_show_string(10, 25, ("For: " + wifi_list[selected_index].ssid).c_str());
    ips200_draw_line(0, 40, SCREEN_WIDTH - 1, 40, RGB565_BLACK);
    
    int y_pos = 50;
    
    // 显示"手动输入"选项
    if (password_selected_index == -1) {
        for (int x = 0; x < SCREEN_WIDTH; x++) {
            for (int y = y_pos - 2; y < y_pos + 18; y++) {
                ips200_draw_point(x, y, RGB565_LIGHTBLUE);
            }
        }
    }
    ips200_show_string(20, y_pos, "Manual Input");
    y_pos += 20;
    
    // 显示匹配的预存密码
    for (size_t i = 0; i < matched_passwords.size(); i++) {
        if ((int)i == password_selected_index) {
            for (int x = 0; x < SCREEN_WIDTH; x++) {
                for (int y = y_pos - 2; y < y_pos + 18; y++) {
                    ips200_draw_point(x, y, RGB565_LIGHTBLUE);
                }
            }
        }
        
        char display_str[64];
        if (matched_passwords[i].is_open) {
            snprintf(display_str, sizeof(display_str), "%s [OPEN]", 
                     matched_passwords[i].ssid.c_str());
        } else {
            snprintf(display_str, sizeof(display_str), "%s [%s]", 
                     matched_passwords[i].ssid.c_str(), 
                     matched_passwords[i].password.c_str());
        }
        ips200_show_string(20, y_pos, display_str);
        y_pos += 20;
    }
    
    // 显示提示
    ips200_show_string(10, SCREEN_HEIGHT - 25, 
                      "KEY0:Up  KEY1:Down  KEY2:Connect");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示连接状态
//-------------------------------------------------------------------------------------------------------------------
void display_connection_status(void) {
    ips200_clear();
    
    if (connected) {
        ips200_show_string(40, 100, "Connected Successfully!");
        ips200_show_string(40, 130, ("SSID: " + connected_ssid).c_str());
        ips200_show_string(40, 150, ("IP: " + ip_address).c_str());
    } else {
        ips200_show_string(40, 100, "Connection Failed!");
        ips200_show_string(40, 130, "Please try again.");
    }
    
    ips200_show_string(40, 200, "Press KEY2 to continue");
}

//-------------------------------------------------------------------------------------------------------------------
// 主函数
//-------------------------------------------------------------------------------------------------------------------
int main(int, char**) 
{
    // 初始化屏幕
    ips200_init("/dev/fb0");
    
    // 初始化按键
    printf("WiFi Manager Started\n");
    
    // 初始扫描
    scan_wifi_networks();
    selected_index = 0;
    
    int last_key0 = -1, last_key1 = -1, last_key2 = -1, last_key3 = -1;
    
    while(1)
    {
        // 读取按键状态
        int key0 = gpio_get_level(KEY_0);
        int key1 = gpio_get_level(KEY_1);
        int key2 = gpio_get_level(KEY_2);
        int key3 = gpio_get_level(KEY_3);
        
        if (show_password_selection) {
            // 密码选择界面
            if (key0 == 1 && last_key0 == 0) {  // 按键0按下（向上）
                password_selected_index--;
                if (password_selected_index < -1) {
                    password_selected_index = (int)matched_passwords.size() - 1;
                }
            }
            else if (key1 == 1 && last_key1 == 0) {  // 按键1按下（向下）
                password_selected_index++;
                if (password_selected_index >= (int)matched_passwords.size()) {
                    password_selected_index = -1;
                }
            }
            else if (key2 == 1 && last_key2 == 0) {  // 按键2按下（连接）
                std::string password;
                
                if (password_selected_index == -1) {
                    // 手动输入模式（这里简化处理，实际应该显示输入界面）
                    password = "12345678";  // 默认密码
                } else if (password_selected_index >= 0 && 
                          password_selected_index < (int)matched_passwords.size()) {
                    password = matched_passwords[password_selected_index].password;
                }
                
                // 尝试连接
                connected = connect_to_wifi(wifi_list[selected_index].ssid, password);
                if (connected) {
                    connected_ssid = wifi_list[selected_index].ssid;
                }
                
                show_password_selection = false;
                display_connection_status();
                system_delay_ms(2000);
            }
            
            if (show_password_selection) {
                display_password_selection();
            }
        } else {
            // 主界面 - WiFi列表
            if (key0 == 1 && last_key0 == 0) {  // 按键0按下（向上）
                if (selected_index > 0) {
                    selected_index--;
                    if (selected_index < scroll_offset) {
                        scroll_offset = selected_index;
                    }
                }
            }
            else if (key1 == 1 && last_key1 == 0) {  // 按键1按下（向下）
                if (selected_index < (int)wifi_list.size() - 1) {
                    selected_index++;
                    if (selected_index >= scroll_offset + 10) {
                        scroll_offset = selected_index - 9;
                    }
                }
            }
            else if (key2 == 1 && last_key2 == 0) {  // 按键2按下（选择）
                if (!wifi_list.empty()) {
                    if (wifi_list[selected_index].has_password) {
                        // 查找匹配的预存密码
                        matched_passwords = find_matched_passwords(wifi_list[selected_index].ssid);
                        show_password_selection = true;
                        password_selected_index = matched_passwords.empty() ? -1 : 0;
                    } else {
                        // 开放网络，直接连接
                        connected = connect_to_wifi(wifi_list[selected_index].ssid, "");
                        if (connected) {
                            connected_ssid = wifi_list[selected_index].ssid;
                        }
                        display_connection_status();
                        system_delay_ms(2000);
                    }
                }
            }
            else if (key3 == 1 && last_key3 == 0) {  // 按键3按下（扫描）
                ips200_show_string(10, SCREEN_HEIGHT - 100, "Scanning...");
                scan_wifi_networks();
                selected_index = 0;
                scroll_offset = 0;
            }
            
            display_wifi_list();
        }
        
        // 更新上一次按键状态
        last_key0 = key0;
        last_key1 = key1;
        last_key2 = key2;
        last_key3 = key3;
        
        system_delay_ms(50);  // 减少延迟，提高响应速度
    }
}