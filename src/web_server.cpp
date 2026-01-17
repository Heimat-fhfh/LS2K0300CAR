#include "web_server.h"
#include <httplib.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <sstream>
#include <random>
#include <csignal>
#include <fstream>
#include <streambuf>
#include "json.hpp"
#include "zf_common_headfile.h"

#define BEEP "/dev/zf_driver_gpio_beep"


// 全局变量定义
std::atomic<bool> running(false);
httplib::Server* g_server = nullptr;

// 智能车控制状态变量
static int power_value = 0;
static int steer_angle = 180;
static bool car_stopped = false;
static bool buzzer_state = false;

// 配置文件路径
static const std::string CONFIG_DIR = "config/";

// 信号处理函数
void signal_handler(int signal) {
    cout << "\n收到终止信号 " << signal << ", 正在关闭服务器..." << endl;
    running = false;
    if (g_server) {
        g_server->stop();
    }
}

// SSE流处理函数
void handle_sse_stream(const httplib::Request& req, httplib::Response& res) {
    // 设置SSE相关的HTTP头
    res.set_header("Content-Type", "text/event-stream");
    res.set_header("Cache-Control", "no-cache");
    res.set_header("Connection", "keep-alive");
    res.set_header("Access-Control-Allow-Origin", "*");
    
    // 设置块传输编码（支持持续流式响应）
    res.set_chunked_content_provider("text/event-stream", 
        [&](size_t offset, httplib::DataSink& sink) {
            // 初始化随机数生成器
            random_device rd;
            mt19937 gen(rd());
            uniform_int_distribution<> temp_dist(20, 35);
            uniform_int_distribution<> humid_dist(30, 80);
            
            int count = 0;
            while (running && count < 100) { // 限制发送100条消息，避免无限循环
                count++;
                
                // 生成随机数据
                int temperature = temp_dist(gen);
                int humidity = humid_dist(gen);
                
                // 构建SSE格式的消息
                stringstream ss;
                
                // 事件类型（可选）
                ss << "event: sensor-data\n";
                
                // 数据行
                ss << "data: {\"temperature\": " << temperature 
                   << ", \"humidity\": " << humidity 
                   << ", \"timestamp\": " << time(nullptr) 
                   << ", \"message\": \"Data #" << count << "\"}\n";
                
                // 注释行（可选，可用于保持连接）
                ss << ": heartbeat\n";
                
                // 空行表示消息结束
                ss << "\n";
                
                string message = ss.str();
                
                // 发送数据块
                if (!sink.write(message.c_str(), message.size())) {
                    cout << "客户端断开连接" << endl;
                    break;
                }
                
                // 等待1秒
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            
            // 发送结束消息
            string end_msg = "event: end\ndata: {\"status\": \"finished\"}\n\n";
            sink.write(end_msg.c_str(), end_msg.size());
            sink.done();
            
            return true;
        }
    );
}

// 读取文件内容的辅助函数
std::string read_file(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return "";
    }
    std::string content((std::istreambuf_iterator<char>(file)),
                       std::istreambuf_iterator<char>());
    return content;
}

// 处理根路径请求，返回静态HTML文件
void handle_root(const httplib::Request& req, httplib::Response& res) {
    string html = read_file("web/index.html");
    if (html.empty()) {
        res.status = 404;
        res.set_content("File not found", "text/plain");
        return;
    }
    res.set_content(html, "text/html");
}

// 设置静态文件处理器
void setup_static_file_handlers(httplib::Server& svr) {
    // 静态文件服务 - 提供CSS文件
    svr.Get("/css/(.*)", [&](const httplib::Request& req, httplib::Response& res) {
        string filename = "web/css/" + req.matches[1].str();
        string content = read_file(filename);
        if (content.empty()) {
            res.status = 404;
            res.set_content("File not found", "text/plain");
            return;
        }
        res.set_content(content, "text/css");
    });
    
    // 静态文件服务 - 提供JS文件
    svr.Get("/js/(.*)", [&](const httplib::Request& req, httplib::Response& res) {
        string filename = "web/js/" + req.matches[1].str();
        string content = read_file(filename);
        if (content.empty()) {
            res.status = 404;
            res.set_content("File not found", "text/plain");
            return;
        }
        res.set_content(content, "application/javascript");
    });
    
    // 静态文件服务 - 提供图片文件（如果需要）
    svr.Get("/image/(.*)", [&](const httplib::Request& req, httplib::Response& res) {
        string filename = "web/image/" + req.matches[1].str();
        string content = read_file(filename);
        if (content.empty()) {
            res.status = 404;
            res.set_content("File not found", "text/plain");
            return;
        }
        // 根据文件扩展名设置正确的MIME类型
        if (filename.find(".png") != string::npos) {
            res.set_content(content, "image/png");
        } else if (filename.find(".jpg") != string::npos || filename.find(".jpeg") != string::npos) {
            res.set_content(content, "image/jpeg");
        } else if (filename.find(".gif") != string::npos) {
            res.set_content(content, "image/gif");
        } else if (filename.find(".svg") != string::npos) {
            res.set_content(content, "image/svg+xml");
        } else {
            res.set_content(content, "application/octet-stream");
        }
    });
}

// 新增API接口处理函数

// 处理功率控制请求
void handle_power_control(const httplib::Request& req, httplib::Response& res) {
    try {
        auto json_data = nlohmann::json::parse(req.body);
        int power = json_data.at("power").get<int>();
        
        // 验证功率范围
        if (power < -100 || power > 100) {
            res.status = 400;
            res.set_content("{\"error\": \"功率值必须在-100到100之间\"}", "application/json");
            return;
        }
        
        power_value = power;
        car_stopped = false; // 取消停止状态
        
        cout << "设置功率: " << power << endl;
        
        res.set_content("{\"status\": \"success\", \"power\": " + std::to_string(power) + "}", "application/json");
    } catch (const std::exception& e) {
        res.status = 400;
        res.set_content("{\"error\": \"无效的请求数据\"}", "application/json");
    }
}

// 处理转向控制请求
void handle_steer_control(const httplib::Request& req, httplib::Response& res) {
    try {
        auto json_data = nlohmann::json::parse(req.body);
        int angle = json_data.at("angle").get<int>();
        
        // 验证角度范围
        if (angle < 0 || angle > 360) {
            res.status = 400;
            res.set_content("{\"error\": \"角度值必须在0到360之间\"}", "application/json");
            return;
        }
        
        steer_angle = angle;
        
        cout << "设置转向角度: " << angle << endl;
        
        res.set_content("{\"status\": \"success\", \"angle\": " + std::to_string(angle) + "}", "application/json");
    } catch (const std::exception& e) {
        res.status = 400;
        res.set_content("{\"error\": \"无效的请求数据\"}", "application/json");
    }
}

// 处理停止车辆请求
void handle_stop_car(const httplib::Request& req, httplib::Response& res) {
    try {
        auto json_data = nlohmann::json::parse(req.body);
        std::string command = json_data.at("command").get<std::string>();
        
        if (command == "stop") {
            car_stopped = true;
            power_value = 0; // 重置功率
            
            cout << "智能车已停止" << endl;
            
            res.set_content("{\"status\": \"stopped\"}", "application/json");
        } else {
            res.status = 400;
            res.set_content("{\"error\": \"无效的停止命令\"}", "application/json");
        }
    } catch (const std::exception& e) {
        res.status = 400;
        res.set_content("{\"error\": \"无效的请求数据\"}", "application/json");
    }
}

// 处理蜂鸣器控制请求
void handle_buzzer_control(const httplib::Request& req, httplib::Response& res) {
    try {
        auto json_data = nlohmann::json::parse(req.body);
        bool state = json_data.at("state").get<bool>();
        
        buzzer_state = state;
        gpio_set_level(BEEP, state ? 0x0 : 0x1); // 假设低电平开启蜂鸣器
        cout << "蜂鸣器状态: " << (state ? "开启" : "关闭") << endl;
        
        res.set_content("{\"status\": \"success\", \"buzzer_state\": " + std::to_string(state) + "}", "application/json");
    } catch (const std::exception& e) {
        res.status = 400;
        res.set_content("{\"error\": \"无效的请求数据\"}", "application/json");
    }
}

// 获取当前配置文件
void handle_get_config(const httplib::Request& req, httplib::Response& res) {
    std::string config_name = req.path_params.at("config_name");
    std::string filepath = CONFIG_DIR + config_name;
    
    if (config_name.find("..") != std::string::npos) {
        res.status = 400;
        res.set_content("{\"error\": \"无效的文件路径\"}", "application/json");
        return;
    }
    
    std::ifstream file(filepath);
    if (!file.is_open()) {
        res.status = 404;
        res.set_content("{\"error\": \"配置文件不存在\"}", "application/json");
        return;
    }
    
    std::string content((std::istreambuf_iterator<char>(file)),
                       std::istreambuf_iterator<char>());
    
    res.set_content(content, "application/json");
}

// 保存配置文件
void handle_save_config(const httplib::Request& req, httplib::Response& res) {
    std::string config_name = req.path_params.at("config_name");
    std::string filepath = CONFIG_DIR + config_name;
    
    if (config_name.find("..") != std::string::npos) {
        res.status = 400;
        res.set_content("{\"error\": \"无效的文件路径\"}", "application/json");
        return;
    }
    
    // 验证JSON格式
    try {
        auto json_data = nlohmann::json::parse(req.body);
        
        // 写入文件
        std::ofstream file(filepath);
        if (!file.is_open()) {
            res.status = 500;
            res.set_content("{\"error\": \"无法写入配置文件\"}", "application/json");
            return;
        }
        
        file << json_data.dump(4); // 格式化输出，缩进4个空格
        file.close();
        
        cout << "配置文件已保存: " << filepath << endl;
        
        res.set_content("{\"status\": \"success\", \"file\": \"" + config_name + "\"}", "application/json");
    } catch (const std::exception& e) {
        res.status = 400;
        res.set_content("{\"error\": \"无效的JSON格式\"}", "application/json");
    }
}

// 重新加载配置
void handle_reload_config(const httplib::Request& req, httplib::Response& res) {
    try {
        auto json_data = nlohmann::json::parse(req.body);
        std::string config_file = json_data.at("config_file").get<std::string>();
        
        // 这里可以添加实际的配置重载逻辑
        cout << "重新加载配置文件: " << config_file << endl;
        
        res.set_content("{\"status\": \"reloaded\", \"config_file\": \"" + config_file + "\"}", "application/json");
    } catch (const std::exception& e) {
        res.status = 400;
        res.set_content("{\"error\": \"无效的请求数据\"}", "application/json");
    }
}

// 上传配置文件
void handle_upload_config(const httplib::Request& req, httplib::Response& res) {
    // 检查Content-Type是否为multipart/form-data
    auto content_type = req.get_header_value("Content-Type");
    if (content_type.find("multipart/form-data") == std::string::npos) {
        res.status = 400;
        res.set_content("{\"error\": \"请使用multipart/form-data格式上传文件\"}", "application/json");
        return;
    }
    
    // 从请求体中解析文件内容
    // 这里简化处理，实际的multipart解析可能需要更复杂的逻辑
    if (!req.body.empty()) {
        try {
            // 尝试解析为JSON来验证格式
            auto json_data = nlohmann::json::parse(req.body);
            
            // 生成文件名（这里使用时间戳避免冲突）
            auto now = std::time(nullptr);
            std::string filename = "uploaded_config_" + std::to_string(now) + ".json";
            std::string filepath = CONFIG_DIR + filename;
            
            // 保存文件
            std::ofstream output_file(filepath);
            if (!output_file.is_open()) {
                res.status = 500;
                res.set_content("{\"error\": \"无法保存文件\"}", "application/json");
                return;
            }
            
            output_file << json_data.dump(4);
            output_file.close();
            
            cout << "配置文件已上传: " << filepath << endl;
            
            res.set_content("{\"status\": \"success\", \"file\": \"" + filename + "\"}", "application/json");
        } catch (const std::exception& e) {
            res.status = 400;
            res.set_content("{\"error\": \"无效的JSON格式\"}", "application/json");
        }
    } else {
        res.status = 400;
        res.set_content("{\"error\": \"没有找到上传的文件内容\"}", "application/json");
    }
}

// 设置路由
void setup_routes(httplib::Server& svr) {
    svr.Get("/", handle_root);
    svr.Get("/events", handle_sse_stream);
    
    // 智能车控制API
    svr.Post("/api/power", handle_power_control);
    svr.Post("/api/steer", handle_steer_control);
    svr.Post("/api/stop", handle_stop_car);
    svr.Post("/api/buzzer", handle_buzzer_control);
    
    // 配置文件API
    svr.Get("/api/config/(.*)", [](const httplib::Request& req, httplib::Response& res) {
        handle_get_config(req, res);
    });
    svr.Put("/api/config/(.*)", [](const httplib::Request& req, httplib::Response& res) {
        handle_save_config(req, res);
    });
    svr.Post("/api/reload-config", handle_reload_config);
    svr.Post("/api/upload-config", handle_upload_config);
    
    // 停止服务器的接口
    svr.Get("/stop", [&](const httplib::Request& req, httplib::Response& res) {
        running = false;
        res.set_content("服务器正在停止...", "text/plain");
    });
    
    // 健康检查接口
    svr.Get("/health", [](const httplib::Request& req, httplib::Response& res) {
        res.set_content("{\"status\": \"ok\"}", "application/json");
    });
}

// 打印服务器信息
void print_server_info() {
    cout << "服务器启动在 http://0.0.0.0:8080" << endl;
    cout << "访问 http://0.0.0.0:8080 查看SSE演示" << endl;
    cout << "SSE流地址: http://0.0.0.0:8080/events" << endl;
    cout << "按Ctrl+C停止服务器" << endl;
}

// 启动web服务器
int start_web_server() {
    httplib::Server svr;
    
    // 设置全局服务器指针用于信号处理
    g_server = &svr;
    
    // 注册信号处理器
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // 设置日志处理
    svr.set_logger([](const httplib::Request& req, const httplib::Response& res) {
        cout << req.method << " " << req.path << " -> " << res.status << endl;
    });
    
    // 设置路由和静态文件处理器
    setup_routes(svr);
    setup_static_file_handlers(svr);
    
    print_server_info();
    
    // 服务器开始运行
    running = true;
    
    // 启动服务器
    if (!svr.listen("0.0.0.0", 8080)) {
        cerr << "启动服务器失败" << endl;
        // 服务器启动失败，设置running为false
        running = false;
        return 1;
    }
    
    // 服务器正常停止后，设置running为false
    running = false;
    return 0;
}
