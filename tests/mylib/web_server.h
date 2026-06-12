#ifndef WEB_SERVER_H
#define WEB_SERVER_H

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

// 简化using声明
using std::cout;
using std::cerr;
using std::endl;
using std::string;
using std::stringstream;
using std::thread;
using std::this_thread::sleep_for;
using std::chrono::seconds;
using std::atomic;
using std::random_device;
using std::mt19937;
using std::uniform_int_distribution;
using std::time;
using std::signal;

// 全局变量声明
extern std::atomic<bool> running;
extern httplib::Server* g_server;

// 函数声明
void signal_handler(int signal);
void handle_sse_stream(const httplib::Request& req, httplib::Response& res);
std::string read_file(const std::string& filename);
void handle_root(const httplib::Request& req, httplib::Response& res);
void setup_routes(httplib::Server& svr);
void setup_static_file_handlers(httplib::Server& svr);
void print_server_info();
int start_web_server();

#endif // WEB_SERVER_H
