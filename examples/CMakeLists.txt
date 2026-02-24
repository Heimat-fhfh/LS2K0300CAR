# PID示例程序CMake配置
cmake_minimum_required(VERSION 3.10)
project(PID_Examples)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加可执行文件
add_executable(pid_example pid_example.cpp)

# 包含目录
target_include_directories(pid_example PRIVATE ../include)

# 链接PID库（如果PID库已编译为静态库）
# 如果PID库是源代码形式，需要添加源文件
if(EXISTS ../src/PID.cpp)
    target_sources(pid_example PRIVATE ../src/PID.cpp)
endif()

# 设置输出目录
set_target_properties(pid_example PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# 编译选项
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(pid_example PRIVATE -g -O0 -Wall -Wextra)
else()
    target_compile_options(pid_example PRIVATE -O2 -Wall)
endif()

# 安装目标（可选）
install(TARGETS pid_example
    RUNTIME DESTINATION bin
)