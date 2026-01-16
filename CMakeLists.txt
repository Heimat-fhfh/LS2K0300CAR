cmake_minimum_required(VERSION 3.16)    # 限制最低CMake版本

# 设置C语言编译器、C++编译器的路径和名称
set(CMAKE_C_COMPILER /opt/loongarch-gnu-toolchain/bin/loongarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /opt/loongarch-gnu-toolchain/bin/loongarch64-linux-gnu-g++)

# 项目名称
project(main)

file(GLOB SRC ${PROJECT_SOURCE_DIR}/src/*.cpp)      # 获取包含的源文件
include_directories(${PROJECT_SOURCE_DIR}/include)  # 指定所需头文件路径
include_directories(/opt/loongarch-gnu-toolchain/loongarch64-linux-gnu/sysroot/usr/include/)
include_directories(${PROJECT_SOURCE_DIR}/third_party/cpp-httplib-master)  # cpp-httplib头文件路径

link_libraries(pthread)

# 指定OpenCV所用头文件路径
include_directories(/home/fhfh/Work/LS2K0300CAR/opencv_4_10_build/include/opencv4)
# 找到opencv所需的库文件并保存到变量中
set(OPENCV_CORE /home/fhfh/Work/LS2K0300CAR/opencv_4_10_build/lib/libopencv_core.so)
set(OPENCV_IMGPROC /home/fhfh/Work/LS2K0300CAR/opencv_4_10_build/lib/libopencv_imgproc.so)
set(OPENCV_HIGHGUI /home/fhfh/Work/LS2K0300CAR/opencv_4_10_build/lib/libopencv_highgui.so)
set(OPENCV_VIDEOIO /home/fhfh/Work/LS2K0300CAR/opencv_4_10_build/lib/libopencv_videoio.so)
set(OPENCV_IMGCODECS /home/fhfh/Work/LS2K0300CAR/opencv_4_10_build/lib/libopencv_imgcodecs.so)
# find_library(OPENCV_CORE      opencv_core      PATHS /home/fhfh/Work/LS2K0300CAR/opencv_4_10_build/lib)
# find_library(OPENCV_IMGPROC   opencv_imgproc   PATHS /home/fhfh/Work/LS2K0300CAR/opencv_4_10_build/lib)
# find_library(OPENCV_HIGHGUI   opencv_highgui   PATHS /home/fhfh/Work/LS2K0300CAR/opencv_4_10_build/lib)
# find_library(OPENCV_VIDEOIO   opencv_videoio   PATHS /home/fhfh/Work/LS2K0300CAR/opencv_4_10_build/lib)
# find_library(OPENCV_IMGCODECS opencv_imgcodecs PATHS /home/fhfh/Work/LS2K0300CAR/opencv_4_10_build/lib)

# link_directories(/home/fhfh/Work/LS2K0300CAR/opencv_4_10_build/lib)
# set(CMAKE_PREFIX_PATH "/home/zmr/Desktop/opencv_4_10_build") 

# 若需使用静态库，可带上下面两行程序，具体可参考龙邱B站视频教程
# link_directories(${PROJECT_SOURCE_DIR}/a)
# link_libraries(ss)

# 添加第三方web库cpp-httplib（HTTP服务器/客户端库）
# 禁用可选依赖以避免交叉编译问题
set(HTTPLIB_USE_OPENSSL_IF_AVAILABLE OFF)
set(HTTPLIB_USE_ZLIB_IF_AVAILABLE OFF)
set(HTTPLIB_USE_BROTLI_IF_AVAILABLE OFF)
set(HTTPLIB_USE_ZSTD_IF_AVAILABLE OFF)
set(HTTPLIB_USE_NON_BLOCKING_GETADDRINFO OFF)
# 启用编译httplib库（默认是头文件库，设为ON可看到编译过程）
set(HTTPLIB_COMPILE ON)
add_subdirectory(third_party/cpp-httplib-master)

# 由源文件生成一个可执行程序
add_executable(main main.cpp ${SRC} ${TEST})

# 连接OpenCV动态库
target_link_libraries(main PUBLIC ${OPENCV_CORE} ${OPENCV_IMGPROC} ${OPENCV_HIGHGUI} ${OPENCV_VIDEOIO} ${OPENCV_IMGCODECS})
# 连接第三方web库cpp-httplib
target_link_libraries(main PUBLIC httplib::httplib)
# 连接多线程pthread库
target_link_libraries(main PUBLIC pthread)
