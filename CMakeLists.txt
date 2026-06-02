cmake_minimum_required(VERSION 3.16)    # 限制最低CMake版本

# 默认按边缘设备工具链构建；服务器端本机构建时可在配置阶段关闭。
option(USE_EDGE_TOOLCHAIN "Build with LoongArch edge toolchain" ON)

# Guard against mixing LoongArch compiler with host OpenCV libs from /usr/local.
if (CMAKE_CXX_COMPILER MATCHES "loongarch64" AND NOT USE_EDGE_TOOLCHAIN)
    message(WARNING "Detected LoongArch compiler; forcing USE_EDGE_TOOLCHAIN=ON to avoid host OpenCV linkage.")
    set(USE_EDGE_TOOLCHAIN ON CACHE BOOL "Build with LoongArch edge toolchain" FORCE)
endif()

# 设置C语言编译器、C++编译器的路径和名称
if (USE_EDGE_TOOLCHAIN)
    set(CMAKE_C_COMPILER /opt/loongarch-gnu-toolchain/bin/loongarch64-linux-gnu-gcc)
    set(CMAKE_CXX_COMPILER /opt/loongarch-gnu-toolchain/bin/loongarch64-linux-gnu-g++)
endif()

# 设置C++标准为C++17（支持filesystem）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 项目名称
project(main)

file(GLOB SRC ${PROJECT_SOURCE_DIR}/src/*.cpp)      # 获取包含的源文件
include_directories(${PROJECT_SOURCE_DIR}/include)  # 指定所需头文件路径
if (USE_EDGE_TOOLCHAIN)
    include_directories(/opt/loongarch-gnu-toolchain/loongarch64-linux-gnu/sysroot/usr/include/)
endif()
include_directories(${PROJECT_SOURCE_DIR}/third_party/cpp-httplib-master)  # cpp-httplib头文件路径

link_libraries(pthread)

# 指定OpenCV所用头文件路径
if (USE_EDGE_TOOLCHAIN)
    include_directories(/home/fhfh/Work/LS2K0300CAR/opencv_4_10_build/include/opencv4)
    set(OPENCV_LIB_DIR /home/fhfh/Work/LS2K0300CAR/opencv_4_10_build/lib)
    # 找到opencv所需的库文件并保存到变量中
    set(OPENCV_CORE ${OPENCV_LIB_DIR}/libopencv_core.so)
    set(OPENCV_IMGPROC ${OPENCV_LIB_DIR}/libopencv_imgproc.so)
    set(OPENCV_HIGHGUI ${OPENCV_LIB_DIR}/libopencv_highgui.so)
    set(OPENCV_VIDEOIO ${OPENCV_LIB_DIR}/libopencv_videoio.so)
    set(OPENCV_IMGCODECS ${OPENCV_LIB_DIR}/libopencv_imgcodecs.so)
    set(OPENCV_DNN ${OPENCV_LIB_DIR}/libopencv_dnn.so)
    set(OPENCV_CALIB3D ${OPENCV_LIB_DIR}/libopencv_calib3d.so)
    set(OPENCV_FEATURES2D ${OPENCV_LIB_DIR}/libopencv_features2d.so)
    set(OPENCV_FLANN ${OPENCV_LIB_DIR}/libopencv_flann.so)
    set(OPENCV_LINK_LIBS
        ${OPENCV_CORE}
        ${OPENCV_IMGPROC}
        ${OPENCV_HIGHGUI}
        ${OPENCV_VIDEOIO}
        ${OPENCV_IMGCODECS}
        ${OPENCV_DNN}
        ${OPENCV_CALIB3D}
        ${OPENCV_FEATURES2D}
        ${OPENCV_FLANN}
    )
else()
    find_package(OpenCV REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})
    set(OPENCV_LIB_DIR "")
    set(OPENCV_LINK_LIBS ${OpenCV_LIBS})
endif()




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

# 交叉链接时为 ld 提供动态库依赖搜索路径，避免 calib3d 的间接依赖找不到。
if (USE_EDGE_TOOLCHAIN)
    target_link_options(main PUBLIC -Wl,-rpath-link,${OPENCV_LIB_DIR})
endif()

# 连接OpenCV动态库
target_link_libraries(main PUBLIC ${OPENCV_LINK_LIBS})
# 连接第三方web库cpp-httplib
target_link_libraries(main PUBLIC httplib::httplib)
# 连接多线程pthread库
target_link_libraries(main PUBLIC pthread)
# 连接C++17 filesystem库（对于某些编译器可能需要）
target_link_libraries(main PUBLIC stdc++fs)

# 上位机图像处理离线测试（WebUI展示）
# 该目标仅用于上位机调试，边缘设备构建时不参与构建。
if (NOT USE_EDGE_TOOLCHAIN)
    set(IMAGE_WEB_TEST_SRC
        ${PROJECT_SOURCE_DIR}/src/libimage_process.cpp
        ${PROJECT_SOURCE_DIR}/src/main_impl.cpp
        ${PROJECT_SOURCE_DIR}/src/camera_calibration.cpp
        ${PROJECT_SOURCE_DIR}/src/vision_transform.cpp
        ${PROJECT_SOURCE_DIR}/src/libdata_process.cpp
        ${PROJECT_SOURCE_DIR}/src/path_side_search.cpp
        ${PROJECT_SOURCE_DIR}/src/track_element_scan.cpp
        ${PROJECT_SOURCE_DIR}/src/path_across.cpp
        ${PROJECT_SOURCE_DIR}/src/path_circle.cpp
        ${PROJECT_SOURCE_DIR}/src/path_refactor.cpp
    )

    add_executable(image_web_test
        ${PROJECT_SOURCE_DIR}/tools/image_web_test_main.cpp
        ${IMAGE_WEB_TEST_SRC}
    )

    if (USE_EDGE_TOOLCHAIN)
        target_link_options(image_web_test PUBLIC -Wl,-rpath-link,${OPENCV_LIB_DIR})
    endif()

    target_link_libraries(image_web_test PUBLIC ${OPENCV_LINK_LIBS})
    target_link_libraries(image_web_test PUBLIC httplib::httplib)
    target_link_libraries(image_web_test PUBLIC pthread)
    target_link_libraries(image_web_test PUBLIC stdc++fs)
else()
    message(STATUS "Skip image_web_test: edge-device build enabled")
endif()
