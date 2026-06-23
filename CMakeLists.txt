cmake_minimum_required(VERSION 3.16)

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

# 设置C/C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 项目名称（固定为 main）
project(main)

# 编译与链接优化标志（仅对边缘工具链启用龙芯架构优化）
if (USE_EDGE_TOOLCHAIN)
    set(ARCH_FLAGS "-march=loongarch64 -mtune=loongarch64")
    set(OPT_FLAGS "-O3 -fPIC -ffunction-sections -fdata-sections -Wall")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OPT_FLAGS} ${ARCH_FLAGS} -pthread -Wno-delete-non-virtual-dtor -Wno-placement-new -Wno-switch -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unused-function")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OPT_FLAGS} ${ARCH_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections")
else()
    # 本机构建时也可添加通用优化（按需）
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -Wall")
endif()

# 源文件收集（递归所有子目录）
file(GLOB_RECURSE SRC CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/src/*.cpp)

# 头文件路径
include_directories(${PROJECT_SOURCE_DIR}/include)
if (USE_EDGE_TOOLCHAIN)
    include_directories(/opt/loongarch-gnu-toolchain/loongarch64-linux-gnu/sysroot/usr/include/)
endif()
include_directories(${PROJECT_SOURCE_DIR}/third_party/cpp-httplib-master)

# OpenCV 依赖配置（统一使用 find_package，边缘模式下手动指定库目录）
if (USE_EDGE_TOOLCHAIN)
    set(CMAKE_PREFIX_PATH "/home/fhfh/Work/LS2K0300CAR/opencv_4_10_build")
    set(OPENCV_LIB_DIR "/home/fhfh/Work/LS2K0300CAR/opencv_4_10_build/lib")
endif()
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV include: ${OpenCV_INCLUDE_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS})

# 第三方 web 库 cpp-httplib（HTTP服务器/客户端库）
set(HTTPLIB_USE_OPENSSL_IF_AVAILABLE OFF)
set(HTTPLIB_USE_ZLIB_IF_AVAILABLE OFF)
set(HTTPLIB_USE_BROTLI_IF_AVAILABLE OFF)
set(HTTPLIB_USE_ZSTD_IF_AVAILABLE OFF)
set(HTTPLIB_USE_NON_BLOCKING_GETADDRINFO OFF)
set(HTTPLIB_COMPILE ON)
add_subdirectory(third_party/cpp-httplib-master)

# 主程序可执行文件
add_executable(main main.cpp ${SRC})

# 为边缘设备设置 rpath-link，避免间接依赖找不到（使用手动定义的 OPENCV_LIB_DIR）
if (USE_EDGE_TOOLCHAIN)
    target_link_options(main PUBLIC -Wl,-rpath-link,${OPENCV_LIB_DIR})
    # 同时设置 INSTALL_RPATH 方便运行时查找
    set_target_properties(main PROPERTIES
        INSTALL_RPATH "${OPENCV_LIB_DIR}"
        BUILD_WITH_INSTALL_RPATH ON
    )
endif()

# 链接依赖库
target_link_libraries(main PUBLIC
    ${OpenCV_LIBS}
    httplib::httplib
    pthread
    stdc++fs   # C++17 filesystem
)

# 上位机图像处理离线测试（WebUI展示），仅非边缘构建时生成
if (NOT USE_EDGE_TOOLCHAIN)
    set(IMAGE_WEB_TEST_SRC
        ${PROJECT_SOURCE_DIR}/src/vision/libimage_process.cpp
        ${PROJECT_SOURCE_DIR}/src/vision/image_display.cpp
        ${PROJECT_SOURCE_DIR}/src/vision/main_impl.cpp
        ${PROJECT_SOURCE_DIR}/src/vision/image_my_zf.cpp
        ${PROJECT_SOURCE_DIR}/src/vision/vision_rings.cpp
        ${PROJECT_SOURCE_DIR}/src/vision/camera_calibration.cpp
        ${PROJECT_SOURCE_DIR}/src/vision/libdata_process.cpp
        ${PROJECT_SOURCE_DIR}/src/vision/path_side_search.cpp
        ${PROJECT_SOURCE_DIR}/src/vision/track_element_scan.cpp
        ${PROJECT_SOURCE_DIR}/src/vision/path_across.cpp
        ${PROJECT_SOURCE_DIR}/src/vision/path_circle.cpp
        ${PROJECT_SOURCE_DIR}/src/vision/path_refactor.cpp
    )

    add_executable(image_web_test
        ${PROJECT_SOURCE_DIR}/tools/image_web_test_main.cpp
        ${IMAGE_WEB_TEST_SRC}
    )

    if (USE_EDGE_TOOLCHAIN)
        target_link_options(image_web_test PUBLIC -Wl,-rpath-link,${OPENCV_LIB_DIR})
        set_target_properties(image_web_test PROPERTIES
            INSTALL_RPATH "${OPENCV_LIB_DIR}"
            BUILD_WITH_INSTALL_RPATH ON
        )
    endif()

    target_link_libraries(image_web_test PUBLIC
        ${OpenCV_LIBS}
        httplib::httplib
        pthread
        stdc++fs
    )
else()
    message(STATUS "Skip image_web_test: edge-device build enabled")
endif()
