#
# [LINUX-PORT] 原生 Ubuntu (x86-64) 构建脚本
#
# 原工程使用 CMake + arm-none-eabi 工具链交叉编译 STM32H723 固件。
# 移植后改为用系统 GCC 编译成原生可执行文件，不再产生 .elf/.bin 固件。
#
# 编译方法：
#   cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
#   cmake --build build -j
# 或直接执行 ./build.sh
#
cmake_minimum_required(VERSION 3.16)

project(sentry_upper_linux C)

# 原工程要求 C11
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# 便于 clangd/VSCode 索引
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

add_executable(${PROJECT_NAME})

# ---------------------------------------------------------------------------
# [LINUX-PORT] 业务层源码（与原工程保持一致，仅排除了硬件相关文件）
# ---------------------------------------------------------------------------
file(GLOB_RECURSE APPLICATION_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/Application/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/BSP/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/math/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/UserMiddlewares/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/IMU/src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/IMU/Algorithm/src/*.c"
)

# [LINUX-PORT] BMI088 走 SPI 的真实驱动无法在 PC 上运行，
# 改由 platform/linux/src/bmi088_sim.c 提供同名的 BMI088_init / BMI088_read。
set(BMI088_HW_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/IMU/BMI088/BMI088Middleware.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/IMU/BMI088/BMI088driver.c"
)
list(REMOVE_ITEM APPLICATION_SOURCES ${BMI088_HW_SOURCES})

# ---------------------------------------------------------------------------
# [LINUX-PORT] 移植层源码
# ---------------------------------------------------------------------------
file(GLOB LINUX_PORT_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/platform/linux/src/*.c"
)

target_sources(${PROJECT_NAME} PRIVATE
    ${APPLICATION_SOURCES}
    ${LINUX_PORT_SOURCES}
)

# ---------------------------------------------------------------------------
# [LINUX-PORT] 头文件路径
# platform/linux/inc 放在最前面，用于覆盖原 Core/Inc 下的 HAL / CubeMX 头文件
# ---------------------------------------------------------------------------
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/platform/linux/inc

    # Application
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/inc

    # BSP
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/inc

    # Math
    ${CMAKE_CURRENT_SOURCE_DIR}/math/inc

    # UserMiddlewares
    ${CMAKE_CURRENT_SOURCE_DIR}/UserMiddlewares/inc

    # IMU
    ${CMAKE_CURRENT_SOURCE_DIR}/IMU/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/IMU/Algorithm/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/IMU/BMI088/inc
)

# [LINUX-PORT] 原工程链接的是 ARM 版 libCMSISDSP.a，此处改为系统数学库
target_link_libraries(${PROJECT_NAME} PRIVATE m pthread)

target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall
    -Wno-unused-parameter
    -Wno-unused-variable
    -Wno-unused-but-set-variable
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    LINUX_PORT=1
)
