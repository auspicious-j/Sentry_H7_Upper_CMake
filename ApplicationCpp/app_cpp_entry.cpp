#include "app_cpp_entry.h"

#include "RobotApplication.hpp"

// 应用对象由 C++ 层独占；C 层只通过 extern "C" 函数访问。
namespace {
robot::application::RobotApplication g_application;
}

// 在 CubeMX 外设初始化完成后调用；当前只启动观测型插件图。
extern "C" void AppCpp_Initialize(void)
{
    (void)g_application.initialize();
}

// 由 FreeRTOS 控制任务每 1 ms 调用一次。
extern "C" void AppCpp_ProcessFrame(void)
{
    (void)g_application.processFrame();
}
