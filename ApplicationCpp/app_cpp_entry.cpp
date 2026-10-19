#include "app_cpp_entry.h"

#include "RobotApplication.hpp"

//=============================================================================
// 文件：app_cpp_entry.cpp
// 所属：应用层 / C 与 C++ 的桥接实现（ApplicationCpp）
//
// 一、文件定位
//   本文件是 app_cpp_entry.h 所声明接口中前两个函数（AppCpp_Initialize、
//   AppCpp_ProcessFrame）的具体实现，作用是把来自 C 侧的调用转发给 C++ 的
//   robot_application 对象。C 代码（main.c、freertos.c）只认识 C 符号，真正的
//   C++ 类与对象都封装在本文件内部，不外泄。
//   （第三个函数 AppCpp_ControlTask 的实现位于 ControlExecutorTask.cpp。）
//
// 二、对象所有权
//   全局仅有唯一一个 robot_application 实例 g_application，它定义在匿名命名
//   空间 namespace { ... } 中：匿名命名空间为其赋予“内部链接”，即该对象只在
//   当前编译单元（本 .cpp）内可见，不会被其他文件按名字引用，从而实现 C++
//   对象的独占与封装。C 层无法直接触及该对象，只能通过下面两个 extern "C"
//   函数间接访问，从而保持 C 与 C++ 之间边界干净。
//
// 三、调用链
//   main.c → AppCpp_Initialize() → g_application.initialize()
//   freertos.c 创建的任务 → AppCpp_ControlTask()（见 ControlExecutorTask.cpp）
//     → 每 1 ms 一帧 → AppCpp_ProcessFrame() → g_application.process_frame()
//=============================================================================

// 应用对象由 C++ 层独占；C 层只通过 extern "C" 函数访问。
// 匿名命名空间赋予其内部链接，仅在当前编译单元可见。
namespace {
// g_application：全局唯一的机器人应用“组合根”对象，负责装配并驱动插件图；
//   在程序启动的静态构造阶段即被构造，供下面两个桥接函数调用。
robot::application::robot_application g_application;
}

// 在 CubeMX 外设初始化完成后调用；当前只启动观测型插件图。
// 用途：初始化 C++ 应用（装配插件图、准备依赖注入的服务）。
// 调用时机：CubeMX 生成的 main.c 在全部外设初始化完成之后调用一次。
// 参数：无。返回值：无。
// (void)：显式丢弃 initialize() 的返回值（plugin_status），避免“返回值未使用”
//   告警；初始化结果保存在应用对象内部。
extern "C" void AppCpp_Initialize(void)
{
    (void)g_application.initialize();
}

// 由 FreeRTOS 控制任务每 1 ms 调用一次。
// 用途：执行一帧插件图（采集输入 → 逐节点计算 → 下发输出）。
// 调用时机：AppCpp_ControlTask() 在其 1 ms 周期的循环中每次调用；注意本函数
//   只运行一帧便返回，周期节奏由调用方用 vTaskDelayUntil 维持。
// 参数：无。返回值：无。
// (void)：显式丢弃 process_frame() 的返回值（process_result），避免告警。
extern "C" void AppCpp_ProcessFrame(void)
{
    (void)g_application.process_frame();
}
