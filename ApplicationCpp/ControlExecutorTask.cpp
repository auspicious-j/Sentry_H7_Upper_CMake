//=============================================================================
// 文件：ControlExecutorTask.cpp
// 所属：应用层 / 平台桥接层（ApplicationCpp）
//
// 一、文件定位
//   本文件是 C++ 插件框架与 FreeRTOS 之间的“粘合层”之一：只负责把插件图
//   （ApplicationCpp/Framework 中的节点网络）按固定节奏“驱动”起来，本身不含
//   任何具体机器人业务逻辑；业务逻辑全部封装在插件节点内部。
//
// 二、帧循环节奏（1 kHz）
//   控制任务以 1 ms 为周期循环执行：每一轮先调用 AppCpp_ProcessFrame() 运行
//   一帧插件图，随后用 vTaskDelayUntil() 阻塞到下一个周期点，从而形成
//   1000 Hz（1 kHz）的控制帧率。这也是机器人闭环控制中常见的控制周期。
//
// 三、为何使用 vTaskDelayUntil 而不是 vTaskDelay / osDelay
//   vTaskDelay / osDelay 是“从现在起再延时 N 个 tick”，会把上一帧自身的计算
//   耗时叠加进周期中，长期运行会造成周期漂移；而 vTaskDelayUntil 以固定的
//   “上一次唤醒时刻”为参考点，即使某一帧计算偏慢，也能把后续帧拉回统一的
//   时间网格，保证周期稳定。注意：插件自身不创建任务、也不调用
//   vTaskDelayUntil，任务调度完全由本层掌握，从而与插件业务保持解耦。
//
// 四、与 C 侧的接口
//   AppCpp_ControlTask 以 extern "C" 导出，供 CubeMX 生成的 C 代码（freertos.c
//   中的任务创建代码）按 C 符号名直接引用，并作为该任务的入口函数。
//=============================================================================

#include "app_cpp_entry.h"

#include "FreeRTOS.h"
#include "task.h"

// 任务属于平台桥接层；插件本身不创建任务、不调用 vTaskDelayUntil。
// 参数 argument：FreeRTOS 创建任务时传入的形参（void* 通用指针），本项目未使用。
// extern "C"：禁用 C++ 名称修饰（name mangling），使该函数以 C 语言符号名
//   AppCpp_ControlTask 导出，便于 C 代码（freertos.c）链接并调用。
extern "C" void AppCpp_ControlTask(void const* argument)
{
    // argument：FreeRTOS 任务参数，当前未使用。
    // (void)argument：显式忽略该参数，避免编译器报“未使用参数”告警。
    (void)argument;
    // vTaskDelayUntil 保证周期参考点固定，避免单次计算时间造成周期漂移。
    // last_wake：上一次被唤醒（阻塞结束）的时刻，作为周期性延时的基准参考点；
    //   单位为 tick（FreeRTOS 系统节拍）。初值取当前 tick，即把“任务刚启动的
    //   时刻”当作第一帧的时间基准。
    TickType_t last_wake = xTaskGetTickCount();
    // period：任务周期，当前为 1 ms（pdMS_TO_TICKS 把毫秒换算成 tick 数，
    //   默认配置下 1 tick = 1 ms）；即每 1 ms 驱动一帧插件图。
    const TickType_t period = pdMS_TO_TICKS(1U);

    for (;;) {
        // 业务图在这里运行；任务调度与插件业务保持分离。
        // AppCpp_ProcessFrame()：执行一帧 C++ 插件图（采集输入 → 逐节点计算 →
        //   下发输出），是插件框架每帧的唯一入口。
        AppCpp_ProcessFrame();
        // 阻塞到“上一次唤醒时刻 + period”这一绝对时间点，实现固定周期调度：
        //   若本帧耗时为 t，则实际阻塞时间为 period - t（不小于 0），从而消除
        //   累计漂移。第一个参数传入 last_wake 的地址，函数会就地更新它。
        vTaskDelayUntil(&last_wake, period);
    }
}
