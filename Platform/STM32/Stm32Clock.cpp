/**
 ******************************************************************************
 * @file    stm32_clock.cpp
 * @brief   stm32_clock（STM32 时钟后端）的实现：直接读取 bsp_dwt 提供的
 *          单调微秒时间轴。
 *
 * 时间来源与精度：
 *  - 底层 DWT->CYCCNT 是 Cortex-M 内核的 32 位周期计数器，按 CPU 主频递增，
 *    分辨率约为一个 CPU 时钟周期（例如 480MHz 时约 2ns），精度远高于 FreeRTOS
 *    的 tick（通常 1ms），因此适合做高精度耗时测量与控制周期统计。
 *  - CYCCNT 是 32 位，回绕（溢出）周期约为 2^32 / 主频（480MHz 时约 8.9s）。
 *    bsp_dwt 内部通过 DWT_CNT_Update() 检测回绕并累加 64 位计数，再换算成
 *    微秒返回，从而对外表现为“单调不减、不会因回绕而跳变”的时钟。
 *  - 本类只测量时间轴，不参与 FreeRTOS 的任务调度/延时，两者互不替代。
 ******************************************************************************
 */

#include "Stm32Clock.hpp"

#include "bsp_dwt.h"

namespace robot::platform::stm32 {

// now_us()：返回自 DWT 计时起点以来的单调微秒数。
// 实现：转发调用 DWT_GetTimeline_us()，该函数内部读取 CYCCNT、处理 32 位
// 回绕并拼接为 64 位计数，再换算成微秒后返回。
// 单位：微秒（us）。单调（不会因回绕而跳变），适合求两次调用之差作为时间间隔。
// DWT 用于高精度测量，不用于替代 FreeRTOS 任务调度。
uint64_t stm32_clock::now_us() const
{
    return DWT_GetTimeline_us();
}

} // namespace robot::platform::stm32
