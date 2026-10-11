/**
 ******************************************************************************
 * @file    Stm32Clock.hpp
 * @brief   STM32 平台时钟后端：把 Cortex-M 内核的 DWT 周期计数器封装成
 *          IClock 接口所要求的“单调微秒时钟”。
 *
 * 设计要点：
 *  - 平台无关的上层（ApplicationCpp）只依赖 robot::platform::IClock；
 *    本文件是 STM32 这一侧的具体实现（后端）。将来迁到 Linux 时，只要换成
 *    另一个基于 clock_gettime(CLOCK_MONOTONIC) 的后端即可，上层代码不必改动。
 *  - “单调时钟”指读数只增不减，只用来测量两个时刻之间的间隔（例如控制周期
 *    dt、代码段耗时），不能用来表示日历时间。
 *  - 时间基准来自 DWT->CYCCNT：它是按 CPU 主频计数的 32 位周期计数器。因为是
 *    32 位，会发生回绕（溢出），底层 bsp_dwt 通过累计回绕次数拼出 64 位计数，
 *    再换算成微秒，细节见 bsp_dwt.c 的 DWT_SysTimeUpdate()。
 *  - DWT 的初始化（使能 TRCENA/CYCCNTENA、设置 CPU 频率）不由本类负责，仍由
 *    现有 CubeMX 用户代码调用 DWT_Init() 完成；本类只做读取。
 ******************************************************************************
 */

#ifndef ROBOT_STM32_CLOCK_HPP
#define ROBOT_STM32_CLOCK_HPP

#include "../../ApplicationCpp/Platform/IClock.hpp"

namespace robot::platform::stm32 {

/**
 * STM32 时钟后端（具体实现类）。
 *
 * 继承 robot::platform::IClock，为其纯虚接口 nowUs() 提供 STM32 实现。
 * final 表示不再被继承；本类无状态（不持有任何成员），nowUs() 为 const，
 * 因此多任务并发读取时无需额外加锁，仅依赖 DWT 硬件计数器。
 *
 * 注意：DWT 初始化仍由现有 CubeMX 用户代码负责，本类假定 DWT 已就绪。
 */
class Stm32Clock final : public robot::platform::IClock {
public:
    // 返回自 DWT 计时起点以来的单调微秒数。
    // 单位：微秒（us）。返回值单调不减（除 64 位计数极小概率回绕外），
    // 适合计算时间间隔，不适合表示挂钟时间。
    uint64_t nowUs() const override;
};

} // namespace robot::platform::stm32

#endif /* ROBOT_STM32_CLOCK_HPP */
