/**
 ******************************************************************************
 * @file	bsp_dwt.c
 * @author  Wang Hongxi
 * @version V1.1.0
 * @date    2022/3/8
 * @brief
 ******************************************************************************
 * @attention
 *
 * [LINUX-PORT] 本文件已针对 Ubuntu 重写：
 *   原实现直接访问 Cortex-M7 的 DWT->CYCCNT（CPU 周期计数器）与 SysTick 寄存器，
 *   这两个寄存器在 x86-64 上不存在。
 *
 *   Linux 等价方案：clock_gettime(CLOCK_MONOTONIC) —— 单调、不受系统时间调整影响，
 *   精度通常为纳秒级（usleep 级别足以满足 1ms 控制环）。
 *   具体封装见 platform/linux/src/hal_port.c 中的 port_monotonic_us()。
 *
 *   对外接口（DWT_Init / DWT_GetDeltaT / DWT_GetTimeline_* / DWT_Delay）保持不变。
 ******************************************************************************
 */
#include "bsp_dwt.h"
#include "port_linux.h" /* [LINUX-PORT] 替代 "main.h" 中的内核寄存器 */

#include <time.h> /* [LINUX-PORT] DWT_Delay 使用 nanosleep */

DWT_Time_t SysTime;
uint64_t CYCCNT64;

/* [LINUX-PORT] 记录初始化时刻，作为时间轴原点 */
static uint64_t s_dwt_start_us = 0;

void DWT_Init(uint32_t CPU_Freq_mHz)
{
    (void)CPU_Freq_mHz; /* [LINUX-PORT] 原为 CPU 主频，用于 CYCCNT 换算；这里不再需要 */
    s_dwt_start_us = port_monotonic_us();
    SysTime.s = 0;
    SysTime.ms = 0;
    SysTime.us = 0;
}

/* [LINUX-PORT] 用 32 位微秒计数器模拟 DWT->CYCCNT（溢出由无符号减法自动处理） */
static uint32_t DWT_GetCounter(void)
{
    return (uint32_t)(port_monotonic_us() - s_dwt_start_us);
}

float DWT_GetDeltaT(uint32_t *cnt_last)
{
    uint32_t cnt_now = DWT_GetCounter();
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / 1000000.0f;
    *cnt_last = cnt_now;

    return dt;
}

double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    uint32_t cnt_now = DWT_GetCounter();
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / 1000000.0;
    *cnt_last = cnt_now;

    return dt;
}

void DWT_SysTimeUpdate(void)
{
    uint64_t now_us = port_monotonic_us() - s_dwt_start_us;

    CYCCNT64 = now_us;

    SysTime.s = (uint32_t)(now_us / 1000000ULL);
    SysTime.ms = (uint16_t)((now_us % 1000000ULL) / 1000ULL);
    SysTime.us = (uint16_t)(now_us % 1000ULL);
}

float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;

    return DWT_Timelinef32;
}

float DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s * 1000 + SysTime.ms + SysTime.us * 0.001f;

    return DWT_Timelinef32;
}

uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();

    uint64_t DWT_Timelinef32 = SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;

    return DWT_Timelinef32;
}

void DWT_Delay(float Delay)
{
    /* [LINUX-PORT] 原为忙等 CPU 周期；Linux 侧改用 nanosleep，避免占满一个核心 */
    if (Delay <= 0.0f)
        return;

    uint32_t us = (uint32_t)(Delay * 1000000.0f);
    port_sleep_ms(us / 1000U);

    uint32_t remain_us = us % 1000U;
    if (remain_us > 0U)
    {
        struct timespec req;
        req.tv_sec = 0;
        req.tv_nsec = (long)remain_us * 1000L;
        nanosleep(&req, NULL);
    }
}
