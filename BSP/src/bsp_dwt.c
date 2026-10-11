/**
 ******************************************************************************
 * @file	bsp_dwt.c
 * @author  Wang Hongxi
 * @version V1.1.0
 * @date    2022/3/8
 * @brief   DWT 周期计数器驱动的实现。
 *         利用 Cortex-M 内核的 DWT->CYCCNT 寄存器对内核时钟周期计数，
 *         提供微秒级的时间测量(GetTimeline/GetDeltaT)与忙等延时(Delay)。
 *         由于 CYCCNT 只有 32 位，本文件通过统计“回绕次数”把它扩展成 64 位时间轴。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "bsp_dwt.h"
#include "main.h"
DWT_Time_t SysTime;   /* 全局系统时间(秒/毫秒/微秒)，定义见 bsp_dwt.h */

/* 内核时钟频率，由 DWT_Init 按传入的 MHz 换算并缓存：
   CPU_FREQ_Hz    = 每秒时钟周期数(Hz)
   CPU_FREQ_Hz_ms = 每毫秒周期数
   CPU_FREQ_Hz_us = 每微秒周期数(如 550MHz 时即 550) */
static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
/* CYCCNT 的回绕(溢出)次数。CYCCNT 是 32 位计数器，计满 2^32 个周期后回到 0，
   用它把 32 位计数扩展成 64 位总时长 */
static uint32_t CYCCNT_RountCount;
/* 上一次读取到的 CYCCNT 值，用于判断是否发生回绕(本次比上次小即为回绕) */
static uint32_t CYCCNT_LAST;
/* 扩展后的 64 位总周期数 = 回绕次数 A* 2^32 + 当前 CYCCNT */
uint64_t CYCCNT64;
/* 内部函数：采样时检测回绕并累加回绕次数(前置声明) */
static void DWT_CNT_Update(void);

/* 初始化 DWT 周期计数器。
   参数 CPU_Freq_mHz：内核时钟频率，单位 MHz。
   使能跟踪与 CYCCNT 计数，并预先算好 Hz/ms/us 三种换算系数。 */
void DWT_Init(uint32_t CPU_Freq_mHz)
{
    /* 使能 DWT 外设(DEMCR 的 TRCENA 位；不置位则 DWT 各寄存器无法使用) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* 将周期计数器 CYCCNT 清 0，作为计时起点 */
    DWT->CYCCNT = (uint32_t)0u;

    /* 使能 CYCCNT 计数(CTRL 的 CYCCNTENA 位，置位后随内核时钟自增) */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;    /* MHz -> Hz：每秒周期数 */
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;     /* 每毫秒周期数 */
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;  /* 每微秒周期数 */
    CYCCNT_RountCount = 0;                   /* 回绕计数清零，与计时起点对齐 */
}

/* 计算距上次调用经过的时间，并同步刷新回绕计数。
   参数 cnt_last：传入传出，保存上次 CYCCNT 采样值，调用后被本次值覆盖。
   返回：两次调用间隔，单位 秒(float)。 */
float DWT_GetDeltaT(uint32_t *cnt_last)
{
    /* 本次 CYCCNT 值(volatile 确保每次都真实读寄存器而非被优化) */
    volatile uint32_t cnt_now = DWT->CYCCNT;
    /* 无符号相减自动处理 32 位回绕：得到两次采样间的周期数，再除以频率得秒数 */
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;   /* 更新历史值，供下次调用使用 */

    DWT_CNT_Update();   /* 同步刷新回绕计数，保证长时间运行时间轴连续 */

    return dt;
}

/* 与 DWT_GetDeltaT 功能相同，但用 double 计算，精度更高、耗时略长。
   参数 cnt_last：传入传出，同上。
   返回：两次调用间隔，单位 秒(double)。 */
double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    /* 本次 CYCCNT 值(volatile 确保真实读寄存器) */
    volatile uint32_t cnt_now = DWT->CYCCNT;
    /* 无符号相减处理回绕，再用 double 除法，减少浮点精度损失 */
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;   /* 更新历史值 */

    DWT_CNT_Update();   /* 同步刷新回绕计数 */

    return dt;
}

/* 根据当前 CYCCNT 与回绕次数刷新全局 SysTime(秒/毫秒/微秒)。 */
void DWT_SysTimeUpdate(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;   /* 当前 CYCCNT 值 */
    /* 逐级取整拆分时间用的临时变量(static 避免每次重新分配) */
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    DWT_CNT_Update();   /* 先更新回绕计数，保证下面 64 位总周期数正确 */

    /* 扩展为 64 位总周期数：回绕次数 A* 2^32 + 当前计数值 */
    CYCCNT64 = (uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;                      /* 总秒数 */
    CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;          /* 秒以下的剩余周期数 */
    SysTime.s = CNT_TEMP1;
    SysTime.ms = CNT_TEMP2 / CPU_FREQ_Hz_ms;                 /* 剩余周期换算成毫秒部分 */
    CNT_TEMP3 = CNT_TEMP2 - SysTime.ms * CPU_FREQ_Hz_ms;     /* 毫秒以下的剩余周期数 */
    SysTime.us = CNT_TEMP3 / CPU_FREQ_Hz_us;                 /* 剩余周期换算成微秒部分 */
}

/* 获取自 DWT_Init 起累计的运行时间，单位 秒(float)。 */
float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();   /* 先刷新 SysTime */

    /* 三级时间合成秒：(秒) + (毫秒换算) + (微秒换算) */
    float DWT_Timelinef32 = SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;

    return DWT_Timelinef32;
}

/* 获取自 DWT_Init 起累计的运行时间，单位 毫秒(float)。 */
float DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();   /* 先刷新 SysTime */

    /* 三级时间合成毫秒：(秒换算) + (毫秒) + (微秒换算) */
    float DWT_Timelinef32 = SysTime.s * 1000 + SysTime.ms + SysTime.us * 0.001f;

    return DWT_Timelinef32;
}

/* 获取自 DWT_Init 起累计的运行时间，单位 微秒(uint64_t，整数、无浮点误差)。 */
uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();   /* 先刷新 SysTime */

    /* 三级时间合成微秒：(秒换算) + (毫秒换算) + (微秒) */
    uint64_t DWT_Timelinef32 = SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;

    return DWT_Timelinef32;
}

/* 检测 CYCCNT 是否回绕并累加回绕次数。
   原理：CYCCNT 为 32 位自增计数，计满后回到 0；当本次读取值小于上次时，
   即认为发生一次回绕，回绕计数 +1，从而把 32 位计数扩展为 64 位总时长。 */
static void DWT_CNT_Update(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;   /* 当前 CYCCNT 值 */

    if (cnt_now < CYCCNT_LAST)   /* 本次值比上次小 -> 判定为回绕 */
        CYCCNT_RountCount++;     /* 回绕次数累加 */

    CYCCNT_LAST = cnt_now;       /* 记录本次值，供下次比较 */
}

/* 忙等式延时：阻塞当前执行流直到经过指定时间。
   参数 Delay：延时时间，单位 秒(float)。
   原理：记录起始 CYCCNT，空循环等待 (当前值 - 起始值) 达到 Delay A* 频率。
   注意：占用 CPU 空转、不进行任务切换，仅适合极短延时。 */
void DWT_Delay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;   /* 延时起始周期数 */
    float wait = Delay;                 /* 待延时的时间(秒) */

    /* 周期差尚未达到目标则继续等待(无符号相减可自然处理回绕) */
    while ((DWT->CYCCNT - tickstart) < wait * (float)CPU_FREQ_Hz)
    {
    }
}
