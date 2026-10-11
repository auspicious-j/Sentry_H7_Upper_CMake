/**
 ******************************************************************************
 * @file	bsp_dwt.h
 * @author  Wang Hongxi
 * @version V1.1.0
 * @date    2022/3/8
 * @brief   DWT 周期计数器驱动的接口。
 *         DWT(Data Watchpoint and Trace，数据观察点与跟踪单元) 是 Cortex-M 内核
 *         自带的调试组件，其中的 CYCCNT 寄存器会在使能后随内核时钟自动自增。
 *         本模块用它实现“微秒级”的时间测量与延时，不占用任何硬件定时器，
 *         精度等于内核时钟周期(1/CPU主频)。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _BSP_DWT_H
#define _BSP_DWT_H


/* 定长整数类型：uint32_t / uint16_t / uint64_t 等 */
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 系统运行时间的“拆分表示”。
   由 DWT_SysTimeUpdate() 刷新，把“自 DWT_Init 起累计的时钟周期数”
   换算成整块的 秒 / 毫秒 / 微秒 三级。 */
typedef struct
{
    uint32_t s;   /* 整数秒。范围 0 ~ 4294967295(约 136 年) */
    uint16_t ms;  /* 秒以下的毫秒部分。范围 0 ~ 999 */
    uint16_t us;  /* 毫秒以下的微秒部分。范围 0 ~ 999 */
} DWT_Time_t;

/* 初始化 DWT 周期计数器。
   参数 CPU_Freq_mHz：内核时钟频率，单位 MHz(如 550 表示 550MHz)。
   内部据此换算出每秒/每毫秒/每微秒的周期数缓存备用，并清 0 计数器。 */
void DWT_Init(uint32_t CPU_Freq_mHz);

/* 计算“距上次调用”经过的时间(差分计时)。
   参数 cnt_last：传入传出，指向保存上次 CYCCNT 采样值的变量，调用后被本次值覆盖。
   返回：两次调用之间的时间，单位 秒(float)。
   用途：周期性调用即可得到每次调用的间隔(测周期/频率)。 */
float DWT_GetDeltaT(uint32_t *cnt_last);

/* 与 DWT_GetDeltaT 功能相同，但全程用 double 计算，精度更高、耗时略长。
   返回：两次调用之间的时间，单位 秒(double)。 */
double DWT_GetDeltaT64(uint32_t *cnt_last);

/* 获取自 DWT_Init 起累计的运行时间。返回单位：秒(float)。 */
float DWT_GetTimeline_s(void);

/* 获取自 DWT_Init 起累计的运行时间。返回单位：毫秒(float)。 */
float DWT_GetTimeline_ms(void);

/* 获取自 DWT_Init 起累计的运行时间。返回单位：微秒(uint64_t，整数、无浮点误差)。 */
uint64_t DWT_GetTimeline_us(void);

/* 阻塞式忙等延时(空循环等待周期计数达标)。
   参数 Delay：延时时间，单位 秒(float)。
   注意：占用 CPU 空转、不进行任务切换，仅适合极短延时。 */
void DWT_Delay(float Delay);

/* 根据当前 CYCCNT 刷新全局 SysTime(秒/毫秒/微秒)。
   一般无需手动调用，各 Timeline 函数内部会自动调用。 */
void DWT_SysTimeUpdate(void);

/* 全局系统时间，各成员单位见 DWT_Time_t。由 DWT_SysTimeUpdate() 更新。 */
extern DWT_Time_t SysTime;

#ifdef __cplusplus
}
#endif

#endif /* BSP_DWT_H_ */
