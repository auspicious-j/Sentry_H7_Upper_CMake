/**
 * @file    cmsis_os.h
 * @brief   [LINUX-PORT] CMSIS-RTOS v1 的 Linux 替代实现（基于 POSIX pthread）
 *
 * 原工程使用 FreeRTOS + CMSIS-RTOS 封装（Middlewares/Third_Party/FreeRTOS）。
 * Linux 上的等价方案是 POSIX 线程：
 *
 *   osThreadCreate  ->  pthread_create        （见 platform/linux/src/tasks_linux.c）
 *   osDelay(ms)     ->  clock_nanosleep       （见 platform/linux/src/os_port.c）
 *   任务挂起/恢复   ->  每个线程一个信号量    （见 platform/linux/src/os_port.c）
 *
 * 注意：Linux 普通用户无法设置实时优先级，任务按 CFS 调度运行；
 *       控制环周期为 1~2ms，在桌面环境下的抖动会比 MCU 大。
 */
#ifndef __CMSIS_OS_H__
#define __CMSIS_OS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"

/* [LINUX-PORT] 与原 CMSIS-RTOS v1 一致：线程句柄就是 void* */
typedef void *osThreadId;
typedef int32_t osStatus;

typedef enum
{
    osPriorityIdle = 0,
    osPriorityLow,
    osPriorityBelowNormal,
    osPriorityNormal,
    osPriorityAboveNormal,
    osPriorityHigh,
    osPriorityRealtime,
    osPriorityError = -1
} osPriority;

#ifndef osOK
#define osOK ((osStatus)0)
#endif

/* ---------------- [LINUX-PORT] 由 os_port.c 实现 ---------------- */
osStatus osDelay(uint32_t millisec);
osStatus osThreadSuspend(osThreadId thread_id);
osStatus osThreadResume(osThreadId thread_id);

uint32_t osKernelSysTick(void);

#ifdef __cplusplus
}
#endif

#endif /* __CMSIS_OS_H__ */
