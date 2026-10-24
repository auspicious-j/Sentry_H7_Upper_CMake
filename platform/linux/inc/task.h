/**
 * @file    task.h
 * @brief   [LINUX-PORT] FreeRTOS 任务 API 的 Linux 替代
 *
 * 只实现工程实际用到的接口：
 *   xTaskNotifyGive / ulTaskNotifyTake  ->  POSIX 信号量
 *   vTaskSuspendAll / xTaskResumeAll    ->  POSIX 递归互斥量
 *   vTaskDelay                          ->  osDelay
 */
#ifndef INC_TASK_H
#define INC_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"

/* [LINUX-PORT] 任务通知等价实现 */
void xTaskNotifyGive(void *xTaskToNotify);
uint32_t ulTaskNotifyTake(BaseType_t xClearCountOnExit, TickType_t xTicksToWait);

/* [LINUX-PORT] 挂起/恢复调度器（这里等价于加锁/解锁） */
void vTaskSuspendAll(void);
BaseType_t xTaskResumeAll(void);

/* [LINUX-PORT] 延时 */
void vTaskDelay(TickType_t xTicksToDelay);

#ifdef __cplusplus
}
#endif

#endif /* INC_TASK_H */
