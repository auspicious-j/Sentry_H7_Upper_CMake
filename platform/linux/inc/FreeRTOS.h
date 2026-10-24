/**
 * @file    FreeRTOS.h
 * @brief   [LINUX-PORT] FreeRTOS 基础类型的极简替代
 *
 * 工程里只有少量 FreeRTOS API 被直接使用（任务通知、挂起调度器、临界区宏），
 * 这里用 pthread 互斥量/信号量实现，见 platform/linux/src/os_port.c。
 */
#ifndef INC_FREERTOS_H
#define INC_FREERTOS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <string.h> /* [LINUX-PORT] 原 FreeRTOS.h 间接提供 memcpy/memset 声明 */

typedef long BaseType_t;
typedef unsigned long UBaseType_t;
typedef uint32_t TickType_t;
typedef uint32_t StackType_t;

#ifndef pdTRUE
#define pdTRUE ((BaseType_t)1)
#endif
#ifndef pdFALSE
#define pdFALSE ((BaseType_t)0)
#endif

/* [LINUX-PORT] 原值 0xFFFFFFFF，表示永久等待 */
#define portMAX_DELAY ((TickType_t)0xFFFFFFFFUL)

#define configTICK_RATE_HZ 1000U

/* [LINUX-PORT] 临界区宏：Linux 下用互斥量保护（hal_port.c 提供实现） */
void portENTER_CRITICAL(void);
void portEXIT_CRITICAL(void);
#define taskENTER_CRITICAL() portENTER_CRITICAL()
#define taskEXIT_CRITICAL() portEXIT_CRITICAL()

/* [LINUX-PORT] FreeRTOS 堆管理 -> 标准 malloc/free（实现见 os_port.c） */
void *pvPortMalloc(size_t xWantedSize);
void vPortFree(void *pv);

#ifdef __cplusplus
}
#endif

#endif /* INC_FREERTOS_H */
