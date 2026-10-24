/**
 * @file    port_linux.h
 * @brief   [LINUX-PORT] 移植层内部公共接口
 *
 * 提供 Linux 侧的“板级支持”原语：
 *   - 单调时钟（替代 DWT / SysTick）
 *   - 日志
 *   - 线程创建（替代 osThreadCreate）
 */
#ifndef PORT_LINUX_H
#define PORT_LINUX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "cmsis_os.h"

/* [LINUX-PORT] 平台初始化：设置日志、忽略 SIGPIPE 等 */
void port_init(void);

/* [LINUX-PORT] 单调时间源，替代 STM32 的 DWT->CYCCNT / HAL_GetTick */
uint64_t port_monotonic_us(void);
uint32_t port_monotonic_ms(void);

/* [LINUX-PORT] 毫秒级睡眠（保证不会提前返回） */
void port_sleep_ms(uint32_t ms);

/* [LINUX-PORT] 日志：默认关闭，环境变量 SENTRY_PORT_VERBOSE=1 打开 */
void port_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
bool port_verbose_enabled(void);

/* [LINUX-PORT] 创建线程（替代 osThreadCreate + osThreadDef） */
osThreadId port_thread_create(const char *name,
                              void (*entry)(void const *),
                              void *arg,
                              int nice_value);

/* [LINUX-PORT] 获取当前线程句柄（用于任务通知等待） */
osThreadId port_thread_current(void);

#ifdef __cplusplus
}
#endif

#endif /* PORT_LINUX_H */
