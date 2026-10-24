/**
 ******************************************************************************
 * @file    os_port.c
 * @brief   [LINUX-PORT] CMSIS-RTOS / FreeRTOS 在 Ubuntu 上的替代实现
 *
 * 对应关系：
 *   osThreadCreate / osThreadDef   ->  pthread_create        （port_thread_create）
 *   osDelay(ms)                    ->  clock_nanosleep
 *   osThreadSuspend / osThreadResume -> 每线程一个二值信号量
 *   xTaskNotifyGive / ulTaskNotifyTake -> 每线程一个计数信号量
 *   vTaskSuspendAll / xTaskResumeAll   -> 递归互斥量
 *
 * 注意：Linux 用户态无法做到 MCU 那样的硬实时，1ms 控制环在桌面环境下
 *       会出现毫秒级抖动。若需要更严格的周期，可考虑 PREEMPT_RT 内核。
 ******************************************************************************
 */
#define _GNU_SOURCE

#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#include "port_linux.h"
#include "stm32h7xx_hal.h"

/* ---------------- [LINUX-PORT] 线程对象 ---------------- */
typedef struct port_thread_obj
{
    pthread_t tid;
    sem_t suspend_sem; /* 挂起/恢复 */
    sem_t notify_sem;  /* 任务通知 */
    char name[32];
    int nice_value;
    int created;
} port_thread_obj_t;

/* 当前线程对应的移植层对象（用于 ulTaskNotifyTake） */
static __thread port_thread_obj_t *s_current_obj = NULL;

osThreadId port_thread_current(void)
{
    return (osThreadId)s_current_obj;
}

typedef struct port_thread_start
{
    port_thread_obj_t *obj;
    void (*entry)(void const *);
    void *arg;
} port_thread_start_t;

static void *port_thread_trampoline(void *p)
{
    port_thread_start_t *start = (port_thread_start_t *)p;
    port_thread_obj_t *obj = start->obj;
    void (*entry)(void const *) = start->entry;
    void *arg = start->arg;
    free(start);

    s_current_obj = obj;

#if defined(__linux__)
    pthread_setname_np(pthread_self(), obj->name);
#endif

    /* nice 值越大优先级越低；普通用户只能调高 nice（降低优先级） */
    if (obj->nice_value != 0)
        setpriority(PRIO_PROCESS, (id_t)syscall(SYS_gettid), obj->nice_value);

    port_log("[port] 线程启动: %s\n", obj->name);

    entry(arg);

    port_log("[port] 线程退出: %s\n", obj->name);
    s_current_obj = NULL;
    return NULL;
}

osThreadId port_thread_create(const char *name,
                              void (*entry)(void const *),
                              void *arg,
                              int nice_value)
{
    if (entry == NULL)
        return NULL;

    port_thread_obj_t *obj = (port_thread_obj_t *)calloc(1, sizeof(*obj));
    if (obj == NULL)
        return NULL;

    port_thread_start_t *start = (port_thread_start_t *)calloc(1, sizeof(*start));
    if (start == NULL)
    {
        free(obj);
        return NULL;
    }

    snprintf(obj->name, sizeof(obj->name), "%s", (name != NULL) ? name : "task");
    obj->nice_value = nice_value;

    sem_init(&obj->suspend_sem, 0, 0);
    sem_init(&obj->notify_sem, 0, 0);

    start->obj = obj;
    start->entry = entry;
    start->arg = arg;

    if (pthread_create(&obj->tid, NULL, port_thread_trampoline, start) != 0)
    {
        sem_destroy(&obj->suspend_sem);
        sem_destroy(&obj->notify_sem);
        free(obj);
        free(start);
        return NULL;
    }

    obj->created = 1;
    return (osThreadId)obj;
}

/* ---------------- [LINUX-PORT] 延时 ---------------- */
osStatus osDelay(uint32_t millisec)
{
    port_sleep_ms(millisec);
    return osOK;
}

void vTaskDelay(TickType_t xTicksToDelay)
{
    port_sleep_ms((uint32_t)xTicksToDelay);
}

uint32_t osKernelSysTick(void)
{
    return HAL_GetTick();
}

/* ---------------- [LINUX-PORT] 挂起 / 恢复 ---------------- */
osStatus osThreadSuspend(osThreadId thread_id)
{
    port_thread_obj_t *obj = (port_thread_obj_t *)thread_id;
    if (obj == NULL)
        return -1;

    /* 与原工程用法一致：只用于挂起“当前线程自身” */
    while (sem_wait(&obj->suspend_sem) == -1 && errno == EINTR)
    {
    }
    return osOK;
}

osStatus osThreadResume(osThreadId thread_id)
{
    port_thread_obj_t *obj = (port_thread_obj_t *)thread_id;
    if (obj == NULL)
        return -1;

    sem_post(&obj->suspend_sem);
    return osOK;
}

/* ---------------- [LINUX-PORT] 任务通知 ---------------- */
void xTaskNotifyGive(void *xTaskToNotify)
{
    port_thread_obj_t *obj = (port_thread_obj_t *)xTaskToNotify;
    if (obj != NULL)
        sem_post(&obj->notify_sem);
}

uint32_t ulTaskNotifyTake(BaseType_t xClearCountOnExit, TickType_t xTicksToWait)
{
    /* 本工程中调用者一定是被通知的线程本身，这里按“当前线程对象”处理 */
    port_thread_obj_t *obj = (port_thread_obj_t *)port_thread_current();
    if (obj == NULL)
        return 0U;

    int ret = 0;
    if (xTicksToWait == portMAX_DELAY)
    {
        while (sem_wait(&obj->notify_sem) == -1 && errno == EINTR)
        {
        }
        ret = 0;
    }
    else
    {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += (time_t)(xTicksToWait / 1000U);
        ts.tv_nsec += (long)(xTicksToWait % 1000U) * 1000000L;
        if (ts.tv_nsec >= 1000000000L)
        {
            ts.tv_sec += 1;
            ts.tv_nsec -= 1000000000L;
        }

        while (sem_timedwait(&obj->notify_sem, &ts) == -1 && errno == EINTR)
        {
        }
        ret = 0;
    }

    if (xClearCountOnExit != pdFALSE)
    {
        int count = 0;
        /* 排空计数，模拟 xClearCountOnExit = pdTRUE 的语义 */
        while (sem_trywait(&obj->notify_sem) == 0)
            count++;
        (void)count;
    }

    return 1U;
}

/* ---------------- [LINUX-PORT] 临界区 / 挂起调度器 ---------------- */
static pthread_mutex_t s_sched_mutex;
static pthread_once_t s_sched_once = PTHREAD_ONCE_INIT;

static void sched_mutex_init(void)
{
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&s_sched_mutex, &attr);
    pthread_mutexattr_destroy(&attr);
}

void vTaskSuspendAll(void)
{
    pthread_once(&s_sched_once, sched_mutex_init);
    pthread_mutex_lock(&s_sched_mutex);
}

BaseType_t xTaskResumeAll(void)
{
    pthread_once(&s_sched_once, sched_mutex_init);
    pthread_mutex_unlock(&s_sched_mutex);
    return pdFALSE;
}

void portENTER_CRITICAL(void)
{
    vTaskSuspendAll();
}

void portEXIT_CRITICAL(void)
{
    (void)xTaskResumeAll();
}

/* ---------------- [LINUX-PORT] FreeRTOS 堆 -> malloc ---------------- */
void *pvPortMalloc(size_t xWantedSize)
{
    return malloc(xWantedSize);
}

void vPortFree(void *pv)
{
    free(pv);
}
