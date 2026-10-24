/**
 ******************************************************************************
 * @file    tasks_linux.c
 * @brief   [LINUX-PORT] 替代 Core/Src/freertos.c（STM32CubeMX 生成的 FreeRTOS 初始化）
 *
 * 原工程在 freertos.c 中用 osThreadDef/osThreadCreate 创建 15 个任务，
 * 然后由 osKernelStart() 启动调度器。
 *
 * Linux 侧对应做法：每个任务对应一个 POSIX 线程（pthread），
 * 由 port_thread_create() 创建；没有“内核启动”这一步，主线程继续运行。
 *
 * 任务周期（osDelay）保持不变，任务函数本体（OS_xxxCallback）不做修改。
 ******************************************************************************
 */
#include <stdio.h>

#include "port_linux.h"
#include "cmsis_os.h"
#include "UserFreertos.h"

/* [LINUX-PORT] 原 freertos.c 中创建的任务入口 */
extern void OS_IMUCallback(void const *argument);
extern void OS_LedCallback(void const *argument);
extern void OS_BeepCallback(void const *argument);
extern void OS_MotorCallback(void const *argument);
extern void OS_ErrorCallback(void const *argument);
extern void OS_Board2BoardCallback(void const *argument);
extern void OS_DetectCallback(void const *argument);
extern void OS_RcCallback(void const *argument);
extern void OS_ShooterCallback(void const *argument);
extern void OS_ChassisCallback(void const *argument);
extern void OS_GimbalCallback(void const *argument);
extern void OS_VisionCallback(void const *argument);

/* [LINUX-PORT] 与原 freertos.c 中的全局句柄保持一致 */
osThreadId defaultTaskHandle = NULL;
osThreadId IMUTaskHandle = NULL;
osThreadId LEDTaskHandle = NULL;
osThreadId BeepTaskHandle = NULL;
osThreadId MotorTaskHandle = NULL;
osThreadId ErrorTaskHandle = NULL;
osThreadId RS485TaskHandle = NULL;
osThreadId SuperCapTaskHandle = NULL;
osThreadId JudgeTaskHandle = NULL;
osThreadId DetectTaskHandle = NULL;
osThreadId RcTaskHandle = NULL;
osThreadId ShooterTaskHandle = NULL;
osThreadId ChassisHandle = NULL;
osThreadId GimbalHandle = NULL;
osThreadId VisionTaskHandle = NULL;

/* [LINUX-PORT] 用 nice 值近似原任务的相对优先级（负数需要 root 权限） */
#define NICE_REALTIME (-10)
#define NICE_HIGH (-5)
#define NICE_ABOVE_NORMAL (-2)
#define NICE_NORMAL (0)
#define NICE_BELOW_NORMAL (5)

void LinuxTasks_Start(void)
{
    IMUTaskHandle = port_thread_create("IMU", OS_IMUCallback, NULL, NICE_NORMAL);
    LEDTaskHandle = port_thread_create("LED", OS_LedCallback, NULL, NICE_BELOW_NORMAL);
    BeepTaskHandle = port_thread_create("Beep", OS_BeepCallback, NULL, NICE_NORMAL);
    MotorTaskHandle = port_thread_create("Motor", OS_MotorCallback, NULL, NICE_NORMAL);

    /* 错误任务启动后立刻挂起自身，由 RC 任务在急停时 osThreadResume 唤醒 */
    ErrorTaskHandle = port_thread_create("Error", OS_ErrorCallback, NULL, NICE_REALTIME);

    RS485TaskHandle = port_thread_create("B2B", OS_Board2BoardCallback, NULL, NICE_HIGH);
    DetectTaskHandle = port_thread_create("Detect", OS_DetectCallback, NULL, NICE_NORMAL);
    RcTaskHandle = port_thread_create("RC", OS_RcCallback, NULL, NICE_ABOVE_NORMAL);
    ShooterTaskHandle = port_thread_create("Shooter", OS_ShooterCallback, NULL, NICE_NORMAL);
    ChassisHandle = port_thread_create("Chassis", OS_ChassisCallback, NULL, NICE_NORMAL);
    GimbalHandle = port_thread_create("Gimbal", OS_GimbalCallback, NULL, NICE_NORMAL);
    VisionTaskHandle = port_thread_create("Vision", OS_VisionCallback, NULL, NICE_NORMAL);

    /* [LINUX-PORT] 原工程中 Judge 任务被 DISABLE_JUDGE_TASK 关闭，
     * SuperCap/Default 任务为空循环，均未创建。 */
    JudgeTaskHandle = NULL;
    SuperCapTaskHandle = NULL;
    defaultTaskHandle = NULL;

    printf("[port] 已创建 %d 个任务线程\n", 12);
}
