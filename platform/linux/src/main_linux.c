/**
 ******************************************************************************
 * @file    main_linux.c
 * @brief   [LINUX-PORT] 替代 Core/Src/main.c（STM32CubeMX 生成的入口）
 *
 * 原 main.c 的启动流程：
 *   MPU_Config() -> HAL_Init() -> SystemClock_Config() -> MX_xxx_Init()（外设）
 *   -> DWT_Init(480) / CAN_Init() / RC_Init() / Beep_Init() / B2B_Init()
 *      / Detect_InitAll()
 *   -> MX_FREERTOS_Init() -> osKernelStart()
 *
 * Linux 侧启动流程：
 *   port_init() -> DWT_Init(480) -> CAN_Init() -> ... -> LinuxTasks_Start()
 *   -> 主线程周期性打印运行状态
 *
 * 说明：这一版的目标是“能在 Ubuntu 上原生编译并跑起来”，
 *       外设相关的初始化都变成了空实现，业务逻辑保持不变。
 ******************************************************************************
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "port_linux.h"
#include "stm32h7xx_hal.h"

#include "bsp_dwt.h"
#include "bsp_can.h"
#include "bsp_ws2812.h"
#include "Beep.h"
#include "Chassis.h"
#include "Gimbal.h"
#include "Judge.h"
#include "Shooter.h"
#include "Vision.h"
#include "imu_temp_ctrl.h"
#include "USER_B2B.h"
#include "USER_RC.h"
#include "USER_Detcet.h"

/* [LINUX-PORT] 由 tasks_linux.c 提供 */
void LinuxTasks_Start(void);

void Error_Handler(void)
{
    printf("[port] Error_Handler() 被调用，进程退出\n");
    exit(EXIT_FAILURE);
}

static void print_status(void)
{
    static uint32_t last_print = 0;
    uint32_t now = HAL_GetTick();

    if ((now - last_print) < 1000U)
        return;
    last_print = now;

    printf("[status] t=%6.1fs  yaw=%8.2f  pitch=%7.2f  roll=%7.2f  "
           "chassisMode=%d  gimbalTargetYaw=%8.2f  fricOpen=%d\n",
           (double)DWT_GetTimeline_s(),
           (double)INS.yaw,
           (double)INS.pitch,
           (double)INS.roll,
           (int)chassis.rotate.mode,
           (double)gimbal.top_yaw.targetAngle,
           (int)shooter.fricOpen);
    fflush(stdout);
}

int main(void)
{
    /* [LINUX-PORT] 用 CLOCK_MONOTONIC 提供 HAL_GetTick 时间基准 */
    port_init();

    printf("==============================================\n");
    printf(" Sentry H7 Upper - Linux 原生移植版\n");
    printf(" 通信/外设已桩化，仅运行业务逻辑\n");
    printf(" 编译时间: %s %s\n", __DATE__, __TIME__);
    printf("==============================================\n");

    /* ---- 与原 main.c 的 USER CODE BEGIN 2 保持一致 ---- */
    DWT_Init(480);       /* 原为 DWT 周期计数器，现为 clock_gettime 封装 */
    CAN_Init();          /* FDCAN -> 空实现（SocketCAN 待接入） */
    RC_Init();           /* 串口遥控 -> 空实现 */
    Beep_Init();         /* PWM 蜂鸣器 -> 空实现 */
    B2B_Init();          /* 上下板串口 -> 空实现 */
    Detect_InitAll();    /* 掉线检测逻辑保留 */

    /* ---- 替代 MX_FREERTOS_Init() + osKernelStart() ---- */
    LinuxTasks_Start();

    /* 主线程：打印运行状态，保持进程存活 */
    for (;;)
    {
        print_status();
        sleep(1);
    }

    return 0;
}
