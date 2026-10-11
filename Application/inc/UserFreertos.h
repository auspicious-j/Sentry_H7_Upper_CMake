/*==============================================================================
 * UserFreertos.h —— 用户 FreeRTOS 任务句柄的外部声明
 *------------------------------------------------------------------------------
 * 说明：
 *   - 本头文件对外暴露各“用户任务”的任务句柄。任务句柄（osThreadId）是 RTOS
 *     用来标识一个任务的唯一编号，由任务创建函数返回；拿到句柄后即可对该任务
 *     执行挂起/恢复/删除等操作（例如急停任务会用它在启动时挂起自身）。
 *   - 各句柄的实际定义，以及任务的优先级、栈大小、创建位置，均在 CubeMX 生成的
 *     FreeRTOS 源文件（如 freertos.c）中；本文件只做 extern 声明，供其它模块
 *     引用。注意：extern 仅作“声明”，不会在此分配存储空间。
 *   - 术语：任务（Task）是 RTOS 中可独立调度执行的代码单元；句柄（Handle）
 *     可理解为任务在系统中的“身份证号”。
 *============================================================================*/
#ifndef _USER_FREERTOS_H_
#define _USER_FREERTOS_H_
#include "cmsis_os.h"                // CMSIS-RTOS 封装层（osThreadId、osThreadSuspend 等接口）

extern osThreadId defaultTaskHandle; // 默认任务句柄（CubeMX 生成，通常用于空闲/兜底处理）
extern osThreadId LEDTaskHandle;     // LED 任务句柄：负责指示灯/灯效刷新
extern osThreadId BeepTaskHandle;    // 蜂鸣器任务句柄：负责提示音/报警音
extern osThreadId MotorTaskHandle;   // 电机任务句柄：负责电机控制指令的下发
extern osThreadId ErrorTaskHandle;   // 错误处理(急停)任务句柄：故障时被唤醒执行急停

#endif                               // _USER_FREERTOS_H_
