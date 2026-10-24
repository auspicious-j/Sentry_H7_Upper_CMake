/**
 ******************************************************************************
 * @file    main.h
 * @brief   [LINUX-PORT] 替代 Core/Inc/main.h（原文件由 STM32CubeMX 生成）
 *
 * 原 main.h 只做两件事：
 *   1. include "stm32h7xx_hal.h"（现在指向 platform/linux/inc 下的 Linux 兼容层）
 *   2. 定义 CubeMX 生成的引脚宏
 *
 * Linux 侧没有物理引脚，但应用层头文件（Chassis.h / Vision.h / Judge.h /
 * Beep.h / USER_Moto.h / USER_Detcet.h）仍然 #include "main.h"，
 * 因此保留同名头文件，避免大面积修改业务代码。
 ******************************************************************************
 */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* [LINUX-PORT] 指向 Linux 兼容层，而不是 STM32 HAL 驱动 */
#include "stm32h7xx_hal.h"

/* [LINUX-PORT] 原 CubeMX 生成的引脚宏，保留以兼容 IMU 驱动等代码 */
#define ACC_CS_Pin GPIO_PIN_0
#define ACC_CS_GPIO_Port ((GPIO_TypeDef *)0)
#define GYRO_CS_Pin GPIO_PIN_3
#define GYRO_CS_GPIO_Port ((GPIO_TypeDef *)0)

/* 原 main.h 中声明的错误处理函数 */
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
