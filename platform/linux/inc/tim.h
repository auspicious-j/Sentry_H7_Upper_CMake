/**
 * @file    tim.h
 * @brief   [LINUX-PORT] 替代 Core/Inc/tim.h
 *
 * Linux 侧对应技术：蜂鸣器可用 ALSA / console 蜂鸣，WS2812 可用 SPI(ws2812-spi) 驱动。
 * 目前这些外设输出均为桩实现（platform/linux/src/hal_port.c）。
 */
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/* [LINUX-PORT] htim3 = WS2812 PWM-DMA, htim12 = 蜂鸣器 PWM */
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim12;

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */
