/**
 * @file    gpio.h
 * @brief   [LINUX-PORT] 替代 Core/Inc/gpio.h
 *
 * Linux 侧没有 GPIO；原工程里 gpio.h 主要用于 IMU 片选，现已由模拟 IMU 取代。
 */
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#ifdef __cplusplus
}
#endif

#endif /* __GPIO_H__ */
