/**
 * @file    spi.h
 * @brief   [LINUX-PORT] 替代 Core/Inc/spi.h
 *
 * 原工程用 SPI2 读取 BMI088。Linux 侧改为模拟 IMU 数据源
 * （platform/linux/src/bmi088_sim.c），此句柄仅用于保持接口兼容。
 */
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/* [LINUX-PORT] 虚拟 SPI 句柄 */
extern SPI_HandleTypeDef hspi2;

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */
