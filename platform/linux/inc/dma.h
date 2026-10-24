/**
 * @file    dma.h
 * @brief   [LINUX-PORT] 替代 Core/Inc/dma.h
 *
 * Linux 侧没有 DMA 外设（CPU 直接搬运内存），这里只保留句柄占位。
 */
#ifndef __DMA_H__
#define __DMA_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/* [LINUX-PORT] 原工程用于串口空闲中断接收的 DMA 句柄 */
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_uart5_rx;

#ifdef __cplusplus
}
#endif

#endif /* __DMA_H__ */
