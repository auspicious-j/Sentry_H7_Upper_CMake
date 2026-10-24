/**
 * @file    usart.h
 * @brief   [LINUX-PORT] 替代 Core/Inc/usart.h
 *
 * Linux 侧对应技术：termios（/dev/ttyUSB0 等）或串口转 socket。
 * 通信相关代码按要求暂时停用，这里只保留句柄定义。
 */
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "dma.h"

/* [LINUX-PORT] 虚拟串口句柄：huart1=裁判系统, huart2=上下板通信, huart5=遥控器 */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart5;

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
