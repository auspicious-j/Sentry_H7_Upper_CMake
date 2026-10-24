/**
 * @file    fdcan.h
 * @brief   [LINUX-PORT] 替代 Core/Inc/fdcan.h
 *
 * Linux 侧对应技术：SocketCAN（PF_CAN / linux/can.h）。
 * 通信相关代码按要求暂时停用，这里只保留句柄定义，便于后续替换实现。
 */
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/* [LINUX-PORT] 三个虚拟 CAN 句柄，函数体为空实现（platform/linux/src/hal_port.c） */
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */
