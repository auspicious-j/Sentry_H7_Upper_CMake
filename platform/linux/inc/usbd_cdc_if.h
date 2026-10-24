/**
 * @file    usbd_cdc_if.h
 * @brief   [LINUX-PORT] USB CDC 虚拟串口接口的 Linux 替代
 *
 * 原工程通过 USB_DEVICE/App/usbd_cdc_if.c 向视觉上位机发送数据。
 * Linux 侧的等价方案是 socket / 串口 / ROS2 话题。
 * 按需求，收发相关代码先停用：CDC_Transmit_HS() 只保留接口，不做实际发送。
 */
#ifndef __USB_CDC_IF_H
#define __USB_CDC_IF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define USBD_OK 0U
#define USBD_BUSY 1U
#define USBD_FAIL 2U

/* [LINUX-PORT] 空实现：数据不发送，仅返回成功（实现见 hal_port.c） */
uint8_t CDC_Transmit_HS(uint8_t *Buf, uint16_t Len);

#ifdef __cplusplus
}
#endif

#endif /* __USB_CDC_IF_H */
