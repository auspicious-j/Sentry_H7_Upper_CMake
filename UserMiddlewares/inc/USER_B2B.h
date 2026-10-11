/*
 * USER_B2B.h
 * 板间通信（board-to-board，B2B）接口头文件。
 * 本文件声明上板与下板之间通过 USART2 交换数据所需的全局变量和接口函数。
 * 术语：B2B = board-to-board，指同一台机器人上"上板"与"下板"两块主控板之间的通信。
 */
#ifndef B2B_H
#define B2B_H // 头文件包含保护宏，防止本头文件被重复包含
#include "stdint.h"

/* B2B 接收缓冲：DMA 把 USART2 收到的原始字节写入此处，容量 256 字节。 */
extern uint8_t usart2RxBuf[256]; // 串口2缓冲区
/* 上板发给下板的急停请求标志：1=请求下板急停，0=正常。 */
extern uint8_t STOPFLAG;
/* 下板回传的故障/急停反馈标志：0=下板处于急停，非 0=正常。 */
extern uint8_t FEEDBACK;
/* 底盘（下板）上报的 yaw 角，单位°（度）。 */
extern float chassis_yaw;

/* 初始化板间通信：启动 USART2 的 DMA 空闲接收，并关闭半传输中断。 */
void B2B_Init(void);
/* 打包并向下板发送一帧数据（帧头 0xAA，帧尾 0xFE，共 64 字节）。 */
void B2B_Transmit(void);
/* 解析下板发来的一帧数据（帧头 0xAB，帧尾 0xFD，共 64 字节）。 */
void B2B_Receive(void);


#endif
