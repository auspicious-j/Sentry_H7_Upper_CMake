#ifndef B2B_H
#define B2B_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

extern uint8_t usart2RxBuf[256]; // 串口2缓冲区
extern uint8_t STOPFLAG;
extern uint8_t FEEDBACK;
extern float chassis_yaw;

void B2B_Init(void);
void B2B_Transmit(void);
void B2B_Receive(void);

#ifdef __cplusplus
}
#endif

#endif
