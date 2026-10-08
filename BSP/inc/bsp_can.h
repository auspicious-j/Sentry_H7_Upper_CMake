#ifndef _BSP_CAN_H_
#define _BSP_CAN_H_

#include "fdcan.h"
#include "cmsis_os.h"
#include <stdbool.h>

//can????? debug?
//can错误计数器 debug用
typedef struct
{
	HAL_StatusTypeDef can1_user_init_error,can2_user_init_error,can3_user_init_error;
	uint16_t can1_send_error,can2_send_error,can3_send_error;
	uint16_t can1_receive_error,can2_receive_error,can3_receive_error;
}CanState;

#define BSP_MOTOR_FEEDBACK_COUNT 5U

// 每个统一电机通道最后一次收到反馈的 HAL tick。
extern volatile uint32_t bsp_motor_feedback_tick[BSP_MOTOR_FEEDBACK_COUNT];

// 标记一个电机通道刚收到反馈。
void BSP_MotorFeedbackMark(uint8_t channel);

// 判断一个电机通道是否在超时时间内收到反馈。
bool BSP_MotorFeedbackIsOnline(uint8_t channel, uint32_t timeout_ms);

//can初始化
void CAN_Init(void);

/****??????****/
/****外部调用函数****/
void USER_CAN_Send(FDCAN_HandleTypeDef* hfdcan,int16_t StdId,uint8_t* tx_data);

#endif
