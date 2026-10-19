extern "C"
{
#include "USER_B2B.h"
#include "Chassis.h"
#include "usart.h"
#include "cmsis_os.h"
#include "Chassis.h"
#include "Gimbal.h"
#include <string.h>
#include "Vision.h"
#include "Judge.h"
}

#include "VisionLink.hpp"

// 平台层中断路由：本文件不再定义 HAL 的共享弱符号回调，只登记自己的处理函数。
#include "IrqDispatch.hpp"

// 云台控制量（大 yaw 速度指令）来自 Gimbal 任务的跨任务邮箱。
#include "GimbalControlMailbox.hpp"
#include "ChassisOutputMailbox.hpp"

extern DMA_HandleTypeDef hdma_usart2_rx;
float chassis_yaw = 0;

/* 需要用到的接收变量*/
uint8_t usart2RxBuf[256]; // 串口2缓冲区
uint8_t STOPFLAG = 0;   //是1下板急停
uint8_t FEEDBACK = 0;		//是0下板急停

uint32_t receive_times;

/* 需要用到的发送变量*/
uint8_t usart2TxBuf[64];

namespace {

// USART2 空闲中断后的处理：交给原有的接收解析函数。
// 由 Platform/STM32/IrqDispatch 按 huart 实例路由进来，本函数不再判断 huart。
void b2b_uart_on_idle(std::uint16_t received_size)
{
    (void)received_size;
    B2B_Receive();
}

} // namespace

// 板间通信初始化
void B2B_Init()
{
    // 先登记路由再开中断：顺序反了会丢掉第一帧。
    if (!robot::platform::stm32::irq_register_uart_rx(&huart2, b2b_uart_on_idle))
    {
        return;
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2RxBuf, sizeof(usart2RxBuf));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

void B2B_Transmit()
{
	// 四舵轮目标值与当前舵角由 Chassis 任务的 Kinematics_Node 通过邮箱发布。
	robot::framework::timed_snapshot<robot::Chassis::chassis_output> chassis_output{};
	robot::Chassis::chassis_output_mailbox().copy_to(chassis_output);

	usart2TxBuf[0] = 0xAA;				 // 帧头
	for(uint8_t i = 0; i < 4; i++)
	{
		float v = chassis_output.value.target_turn_angle_deg[i] - chassis_output.value.now_turn_angle_deg[i];   // 要发送的那个 float
		uint8_t *p = (uint8_t *)&v;
		usart2TxBuf[1 + 4*i] = p[0];
		usart2TxBuf[2 + 4*i] = p[1];
		usart2TxBuf[3 + 4*i] = p[2];
		usart2TxBuf[4 + 4*i] = p[3];
	} //1-16 舵电机误差角度
	for (uint8_t i = 0; i < 4; i++)
	{
		usart2TxBuf[17 + i * 2] = (uint8_t)chassis_output.value.target_drive_speed[i];
		usart2TxBuf[17 + i * 2 + 1] = (uint8_t)(chassis_output.value.target_drive_speed[i] >> 8);
	} // 17-24 轮电机目标速度
	{
		// 大 yaw 速度指令由 Gimbal 任务的 Control_Node 通过邮箱发布。
		robot::framework::timed_snapshot<robot::Gimbal::gimbal_control> control{};
		robot::Gimbal::gimbal_control_mailbox().copy_to(control);
		float v = control.value.base_yaw_speed_cmd;   // 要发送的那个 float
		uint8_t *p = (uint8_t *)&v;
		usart2TxBuf[25] = p[0];
		usart2TxBuf[26] = p[1];
		usart2TxBuf[27] = p[2];
		usart2TxBuf[28] = p[3];
	}	//25-28 yaw电机目标速度

	//29-35 哨兵自主指令：从视觉快照取最新值（无新帧时保留上一次，与旧全局语义一致）。
	{
		USER_SentryCmd_t sentry_command{};
		robot::framework::timed_snapshot<robot::io::vision::vision_frame_snapshot> vision_copy{};
		robot::io::vision::vision_frame_mailbox().copy_to(vision_copy);
		sentry_command.sentry_mode = vision_copy.value.sentry_mode;
		sentry_command.armor_mode = vision_copy.value.armor_mode;
		sentry_command.look_yaw = vision_copy.value.look_yaw;
		sentry_command.look_pitch = vision_copy.value.look_pitch;
		sentry_command.energy_activation = vision_copy.value.energy_activation;
		sentry_command.buy_projectile = vision_copy.value.buy_projectile;
		sentry_command.buy_life = vision_copy.value.buy_life;
		sentry_command.remote_buy_blood = vision_copy.value.remote_buy_blood;
		sentry_command.remote_buy_bullet = vision_copy.value.remote_buy_bullet;
		memcpy(&usart2TxBuf[29], &sentry_command, sizeof(sentry_command));
	}

	usart2TxBuf[62]  = STOPFLAG;    // 急停标志
	usart2TxBuf[63] = 0xFE;				 // 帧尾
		
	HAL_UART_Transmit_DMA(&huart2, usart2TxBuf, sizeof(usart2TxBuf));
}


void B2B_Receive(void)
{
    if (usart2RxBuf[0] == 0xAB && usart2RxBuf[63] == 0xFD)
	{
	    for(uint8_t i = 0; i < 4; i++)
		{
		    float v;
		    uint8_t *p = (uint8_t *)&v;
		    p[0] = usart2RxBuf[1 + 4*i];
		    p[1] = usart2RxBuf[2 + 4*i];
		    p[2] = usart2RxBuf[3 + 4*i];
		    p[3] = usart2RxBuf[4 + 4*i];
		    chassis.motors[i].TurnAngle = v;
		} //解析舵电机当前角度（单圈）单位° 1-16
		
	    for (uint8_t i = 0; i < 4; i++)
		{
		    chassis.motors[i].now_Speed = (int16_t)usart2RxBuf[17 + i * 2] | (int16_t)usart2RxBuf[17 + i * 2 + 1] << 8;
		}//轮电机当前速度    17-24
		{
		    float v;
		    uint8_t *p = (uint8_t *)&v;
		    p[0] = usart2RxBuf[25];
		    p[1] = usart2RxBuf[26];
		    p[2] = usart2RxBuf[27];
		    p[3] = usart2RxBuf[28];
		    gimbal.base_yawMotor.nowAngle = v;
		}	//解析大yaw电机当前角度（单圈）单位° 25-28	
		// {
		// 	    float v;
		// 	    uint8_t *p = (uint8_t *)&v;
		// 	    p[0] = usart2RxBuf[29];
		// 	    p[1] = usart2RxBuf[30];
		// 	    p[2] = usart2RxBuf[31];
		// 	    p[3] = usart2RxBuf[32];
		// 	    gimbal.base_yawMotor.para.vel = v;
		// }	//解析大yaw电机当前速度  单位rad/s 29-32	
	    memcpy(&USER_JudgeData, &usart2RxBuf[29], sizeof(JudgeData_t)); //解析裁判系统数据 29-58

	    FEEDBACK = usart2RxBuf[62];
	}
}

/************************freertos任务****************************/
extern "C" void OS_Board2BoardCallback(void const *argument)
{
    while (1)
	{
	    B2B_Transmit();
	    osDelay(1);
	}
}
