/*
 * Judge.c —— 裁判系统数据解析实现。
 *
 * 职责：
 *  1) 接收裁判系统经 USART1(DMA + 空闲中断)下发的原始字节流，按协议拆帧；
 *  2) 对每一帧做帧头 CRC8 与整帧 CRC16 校验，校验通过后按命令码把数据段
 *     拷贝进对应的全局结构体(GameState/PowerHeatData 等)；
 *  3) 通过 JUDGE_xxx 系列函数向外提供统一的读取接口；
 *  4) Judge_update() 把分散的数据汇总为 USER_JudgeData，供上层/AI 使用。
 *
 * 数据流向：
 *  硬件 USART1 → DMA → usart1RxBuf(接收缓冲区) → 中断回调
 *  → JUDGE_Read_Data()(拆帧 + 校验 + 分发) → 各 ext_xxx_t 全局结构体
 *  → Judge_update() → USER_JudgeData → 其他模块。
 *
 * 注意：本文件主要负责“接收/解析”，发送相关逻辑(Task_Judge_Callback)当前为空。
 */
#include "Judge.h"        //裁判系统数据结构与接口定义
#include "string.h"       //memcpy / memset
#include "cRc.h"          //CRC8 / CRC16 校验
#include "usart.h"        //USART1 句柄 huart1
#include "userfreertos.h" //FreeRTOS 相关
#include "usart.h"
#include "USER_B2B.h"     //板间通信
#include "myQueue.h"      //队列

#include <stdbool.h>
#define DISABLE_JUDGE_TASK   //定义后禁用裁判系统任务(见文件末尾 #ifdef EN_JUDGE_TASK)

/*****************系统数据定义**********************/
// 各命令码对应的数据段全局实例(名字 → 命令码 ID)，由 JUDGE_Read_Data 填充
ext_game_status_t GameState;						   // 0x0001 比赛状态
ext_game_result_t GameResult;						   // 0x0002 比赛结果
ext_game_robot_HP_t GameRobotHP;					   // 0x0003 各方机器人血量
ext_event_data_t EventData;							   // 0x0101 场地事件
ext_referee_warning_t RefereeWarning;				   // 0x0104 裁判警告
ext_dart_info_t DartRemainingTime;					   // 0x0105 飞镖发射口倒计时
ext_game_robot_status_t GameRobotState;				   // 0x0201 机器人状态(血量/热量上限/功率上限等)
ext_power_heat_data_t PowerHeatData;				   // 0x0202 实时功率热量
ext_game_robot_pos_t GameRobotPos;					   // 0x0203 机器人位置
ext_buff_musk_t BuffMusk;							   // 0x0204 机器人增益
ext_robot_hurt_t RobotHurt;							   // 0x0206 伤害状态
ext_shoot_data_t ShootData;							   // 0x0207 实时射击
ext_bullet_remaining_t BulletRemaining;				   // 0x0208 子弹剩余发射数
ext_rfid_status_t RfidStatus;						   // 0x0209 RFID 状态
ext_dart_client_cmd_t DartClientCmd;				   // 0x020A 飞镖客户端指令
sentry_info_t    SentryDecision;			//0x20D 哨兵当前状态



xFrameHeader FrameHeader; // 帧头解析暂存：每次解析前先把 5 字节帧头拷贝到这里
/****************************************************/

JudgeData_t USER_JudgeData;      // 汇总后的裁判数据(对外接口使用)
USER_SentryCmd_t USER_SentryCmd; // 哨兵机器人指令(本机决策输出)


bool Judge_Data_TF = false; // 裁判数据是否可用：true 表示最近一帧 CRC 校验通过(辅助函数判断用)

// 发送队列
Queue judgeQueue = EMPTY_QUEUE;
// 发送队列数据保存区
JudgeTxFrame judgeQueueBuf[JUDGE_QUEUE_SIZE];

// 串口接收缓冲区(DMA 直接把 USART1 收到的数据写入这里)
uint8_t usart1RxBuf[JUDGE_MAX_RX_LENGTH];

uint16_t shootNum = 0; // 统计发弹量：每收到一帧 0x0207 射击数据 +1

/**************裁判系统数据辅助****************/

/**
 * @brief  解析一段裁判系统数据(可能含多帧)，逐帧完成校验并按命令码分发到对应结构体。
 * @param  ReadFromUsart 指向接收缓冲区首字节的指针(通常为 usart1RxBuf)。
 * @retval true  成功解析出至少一帧(帧头 CRC8 与整帧 CRC16 均通过)
 * @retval false 未解析出有效帧(起始字节不对或校验失败)
 * @attention 为保证速度，本函数通常在串口接收中断/回调中调用；内部会自动递归处理
 *            同一包中的多帧数据；校验通过后才写数据，避免脏数据污染全局变量。
 */
bool JUDGE_Read_Data(uint8_t *ReadFromUsart)
{
    bool retval_tf = false; // 数据正确与否标志，每次调用先默认视为错误

    uint16_t judge_length; // 统计一帧数据长度(用于 CRC16 校验)
    int CmdID = 0;		   // 数据命令码解析结果

	/***------------------*****/
	// 空指针保护：无数据包则不处理(返回 -1 表示入参非法)
    if (ReadFromUsart == NULL)
	{
	    return -1;
	}
	// 先把 5 字节帧头拷贝到 FrameHeader，后续据此计算整帧长度
    memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);

	// 判断起始字节是否为 0xA5，是则说明此处可能是一帧的起点
    if (ReadFromUsart[SOF] == JUDGE_FRAME_HEADER)
	{
		// 帧头 CRC8 校验：只校验前 5 字节，避免把总线噪声误判成帧头
	    if (Verify_CRC8_Check_Sum(ReadFromUsart, LEN_HEADER))
		{
			// 计算整帧长度 = 数据段长度 + 帧头5 + 命令码2 + 帧尾2，用于 CRC16 校验
		    judge_length = ReadFromUsart[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
			;

			// 整帧 CRC16 校验：帧头+命令码+数据段全部参与，通过才认为数据可信
		    if (Verify_CRC16_Check_Sum(ReadFromUsart, judge_length))
			{
			    retval_tf = true; // 两道校验都通过，这一帧数据可用

			    CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]); // 命令码为小端：高字节在偏移6、低字节在偏移5
				// 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
			    switch (CmdID)
				{
			    case ID_game_state: // 0x0001
				    memcpy(&GameState, (ReadFromUsart + DATA), LEN_game_state);
				    break;

			    case ID_game_result: // 0x0002
				    memcpy(&GameResult, (ReadFromUsart + DATA), LEN_game_result);
				    break;

			    case ID_game_robot_HP: // 0x0003
				    memcpy(&GameRobotHP, (ReadFromUsart + DATA), LEN_game_robot_HP);
				    break;

			    case ID_event_data: // 0x0101
				    memcpy(&EventData, (ReadFromUsart + DATA), LEN_event_data);
				    break;


			    case ID_referee_warning: // 0x0104
				    memcpy(&RefereeWarning, (ReadFromUsart + DATA), LEN_referee_warning);
				    break;

			    case ID_dart_remaining_time: // 0x0105
				    memcpy(&DartRemainingTime, (ReadFromUsart + DATA), LEN_dart_remaining_time);
				    break;

			    case ID_game_robot_state: // 0x0201
				    memcpy(&GameRobotState, (ReadFromUsart + DATA), LEN_game_robot_state);
				    break;

			    case ID_power_heat_data: // 0x0202
				    memcpy(&PowerHeatData, (ReadFromUsart + DATA), LEN_power_heat_data);
				    break;

			    case ID_game_robot_pos: // 0x0203
				    memcpy(&GameRobotPos, (ReadFromUsart + DATA), LEN_game_robot_pos);
				    break;

			    case ID_buff_musk: // 0x0204
				    memcpy(&BuffMusk, (ReadFromUsart + DATA), LEN_buff_musk);
				    break;

			    case ID_robot_hurt: // 0x0206
				    memcpy(&RobotHurt, (ReadFromUsart + DATA), LEN_robot_hurt);
				    break;

			    case ID_shoot_data: // 0x0207
				    memcpy(&ShootData, (ReadFromUsart + DATA), LEN_shoot_data);
				    shootNum++; // 触发一次则是发射了一颗
					// Vision_SendShootSpeed(ShootData.bullet_speed);
				    break;

			    case ID_bullet_remaining: // 0x0208
				    memcpy(&BulletRemaining, (ReadFromUsart + DATA), LEN_bullet_remaining);
				    break;

			    case ID_rfid_status: // 0x0209
				    memcpy(&RfidStatus, (ReadFromUsart + DATA), LEN_rfid_status);
				    break;
			    case ID_sentry_status: // 0x020D
				    memcpy(&SentryDecision, (ReadFromUsart + DATA), LEN_sentry_status);
				    break;
				}
				// 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,用来判断一个数据包是否有多帧数据
			    if (*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
				{
					// 如果一个数据包出现了多帧数据,则再次读取
				    JUDGE_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
				}
			}
		}
		// 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,用来判断一个数据包是否有多帧数据
	    if (*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{
			// 如果一个数据包出现了多帧数据,则再次读取
		    JUDGE_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
		}
	}

    if (retval_tf)
	{
	    Judge_Data_TF = true; // 辅助函数用
	}
    else // 只要CRC16校验不通过就为FALSE
	{
	    Judge_Data_TF = false; // 辅助函数用
	}

    return retval_tf; // 对数据正误做处理
// 	LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);
// 	// IT trans
//	LL_USART_EnableIT_IDLE(USART1);
// 	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_0, (uint32_t)usart1RxBuf);
// 	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, sizeof(usart1RxBuf));
// 	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_0, (uint32_t)&USART1->RDR);
// 	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_0);
// 	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
//  LL_USART_EnableDMAReq_RX(USART1);
//
// 	// send
// 	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_1, (uint32_t)&USART1->TDR);
// 	LL_USART_EnableDMAReq_TX(USART1);
// 	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_1);
//    if (LL_USART_IsActiveFlag_IDLE(USART1) && LL_USART_IsEnabledIT_IDLE(USART1))
//    {
//        LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);
////        int usart1RxLen=JUDGE_MAX_RX_LENGTH - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_0);
//        // 解析串口数据
//                JUDGE_Read_Data(usart1RxBuf);
////                judgedata_update();
//                memset(usart1RxBuf,0,sizeof(usart1RxBuf));
//                LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0,JUDGE_MAX_RX_LENGTH);
//                LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
//                LL_USART_ClearFlag_IDLE(USART1);
//    }
////串口1中断回调
//void USER_USART1_IRQHandler()
//{
//	if (LL_USART_IsActiveFlag_IDLE(USART1) && LL_USART_IsEnabledIT_IDLE(USART1))
//	{
//		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);
//		// 获取接收到的数据长度
//		uint16_t rxLen = JUDGE_MAX_RX_LENGTH - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_0);
//		JUDGE_Read_Data(usart1RxBuf);
//		memset(usart1RxBuf,0,sizeof(usart1RxBuf));
//		LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, JUDGE_MAX_RX_LENGTH);
//		LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
//		LL_USART_ClearFlag_IDLE(USART1);
//		Detect_Update(DeviceID_Judge);
//	}
//}
//	while (LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_1) &&
//		   !LL_DMA_IsActiveFlag_TC1(DMA2))
//		// 设置超时，避免死等
//		static uint32_t timeout = 0;
//		if (++timeout > 10000)
//		{
//			LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_1);
//			LL_DMA_ClearFlag_TC1(DMA2);
//			timeout = 0;
//			break;
//		}
//	LL_DMA_ClearFlag_TC1(DMA2);
//	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_1);
//	LL_DMA_ConfigTransfer(DMA2, LL_DMA_STREAM_1,
//						  LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
//							  LL_DMA_MEMORY_INCREMENT |
//							  LL_DMA_PERIPH_NOINCREMENT |
//							  LL_DMA_PDATAALIGN_BYTE |
//							  LL_DMA_MDATAALIGN_BYTE);
//	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_1, (uint32_t)&USART1->TDR);
//	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_1, (uint32_t)tx_buffer);
//	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_1, size);
//	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);
}

extern DMA_HandleTypeDef hdma_usart1_rx;

// 裁判系统掉线回调函数
void Judge_UartLostCallback()
{

	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,usart1RxBuf,sizeof(usart1RxBuf));
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx , DMA_IT_HT);
//	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);
//	// 清除所有标志位
//	LL_USART_ClearFlag_IDLE(USART1);
//	LL_USART_ClearFlag_ORE(USART1);
//	LL_USART_ClearFlag_FE(USART1);
//	LL_USART_ClearFlag_NE(USART1);
//	LL_USART_ClearFlag_PE(USART1);
//	memset(usart1RxBuf, 0, sizeof(usart1RxBuf));
//	Judge_Data_TF = FALSE;
//	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, sizeof(usart1RxBuf));
//	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_0, (uint32_t)usart1RxBuf);
//	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
}
// 裁判系统初始化


void JUDGE_Init()
{
	//	USART1_dma_init();
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,usart1RxBuf,sizeof(usart1RxBuf));
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx , DMA_IT_HT);
}


// 获取己方颜色
RobotColor JUDGE_GetSelfColor()
{
    if (JUDGE_GetSelfID() > 10) // 蓝方
	{
	    return RobotColor_Blue;
	}
    else // 红方
	{
	    return RobotColor_Red;
	}
}

// 获取自身ID
uint8_t JUDGE_GetSelfID()
{
    return GameRobotState.robot_id;
}

// 获取客户端ID
uint16_t JUDGE_GetClientID()
{
    return 0x100 + GameRobotState.robot_id;
}

// 获取机器人坐标
void JUDGE_GetPosition(float *x, float *y)
{
	*x = GameRobotPos.x;
	*y = GameRobotPos.y;
}

// 获取底盘功率限制
uint8_t JUDGE_GetChassisPowerLimit()
{
    return GameRobotState.chassis_power_limit;
}

// 判断发射电源是否输出
bool JUDGE_GetShooterOutputState()
{
    return GameRobotState.power_management_shooter_output;
}

bool JUDGE_GetGimbalOutputState()
{
    return GameRobotState.power_management_gimbal_output;
}

bool JUDGE_GetChassisOutputState()
{
    return GameRobotState.power_management_chassis_output;
}

// 获取枪口热量限制
uint16_t JUDGE_GetHeatLimit()
{
    return GameRobotState.shooter_barrel_heat_limit;
}
// 获取当前枪管热量
uint16_t JUDGE_GetNowHeat()
{
    return PowerHeatData.shooter_id1_17mm_cooling_heat;
}
// 获取射速限制
uint16_t JUDGE_GetShootSpeedLimit()
{
    return 25;
}

// 获取底盘缓冲能量
uint16_t JUDGE_GetPowerBuffer()
{
    return PowerHeatData.chassis_power_buffer;
}

// 获取剩余枪口热量
int16_t JUDGE_GetRemainHeat()
{
    return (int16_t)USER_JudgeData.shooter_barrel_heat_limit - (int16_t)USER_JudgeData.shooter_id1_17mm_cooling_heat;
}

// 剩余发弹数
uint16_t JUDGE_GetRemain_17_Num()
{
    return BulletRemaining.projectile_allowance_17mm;
}

// 裁判系统数据是否有效
bool JUDGE_IsValid(void)
{
    return Judge_Data_TF;
}

// 扣血原因
uint8_t HP_deduction_reason()
{
    return RobotHurt.hurt_type;
}

// 读取当前血量
uint16_t JUDGE_GetHP()
{
    return GameRobotState.current_HP;
}

// 获取冷却速度
uint16_t JUDGE_GetCoolingValue()
{
    return USER_JudgeData.shooter_barrel_cooling_value;
}

//获取弹速
float JUDGE_GetInitial_speed()
{
    return ShootData.initial_speed;
}

//重置挨打的装甲板id
void JUDGE_ResetHurtArmorID()
{
    RobotHurt.armor_id=0;
}
//获取挨打的装甲板id
uint8_t JUDGE_GetHurtArmorID()
{
    return RobotHurt.armor_id;
}

// 获取哨兵信息
uint32_t JUDGE_GetSentryInfo()
{
    return SentryDecision.sentry_info;
}

// 获取哨兵信息2
uint16_t JUDGE_GetSentryInfo2()
{
    return SentryDecision.sentry_info_2;
}

//获取前哨站和基地血量
uint16_t JUDGE_GetAllyOutpostHP()
{
    return GameRobotHP.ally_outpost_HP;
}

uint16_t JUDGE_GetAllyBaseHP()
{
    return GameRobotHP.ally_base_HP;
}

void Judge_Receive()
{
    JUDGE_Read_Data(usart1RxBuf);
    memset(usart1RxBuf,0,sizeof(usart1RxBuf));
}

void Judge_update()
{
    USER_JudgeData.game_progress = GameState.game_progress;
    USER_JudgeData.remain_time = GameState.stage_remain_time;
    USER_JudgeData.current_hp = JUDGE_GetHP();
    USER_JudgeData.projectile = JUDGE_GetRemain_17_Num();

    USER_JudgeData.sentry_info = JUDGE_GetSentryInfo();
    USER_JudgeData.sentry_info_2 = JUDGE_GetSentryInfo2();
	//是否挨揍 bit0
    if (JUDGE_GetHurtArmorID()!=0&&RobotHurt.hurt_type == 0)// 被弹丸
	{
	    USER_JudgeData.sentry_info_3 |= (1 << 0);
	    JUDGE_ResetHurtArmorID();
	}
    else
	{
	    USER_JudgeData.sentry_info_3 &=~(1 << 0);
	}

	// bit1 是否检测到堡垒
    if (RfidStatus.rfid_status & (1 << 17))
		USER_JudgeData.sentry_info_3 |= (1 << 1);
    else
		USER_JudgeData.sentry_info_3 &= ~(1 << 1);

	// bit2 是否检测到补给区（与兑换站不重叠）
    if (RfidStatus.rfid_status & (1 << 19))
		USER_JudgeData.sentry_info_3 |= (1 << 2);
    else
		USER_JudgeData.sentry_info_3 &= ~(1 << 2);

	// bit3 补给区（与兑换站重叠）
    if (RfidStatus.rfid_status & (1 << 20))
		USER_JudgeData.sentry_info_3 |= (1 << 3);
    else
		USER_JudgeData.sentry_info_3 &= ~(1 << 3);

	// bit4 能量 <30%
    if (BuffMusk.remaining_energy != 0x80)
	{
		if (BuffMusk.remaining_energy & (1 << 3))
			USER_JudgeData.sentry_info_3 &= ~(1 << 4);
		else
			USER_JudgeData.sentry_info_3 |= (1 << 4);
	}
    else
	{
		USER_JudgeData.sentry_info_3 &= ~(1 << 4);
	}

	// 对方前哨站增益点 bit5
    if (RfidStatus.rfid_status & (1 << 18))
		USER_JudgeData.sentry_info_3 |= (1 << 5);
    else
		USER_JudgeData.sentry_info_3 &= ~(1 << 5);

	// 对方堡垒增益点 bit6
    if (RfidStatus.rfid_status & (1 << 24))
		USER_JudgeData.sentry_info_3 |= (1 << 6);
    else
		USER_JudgeData.sentry_info_3 &= ~(1 << 6);

    USER_JudgeData.ally_outpost_hp = JUDGE_GetAllyOutpostHP();
    USER_JudgeData.ally_base_hp = JUDGE_GetAllyBaseHP();


    USER_JudgeData.ally_outpost_hp = JUDGE_GetAllyOutpostHP();
    USER_JudgeData.ally_base_hp = JUDGE_GetAllyBaseHP();
    USER_JudgeData.shooter_barrel_cooling_value = JUDGE_GetCoolingValue();	// 获取冷却速度
    USER_JudgeData.shooter_barrel_heat_limit = JUDGE_GetHeatLimit();				//获取热量限制
    USER_JudgeData.shooter_id1_17mm_cooling_heat = JUDGE_GetNowHeat(); // 获取17mm枪口热量
    USER_JudgeData.power_management_chassis_output = JUDGE_GetChassisOutputState();
    USER_JudgeData.power_management_gimbal_output = JUDGE_GetGimbalOutputState();
    USER_JudgeData.power_management_shooter_output = JUDGE_GetShooterOutputState();
    USER_JudgeData.initial_speed = JUDGE_GetInitial_speed();				 			//获取当前弹速
    USER_JudgeData.self_color = JUDGE_GetSelfColor();
}

/**********************freertos任务*********************************/
// 裁判系统发送任务回调
void Task_Judge_Callback()
{
//    if (Queue_IsEmpty(&judgeQueue))
//	    return;
//	// 取队头的消息发送
//    JudgeTxFrame *frame = (JudgeTxFrame *)Queue_Dequeue(&judgeQueue);
////    USART1_DMA_Send((uint8_t *)frame->data, frame->frameLength);
//    HAL_UART_Transmit_DMA(&huart1,(uint8_t*)frame->data,frame->frameLength);
//	if (Queue_IsEmpty(&judgeQueue))
//		return;
//	JudgeTxFrame *frame = (JudgeTxFrame *)Queue_Dequeue(&judgeQueue);
////	USART1_DMA_Send((uint8_t *)frame->data, frame->frameLength);
//	HAL_UART_Transmit_DMA(&huart1,(uint8_t*)frame->data,frame->frameLength);
}


#ifdef EN_JUDGE_TASK
void OS_JudgeCallback(void const *argument)
{

    osDelay(500);
    for (;;)
	{
	    Judge_update();
//	    Task_Judge_Callback();
	    //		Task_Judge_Callback();
	    osDelay(100);
	}
}
#endif
