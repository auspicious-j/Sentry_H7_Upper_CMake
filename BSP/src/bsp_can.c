// ============================================================================
// 文件：bsp_can.c
// 作用：CAN(FDCAN) 外设驱动实现——过滤器配置、发送、接收中断回调与按 ID 分发。
//   数据流：CAN 硬件收到报文 → 触发 RxFifo0 中断 → HAL_FDCAN_RxFifo0Callback
//           → 按句柄(hfdcan1/2/3)分发到 CANx_Rx0Callback → 按帧 ID 解析并更新对应电机。
//   术语：FIFO0 = 接收 FIFO 通道 0；标准帧 ID = 11 位报文标识(取值 0x000~0x7FF)。
//   include 说明：bsp_can.h 本模块接口；fdcan.h 提供 hfdcan1/2/3 句柄；
//                 USER_Moto.h / Shooter.h / Gimbal.h 提供各电机结构与更新接口。
// ============================================================================
#include "bsp_can.h"
#include "fdcan.h"
#include "USER_Moto.h"
#include "Shooter.h"
#include "Gimbal.h"

// 全局 CAN 状态与错误计数实例(类型见 bsp_can.h 的 CanState)，仅供 debug 观察。
CanState can_state;

// 各电机通道最后一次收到反馈的 HAL tick 时间戳数组。
//   单位：ms；索引=统一通道号(0 ~ BSP_MOTOR_FEEDBACK_COUNT-1)。
//   值为 0 表示该通道从未收到过反馈。
//   volatile：由 CAN 接收中断写入、被任务读取，禁止编译器优化其访问。
volatile uint32_t bsp_motor_feedback_tick[BSP_MOTOR_FEEDBACK_COUNT] = {0U};

// 记录电机反馈接收时间：把当前时刻写入指定通道。
// 参数 channel：统一电机通道号(0 ~ BSP_MOTOR_FEEDBACK_COUNT-1)。
// 返回值：无。通道号越界时直接忽略，防止数组越界写。
void BSP_MotorFeedbackMark(uint8_t channel)
{
    // 边界检查：通道号非法则丢弃本次标记。
    if (channel < BSP_MOTOR_FEEDBACK_COUNT)
    {
        // 用 HAL 的 1ms 计数作为时间戳，供在线检测做超时判断。
        bsp_motor_feedback_tick[channel] = HAL_GetTick();
    }
}

// 检查电机反馈是否超时(即该通道是否在线)。
// 参数 channel   ：统一电机通道号(0 ~ BSP_MOTOR_FEEDBACK_COUNT-1)。
// 参数 timeout_ms：允许的最大静默时间(单位 ms)。
// 返回值：true=在线；false=离线，或通道号非法/从未收到过反馈。
bool BSP_MotorFeedbackIsOnline(uint8_t channel, uint32_t timeout_ms)
{
    // 通道号非法，或时间戳为 0(从未收到过反馈)→ 判为离线。
    if (channel >= BSP_MOTOR_FEEDBACK_COUNT || bsp_motor_feedback_tick[channel] == 0U)
    {
        return false;
    }
    // 用无符号相减得“已静默多久”，可自然处理 HAL tick 回绕；不超过超时即在线。
    return (HAL_GetTick() - bsp_motor_feedback_tick[channel]) <= timeout_ms;
}

/************** 内部工具函数声明(各 CAN 接收回调) ***********************/
// 三个回调分别对应 CAN1/2/3 的接收分发；用途与参数见各自定义处。
void CAN1_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header,uint8_t *rxdata);
//can1接收回调
void CAN2_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header,uint8_t *rxdata);
//can2接收回调
void CAN3_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header,uint8_t *rxdata);
//can3接收回调
/****************** 初始化 ***************************/
// CAN 初始化：配置过滤器/全局过滤器/中断，再依次启动 CAN1、CAN2、CAN3。
// 三个通道共用同一份 filter 配置(接收全部标准帧到 RXFIFO0)。
// 无参数、无返回值；在系统启动阶段调用一次。

void CAN_Init()
{
	FDCAN_FilterTypeDef filter;                   	//< 过滤器配置结构体：描述 FDCAN 的接收过滤规则
	filter.IdType       = FDCAN_STANDARD_ID;       	//< 帧 ID 类型：标准帧(11 位 ID)
	filter.FilterIndex  = 0;                      	//< 过滤器编号：标准 ID 可用 0~127，此处用 0 号
	filter.FilterType   = FDCAN_FILTER_MASK;       	//< 过滤器类型：掩码模式(用 FilterID2 的掩码位比较)
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; 	//< 命中后送入接收 FIFO0，待 RxFifo0 回调取走
	filter.FilterID1    = 0x00000000;                   	//< 要匹配的 ID(掩码模式下与 FilterID2 共同决定匹配范围)
	filter.FilterID2    = 0x00000000;  //< 掩码值：0 表示不比较任何位，即接收全部标准 ID
	
	// 用同一份 filter 分别配置三个 CAN 通道。
	HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);   //< 配置过滤器	
	// 全局过滤器：未匹配的帧与远程帧统一拒收(本项目只用经典数据帧)。
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT,FDCAN_REJECT,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // 使能fifo0接收到新信息中断
	// 使能 Bus-Off(总线脱离)中断：CAN 发生严重错误时由错误回调处理。
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0);
	// 设置接收 FIFO0 水位为 1：FIFO 中报文数达到水位就触发接收中断。
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1,FDCAN_CFG_RX_FIFO0, 1);
	// 启动 CAN1：进入正常收发状态(应在所有配置完成后调用)。
	HAL_FDCAN_Start(&hfdcan1);                   //< 使能can

	// ===== CAN2：复用同一份 filter，重复相同配置 =====
	HAL_FDCAN_ConfigFilter(&hfdcan2, &filter);   //< 配置过滤器	
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT,FDCAN_REJECT,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE);
	// 设置接收 FIFO0 水位为 1。
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan2,FDCAN_CFG_RX_FIFO0, 1);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // 使能fifo0接收到新信息中断
	// 使能 Bus-Off 中断。
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_BUS_OFF, 0);
	// 启动 CAN2。
	HAL_FDCAN_Start(&hfdcan2);                   //< 使能can
	
	// ===== CAN3：复用同一份 filter，重复相同配置 =====
	HAL_FDCAN_ConfigFilter(&hfdcan3, &filter);   //< 配置过滤器	
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT,FDCAN_REJECT,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE);
	// 设置接收 FIFO0 水位为 1。
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan3,FDCAN_CFG_RX_FIFO0, 1);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // 使能fifo0接收到新信息中断
	// 使能 Bus-Off 中断。
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_BUS_OFF, 0);
	// 启动 CAN3。
	HAL_FDCAN_Start(&hfdcan3);                   //< 使能can
}

// RXFIFO0 接收中断回调：HAL 库在报文到达 FIFO0 时调用本函数。
// 参数 hfdcan   ：触发中断的 CAN 句柄，用于区分 CAN1/2/3。
// 参数 RxFifo0ITs：触发标志位(本函数未使用，直接按句柄分发)。
// 返回值：无。流程：取报文 → 成功则分发到对应 CANx_Rx0Callback；
//         失败则累加对应接收错误计数；最后重新使能 FIFO0 中断。
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	HAL_StatusTypeDef	if_can_get_message_ok;  // 取报文结果：HAL_OK 表示成功
	FDCAN_RxHeaderTypeDef rx_header;  // 保存本次收到的帧信息(ID、长度等)
	uint8_t rx_data[8];  // 保存本次收到的 8 字节数据

	// ---- CAN1 分支 ----
	if(hfdcan == &hfdcan1)
	{
		if_can_get_message_ok = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
		if(if_can_get_message_ok == HAL_OK)
		{
			
			CAN1_Rx0Callback(&rx_header,rx_data);
			
		}
		else
		{
			can_state.can1_receive_error++;  // 取报文失败：CAN1 接收错误计数 +1
		}
		HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);  // 重新使能 FIFO0 新报文中断
	}
	// ---- CAN2 分支 ----
	else if(hfdcan == &hfdcan2)
	{
		if_can_get_message_ok = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
		if(HAL_OK == if_can_get_message_ok)
		{
      CAN2_Rx0Callback(&rx_header,rx_data);  
		}
		else
		{
			can_state.can2_receive_error++;  // 取报文失败：CAN2 接收错误计数 +1
		}
		HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);  // 重新使能 FIFO0 新报文中断
	}
	// ---- CAN3 分支 ----
	else if(hfdcan == &hfdcan3)
	{
		if_can_get_message_ok = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
		if(HAL_OK == if_can_get_message_ok)
		{
			CAN3_Rx0Callback(&rx_header,rx_data);   
			
		}
		else
		{
			can_state.can3_receive_error++;  // 取报文失败：CAN3 接收错误计数 +1
		}
		HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);  // 重新使能 FIFO0 新报文中断
	}
        
}

// CAN1 接收回调：解析并分发 CAN1 上的电机报文。
// 参数 rx_header：接收帧头(Identifier 即帧 ID)；参数 rxdata：8 字节数据指针。
// 返回值：无。DJI 电机报文格式：data[0..1] 机械角度、[2..3] 转速、[4..5] 电流、[6] 温度。
void CAN1_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header,uint8_t *rxdata)
{
	uint8_t whichMotor;  // 摩擦轮电机在 fricMotor[] 中的下标：0x201→0，0x202→1
	switch(rx_header->Identifier)
	{
		// 摩擦轮电机(两个)：ID 0x201、0x202
		case 0x201:
		case 0x202:
        {
					whichMotor = rx_header->Identifier - 0x201;  // 由 ID 反推下标(0x201→0，0x202→1)
					DJIMotor_Update(&shooter.fricMotor[whichMotor], (rxdata[0]<<8 | rxdata[1]), (rxdata[2]<<8 | rxdata[3]),(rxdata[4]<<8|rxdata[5]),rxdata[6]);
                    BSP_MotorFeedbackMark(whichMotor);
        }
				break;
		// 拨弹(trigger)电机：ID 0x203
		case 0x203:
				{
					DJIMotor_Update(&shooter.triggerMotor, (rxdata[0] << 8 | rxdata[1]), (rxdata[2] << 8 | rxdata[3]), (rxdata[4] << 8 | rxdata[5]), rxdata[6]);
                    BSP_MotorFeedbackMark(2U);  // 通道 2 = 拨弹电机
				}
				break;
		//未知信息
		default:
				break;
	}
}


// CAN2 接收回调：解析并分发 CAN2 上的电机报文。
// 参数 rx_header：接收帧头；参数 rxdata：数据指针。返回值：无。
// 说明：CAN2 上既有达妙4310 电机(自定义协议)，也有 DJI 电机(ID 0x205)。
void CAN2_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header,uint8_t *rxdata)
{
	switch(rx_header->Identifier)
	{
		// 达妙 4310 电机(ID 0x11)：pitch 轴，按该电机的自定义协议解析
		case 0x11:
        {
					dm4310_fbdata(&gimbal.pitchMotor,rxdata);
                    BSP_MotorFeedbackMark(4U);  // 通道 4 = pitch 电机
//			Detect_Update(DeviceID_Turn_Motor1+whichMotor);	
        }
		break;
		// DJI 上 yaw 电机：ID 0x205
		case 0x205:
			{
					DJIMotor_Update(&gimbal.top_yawMotor,(rxdata[0] << 8 | rxdata[1]), (rxdata[2] << 8 | rxdata[3]), (rxdata[4] << 8 | rxdata[5]), rxdata[6]);
                    BSP_MotorFeedbackMark(3U);  // 通道 3 = top_yaw 电机
			}
        break;
		//未知信息
		default:
				break;
	}
}
            
// CAN3 接收回调：预留给挂载在 CAN3 上的设备(如超级电容)。
// 参数 rx_header：接收帧头；参数 rxdata：数据指针。返回值：无。
// 当前仅保留分发框架，尚未实际解析任何报文。
void CAN3_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header,uint8_t *rxdata)
{   
	switch(rx_header->Identifier)
	{
		// 0x300：超级电容(Supercap)反馈 ID(当前未启用解析)
		case 0x300:
			// 预留：超级电容解析(已注释，未启用)
			break;
        default :
            break;
		} 
}


//========================= 外部调用函数 =========================
// 统一 CAN 发送接口：向指定 CAN 总线发送一帧标准数据帧。
// 参数 hfdcan ：目标 CAN 句柄(&hfdcan1/&hfdcan2/&hfdcan3)。
// 参数 StdId  ：标准帧 ID(11 位，0x000~0x7FF)。
// 参数 tx_data：待发送数据缓冲区指针，固定发送 8 字节。
// 返回值：无。发送失败仅累加对应错误计数，不阻塞、不重发。
void USER_CAN_Send(FDCAN_HandleTypeDef* hfdcan,int16_t StdId,uint8_t* tx_data)
{
	FDCAN_TxHeaderTypeDef tx_header;  // 发送帧头：描述 ID、帧类型、长度等
    tx_header.Identifier = StdId;  // 目标标准帧 ID
	tx_header.IdType = FDCAN_STANDARD_ID;  // 标准帧(11 位 ID)
 	tx_header.TxFrameType = FDCAN_DATA_FRAME;  // 数据帧(非远程帧)
  	tx_header.DataLength = 8;  // 数据长度 8 字节(经典 CAN) 
 	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  // 错误状态：主动
 	tx_header.BitRateSwitch = FDCAN_BRS_OFF;  // 不切换波特率(经典 CAN)
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;  // 经典 CAN 帧格式(非 CAN-FD)
	tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;  // 不生成发送事件
	tx_header.MessageMarker = 0;  // 消息标记(无特殊用途)
		
    vTaskSuspendAll();  // 挂起调度器：保证入队期间不被打断(临界区)
	// 将报文放入发送 FIFO 队列；返回非 HAL_OK 视为发送失败。
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan,&tx_header,tx_data)!= HAL_OK)
	{
		if(hfdcan==&hfdcan1)
		{
			can_state.can1_send_error++;  // CAN1 发送失败计数 +1
		}
		else if(hfdcan==&hfdcan2)
		{
			can_state.can2_send_error++;  // CAN2 发送失败计数 +1
		}
		else if(hfdcan==&hfdcan3)
		{
			can_state.can3_send_error++;  // CAN3 发送失败计数 +1		
		}
	}
  	xTaskResumeAll();  // 恢复调度器
}

// 检查 CAN 总线是否处于 Bus-Off 状态，若是则清 INIT 位使控制器重新初始化(自动恢复)。
// 参数 hfdcan：目标 CAN 句柄。返回值：无。
static void check_can_bus(FDCAN_HandleTypeDef *hfdcan)
{
	FDCAN_ProtocolStatusTypeDef protocolStatus;  // CAN 协议状态(含 BusOff 等标志)

	HAL_FDCAN_GetProtocolStatus(hfdcan, &protocolStatus);  // 读取当前协议状态
	// 发生总线脱离(Bus-Off)：清 INIT 位使控制器重新参与总线
	if (protocolStatus.BusOff)
	{
		CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);  // 复位 CCCR 的 INIT 位 → 退出初始化、重新接入总线
	}
}

// FDCAN 错误状态回调：CAN 出现 Bus-Off 等错误时由 HAL 调用。
// 参数 hfdcan       ：触发回调的 CAN 句柄。
// 参数 ErrorStatusITs：错误中断标志(CAN1/2/3 分别判断)。
// 返回值：无。仅对 Bus-Off 事件调用 check_can_bus() 做自动恢复。
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
   // ---- CAN1：若发生 Bus-Off 则尝试恢复 ----
   if (hfdcan == &hfdcan1) {
     if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET) {

      check_can_bus(hfdcan);
			 
    }
  }
	// ---- CAN2：若发生 Bus-Off 则尝试恢复 ----
	if (hfdcan == &hfdcan2) {
     if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET) {

      check_can_bus(hfdcan);
			 
    }
  }
	 // ---- CAN3：若发生 Bus-Off 则尝试恢复 ----
	 if (hfdcan == &hfdcan3) {
     if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET) {

      check_can_bus(hfdcan);
			 
    }
  }
}
