#ifndef _BSP_CAN_H_
#define _BSP_CAN_H_

// ============================================================================
// 文件：bsp_can.h
// 作用：CAN 外设驱动(bsp_can.c)的对外接口声明。
//   术语：CAN = Controller Area Network，一种差分、多主、带仲裁的总线，机器人
//         内部各电调/传感器间常用。
//         FDCAN = Flexible Data-rate CAN，STM32H7 上的 CAN 控制器，兼容经典 CAN
//         与 CAN-FD(更高数据段波特率、单帧最多 64 字节)。
//   本文件导出：
//     - CanState：CAN 初始化/收发错误计数，仅供 debug 观察。
//     - 电机反馈在线检测接口 BSP_MotorFeedbackMark / BSP_MotorFeedbackIsOnline。
//     - CAN_Init()：初始化 CAN1/2/3，配置过滤器并开启接收中断。
//     - USER_CAN_Send()：统一发送接口，供上层业务调用。
//   说明：本文件只做声明，具体实现见 BSP/src/bsp_can.c。
// ============================================================================

#include "fdcan.h"
#include "cmsis_os.h"
#include <stdbool.h>

// CAN 运行状态与错误计数结构体。
// 目的：debug 时观察 CAN 是否初始化成功、收发有无出错，便于定位总线问题；
//       这些字段只做统计/观察用，不参与任何控制逻辑。
typedef struct
{
	// 通道 1/2/3 用户层初始化返回值：HAL_OK 表示初始化成功。
	// 类型 HAL_StatusTypeDef 是 HAL 库通用返回码(HAL_OK/HAL_ERROR/HAL_BUSY/HAL_TIMEOUT)。
	HAL_StatusTypeDef can1_user_init_error,can2_user_init_error,can3_user_init_error;
	// 通道 1/2/3 发送失败累计次数：每次发送(AddMessageToTxFifoQ)非 HAL_OK 时 +1。
	uint16_t can1_send_error,can2_send_error,can3_send_error;
	// 通道 1/2/3 接收失败累计次数：每次取报文(GetRxMessage)非 HAL_OK 时 +1。
	uint16_t can1_receive_error,can2_receive_error,can3_receive_error;
}CanState;

// 统一电机反馈通道数量：把底盘/云台等不同总线上的电机映射成从 0 开始的统一"通道号"，
// 用于记录"每个通道最后一次收到反馈的时刻"。当前共 5 个通道。
#define BSP_MOTOR_FEEDBACK_COUNT 5U

// 每个统一电机通道最后一次收到反馈的时间戳数组。
//   单位：ms(HAL_GetTick() 返回值，HAL 的 1ms 计数)。
//   索引：通道号 0 ~ BSP_MOTOR_FEEDBACK_COUNT-1。
//   取值：最近一次收到该通道反馈时的 HAL tick；为 0 表示从未收到过。
//   volatile：该数组在 CAN 接收中断里被写、在任务里被读，禁止编译器优化掉访问。
extern volatile uint32_t bsp_motor_feedback_tick[BSP_MOTOR_FEEDBACK_COUNT];

// 标记一个电机通道"刚刚收到反馈"。
// 参数 channel：统一电机通道号，范围 0 ~ BSP_MOTOR_FEEDBACK_COUNT-1；越界会被忽略。
// 作用：把当前 HAL_GetTick() 写入该通道，刷新其"最后一次反馈时刻"。
// 调用时机：通常在某电机报文解析完成后、于 CAN 接收回调中调用。
// 返回值：无。
void BSP_MotorFeedbackMark(uint8_t channel);

// 判断一个电机通道是否在超时时间内收到过反馈(即"是否在线")。
// 参数 channel   ：统一电机通道号，范围 0 ~ BSP_MOTOR_FEEDBACK_COUNT-1。
// 参数 timeout_ms：允许的最大静默时间，单位 ms；超过则判为离线。
// 返回：true = 在线(在 timeout_ms 内收到过反馈)；false = 离线或通道号非法。
bool BSP_MotorFeedbackIsOnline(uint8_t channel, uint32_t timeout_ms);

// 初始化 CAN 外设：配置过滤器、开启 FIFO0 新报文中断与 Bus-Off 中断，
// 并依次启动 CAN1/2/3。系统启动时调用一次。无参数、无返回值。
void CAN_Init(void);

//========================= 外部调用函数 =========================
// USER_CAN_Send：统一 CAN 发送接口。
// 参数 hfdcan ：目标 CAN 句柄(&hfdcan1 / &hfdcan2 / &hfdcan3)。
// 参数 StdId  ：标准帧 ID(11 位，取值 0x000 ~ 0x7FF)，接收端据此识别报文来源。
// 参数 tx_data：待发送数据缓冲区指针，固定发送 8 字节(经典 CAN 帧)。
// 返回：无。发送失败时不阻塞、不重发，仅累加对应的 can*_send_error 计数。
void USER_CAN_Send(FDCAN_HandleTypeDef* hfdcan,int16_t StdId,uint8_t* tx_data);

#endif
