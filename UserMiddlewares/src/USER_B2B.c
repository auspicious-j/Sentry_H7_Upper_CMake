/*
 * USER_B2B.c
 * 板间通信（board-to-board，B2B）实现文件。
 * 本文件实现上板（本工程）与下板之间的 USART2 双向数据交换：
 *   - 上板 到 下板：舵轮误差角、轮电机目标速度、云台 yaw 目标速度、
 *     哨兵自主指令、急停标志，见 B2B_Transmit。
 *   - 下板 到 上板：舵电机当前角度、轮电机当前速度、大 yaw 电机角度、
 *     裁判系统数据、下板反馈标志，见 B2B_Receive。
 * 术语说明：
 *   B2B   ：board-to-board，即同一台机器人两块主控板之间的板间通信。
 *   帧    ：一次完整收发的定长数据包（本工程固定 64 字节），用帧头/帧尾标识起止。
 *   单圈角：电机编码器在一圈内的绝对角度，单位°（度），范围约 0 到 360。
 * 并发说明：接收由 USART2 空闲中断/DMA 触发，快照的发布与读取用短临界区保护。
 */
#include "USER_B2B.h"
#include "robot_build_config.h"
#include "STM32/ChassisFeedbackBridge.h"
#include "Chassis.h"
#include "usart.h"
#include "cmsis_os.h"
#include "Chassis.h"
#include "Gimbal.h"
#include <string.h>
#include "Vision.h"
#include "Judge.h"

/* USART2 接收 DMA 句柄（CubeMX 生成）。B2B_Init 用它关闭“半传输”中断：
 * 空闲中断方式只在整帧结束时触发，无需在半缓冲区满时打断。 */
extern DMA_HandleTypeDef hdma_usart2_rx;
/* 底盘（下板）上报的 yaw 角，全局共享，单位°（度），初值 0。 */
float chassis_yaw = 0;

/* 需要用到的接收变量*/
/* 上板接收缓冲：DMA 把 USART2 收到的原始字节写入此处，容量 256 字节；
 * 收到完整帧后由 B2B_Receive() 按固定偏移解析。 */
uint8_t usart2RxBuf[256]; // 串口2缓冲区
/* 上板发给下板的急停请求：1=请求下板急停，0=正常。 */
uint8_t STOPFLAG = 0;   //是1下板急停
/* 下板回传的故障/急停反馈：0=下板处于急停，非 0=正常；初值 0。 */
uint8_t FEEDBACK = 0;		//是0下板急停

/* 接收帧计数（调试统计用），上电初值 0。 */
uint32_t receive_times;

/* 需要用到的发送变量*/
/* 上板发送缓冲：按固定协议打包后一次性 DMA 发出，容量 64 字节（帧长同为 64）。 */
uint8_t usart2TxBuf[64];

#if ROBOT_ENABLE_CHASSIS_OBSERVER
// 仅由 B2B 接收回调写入；任务通过短临界区取得完整样本。
// volatile：该结构可能在中断上下文中被修改，避免编译器优化掉对它的读取。
static volatile RobotChassisFeedbackSnapshot chassis_feedback_snapshot = {0};

// 旧解码完成后发布快照；不改变旧 chassis 对象或发送内容。
// 该函数在 USART2 接收回调上下文中调用，故写入无需加锁。
static void B2B_PublishChassisSnapshot(void)
{
    uint8_t index; // 轮组下标，保持旧协议顺序。
    for (index = 0U; index < ROBOT_CHASSIS_FEEDBACK_WHEELS; ++index)
    {
        chassis_feedback_snapshot.steering_deg[index] = chassis.motors[index].TurnAngle; // 舵向电机当前角度，单位°
        chassis_feedback_snapshot.drive_rpm[index] = chassis.motors[index].now_Speed;    // 驱动轮当前转速，单位 rpm
    }
    chassis_feedback_snapshot.lower_feedback = FEEDBACK;        // 下板反馈标志，原样透传
    chassis_feedback_snapshot.received_tick_ms = HAL_GetTick(); // 记录本次接收时刻（ms），用于判断数据新鲜度
    chassis_feedback_snapshot.receive_count++;                  // 累计收到的帧数，自增
    chassis_feedback_snapshot.received = 1U;                    // 置位“已收到过数据”标志
}
#endif

// destination：接收结果；只供任务读取，不允许空指针写入。
void RobotChassisFeedback_Read(RobotChassisFeedbackSnapshot* destination)
{
    if (destination == NULL)
    {
        return;
    }
#if ROBOT_ENABLE_CHASSIS_OBSERVER
    const uint32_t saved_primask = __get_PRIMASK(); // 调用前中断屏蔽状态。
    __disable_irq(); // 进入临界区：关中断，避免读到写了一半的快照
    __DMB();         // 数据内存屏障：确保屏障前的内存访问已完成
    // 临界区只复制固定大小快照和读取 tick，不进行解算。
    *destination = chassis_feedback_snapshot;
    destination->sampled_tick_ms = HAL_GetTick();
    __DMB();                      // 屏障：确保快照复制完成后才恢复中断
    __set_PRIMASK(saved_primask); // 恢复调用前的中断屏蔽状态
#else
    const RobotChassisFeedbackSnapshot empty = {0}; // 关闭观察时的空快照。
    *destination = empty;
#endif
}

// 板间通信初始化
// 启动 USART2 的“空闲+DMA”接收：DMA 自动把收到的字节搬进 usart2RxBuf，
// 总线空闲时结束本次接收并触发回调；同时关闭 DMA 半传输(HT)中断，
// 因为整帧接收只需在结束时通知，不希望缓冲半满时被反复打断。
void B2B_Init()
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2RxBuf, sizeof(usart2RxBuf)); // 启动空闲+DMA 接收，长度=缓冲区大小
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT); // 关闭半传输中断
}

/* 打包并通过 USART2 向下板发送一帧（帧长固定 64 字节）。
 * 参数：无；返回值：无。
 * 帧布局：0xAA 帧头 | 1-16 四个舵轮误差角(float) | 17-24 四个轮电机目标速度(int16)
 *          | 25-28 yaw 目标速度(float) | 29-35 哨兵自主指令 | 62 急停标志 | 0xFE 帧尾。*/
void B2B_Transmit()
{
	usart2TxBuf[0] = 0xAA;				 // 帧头：一帧的起始标志，固定为 0xAA
	for(uint8_t i = 0; i < 4; i++)
	{
		float v = chassis.motors[i].targetTurnAngle - chassis.motors[i].now_angle;   // 要发送的那个 float
		uint8_t *p = (uint8_t *)&v; // 用字节指针访问 float 的 4 个字节，便于按字节拆包
		usart2TxBuf[1 + 4*i] = p[0]; // 小端序：先发最低字节
		usart2TxBuf[2 + 4*i] = p[1];
		usart2TxBuf[3 + 4*i] = p[2];
		usart2TxBuf[4 + 4*i] = p[3];
	} //1-16 舵电机误差角度
	for (uint8_t i = 0; i < 4; i++)
	{
		usart2TxBuf[17 + i * 2] = chassis.motors[i].targetDriveSpeed;         // 目标速度低字节
		usart2TxBuf[17 + i * 2 + 1] = chassis.motors[i].targetDriveSpeed >> 8; // 目标速度高字节（小端）
	} // 17-24 轮电机目标速度
	{
		float v = gimbal.base_yaw.imuPID.output;   // 要发送的那个 float
		v = gimbal.base_yaw.imuPID.outer.output;   // 实际取外环输出，覆盖上一行的取值
		uint8_t *p = (uint8_t *)&v;
		usart2TxBuf[25] = p[0];
		usart2TxBuf[26] = p[1];
		usart2TxBuf[27] = p[2];
		usart2TxBuf[28] = p[3];
	}	//25-28 yaw电机目标速度

	memcpy(&usart2TxBuf[29], &USER_SentryCmd, sizeof(USER_SentryCmd_t)); //29-35 哨兵自主指令

	usart2TxBuf[62]  = STOPFLAG;    // 急停标志
	usart2TxBuf[63] = 0xFE;				 // 帧尾：一帧的结束标志，固定为 0xFE

	HAL_UART_Transmit_DMA(&huart2, usart2TxBuf, sizeof(usart2TxBuf)); // 以 DMA 方式整帧发送
}


/* 解析下板通过 USART2 发来的一帧数据（帧长固定 64 字节）。
 * 参数：无；返回值：无。
 * 与发送帧对应：先校验帧头 0xAB、帧尾 0xFD，再按固定偏移把字节拼回各物理量。*/
void B2B_Receive(void)
{
    if (usart2RxBuf[0] == 0xAB && usart2RxBuf[63] == 0xFD) // 帧头 0xAB、帧尾 0xFD 同时匹配才视为有效帧
	{
	    for(uint8_t i = 0; i < 4; i++) // 遍历 4 个舵轮电机
		{
		    float v;
		    uint8_t *p = (uint8_t *)&v; // 用字节指针按字节填充 float
		    p[0] = usart2RxBuf[1 + 4*i]; // 按小端序取 4 个字节
		    p[1] = usart2RxBuf[2 + 4*i];
		    p[2] = usart2RxBuf[3 + 4*i];
		    p[3] = usart2RxBuf[4 + 4*i];
		    chassis.motors[i].TurnAngle = v; // 写回舵电机当前角度
		} //1-16 舵电机当前角度（单圈）单位° 

	    for (uint8_t i = 0; i < 4; i++)
		{
		    chassis.motors[i].now_Speed = (int16_t)usart2RxBuf[17 + i * 2] | (int16_t)usart2RxBuf[17 + i * 2 + 1] << 8; // 低字节 | 高字节<<8 组合为有符号 16 位转速
		} //17-24 轮电机当前速度
		{
		    float v;
		    uint8_t *p = (uint8_t *)&v;
		    p[0] = usart2RxBuf[25];
		    p[1] = usart2RxBuf[26];
		    p[2] = usart2RxBuf[27];
		    p[3] = usart2RxBuf[28];
		    gimbal.base_yawMotor.nowAngle = v; // 写回大 yaw 电机当前角度
		}	//25-28 大yaw电机当前角度（单圈）单位°
		// {
		// 	    float v;
		// 	    uint8_t *p = (uint8_t *)&v;
		// 	    p[0] = usart2RxBuf[29];
		// 	    p[1] = usart2RxBuf[30];
		// 	    p[2] = usart2RxBuf[31];
		// 	    p[3] = usart2RxBuf[32];
		// 	    gimbal.base_yawMotor.para.vel = v;
		// }	//解析大yaw电机当前速度  单位rad/s 29-32
	    // 		float v;
	    // 		uint8_t *p = (uint8_t *)&v;
	    // 		p[0] = usart2RxBuf[29];
	    // 		p[1] = usart2RxBuf[30];
	    // 		p[2] = usart2RxBuf[31];
	    // 		p[3] = usart2RxBuf[32];
	    // 		gimbal.base_yawMotor.para.vel = v;
	    memcpy(&USER_JudgeData, &usart2RxBuf[29], sizeof(JudgeData_t)); //解析裁判系统数据 29-58

	    FEEDBACK = usart2RxBuf[62]; // 读取下板反馈标志（第 62 字节）
#if ROBOT_ENABLE_CHASSIS_OBSERVER
        B2B_PublishChassisSnapshot(); // 在旧帧校验及解析完成后记录反馈。
#endif
	}
}

/************************freertos任务****************************/
/* 板间通信周期任务：周期性地向下板发送一帧控制数据。
 * 参数 argument：RTOS 任务入参，本函数未使用。*/
void OS_Board2BoardCallback(void const *argument)
{
    while (1)
	{
	    B2B_Transmit(); // 发送一帧
	    osDelay(1);     // 延时 1 个 tick，控制发送周期
	}
}
