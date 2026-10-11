#include "USER_Detcet.h"
#include "bsp_can.h"
#include "UserFreertos.h"
#include "Beep.h"
#include "Judge.h"
#include "USER_B2B.h"

/* ============================================================================
 * 文件：USER_Detect.c
 * 模块：掉线检测（设备在线/离线状态监控）实现
 * 说明：维护 detectList[]，通过“刷新时间戳 + 周期超时判断”，实现对底盘电机、
 *       拨盘电机、云台电机、拨弹电机、板间通信(B2B)、裁判系统等设备的在线检测。
 *       检测到掉线时优先执行设备自定义的 lostFunc，否则执行默认处理
 *       Detect_DefaultLostHandler()（其中对 B2B 掉线会尝试重新初始化板间通信）。
 * 工作原理：
 *       1) 收包侧调用 Detect_Update(id) 刷新 lastRecieveTime；
 *       2) 后台任务周期调用 OS_DetectCallback() -> Task_Detect_Callback()；
 *       3) 回调遍历所有设备，用 (当前时间 - 上次收包时间) 与 maxInterval 比较：
 *          超出 -> isLost=1(离线) 并触发 lost 回调；
 *          未超 -> isLost=0(在线) 并触发 recover 回调。
 * 注：时间基准为 HAL_GetTick()（单位毫秒，依赖 SysTick，需在调度前初始化）。
 * ==========================================================================*/

// B2B(板间 RS485)掉线重连计数：每次检测到 B2B 掉线时自增，
// 用于区分“刚上电/刚上线时的短暂无数据”与“真正持续掉线”，详见默认掉线处理函数。
uint32_t rs485_cnt = 0;

/****内部函数声明****/
// 默认掉线处理函数
// 参数：deviceID —— 掉线设备的标识（DeviceID_xxx）
// 作用：当设备未注册自定义 lostFunc 时，由检测任务调用此默认处理。
void Detect_DefaultLostHandler(uint8_t deviceID);
// 默认上线处理函数
// 参数：deviceID —— 恢复在线设备的标识（DeviceID_xxx）
// 作用：当设备未注册自定义 recoverFunc 时，由检测任务调用此默认处理（当前为空实现）。
void Detect_DefaultRecoverHandler(uint8_t deviceID);
// 初始化单个设备
// 参数：deviceID    —— 设备标识（DeviceID_xxx，作为 detectList 下标）
//       maxInterval —— 允许的最大收包间隔（单位：ms），超过判离线
//       lostFunc    —— 掉线回调（可为 NULL，表示使用默认处理）
//       recoverFunc —— 恢复回调（可为 NULL，表示使用默认处理）
// 作用：写入指定设备在 detectList[] 中的检测参数（间隔与两个回调）。
void Detect_InitDevice(uint8_t deviceID, uint32_t maxInterval, void (*lostFunc)(void), void (*recoverFunc)(void));

// 设备离线信息列表
// 下标 = 设备标识（DeviceID_xxx），长度 = DETECT_DEVICE_NUM；本文件为其唯一定义处。
DetectDevice detectList[DETECT_DEVICE_NUM];

// 更新指定id设备的状态
// 参数：deviceID —— 设备标识（DeviceID_xxx），用作 detectList 下标
// 作用：把该设备“最近收包时刻”刷新为当前系统毫秒计数（喂狗）。
// 调用时机：应在收到该设备数据的处理流程中调用，表示该设备仍然在线。
void Detect_Update(uint8_t deviceID)
{
    // 记录当前时刻（HAL_GetTick 返回自系统启动以来的毫秒数）
    detectList[deviceID].lastRecieveTime = HAL_GetTick();
}



// 默认掉线处理函数，若lostFunc==NULL则调用
// 参数：deviceID —— 掉线设备的标识（DeviceID_xxx）
// 作用：处理未注册自定义 lostFunc 的设备的掉线事件：
//   - 拨盘电机(Turn_Motor1~4)：仅预留位置（提示音代码已被注释），暂不做处理；
//   - 板间通信(B2B)：累加计数区分“上电对齐”与“持续掉线”，最后尝试重新初始化。
void Detect_DefaultLostHandler(uint8_t deviceID)
{
    // 拨盘电机掉线分支：原设计为播放提示音，现已被注释，暂不处理
    if(deviceID==DeviceID_Turn_Motor1||deviceID==DeviceID_Turn_Motor2||deviceID==DeviceID_Turn_Motor3||deviceID==DeviceID_Turn_Motor4){
		// Beep_PlayNotes((Note[]){{T_M1,D_Sixteenth},{T_M3,D_Sixteenth},{T_M5,D_Sixteenth},{T_H1,D_Sixteenth},{T_None,D_Quarter}},5);
	}
    // 板间通信(B2B)掉线分支
    if (deviceID == DeviceID_B2B)
	{
	    // 掉线次数累加（仅 B2B 掉线时执行）
	    rs485_cnt++;
	    // 前两次掉线直接返回：刚上电时下板尚未收到上板发送的对齐时间，属正常现象，不重连
	    if (rs485_cnt <= 2)
		    return; // 双板485初次进入等上板发送对齐时间

	    // 连续掉线超过 10 次：预留的错误处理位置（错误任务恢复已被注释）
	    if (rs485_cnt > 10)
		{
//		    osThreadResume(ErrorTaskHandle);
		//			osThreadResume(ErrorTaskHandle);
		}
	    // 尝试重新初始化板间通信，以恢复通信
	    B2B_Init();
	}
}

// 默认掉线处理函数，若recoverFunc==NULL则调用
// 参数：deviceID —— 恢复在线设备的标识（DeviceID_xxx）
// 作用：默认的“恢复上线”处理，目前为空实现（不做任何动作）。
//       如需在设备恢复时做提示/重连，可在此补充或改用自定义 recoverFunc。
void Detect_DefaultRecoverHandler(uint8_t deviceID)
{
}

// 初始化一个设备的掉线检测信息(设备id，数据接收最大间隔，掉线处理回调函数)
// 参数：deviceID    —— 设备标识（DeviceID_xxx，作为 detectList 下标）
//       maxInterval —— 允许的最大收包间隔（单位：ms）
//       lostFunc    —— 掉线回调（NULL 表示使用默认处理）
//       recoverFunc —— 恢复回调（NULL 表示使用默认处理）
// 作用：设置指定设备的间隔阈值与回调，并默认标记为“在线”（isLost=0）。
void Detect_InitDevice(uint8_t deviceID, uint32_t maxInterval, void (*lostFunc)(void), void (*recoverFunc)(void))
{
    detectList[deviceID].maxInterval = maxInterval;		// 收包间隔阈值（ms）
    detectList[deviceID].isLost = 0;					// 初始按在线处理
    detectList[deviceID].lostFunc = lostFunc;			// 注册掉线回调
    detectList[deviceID].recoverFunc = recoverFunc;		// 注册恢复回调
}

// 初始化所有设备的掉线检测信息
// 作用：逐个注册每个设备的收包间隔阈值（单位：ms）与回调；
//       必须在启动检测任务(OS_DetectCallback)之前调用一次。
// 说明：除裁判系统(Judge)注册了掉线回调 Judge_UartLostCallback 外，
//       其余设备均传 NULL（掉线/恢复走默认处理）。
void Detect_InitAll()
{
    // 底盘电机：100ms 未收到 CAN 反馈即判离线
    Detect_InitDevice(DeviceID_ChassisMotor1, 100, NULL, NULL);
    Detect_InitDevice(DeviceID_ChassisMotor2, 100, NULL, NULL);
    Detect_InitDevice(DeviceID_ChassisMotor3,100, NULL, NULL);
    Detect_InitDevice(DeviceID_ChassisMotor4, 100, NULL, NULL);

    // 拨盘电机：200ms 阈值（较底盘宽松）
    Detect_InitDevice(DeviceID_Turn_Motor1, 200, NULL, NULL);
    Detect_InitDevice(DeviceID_Turn_Motor2, 200, NULL, NULL);
    Detect_InitDevice(DeviceID_Turn_Motor3, 200, NULL, NULL);
    Detect_InitDevice(DeviceID_Turn_Motor4, 200, NULL, NULL);

    // 云台 Yaw 电机：100ms 阈值
    Detect_InitDevice(DeviceID_YawMotor, 100, NULL, NULL);
    // 上电即把云台电机标记为离线(isLost=1)，待其发来数据后由检测任务置回在线
    detectList[DeviceID_YawMotor].isLost = 1;

    // 板间通信(B2B)：500ms 阈值（RS485 通信周期较长，阈值放宽）
    Detect_InitDevice(DeviceID_B2B, 500, NULL, NULL);

    // 裁判系统：500ms 阈值；掉线时回调 Judge_UartLostCallback（如提示/告警）
    Detect_InitDevice(DeviceID_Judge, 500, Judge_UartLostCallback, NULL);
}

// 外部定义的一个标志变量（定义在其他 .c 文件中）；本文件当前未使用，保留声明。
extern uint8_t FLAG;

// 掉线检测任务回调
// 作用：遍历 detectList[] 中的所有设备，依据“距上次收包的时间”与各自阈值，
//       判定在线/离线，并执行对应回调、更新 isLost 标志。
// 说明：非阻塞，可用于周期任务；时间基准为 HAL_GetTick()（ms）。
void Task_Detect_Callback()
{
    // 本轮遍历统一使用同一时间基准，避免中途时间漂移
    uint32_t presentTime = HAL_GetTick();

	// 逐个设备检查
	for (uint8_t id = 0; id < DETECT_DEVICE_NUM; id++)
	{
		// 判定是否掉线
		// 距上次收包时长 = presentTime - lastRecieveTime；无符号计数天然处理回绕，
		// 超过 maxInterval 即认为掉线
		if (presentTime - detectList[id].lastRecieveTime > detectList[id].maxInterval)
		{
			// 判定执行默认还是自定义的处理函数
			if (detectList[id].lostFunc == NULL)
				Detect_DefaultLostHandler(id);
			else
				detectList[id].lostFunc();
			// 更新标识
			detectList[id].isLost = 1;
		}
		// 未超时（接收间隔在阈值内）：判为在线，执行恢复处理
		else if (presentTime - detectList[id].lastRecieveTime <= detectList[id].maxInterval)
		{
			// 判定执行默认还是自定义的处理函数
			if (detectList[id].recoverFunc == NULL)
				Detect_DefaultRecoverHandler(id);
			else
				detectList[id].recoverFunc();
			// 更新标识
			detectList[id].isLost = 0;
		}
	}
}

// 获取设备离线情况
// 参数：deviceID —— 设备标识（DeviceID_xxx）
// 返回值 1-离线 0-在线
uint8_t Detect_IsDeviceLost(uint8_t deviceID)
{
    // 直接返回检测任务维护的离线标志
    return detectList[deviceID].isLost;
}

// 掉线检测任务入口（供 FreeRTOS 任务体调用）
// 参数：argument —— FreeRTOS 任务参数，本任务未使用
// 作用：先延时 2s 等待上电稳定与设备初始化，再以 10ms 周期调用
//       Task_Detect_Callback() 进行掉线检测（10ms 远小于各设备阈值，反应足够灵敏）。
// 说明：本函数为死循环任务体，不会返回。
void OS_DetectCallback(void const *argument)
{
    osDelay(2000);
    for (;;)
	{
	    Task_Detect_Callback();
	    osDelay(10);
	}
}
