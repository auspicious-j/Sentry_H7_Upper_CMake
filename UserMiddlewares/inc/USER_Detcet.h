#ifndef _USER_DETECT_H_
#define _USER_DETECT_H_

#include "main.h"

/* ============================================================================
 * 文件：USER_Detcet.h
 * 模块：掉线检测（设备在线/离线状态监控）
 * 说明：本模块为板上所有关键外设/子模块提供统一的“喂狗式”掉线检测。
 *       每个被监控设备对应 detectList[] 中的一个条目；对应设备每次成功收到
 *       数据时调用 Detect_Update() 刷新时间戳，后台任务周期性调用
 *       Task_Detect_Callback() 检查各设备距上次收到数据的时间是否超过
 *       各自允许的最大间隔（maxInterval），超时即判为离线并执行掉线回调
 *       （lostFunc）；恢复在线则执行恢复回调（recoverFunc）。
 * 术语：
 *   - 掉线/离线(lost)：超过 maxInterval 仍未收到该设备的数据。
 *   - 喂狗(Update)：设备收到数据时刷新时间戳，表示“还活着”。
 * 注：枚举下标即为 detectList[] 的下标，顺序不可随意调整。
 * ==========================================================================*/

// 连接在主板上的设备的标识(即设备离线信息列表中的下标)
// 该匿名枚举的取值用作 detectList[] 的数组下标，唯一定位一个被监控设备；
// 因此枚举成员的先后顺序 = 数组下标顺序，新增设备请追加在末尾。
enum
{
	// 底盘电机
    // 底盘 4 个驱动电机（如 M3508），下标 0~3
    DeviceID_ChassisMotor1 = 0,	// 底盘电机 1（下标 0）
    DeviceID_ChassisMotor2,		// 底盘电机 2（下标 1）
    DeviceID_ChassisMotor3,		// 底盘电机 3（下标 2）
    DeviceID_ChassisMotor4,		// 底盘电机 4（下标 3）

  // 拨盘/摩擦轮电机（共 4 个），下标 4~7（缩进沿用原文件）
  DeviceID_Turn_Motor1,			// 拨盘电机 1（下标 4）
  DeviceID_Turn_Motor2,			// 拨盘电机 2（下标 5）
  DeviceID_Turn_Motor3,			// 拨盘电机 3（下标 6）
  DeviceID_Turn_Motor4,			// 拨盘电机 4（下标 7）
	// 云台电机
    DeviceID_YawMotor,			// 云台 Yaw 轴电机（下标 8）
	// 拨弹电机
    DeviceID_TrigMotor,			// 拨弹（Trigger）电机（下标 9）
	// 板间通信
    DeviceID_B2B,				// 上下板板间通信（Board-to-Board，RS485，下标 10）
	// 裁判系统接收
    DeviceID_Judge,				// 裁判系统串口接收（下标 11）

    DETECT_DEVICE_NUM // 放在最后，代表设备数量(设备离线信息列表长度)
};

// 掉线检测结构体，每个设备对应一个结构体变量
// 保存单个设备的掉线检测所需信息：允许静默时长、最近收包时刻、当前在线状态，
// 以及掉线/恢复时需要执行的回调函数。
typedef struct _DetectDevice
{
    // 允许的最大静默时长（单位：ms）。距 lastRecieveTime 超过该值即判离线；
    // 由 Detect_InitDevice() 设置，各设备取值不同（见 Detect_InitAll）。
    uint32_t maxInterval;	   // 最大收包间隔阈值（ms），超时判离线
    // 最近一次收到该设备数据的时刻（HAL_GetTick() 提供的系统毫秒计数），
    // 由 Detect_Update() 在每次收包时刷新。
    uint32_t lastRecieveTime;  // 最近收包时刻（ms，来自 HAL_GetTick）
    // 离线标志：1=已离线，0=在线。由 Task_Detect_Callback() 维护，
    // 通过 Detect_IsDeviceLost() 对外提供查询。
    uint8_t isLost;			   // 离线标志：1=离线，0=在线
    // 掉线处理回调（函数指针）；为 NULL 时改用默认处理 Detect_DefaultLostHandler()。
    void (*lostFunc)(void);	   // 掉线回调（NULL→走默认处理）
    // 恢复连接（重新上线）处理回调（函数指针）；为 NULL 时改用默认处理
    // Detect_DefaultRecoverHandler()。
    void (*recoverFunc)(void); // 恢复回调（NULL→走默认处理）
} DetectDevice;

/****接口函数声明****/
// 更新数据接收时间
// 参数：deviceID —— 设备标识（DeviceID_xxx 枚举值），用作 detectList 下标
// 作用：刷新该设备的“最近收包时刻”，表示其仍然在线（喂狗）。
// 调用时机：应在收到该设备数据的中断/任务中调用。
void Detect_Update(uint8_t deviceID);
// 获取设备丢失情况
// 参数：deviceID —— 设备标识（DeviceID_xxx 枚举值）
// 返回值：1=离线，0=在线
uint8_t Detect_IsDeviceLost(uint8_t deviceID);
// 初始化
// 作用：初始化所有被监控设备的掉线检测参数（间隔阈值与回调），
//       必须在启动检测任务前调用一次。
void Detect_InitAll(void);
// 任务回调
// 作用：周期性检查各设备是否掉线/恢复并触发相应回调；
//       函数本身不含延时，由调用者在循环中按固定周期调用。
void Task_Detect_Callback(void);

// 设备离线信息列表：下标即设备标识（DeviceID_xxx），长度 DETECT_DEVICE_NUM。
// 定义在 USER_Detect.c 中，供其他模块查询设备在线状态时引用。
extern DetectDevice detectList[DETECT_DEVICE_NUM];

#endif
