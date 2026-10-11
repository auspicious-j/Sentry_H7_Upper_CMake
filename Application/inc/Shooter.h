/*
 * ============================================================================
 *  Shooter.h —— 发射机构控制模块 头文件
 * ============================================================================
 *  所属工程：RoboMaster 哨兵机器人 STM32H7 上位机（C 语言应用层）。
 *
 *  术语先讲清楚（后面注释会反复用到）：
 *    摩擦轮(friction wheel)：一对高速反向旋转的橡胶轮，弹丸从两轮间挤过时被加速射出。
 *    拨弹机构(trigger)：把弹丸逐个推入摩擦轮的机构；本工程用步进式拨弹盘，
 *                      电机每转过固定角度就放出一发弹。
 *    堵转(block/stall)：拨弹机构被卡住、电机实际角度长时间追不上目标角度的异常状态。
 *    热量(heat)：裁判系统对发射频率的冷却限制；剩余热量越低越要拉开发射间隔，
 *                否则连续发射会导致枪口热量超限被判违规。
 *
 *  本文件只定义“数据结构 + 对外接口”：
 *    - Shooter 结构体：保存发射系统的全部状态（发弹量、工作状态、两台摩擦轮与一台
 *      拨弹电机的句柄、摩擦轮转速、弹速补偿量、堵转计数等）；
 *    - 全局实例 shooter；
 *    - Shooter_Init()：发射系统初始化接口。
 *  具体控制逻辑（摩擦轮调速、拨弹状态机、堵转反转、热量限制）见 Application/src/Shooter.c。
 * ============================================================================
 */
#ifndef _SHOOTER_H_
#define _SHOOTER_H_

/* ---------- 依赖头文件（电机抽象、bool 类型等） ---------- */
#include "Moto.h"
#include "stdbool.h"
#include "USER_Moto.h"

#include <stdbool.h>

// 拨弹机构工作状态机（shooter.workState 的取值来源）
// 由 Shooter.c 中 Task_Shooter_Callback() 的 switch 处理；除 TRIGGER_REVERSE 外，处理完都会回到 IDLE。
enum
{
    IDLE = 0, //空闲，不拨弹（初始状态，数值 0）
    TRIGGER, //单次拨弹：收到一次发射指令，拨弹盘推出一发弹丸后回到 IDLE
    TRIGGER_REVERSE, //堵转后反转拨弹机构（被卡住时先反转回退再小幅正转脱困）
    TRIGGER_CONTINUE, //连续拨弹：连发模式，单次推弹角度比单发更大
    TRIGGER_DOUBLE //双击拨弹：预留状态（当前 switch 未处理，走 default 分支）
};

typedef struct
{
    int number;				  // 发弹量（累计已发射弹丸数，每拨弹一次 +1）
    uint8_t workState;		  // 工作状态（取值为上面枚举的 IDLE/TRIGGER/...）

    DJI_Motor_t triggerMotor; // 拨弹电机（步进式拨弹盘的驱动电机，负责推出弹丸）
    DJI_Motor_t fricMotor[2]; // 摩擦轮电机[2]：两轮反向旋转，弹丸从轮间挤过被加速发射

    bool fricOpenFlag;	  // 摩擦轮开启标志（true=旋转待发射，false=停转）

    int16_t fricSpd;	  // 摩擦轮目标转速（单位：电机转速；默认 6500，随实测弹速微调）
    float ave_bullet_speed; // 弹速滑动平均（单位：m/s，预留，本文件当前未直接使用）
    float bullet_speed; // 最新实测弹丸出膛速度（单位：m/s，来自裁判系统 initial_speed）

    float distance;		 // 测距（单位：m，用于弹道/弹速补偿，预留，本文件当前未直接使用）
    float last_bullet_speed; // 上一次弹速值（用于判断是否“收到新的弹速消息”）
    uint8_t box; // 弹仓/存弹量相关信息（预留，本文件当前未直接使用）

    struct
	{
	    uint16_t judgeCnt, reverseCnt; // 堵转判定计数器,反转计数器（judgeCnt 累加超过阈值即判定堵转）
	    uint16_t fric_judgeCnt, fric_reverseCnt; // 摩擦轮堵转判定/反转计数（预留，当前未使用）
	    bool state; // 堵转状态标志（预留，当前未使用）
	} block; // 堵转处理相关数据（拨弹机构被卡住时的计数与状态）

} Shooter;

extern Shooter shooter; // 发射机构全局实例：全工程共享的一整套发射系统状态

// 发射系统初始化：设置摩擦轮默认转速、初始化各电机 PID，并把工作状态复位为空闲。
// 参数：无。返回值：无。
// 应在电机、裁判系统初始化之后调用（通常由 OS_ShooterCallback 任务开头调用一次）。
void Shooter_Init(void);

#endif
