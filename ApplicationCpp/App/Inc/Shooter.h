#ifndef _SHOOTER_H_
#define _SHOOTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "USER_Moto.h"

#include <stdbool.h>
#include <stdint.h>

enum
{
    IDLE = 0,
    TRIGGER,
    TRIGGER_REVERSE,
    TRIGGER_CONTINUE,
    TRIGGER_DOUBLE
};

// 发射机构在 C 侧只剩两样东西：
//   1) 跨图请求（Gimbal Output → Shooter Input）：command / fricOpen / idToTrigger；
//   2) 跨图事件（Vision Target → Shooter Logic）：triggeringId / hitAtTick / triggeringHitTick。
// 电机反馈与 PID 也在结构体里（Moto 任务使用）。
// 其余业务状态（fricSpd / bullet_speed / 堵转计数 / 拨弹目标角）已归 Shooter_Logic_Node 私有。
typedef struct
{
    uint8_t command; // 后workstate

    int8_t idToTrigger;// 视觉传来的符id，我这边打完之后回答视觉这个id的符可以屏蔽了
    int8_t triggeringId;// 当前触发的符id
    uint32_t triggeringHitTick;// 当前准备发射的预期命中时长
    uint32_t hitAtTick[5];// 预计的具体命中时刻

    DJI_Motor_t triggerMotor; // 拨弹电机和摩擦轮电机
    DJI_Motor_t fricMotor[2];

    bool fricOpen;	  // 摩擦轮开启标志  0开1关
} Shooter;

extern Shooter shooter;

// 初始状态已归 Shooter_Logic_Node；电机 PID 归 Motor 任务。

#ifdef __cplusplus
}
#endif

#endif
