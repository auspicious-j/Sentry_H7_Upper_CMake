#ifndef _SHOOTER_H_
#define _SHOOTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "USER_Moto.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    IDLE = 0,          // 待机：不拨弹
    TRIGGER,           // 单发：拨一次 1/9 圈
    TRIGGER_REVERSE,   // 堵转反转：反转 1.5 圈后退回 0.5 圈
    TRIGGER_CONTINUE,  // 连发：到位后连续拨弹
    TRIGGER_DOUBLE     // 双发（当前未使用）
} Shooter_State_t;

// 发射机构在 C 侧只剩两样东西：
//   1) 跨图请求（Gimbal Output → Shooter Input）：shooter_state / fric_open / id_to_trigger；
//   2) 跨图事件（Vision Target → Shooter Logic）：triggeringId / expected_hit_delay_ms / expected_hit_at_tick。
// 电机反馈与 PID 也在结构体里（Moto 任务使用）。
// 其余业务状态（fricSpd / bullet_speed / 堵转计数 / 拨弹目标角）已归 Shooter_Logic_Node 私有。
typedef struct
{
    Shooter_State_t shooter_state;  // 拨弹状态机指令

    int8_t id_to_trigger;       // 视觉传来的符id，我这边打完之后回答视觉这个id的符可以屏蔽了
    int8_t triggeringId;        // 当前触发的符id
    uint32_t expected_hit_delay_ms;   // 预计还需多久命中（时长，ms），与 expected_hit_at_tick（时刻）区分
    uint32_t expected_hit_at_tick[5]; // 预计的具体命中时刻

    DJI_Motor_t triggerMotor;   // 拨弹电机和摩擦轮电机
    DJI_Motor_t fricMotor[2];

    bool fric_open;   // 摩擦轮开启标志  0开1关
} Shooter;

extern Shooter shooter;

// 初始状态已归 Shooter_Logic_Node；电机 PID 归 Motor 任务。

#ifdef __cplusplus
}
#endif

#endif
