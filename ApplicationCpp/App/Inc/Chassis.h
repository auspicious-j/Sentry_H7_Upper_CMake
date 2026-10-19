#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "USER_Moto.h"
#include "ChassisMode.h"

#define WHEELSPEED_MAX 8000

// 底盘只剩硬件反馈：四轮舵角与轮速（B2B 中断写入，Io/Inputs 读取）。
// 运动学状态（几何、零点、斜坡、旋转 PID、三轴速度）归 Chassis_Kinematics_Node 私有，
// 四轮目标通过 chassis_output_mailbox() 交给 RS485 任务。
typedef struct _Chassis
{
    Double_motor_t motors[4];
} Chassis_t;

extern Chassis_t chassis;

#ifdef __cplusplus
}
#endif

#endif
