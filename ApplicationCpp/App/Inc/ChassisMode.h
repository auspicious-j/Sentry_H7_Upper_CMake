#ifndef _CHASSIS_MODE_H_
#define _CHASSIS_MODE_H_

#ifdef __cplusplus
extern "C" {
#endif

/* 底盘模式枚举单独成头：图节点（C++）只需要这两个 enum，
   不应该为了它把带 HAL 的 Chassis.h 拖进 .hpp。 */

typedef enum
{
    ChassisMode_Follow,	  // 底盘跟随云台模式
    ChassisMode_Spin,	  // 小陀螺模式
} ChassisMode;

typedef enum
{
    Chassis_Control,
    Chassis_AI
} chassis_pattern;

#ifdef __cplusplus
}
#endif

#endif
