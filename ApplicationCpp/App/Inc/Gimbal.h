#ifndef _GIMBAL_H_
#define _GIMBAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "USER_Moto.h"
#include "stdbool.h"
#include "slope.h"
#include "PID.h"

#define TOP_YAW_OFFSET 2710  //此处校准小yaw
#define INIT_YAW_ANGLE -85.0f   //此处校准大yaw
#define PITCH_MOTOR_ZERO_POS -1.048f //此处校准pitch

#define PITCH_DIRECTION -1
#define MASS_G 9.81f
#define PITCH_MASS 1.3f
#define PITCH_R 0.07287f   // pitch中心及其距离

// 云台只剩硬件反馈（三个电机）。业务状态（角度/PID/目标/限幅）全部归插件图节点，
// 见 Gimbal_Input_Node / Gimbal_Control_Node / Gimbal_State_Node。
typedef struct 
{
  DM_motor_t base_yawMotor;
  DJI_Motor_t top_yawMotor;
  DM_motor_t pitchMotor;
} Gimbal_t;

extern Gimbal_t gimbal;

#ifdef __cplusplus
}
#endif

#endif
