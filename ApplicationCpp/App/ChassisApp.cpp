extern "C"
{
#include "Chassis.h"
#include "Chassis_GraphApi.h"
#include "cmsis_os.h"
#include "USER_Moto.h"
#include "bsp_can.h"
#include "USER_RC.h"
#include "Gimbal.h"
#include "Vision.h"
#include "degree.h"
#include "arm_math.h"

#include <math.h>
}

Chassis_t chassis = {0};

// 底盘初始值配置
void Chassis_Init()
{
    // 底盘尺寸信息（用于解算轮速）
    chassis.info.wheelbase = 320;
    chassis.info.wheeltrack = 320;
    chassis.info.wheelRadius = 115;
    chassis.info.offsetX = 0; // 15
    chassis.info.offsetY = 0; //-10
    // 移动参数初始化

    // 旋转参数初始化
    chassis.rotate.InitAngle = INIT_YAW_ANGLE; 
    chassis.rotate.InitpitchAngle = 1290;

	chassis.motors[0].TurnOffset= -304.57 - 30 + 180; //
	chassis.motors[1].TurnOffset= -44.78 + 30 + 45 + 180;//
	chassis.motors[2].TurnOffset= -119.49 + 30 + 90 + 15 + 180;//  //此处校准舵电机 正常加减30°的倍数
	chassis.motors[3].TurnOffset= -6.5 + 90 - 15 + 180;//

    // 斜坡函数初始化
    Slope_Init(&chassis.move.xSlope, 40, 0);
    Slope_Init(&chassis.move.ySlope, 40, 0);
    Slope_Init(&chassis.move.spinSlope, 0.1, 0);
    Slope_Init(&chassis.move.outputSlope, 0.1, 0);
    Slope_Init(&chassis.move.chargeSlope, 0.15, 0);

    Chassis_InitPID();
}

// 底盘PID值配置
void Chassis_InitPID()
{
    PID_Init(&chassis.rotate.pid, 0.25, 0, 6, 2, 15); // 15    PID_Init(&chassis.rotate.pid, 0.4, 0.001, 0.15, 0, 15); // 15
    PID_SetDeadzone(&chassis.rotate.pid, 0.1);
}

// 小陀螺三轴速度最大值配置
void Spin_SpeedUpdate()
{
    chassis.move.maxVw = 3.25f; // chassis.move.maxPower * 0.225f + 3.25f
    if (chassis.move.maxVw <= 0.5f)
    {
        chassis.move.maxVw = 0.5f;
    }

    if (chassis.rotate.mode == ChassisMode_Spin)
    {
        chassis.move.maxVx = 2159.6f;//0.4985f * chassis.move.maxPower * chassis.move.maxPower - 40.994f * chassis.move.maxPower + 2159.6f
        if (chassis.move.maxVx <= 0)
            chassis.move.maxVx = 0;
        chassis.move.maxVy = chassis.move.maxVx;
    }
    else
    {
        chassis.move.maxVx = 5500;
        chassis.move.maxVy = 5500;
    }
}

// 斜坡更新
void Chassis_UpdateSlope()
{
    Slope_NextVal(&chassis.move.xSlope);
    Slope_NextVal(&chassis.move.ySlope);
    Slope_NextVal(&chassis.move.spinSlope);
    Spin_SpeedUpdate();
    float rotateRatio = (chassis.info.wheelbase + chassis.info.wheeltrack) / 4.0f;
    chassis.move.maxVx = WHEELSPEED_MAX / 60.0f / (268.f / 17.f) * 2 * PI * chassis.info.wheelRadius;
    chassis.move.maxVy = chassis.move.maxVx;
    chassis.move.maxVw = WHEELSPEED_MAX / rotateRatio / 60.0f / (268.f / 17.f) * 2.0f * PI * chassis.info.wheelRadius * 1.0f / 1.414f;
}

/*************底盘操控任务******************/

extern "C" void OS_ChassisCallback(void const * argument)
{
    osDelay(500);
    Chassis_Init();

    // 业务由 C++ 插件图执行；旧的 Task_Chassis_Callback / Chassis_ModeControl 已删除。
    AppCpp_Chassis_Init();

    for(;;)
    {
        AppCpp_Chassis_ProcessFrame();
        osDelay(2);
    }
}
