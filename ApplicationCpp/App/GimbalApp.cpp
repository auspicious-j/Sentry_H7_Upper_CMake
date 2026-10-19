extern "C"
{
#include "Gimbal.h"
#include "Gimbal_GraphApi.h"
#include "cmsis_os.h"
#include "imu_temp_ctrl.h"
#include "USER_RC.h"
#include "Vision.h"
#include "Chassis.h"
#include "Shooter.h"
#include "degree.h"
#include "Judge.h"
#include "PID.h"
#include "arm_math.h"

#include <math.h>
}

Gimbal_t gimbal;

// region Initialize
static void Gimbal_InitPID(void)
{
    /*pitch由陀螺仪控制*/
    PID_Init(&gimbal.pitch.imuPID.inner, 6, 0, 0.4, 500, 7000);
    DEPID_Init(&gimbal.pitch.imuPID.deOuter, 75, 0.7, 120, 200, 7000, 0.6);//45, 0.7, 50
    PID_Init(&gimbal.pitch.imuPID.outer,-0.8,-0.001,-8,1.3,10);
    PID_Init(&gimbal.pitch.MIT_PID,100,0,2,0,0);

    PID_Init(&gimbal.base_yaw.imuPID.inner,5.8,0.02,3.5,1000,7000);
    DEPID_Init(&gimbal.base_yaw.imuPID.deOuter,37,0.03,173,700,2000,0.45);
    PID_Init(&gimbal.base_yaw.imuPID.outer,300,0.5,5000,1000,15000);

    PID_Init(&gimbal.top_yaw.imuPID.inner, 50, 0.1, 10, 7000, 30000);//270 0.3  300
    DEPID_Init(&gimbal.top_yaw.imuPID.deOuter, 150, 0.8, 300, 100, 30000, 0.7);
}

// 初始化云台
void Gimbal_Init(void)
{
    gimbal.pitch.pitchMax = 34; // 设定pitch角度限幅
    gimbal.pitch.pitchMin = -20;

    gimbal.pitch.initAngle = 0; // 陀螺仪pitch开机角度 *0表示pitch水平
    gimbal.pitch.targetAngle = gimbal.pitch.initAngle;

    gimbal.base_yaw.initAngle = 0; // 陀螺仪yaw开机角度   *0表示yaw不动
    gimbal.base_yaw.targetAngle = gimbal.base_yaw.initAngle;

    gimbal.top_yaw.initAngle = 0;
    gimbal.top_yaw.targetAngle = gimbal.top_yaw.initAngle;

    Gimbal_InitPID();         // 初始化PID参数
}
// endregion

/*************云台操控任务******************/

extern "C" void OS_GimbalCallback(void const *argument)
{
    osDelay(1500);
    Gimbal_Init();

    // 业务由 C++ 插件图执行；旧的 Task_Gimbal_Callback 及其辅助函数已删除。
    AppCpp_Gimbal_Init();

    for (;;)
    {
        AppCpp_Gimbal_ProcessFrame();
        osDelay(1);
    }
}
