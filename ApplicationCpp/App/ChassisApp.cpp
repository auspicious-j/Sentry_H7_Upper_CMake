extern "C"
{
#include "Chassis.h"
#include "Chassis_GraphApi.h"
#include "cmsis_os.h"
}

// 底盘运动学状态（几何、零点、斜坡、PID、速度）归 Chassis_Kinematics_Node 所有，
// 在它的 configure() 里初始化；这里只保留 C 侧结构体定义与任务入口。
Chassis_t chassis = {0};

/*************底盘操控任务******************/

extern "C" void OS_ChassisCallback(void const * argument)
{
    osDelay(500);

    AppCpp_Chassis_Init();

    for(;;)
    {
        AppCpp_Chassis_ProcessFrame();
        osDelay(2);
    }
}
