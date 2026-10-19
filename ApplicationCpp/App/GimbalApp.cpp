extern "C"
{
#include "Gimbal.h"
#include "Gimbal_GraphApi.h"
#include "cmsis_os.h"
}

Gimbal_t gimbal;

// 云台限幅归 Gimbal_Input_Node 私有成员，三组 PID 归 Gimbal_Control_Node；
// 角度/PID 输出/测量量分别走端口与邮箱。这里只保留 C 侧结构体定义与任务入口。

/*************云台操控任务******************/

extern "C" void OS_GimbalCallback(void const *argument)
{
    osDelay(1500);

    AppCpp_Gimbal_Init();

    for (;;)
    {
        AppCpp_Gimbal_ProcessFrame();
        osDelay(1);
    }
}
