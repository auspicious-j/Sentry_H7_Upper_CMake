extern "C"
{
#include "Shooter.h"
#include "Shooter_GraphApi.h"
#include "cmsis_os.h"
}

// 发射业务状态归 Shooter_Logic_Node（configure() 初始化摩擦轮基准转速）；
// 电机 PID 与累计角初始化归 Motor 任务。这里只保留 C 侧结构体定义与任务入口。
Shooter shooter = { .idToTrigger = -1, .triggeringId = -1 };

/*************发射机构任务******************/

extern "C" void OS_ShooterCallback(void const * argument)
{
    osDelay(100);

    AppCpp_Shooter_Init();

    for(;;)
	{
	    AppCpp_Shooter_ProcessFrame();
	    osDelay(10);
	}
}
