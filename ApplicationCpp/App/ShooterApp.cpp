extern "C"
{
#include "Shooter.h"
#include "Shooter_GraphApi.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "Chassis.h"
#include "USER_RC.h"
#include "Beep.h"
#include "Vision.h"
#include "Gimbal.h"
#include "Judge.h"

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
}

Shooter shooter = { .idToTrigger = -1, .triggeringId = -1 };

void Shooter_InitPID(void);

//射击系统初始化
void Shooter_Init()
{
    shooter.fricSpd = 6500;
    Shooter_InitPID();						// m初始化电机pid
    shooter.fricMotor[0].targetSpeed = -0;
    shooter.fricMotor[1].targetSpeed = +0;
    shooter.command = IDLE;
}

void Shooter_InitPID()
{
    Motor_StartCalcAngle(&shooter.triggerMotor);							 // 初始化电机角度累计
    PID_Init(&shooter.triggerMotor.anglePID.inner, 6, 0.15, 8, 6000, 16000); // 6 0.15 8//10000
    PID_Init(&shooter.triggerMotor.anglePID.outer, 0.8, 0, 12, 0.12, 16000); // 1.5 0 2.3   3.27 maxoutput 9000->7000

    PID_Init(&shooter.fricMotor[0].speedPID, 25, 0, 5, 0, 16000); // 摩擦轮
    PID_Init(&shooter.fricMotor[1].speedPID, 25, 0, 5, 0, 16000);
}

/*************发射机构任务******************/

extern "C" void OS_ShooterCallback(void const * argument)
{
    osDelay(100);
    Shooter_Init();
    PID_Clear(&shooter.triggerMotor.anglePID.inner);
    PID_Clear(&shooter.triggerMotor.anglePID.outer);
    osDelay(100);

    // 业务由 C++ 插件图执行；旧的 Task_Shooter_Callback / Heat_Limit 已删除。
    AppCpp_Shooter_Init();

    for(;;)
	{
	    AppCpp_Shooter_ProcessFrame();
	    osDelay(10);
	}
}
