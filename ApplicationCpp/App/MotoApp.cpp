extern "C"
{
#include "Moto.h"
#include "cmsis_os.h"
#include "PID.h"
#include "bsp_can.h"
#include "USER_Moto.h"
#include "Shooter.h"
#include "Gimbal.h"
#include "Judge.h"
}

#include "GimbalControlMailbox.hpp"
#include "ShooterOutputMailbox.hpp"


static void Motor_ClearErr(FDCAN_HandleTypeDef *hfdcan, uint8_t id, uint8_t state)
{
    if(state == 0)
        enable_motor_mode(hfdcan, id, MIT_MODE);
    else if(state != 1)
        clear_err(hfdcan, id, MIT_MODE);
}

// 电机侧 PID 与累计角初始化（原来在 App/ 的 Shooter_InitPID 里）。
static void Motor_InitPIDs()
{
    Motor_StartCalcAngle(&shooter.triggerMotor);
    PID_Init(&shooter.triggerMotor.anglePID.inner, 6, 0.15, 8, 6000, 16000);
    PID_Init(&shooter.triggerMotor.anglePID.outer, 0.8, 0, 12, 0.12, 16000);
    PID_Init(&shooter.fricMotor[0].speedPID, 25, 0, 5, 0, 16000);
    PID_Init(&shooter.fricMotor[1].speedPID, 25, 0, 5, 0, 16000);

    PID_Clear(&shooter.triggerMotor.anglePID.inner);
    PID_Clear(&shooter.triggerMotor.anglePID.outer);
}

static void Task_CANMotors_Callback()
{
	    // 云台控制量由 Gimbal 任务的 Control_Node 通过邮箱发布（跨任务，最新值语义）。
	    // 以前直接读 gimbal.*.imuPID.output：没有同步、没有帧号，靠任务周期相对快慢。
	    robot::framework::timed_snapshot<robot::Gimbal::gimbal_control> control{};
	    robot::Gimbal::gimbal_control_mailbox().copy_to(control);

	    // 发射目标量由 Shooter 任务的 Logic_Node 通过邮箱发布。
	    robot::framework::timed_snapshot<robot::Shooter::shooter_output> shooter_output{};
	    robot::Shooter::shooter_output_mailbox().copy_to(shooter_output);

	    PID_SingleCalc(&shooter.fricMotor[0].speedPID,shooter_output.value.fric_target_speed[0],shooter.fricMotor[0].speed);
	    PID_SingleCalc(&shooter.fricMotor[1].speedPID,shooter_output.value.fric_target_speed[1],shooter.fricMotor[1].speed);
	    Motor_CalcAngle(&shooter.triggerMotor);
	    PID_CascadeCalc(&shooter.triggerMotor.anglePID,shooter_output.value.trigger_target_angle,shooter.triggerMotor.totalAngle,shooter.triggerMotor.speed);
  	    mit_ctrl(&hfdcan2,0x01,0,control.value.pitch_outer_output,0,1.5,control.value.pitch_mit_torque); //mit速度环 自己写位置环 +力矩前馈
	    USER_CAN_SetMotorCurrent(&hfdcan1,0x200,shooter.fricMotor[0].speedPID.output,shooter.fricMotor[1].speedPID.output,shooter.triggerMotor.anglePID.output,0);
	    USER_CAN_SetMotorCurrent(&hfdcan2,0x1FF,control.value.top_yaw_current,0,0,0);
}


static void Task_ClearError_Callback()
{
  Motor_ClearErr(&hfdcan2,0x01,gimbal.pitchMotor.para.state);
}

extern "C" void OS_MotorCallback(void const * argument)
{
    // PID 与累计角在任务入口立即初始化：早于 Shooter 图开始累积拨弹目标角。
    Motor_InitPIDs();

    osDelay(1000);
    enable_motor_mode(&hfdcan2, 0x01, MIT_MODE);
    for (;;)
    {
	    Task_CANMotors_Callback();
        Task_ClearError_Callback();
        osDelay(1);  // 1ms循环
    }
}
