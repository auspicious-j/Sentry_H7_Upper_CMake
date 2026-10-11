/* =====================================================================
 * Moto.c —— 电机封装与收发调度(应用层)
 * ---------------------------------------------------------------------
 * 本文件把"上层控制量"翻译成 CAN / MIT 帧发给电机, 再把电机反馈读回来控制。
 * 术语:
 *   CAN   : 控制器局域网总线, 一帧含 8 字节负载, 可同时驱动同一广播 ID 下的多台电机。
 *   MIT   : 一种"位置 + 速度 + 力矩"混合控制帧格式(常见于达妙等电机), 可只填其中几项。
 *   ID    : 电机在总线上的编号(0x01 / 0x200 / 0x1FF 等), 决定这一帧被哪台电机接收。
 *   hfdcan1 / hfdcan2 : 两路 FDCAN 外设句柄, 分别挂在不同总线上的电机。
 *   PID   : 比例-积分-微分控制器, 让实际值跟随目标值。
 *   imuPID: 以 IMU(惯性测量单元)角度为反馈的 PID, 用于云台姿态。
 *   fricMotor    : 摩擦轮电机(发射机构), 靠转速把弹丸甩出。
 *   triggerMotor : 拨弹电机, 靠转角把弹丸推入摩擦轮。
 *   gimbal : 云台, 含 top_yaw(上 yaw 轴) 与 pitch(俯仰轴)。
 * 单位约定(具体以电机型号与工程内宏为准):
 *   PID.output 为电流原始值(乘放大系数后为安培); speed 为转速(rpm 或 rad/s);
 *   totalAngle / targetAngle 为累计角度(圈或弧度)。
 * 线程模型: 由 OS_MotorCallback 任务每 1ms 调用一次, 完成"算 PID + 发帧"一整拍。
 * ===================================================================== */
#include "Moto.h"        // 本模块对外声明头(含 include guard)
#include "PID.h"         // PID_* 控制器: 单级 / 串级运算
#include "bsp_can.h"     // 板级 CAN 发送封装(USER_CAN_SetMotorCurrent 等)
#include "USER_Moto.h"   // 电机对象与 MIT 模式收发(mit_ctrl / enable_motor_mode / clear_err)
#include "Shooter.h"     // 发射机构对象 shooter(摩擦轮 / 拨弹电机)
#include "Gimbal.h"      // 云台对象 gimbal(yaw / pitch)
#include "Judge.h"       // 裁判系统数据(供上层决策, 本文件间接使用)


/* Motor_ClearErr —— 电机"使能 / 清错"统一入口
 * 参数:
 *   hfdcan : 目标电机所在的 CAN 句柄(决定把帧发到哪条总线)
 *   id     : 目标电机在总线上的编号
 *   state  : 电机当前状态字(通常取自反馈结构里的 para.state)
 * 返回: 无
 * 说明:
 *   state == 0 表示电机未使能 -> 发使能帧, 让其进入 MIT 工作模式;
 *   state != 1 表示处于(非正常态的)异常   -> 发清错帧, 清除电机故障;
 *   即 state 为 0 或 1 时视为"正常 / 无需清错"。 */
void Motor_ClearErr(FDCAN_HandleTypeDef *hfdcan, uint8_t id, uint8_t state)
{
    if(state == 0)                        // 未使能: 发使能帧, 切换到 MIT 模式
        enable_motor_mode(hfdcan, id, MIT_MODE);
    else if(state != 1)                   // 非正常态: 发清错帧, 清除电机故障
        clear_err(hfdcan, id, MIT_MODE);
}

/* Task_CANMotors_Callback —— 每 1ms 执行一次的核心周期任务
 * 作用: 先根据各电机的最新反馈算出 PID 输出, 再把输出打包成 CAN 帧发出。
 * 参数: 无
 * 返回: 无
 * 说明: 本函数是"算 -> 发"的中枢, 顺序很重要: 必须先更新各 PID / 角度,
 *       再发送, 否则发出的是上一拍的旧值。 */
void Task_CANMotors_Callback()
{
	    PID_SingleCalc(&shooter.fricMotor[0].speedPID,shooter.fricMotor[0].targetSpeed,shooter.fricMotor[0].speed);   // 摩擦轮0: 转速单级 PID, 实测 speed 跟随目标 targetSpeed
	    PID_SingleCalc(&shooter.fricMotor[1].speedPID,shooter.fricMotor[1].targetSpeed,shooter.fricMotor[1].speed);   // 摩擦轮1: 同上, 两轮转速一致以稳定出弹
	    Motor_CalcAngle(&shooter.triggerMotor);                                                                       // 拨弹电机: 由编码器累加出累计角度 totalAngle, 供角度环使用
	    PID_CascadeCalc(&shooter.triggerMotor.anglePID,shooter.triggerMotor.targetAngle,shooter.triggerMotor.totalAngle,shooter.triggerMotor.speed);   // 拨弹电机: 角度-转速串级 PID(外环角度, 内环转速)
	    // 下面三种 pitch 轴(俯仰)控制方式, 仅保留启用其一:
	    //   1) 纯力矩: 只给力矩, 位置与速度填 0(对应下一行被注释的调用);
	    //   2) 速度 + 位置 + 力矩前馈: MIT 完整模式, kp / kd 取自 MIT_PID;
	    //   3) 当前采用: MIT 速度环 + 自写位置外环, 1.5 为该外环 kd, 力矩为前馈。
	    //		mit_ctrl(&hfdcan2,0x01,0,0,0,0,gimbal.pitch.imuPID.output);  //纯力矩控制
	    //		mit_ctrl(&hfdcan2,0x01,p_target,0,gimbal.pitch.MIT_PID.kp,gimbal.pitch.MIT_PID.kd,gimbal.pitch.imuPID.output); //mit速度位置加力矩前馈
	    mit_ctrl(&hfdcan2,0x01,0,gimbal.pitch.imuPID.outer.output,0,1.5,gimbal.pitch.imuPID.output); // MIT 速度环: 位置外环取 imuPID.outer.output, kd=1.5, 力矩前馈=imuPID.output
	    USER_CAN_SetMotorCurrent(&hfdcan1,0x200,shooter.fricMotor[0].speedPID.output,shooter.fricMotor[1].speedPID.output,shooter.triggerMotor.anglePID.output,0);   // 0x200 广播帧一次驱动 3 台: 依次为摩擦轮0/1 电流 + 拨弹电机角度环输出, 末位 0 为保留电机
	    USER_CAN_SetMotorCurrent(&hfdcan2,0x1FF,gimbal.top_yaw.imuPID.output,0,0,0);   // 0x1FF 广播帧: 该帧第 1 台电机为上 yaw, 给其电流, 其余 3 位留 0
}


/* Task_ClearError_Callback —— 周期性检查并复位云台 pitch 电机异常
 * 参数: 无
 * 返回: 无
 * 说明: 把 pitch 电机反馈里的状态字 para.state 交给 Motor_ClearErr,
 *       由其决定是使能 / 清错 / 不动。 */
void Task_ClearError_Callback()
{
  Motor_ClearErr(&hfdcan2,0x01,gimbal.pitchMotor.para.state);   // 对 hfdcan2 上 ID=0x01 的 pitch 电机按状态清错
}

/* OS_MotorCallback —— 电机收发任务入口(FreeRTOS 任务体)
 * 参数:
 *   argument : 任务创建参数, 本任务不使用
 * 返回: 无(死循环任务, 不会返回)
 * 说明:
 *   先延时 1000ms 等系统与电机上电稳定, 再使能 pitch 电机;
 *   随后进入 1ms 周期循环: 计算并发帧 + 检查清错。 */
void OS_MotorCallback(void const * argument)
{
    osDelay(1000);                                // 上电后先等 1s, 待电机与总线稳定
    enable_motor_mode(&hfdcan2, 0x01, MIT_MODE);  // 使能 hfdcan2 上的 0x01 电机, 进入 MIT 模式
    for (;;)
    {
	    Task_CANMotors_Callback();                // 计算各 PID 并发送电机控制帧
        Task_ClearError_Callback();               // 检查并清除电机故障
        osDelay(1);  // 1ms循环
    }
}

