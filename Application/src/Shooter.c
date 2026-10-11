#include "Shooter.h"
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

Shooter shooter; // 发射机构全局实例（定义；供各任务函数读写发射系统状态）
uint32_t t = 0; // 线性火控延时（单位：ms，由 Heat_Limit() 按剩余热量算出；发弹后用 osDelay(t) 拉开射速间隔，防止热量超限）

// 内部函数前向声明：初始化拨弹电机与摩擦轮电机的 PID 参数（定义见下方）
void Shooter_InitPID(void);

// 射击系统初始化：设定摩擦轮默认转速、初始化各电机 PID，并把状态复位为空闲
// 参数：无。返回值：无。
void Shooter_Init()
{
    shooter.fricSpd = 6500; // 摩擦轮默认目标转速（电机转速，约 6500）
Shooter_InitPID();						// m初始化电机pid（调用下方 PID 初始化；注意此行未缩进）
    shooter.fricMotor[0].targetSpeed = -0; // 摩擦轮0 目标速度先置 0（负号表示其转向与轮1相反）
    shooter.fricMotor[1].targetSpeed = +0; // 摩擦轮1 目标速度先置 0（正号表示正向旋转）
    shooter.workState = IDLE; // 初始工作状态：空闲（不拨弹）
}

// 初始化发射系统各电机的 PID 控制器
// 参数：无。返回值：无。
// 说明：拨弹电机用“角度环(外) + 速度环(内)”串级 PID 控制转动角度；摩擦轮电机只用速度环 PID 控制转速。
// PID_Init(pid, p, i, d, maxSum, maxOut)：p/i/d 为比例/积分/微分系数，maxSum 为积分限幅，maxOut 为输出限幅。
void Shooter_InitPID()
{
Motor_StartCalcAngle(&shooter.triggerMotor);							 // 初始化电机角度累计（开始累计 totalAngle，用于精确拨弹）
    PID_Init(&shooter.triggerMotor.anglePID.inner, 6, 0.15, 8, 6000, 16000); // 拨弹电机 角度环内层(速度环)：p=6 i=0.15 d=8，积分限幅6000，输出限幅16000
    PID_Init(&shooter.triggerMotor.anglePID.outer, 0.8, 0, 12, 0.12, 16000); // 拨弹电机 角度环外层(位置环)：p=0.8 i=0 d=12，积分限幅0.12，输出限幅16000

    PID_Init(&shooter.fricMotor[0].speedPID, 25, 0, 5, 0, 16000); // 摩擦轮0 速度环：p=25 i=0 d=5，积分限幅0，输出限幅16000
    PID_Init(&shooter.fricMotor[1].speedPID, 25, 0, 5, 0, 16000); // 摩擦轮1 速度环：参数同上（保证左右两轮转速一致）
}

// 热量限制：根据裁判系统剩余热量动态计算发弹间隔 t（线性火控延时）
// 原理：热量充足→间隔为 0（可连发）；热量接近上限→拉大间隔（错峰发射，防止枪口热量超限被判违规）
// 参数：无。返回值：恒为 true（本函数不直接禁止发射，只通过写入全局 t 来限制射速）
bool Heat_Limit()
{
    float d = 0.6; // 阈值（历史遗留变量，下方用 (void)d 显式忽略，当前无实际作用）
	(void)d;	   // 阈值（把变量 d 标记为“已使用”，避免编译器未使用变量告警）
    float k = 1;   // 防止浪费未使用热量,增加冷却缩减系数（节拍系数：1=标准射速，越小越快）
    if (JUDGE_GetRemainHeat() < 20)
	{
	    k = 5; // 剩余热量 < 20：非常紧张（本分支未更新 t，沿用上一次的发弹间隔）
	    return true;
	}
    else if (JUDGE_GetRemainHeat() <= 35)
	{
	    k = 1; // 剩余热量 20~35：中等偏紧
	    t = k * 1000 * 10 / JUDGE_GetCoolingValue() * 1.01f; // 发弹间隔(ms)=1000*10/冷却速度*1.01（留 1% 余量）
	    return true;
	}
    else if (JUDGE_GetRemainHeat() <= 100)
	{
	    k = 0.5; // 剩余热量 35~100：较充裕
	    t = k * 1000 * 10 * (-JUDGE_GetRemainHeat() + 100) / (100 - 30) / JUDGE_GetCoolingValue(); // 间隔随剩余热量线性缩短（越接近 100 越快）

	    return true;
	}
    else
	{
	    t = 0; // 剩余热量 > 100：热量充足，不额外延时（允许连续发射）
	    k = 0.5;
	    return true;
	}
}

// 发射机构周期回调（每 10ms 由 OS_ShooterCallback 调用一次）
// 职责：①按实测弹速微调摩擦轮转速；②开关摩擦轮；③堵转检测与反转脱困；④按状态机拨弹。
// 本函数含 osDelay，会阻塞当前射击任务，是射速节拍控制的执行点。
void Task_Shooter_Callback()
{
    static uint8_t over_speed, down_speed=0; // 弹速偏高/偏低的连续计数（防抖：累计到阈值才真正调节摩擦轮转速）
//    heat=JUDGE_GetRemainHeat(); //（已注释）原用于缓存剩余热量，现每个周期直接调用 Heat_Limit()，无需该变量
//	heat=JUDGE_GetRemainHeat(); //（已注释）同上
    if (USER_JudgeData.initial_speed > 10) // 裁判系统上报弹速 > 10 m/s 视为有效测量（滤掉 0/异常值）
	{
	shooter.bullet_speed = USER_JudgeData.initial_speed; // 记录实测弹速（m/s），供下方摩擦轮调速使用
	}

    if (shooter.bullet_speed != shooter.last_bullet_speed) // 弹速发生变化 → 认为收到一帧新弹速消息
	{
	    //22
	    if (shooter.bullet_speed > 22.5f)//23（期望弹速约 22 m/s；偏高：降低摩擦轮转速）
		{
		    over_speed++; // 偏高计数 ++

		    if (over_speed >= 2) // 连续 2 次偏高才调节，防止测量抖动导致误调
			{
			    over_speed = 0; // 清计数
			    shooter.fricSpd -= 60; // 摩擦轮目标转速下调 60（弹速随之下降）
			    down_speed = 0; // 偏高说明“偏低”计数失效，清零
			}
		}
	    else if (shooter.bullet_speed < 21.5f)//21（偏低：提高摩擦轮转速）
		{
		    down_speed++; // 偏低计数 ++

		    if (down_speed >= 4) // 连续 4 次偏低才调节（偏低比偏高更难触发，更保守）
			{
			    down_speed = 0;
			    shooter.fricSpd += 50; // 摩擦轮目标转速上调 50（弹速随之上升）
			}
		}
	}

    shooter.last_bullet_speed = shooter.bullet_speed; // 更新“上一次弹速”，供下次比较是否收到新消息

    if (shooter.fricOpenFlag) // 摩擦轮开启：按 fricSpd 输出目标转速
	{
	    shooter.fricMotor[0].targetSpeed = -shooter.fricSpd; // 摩擦轮0 反向旋转（两轮对滚夹持弹丸）
	    shooter.fricMotor[1].targetSpeed = shooter.fricSpd; // 摩擦轮1 正向旋转（与轮0 方向相反）
	}
    else // 摩擦轮关闭：两轮目标转速置 0 停转
	{
	    shooter.fricMotor[0].targetSpeed = 0; // 摩擦轮速度
	    shooter.fricMotor[1].targetSpeed = 0;
	}


//堵转处理
//电机角度与目标角度相差超过10度则进行堵转判定
// 原理：正常拨弹时电机能跟上目标角度；若实际总角度 totalAngle 长时间落后目标角度 targetAngle
//       超过 10 度，说明拨弹盘被卡住（堵转）。
    if (abs(shooter.triggerMotor.totalAngle - shooter.triggerMotor.targetAngle) > MOTOR_M3508_DGR2CODE(10) && shooter.workState != TRIGGER_REVERSE) // 角度误差>10度 且 当前不在反转流程中
	//	Shooter_state(shooter.fricOpenFlag);
	{
	    shooter.block.judgeCnt++; //堵转判定计数器++（误差持续超限时累加）

	    if (shooter.block.judgeCnt > 100) //计数器达到一定值，则判定为堵转 ，触发反转（连续 100 次超限≈持续卡住）
		{
		    shooter.block.judgeCnt = 0;
		    shooter.workState = TRIGGER_REVERSE; // 切换到“反转脱困”状态
		}
	}
    else //与目标值相差小于10度，拨弹状态正常，将堵转判定计数器归零（防止误判累积）
	{
	    shooter.block.judgeCnt = 0;
	}
	Heat_Limit(); // 每周期刷新一次发弹间隔 t（为下方拨弹提供节拍）
    //拨弹
	switch(shooter.workState)
	{
	case TRIGGER: // 单次拨弹：推出 1/9 圈弹量
		if (shooter.fricOpenFlag && (!JUDGE_IsValid() || Heat_Limit()))   //未安装裁判系统 或 裁判系统剩余热量大于100 允许发射
		{
			if(shooter.triggerMotor.targetAngle-shooter.triggerMotor.totalAngle<MOTOR_M3508_DGR2CODE(9)) // 距目标角度不足 9 度→上次拨弹已完成，可安排下一次
			{
				shooter.triggerMotor.targetAngle+=MOTOR_M3508_DGR2CODE(360*1/9.0*1);  //每次转动1/9圈（推出 1 发弹丸）
				shooter.workState=IDLE;
				shooter.number +=1; // 发弹量 +1
				osDelay(t); // 按热量限制算出的节拍延时（ms），控制射速
			}
		}
		else
		{
			shooter.workState=IDLE; // 条件不满足（摩擦轮未开或热量不允许）→ 取消本次拨弹
		}
	break;

	case TRIGGER_CONTINUE: // 连续拨弹：每次推 2/9 圈弹量（连发）
		if (shooter.fricOpenFlag && (!JUDGE_IsValid() || Heat_Limit()))   //未安装裁判系统 或 裁判系统剩余热量大于100 允许发射
		{
			if (shooter.triggerMotor.targetAngle-shooter.triggerMotor.totalAngle<MOTOR_M3508_DGR2CODE(9)) // 距目标角度不足 9 度→可安排下一次
			{
				shooter.triggerMotor.targetAngle+=MOTOR_M3508_DGR2CODE(360*1/9.0*2);  //每次转动1/9圈（连发：单次推弹角度为单发的两倍，即 2/9 圈）
				shooter.number +=1; // 发弹量 +1
				osDelay(t); // 按热量限制算出的节拍延时（ms）
			}
		}
		else
		{
			shooter.workState=IDLE; // 条件不满足 → 取消本次拨弹
		}
	break;

	case TRIGGER_REVERSE: // 堵转脱困：先反向回退把卡住的弹退出，再小幅正转复位
					//嘀嘀嘀
		Beep_PlayNotes((Note[]){{T_M1,D_Sixteenth},{T_M1,D_Sixteenth},{T_M1,D_Sixteenth}},3); // 蜂鸣器响 3 声短音，提示堵转
		shooter.triggerMotor.targetAngle -= MOTOR_M3508_DGR2CODE(360*1.5/9.0*2); //电机反向拨动1.5/9圈（退回被卡的弹丸）
		osDelay(500);
		shooter.triggerMotor.targetAngle += MOTOR_M3508_DGR2CODE(360*0.5/9.0*2); //正转0.5/9圈（回到正常位置的余量）
		shooter.workState=IDLE; // 脱困完成，回到空闲
	break;

	default: // 其余状态（含预留的 TRIGGER_DOUBLE）不做处理
	break;
	}
}

// 发射机构 FreeRTOS 任务入口（调度器启动后由框架调用一次）
// 参数：argument —— 任务创建参数（本任务未使用，可为 NULL）。
// 返回值：无（内部为 for(;;) 死循环，永不返回）。
void OS_ShooterCallback(void const * argument)
{
    osDelay(100); // 上电后等待 100ms，待电机/裁判系统初始化稳定
    Shooter_Init(); // 初始化发射系统（摩擦轮转速、各电机 PID、状态）
    PID_Clear(&shooter.triggerMotor.anglePID.inner); // 清空拨弹电机角度环内层积分，避免残留量冲击
    PID_Clear(&shooter.triggerMotor.anglePID.outer); // 清空拨弹电机角度环外层积分
    osDelay(100); // 再等 100ms 让 PID 状态稳定

    for(;;) // 周期任务主循环
	{
	    if(true /* JUDGE_GetShooterOutputState()||1 */) // 目前恒为真（原打算由裁判系统输出状态控制），始终执行回调
		{
		    Task_Shooter_Callback(); // 执行一次发射控制
		}
	    osDelay(10); // 10ms 周期（100Hz）
	}
}


