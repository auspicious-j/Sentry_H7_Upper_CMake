/**
 * @file    Chassis.h
 * @brief   哨兵四舵轮底盘控制模块：对外接口与数据结构定义
 * @details 底盘由四个“舵轮”构成（每个舵轮 = 1个驱动电机 + 1个转向电机，共8个电机）。
 *          运动学解算：把底盘平移速度(vx,vy)与自转角速度(vw)分解到四个舵轮的合速度矢量，
 *          再解算出每个舵轮的“驱动轮目标转速(RPM) + 转向目标角度(°)”下发电机；
 *          运动控制：底盘跟随云台（用相对角PID回正）或小陀螺持续自转，速度经斜坡发生器限加速平滑；
 *          速度上限：由电机最大转速反推平移/旋转最大速度（换算见 Chassis.c 的 Chassis_UpdateSlope）。
 *          术语：舵轮 = 既能驱动又能独立转向的轮组；小陀螺 = 底盘边自转边移动的机动方式。
 *          实现见 Chassis.c，外部通过 Chassis_Init / OS_ChassisCallback 等接口驱动。
 */
#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "main.h"
#include "pid.h"
#include "Slope.h"
#include "USER_Moto.h"

// 驱动轮最大电机转速(RPM)；用于反推底盘最大平移/旋转速度（换算见 Chassis.c 的 Chassis_UpdateSlope）
#define WHEELSPEED_MAX 8000

typedef enum
{
	ChassisMode_Follow,	  // 底盘跟随云台：底盘朝向自动跟随云台，使云台-底盘相对角回到0°
	ChassisMode_Spin,	  // 小陀螺：底盘持续自转，云台独立瞄准
} ChassisMode;

typedef enum{
    Chassis_control, // 手动遥控：速度与模式由遥控器(rcInfo)给定
    Chassis_AI // AI自动：速度与模式由视觉上位机(Vision)下发
}chassis_pattern;

typedef struct _Chassis
{
	// 底盘尺寸信息（几何参数，用于运动学解算；长度单位 mm）
	struct Info
	{
		float wheelbase;   // 轴距：前后轮中心距(mm)
		float wheeltrack;  // 轮距：左右轮中心距(mm)
		float wheelRadius; // 轮半径(mm)，用于线速度与驱动轮转速互换
		float offsetX;	   // 重心相对几何中心的x方向偏移(mm)，用于自转线速度分量补偿
		float offsetY;      // 重心相对几何中心的y方向偏移(mm)，用于自转线速度分量补偿
	} info;
	// 四组舵轮电机：[0]左前 [1]右前 [2]左后 [3]右后；每组含1个驱动电机+1个转向电机(共8个电机)
	Double_motor_t motors[4]; // 四个舵轮电机组，索引见上方说明
	
	// 底盘移动信息（平移/自转速度及其上限）
	struct Move
	{
		float vx; // 当前x(左右)平移速度(mm/s)，已由云台系旋转变换到底盘系
		float vy; // 当前y(前后)平移速度(mm/s)，已由云台系旋转变换到底盘系
		float vw; // 当前自转角速度(rad/s)，逆时针为正

		float maxVx, maxVy, maxVw; // 三轴速度上限：maxVx/maxVy(mm/s)、maxVw(rad/s)，由 Chassis_UpdateSlope 刷新

		float Wheelangle[4]; // （预留）四轮角度缓存数组(°)，当前实现未使用
		Slope xSlope, ySlope, outputSlope, chargeSlope, spinSlope; // 速度斜坡发生器：对目标速度限加速平滑；xSlope/ySlope 用于平移、spinSlope 用于自转，output/charge 为功率相关预留
	} move;
	
	// 旋转相关信息
	struct
	{
		PID pid;				// 旋转PID：输入云台-底盘相对角偏差(°)，输出底盘旋转速度(rad/s)
		float relativeAngle;	// 云台与底盘的相对偏角(°)，跟随模式下回正目标为0°
    	float align_yaw; 		// 视觉下发的对齐目标角(°)：底盘与起伏路段/坡道方向对齐时使用
		float InitAngle;		// 云台与底盘机械对齐时的yaw编码器角度(°)，作为相对角解算基准 
		int16_t InitpitchAngle; // 云台处于水平时的pitch编码器原始值
		float nowAngle;			// 当前云台yaw角度(°)，由编码器换算得到
		ChassisMode mode;		// 当前底盘旋转模式（跟随/小陀螺）
	} rotate;
	chassis_pattern pattern; // 当前控制来源（手动遥控/AI自动）
} Chassis_t;

extern Chassis_t chassis; // 底盘全局状态实例（全局唯一）
extern float vx,vy,vw; // 视觉(AI)下发的底盘速度指令：vx左右(mm/s)、vy前后(mm/s)、vw自转(rad/s)

void Chassis_InitPID(void); // 初始化底盘旋转PID（Kp/Ki/Kd 与死区），由 Chassis_Init 调用

#endif
