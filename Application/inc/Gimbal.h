#ifndef _GIMBAL_H_
#define _GIMBAL_H_

/******************************************************************************
 * @file    Gimbal.h
 * @brief   云台（Gimbal）控制模块 —— 头文件。
 *          云台指承载摄像头/发射机构、可绕轴转动的支架。本工程云台有三个
 *          自由度（三根轴）：大 yaw（底座水平旋转，DM 电机）、小 yaw（顶部
 *          水平微调，DJI 电机）、pitch（俯仰上下摆动，DM 电机）。
 *          本文件声明各轴标定常量、云台状态结构体 Gimbal_t 及对外接口。
 ******************************************************************************/

#include "USER_Moto.h"
#include "stdbool.h"
#include "slope.h"
#include "PID.h"

/* ======================= 标定常量（每台车需实测校准） ======================= */

// 小 yaw（顶部水平轴）电机机械零位偏置：小 yaw 正对正前方时读到的编码器原始计数。
// 单位=编码器计数，满量程 0~8191 对应 0~360°。
// 作用：把电机原始角度换算成真实角度，公式见 Gimbal.c：(angle - OFFSET)/8192*360。
#define TOP_YAW_OFFSET 2710  //此处校准小yaw

// 大 yaw（底座水平轴）开机目标角度：单位=度。开机时相对 IMU 零点的默认朝向偏置（-85° 即开机向左偏 85°）。
#define INIT_YAW_ANGLE -85.0f   //此处校准大yaw

// pitch（俯仰轴）电机机械零位对应角度：单位=弧度，2.094 rad = 120° = 2π/3，用于 pitch 电机位置换算。
#define PITCH_MOTOR_ZERO_POS 2.094f //此处校准pitch

/* ======================= pitch 重力补偿物理参数 ======================= */

// pitch 轴方向系数，取 +1 或 -1，用于适配电机实际安装方向（此处 -1 表示反向安装）。
#define PITCH_DIRECTION -1

// 重力加速度，单位 m/s^2，用于计算 pitch 重力补偿力矩。
#define MASS_G 9.81f

// pitch 总质量，单位 kg，用于计算 pitch 重力补偿力矩。
#define PITCH_MASS 1.3f

// pitch 质心到转轴的距离（力臂），单位 m，用于计算 pitch 重力补偿力矩。
#define PITCH_R 0.07287f   // pitch中心及其距离

/**
 * @brief 云台全局状态结构体：汇总三个轴（base_yaw 大 yaw、top_yaw 小 yaw、pitch 俯仰）
 *        的角度/角速度/PID，以及三个驱动电机对象。角度单位统一为「度」，角速度单位「度/秒」。
 */
typedef struct {
  // base_yaw：大 yaw 轴（底座水平旋转，由 DM 电机驱动）状态
  struct {
    float initAngle;                                // 大 yaw 开机标定角度（度）
    float angle, lastAngle, totalAngle, totalRound; // angle 当前角(度)；lastAngle 上次角(度)；totalAngle 跨圈累计角(度，用于消除±180°突跳)；totalRound 累计圈数
    float gyro;                                     // 大 yaw 角速度（度/秒），来自 IMU
    float targetAngle, lastTargetAngle;             // targetAngle 目标角(度)；lastTargetAngle 上次目标角(度)
    CascadePID imuPID; // 大 yaw 串级 PID（外环角度 + 内环角速度），陀螺仪闭环
  } base_yaw;

  // top_yaw：小 yaw 轴（顶部水平微调，由 DJI 电机驱动）状态
  struct {
    float initAngle;                                // 小 yaw 开机标定角度（度）
    float angle, lastAngle, totalAngle, totalRound; // angle 当前角(度)；lastAngle 上次角(度)；totalAngle 跨圈累计角(度)；totalRound 累计圈数
    float gyro;                                     // 小 yaw 角速度（度/秒），来自 IMU
    float targetAngle, lastTargetAngle;             // targetAngle 目标角(度)；lastTargetAngle 上次目标角(度)
    CascadePID imuPID; // 小 yaw 串级 PID（外环角度 + 内环角速度）
  } top_yaw;

  // pitch：俯仰轴（上下摆动，由 DM 电机驱动）状态
  struct
  {
    float initAngle;                            // pitch 开机标定角度（度）
    float angle, lastAngle; // angle 当前俯仰角(度)；lastAngle 上次俯仰角(度)
    float gyro;                                 // pitch 角速度（度/秒），来自 IMU（取 -INS.gyro[0]）
    float targetAngle, lastTargetAngle;         // targetAngle 目标俯仰角(度)；lastTargetAngle 上次目标俯仰角(度)
    float pitchMax, pitchMin; // 俯仰角目标限幅（度）：上限 pitchMax、下限 pitchMin，防止打到机械限位
    PID MIT_PID;                                // 达妙电机 MIT 位置环 PID
    CascadePID imuPID;        // pitch 串级 PID（外环角度 + 内环角速度），陀螺仪闭环
  } pitch;

  DM_motor_t base_yawMotor; // 大 yaw 达妙(DM)电机对象（含反馈与参数）
  DJI_Motor_t top_yawMotor; // 小 yaw 大疆(DJI)电机对象
  DM_motor_t pitchMotor; // pitch 达妙(DM)电机对象
} Gimbal_t;

extern Gimbal_t gimbal; // 云台全局实例，所有云台状态与参数集中于此

/**
 * @brief  初始化云台：设置 pitch 上下限幅、各轴开机角度与目标角度，并初始化各 PID。
 * @param  无
 * @return 无
 */
void Gimbal_Init(void);

/**
 * @brief  判断是否强制进入视觉（自瞄）控制，由遥控器拨轮位置决定。
 * @param  无
 * @return true=强制视觉控制；false=跟随遥控器摇杆手动控制
 */
bool Gimbal_VisionForced(void);

#endif
