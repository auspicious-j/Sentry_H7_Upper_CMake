/******************************************************************************
 * @file    Gimbal.c
 * @brief   云台（Gimbal）控制 —— 应用层实现。
 *          云台三轴：大 yaw（底座水平旋转，DM 电机）、小 yaw（顶部水平微调，
 *          DJI 电机）、pitch（俯仰摆动，DM 电机）。
 *          本文件负责：云台初始化与 PID 参数、目标角度解算（遥控器/视觉/扫描）、
 *          角度环绕（±180°）与限幅处理，以及三轴的 PID 输出计算。
 *          约定：角度单位统一「度」，角速度单位「度/秒」，加给电机前再做单位换算。
 ******************************************************************************/

#include "Gimbal.h"
#include "imu_temp_ctrl.h"
#include "USER_RC.h"
#include "Vision.h"
#include "Chassis.h"
#include "Shooter.h"
#include "arm_math.h"
#include "degree.h"

// 角度->弧度换算系数 π/180≈0.01745329，用于把「度」转成 arm_math 需要的弧度。
#define DEG_TO_RAD 0.01745329f  // PI / 180

// 小 yaw 相对大 yaw 的最大偏转量（度）：小 yaw 只能在大 yaw 目标角 ±45° 内微调。
#define TOP_YAW_LIMIT 45.0f

// 大 yaw 转动惯量估计值（单位 kg·m^2），供前馈/力矩估算使用（当前实现未直接用到）。
#define BASE_YAW_J 0.04575f

Gimbal_t gimbal; // 云台全局状态实例（结构体定义见 Gimbal.h）

// （历史遗留变量：visionFindAver，曾用于视觉数据平均滤波，当前未启用）
// float visionFindAver;

// 内部函数：初始化各轴 PID 参数（仅本文件内部使用，故未在 Gimbal.h 声明）
void Gimbal_InitPID(void);

/********************初始化************************/

/**
 * @brief  云台初始化：设定 pitch 上下限幅、把各轴开机角度与目标角度归零，并初始化 PID。
 * @param  无
 * @return 无
 * @note   各轴开机角度均取 0，即把「上电瞬间的 IMU 姿态」当作该轴的角度零点。
 */
void Gimbal_Init()
{
    gimbal.pitch.pitchMax = 34; // pitch 目标角上限（度），限制抬升幅度，防止打到机械限位
    gimbal.pitch.pitchMin = -20; // pitch 目标角下限（度），限制下压幅度

    gimbal.pitch.initAngle = 0; // 陀螺仪pitch开机角度 *0表示pitch水平
    gimbal.pitch.targetAngle = gimbal.pitch.initAngle; // pitch 目标角先对齐开机角（开机静止）

    gimbal.base_yaw.initAngle = 0; // 陀螺仪yaw开机角度   *0表示yaw不动
    gimbal.base_yaw.targetAngle = gimbal.base_yaw.initAngle; // 大 yaw 目标角对齐开机角

    gimbal.top_yaw.initAngle = 0; // 小 yaw 开机角度取 0
    gimbal.top_yaw.targetAngle = gimbal.top_yaw.initAngle; // 小 yaw 目标角对齐开机角

    Gimbal_InitPID();         // 初始化PID参数
}

/**
 * @brief  初始化云台各轴的 PID 参数（含串级 PID 的内环、外环与微分先行环）。
 * @param  无
 * @return 无
 * @note   参数格式：
 *         PID_Init(pid,p,i,d,maxSum,maxOut) —— kp、ki、kd、积分限幅、输出限幅；
 *         DEPID_Init(pid,p,i,d,maxI,maxOut,gama) —— kp、ki、kd、积分限幅、输出限幅、微分先行滤波系数。
 *         行尾注释为历史/备选调参值，便于回溯。
 */
void Gimbal_InitPID()
{
    /*pitch由陀螺仪控制*/
    PID_Init(&gimbal.pitch.imuPID.inner, 6, 0, 0.4, 500, 7000); // pitch 内环(角速度环): kp=6,ki=0,kd=0.4,积分限幅=500,输出限幅=7000
    DEPID_Init(&gimbal.pitch.imuPID.deOuter, 75, 0.7, 120, 200, 7000, 0.6);//45, 0.7, 50  // pitch 外环(角度环,微分先行): kp=75,ki=0.7,kd=120,积分限幅=200,输出限幅=7000,gama=0.6
    PID_Init(&gimbal.pitch.imuPID.outer,-0.8,-0.001,-8,1.3,10); // pitch 外环 PID(备选): kp=-0.8,ki=-0.001,kd=-8,积分限幅=1.3,输出限幅=10
    PID_Init(&gimbal.pitch.MIT_PID,100,0,2,0,0); // pitch 达妙 MIT 位置环 PID: kp=100,ki=0,kd=2,积分限幅=0,输出限幅=0(不限)

    PID_Init(&gimbal.base_yaw.imuPID.inner,5.8,0.02,3.5,1000,7000); // 大 yaw 内环: kp=5.8,ki=0.02,kd=3.5,积分限幅=1000,输出限幅=7000
    DEPID_Init(&gimbal.base_yaw.imuPID.deOuter,37,0.03,173,700,2000,0.45); // 大 yaw 外环(微分先行): kp=37,ki=0.03,kd=173,积分限幅=700,输出限幅=2000,gama=0.45
    PID_Init(&gimbal.base_yaw.imuPID.outer,300,0.5,5000,1000,15000); // 大 yaw 外环 PID: kp=300,ki=0.5,kd=5000,积分限幅=1000,输出限幅=15000

    PID_Init(&gimbal.top_yaw.imuPID.inner, 50, 0.1, 10, 7000, 30000);//270 0.3  300  // 小 yaw 内环: kp=50,ki=0.1,kd=10,积分限幅=7000,输出限幅=30000
    DEPID_Init(&gimbal.top_yaw.imuPID.deOuter, 150, 0.8, 300, 100, 30000, 0.7); // 小 yaw 外环(微分先行): kp=150,ki=0.8,kd=300,积分限幅=100,输出限幅=30000,gama=0.7
//
}

/**
 * @brief  汇总 IMU 与电机反馈，更新三轴当前角度/角速度，并把目标角换算到连续角。
 * @param  无
 * @return 无
 * @note   核心作用：
 *         1) 从 IMU 取 yaw/pitch 角与角速度；
 *         2) 用「小 yaw 电机相对大 yaw 的机械夹角」反推大 yaw 的真实 IMU 角度；
 *         3) 用 ModularDegreeTowards 把角度展开成「跨圈连续角」，避免 ±180° 突发跳变。
 */
void Gimbal_UpdataAngle()
{
    // region 传感器数据传递
    gimbal.top_yaw.gyro = INS.gyro[2]; // 小 yaw 角速度 = IMU 绕 Z 轴(yaw)角速度，单位度/秒
    gimbal.top_yaw.angle = INS.yaw; // 小 yaw 当前角 = IMU 航向角（顶部直接由 IMU 测得），单位度
    gimbal.base_yaw.angle = gimbal.top_yaw.angle - (gimbal.top_yawMotor.angle - TOP_YAW_OFFSET) / 8192.0 * 360.0; //得出大yaw的imu角度
    // 说明：大 yaw 无独立 IMU，用「小 yaw 的 IMU 角」减去「小 yaw 电机相对大 yaw 的机械夹角」反推得到。
    //       机械夹角 = (电机原始计数 - 零位偏置) / 8192 * 360，其中 8192 为编码器一圈计数。
    // gimbal.base_yaw.gyro = -gimbal.base_yawMotor.para.vel;   //加负号因为电机倒置 暂时不用 上下板通信不够用
    gimbal.pitch.gyro = -INS.gyro[0]; // pitch 角速度 = -IMU 绕 X 轴角速度（取负因 IMU 安装方向），单位度/秒
    gimbal.pitch.angle = INS.pitch; // pitch 当前角 = IMU 俯仰角，单位度

    gimbal.top_yaw.totalAngle += ModularDegreeTowards(gimbal.top_yaw.angle - gimbal.top_yaw.lastAngle, 0.0f); // 累计角：把本拍角度增量(取最接近值)累加，得到跨圈连续的 totalAngle
    gimbal.top_yaw.targetAngle = ModularDegreeTowards(gimbal.top_yaw.targetAngle, gimbal.top_yaw.totalAngle); // 把目标角折算到与 totalAngle 同一圈内，避免目标绕圈

    // endregion
    // region yaw相关角度计算
    // target超出total过半圈时修正
    gimbal.top_yaw.lastAngle = gimbal.top_yaw.angle; // 记录本拍角度，供下一拍求增量
    gimbal.base_yaw.totalAngle = gimbal.top_yaw.totalAngle - (gimbal.top_yawMotor.angle - TOP_YAW_OFFSET) / 8192.0 * 360.0; // 大 yaw 累计角 = 小 yaw 累计角 - 小 yaw 电机机械夹角

    if (chassis.pattern == Chassis_AI && rcInfo.left == 2) // 加弹
    {
        gimbal.base_yaw.targetAngle += 45; // AI 模式下拨到加弹档：大 yaw 目标角整体偏转 45°，让出加弹位
    }
}

/* ==================== 自动扫描（无目标时来回搜索）参数 ==================== */

float yaw_angle = 0.0f;      // 当前扫描角（度）（预留给统计用，当前扫描以 targetAngle 累加实现）
float yaw_speed = 60.0f;     // 扫描速度（度/秒），yaw 每拍累加角 = 速度 * dt

// 扫描参数配置
float pitch_phase = 0.0f;       // pitch 正弦扫描的相位（弧度），每拍递增 2π*f*dt 并环绕到 [0,2π)
float pitch_freq = 1.0f;        // 扫描频率 Hz，pitch 正弦摆动的频率
float pitch_amp = 10.0f;        // 振幅 (5 - (-15)) / 2 = 10
float pitch_offset = -5.0f;     // 中心角 (-15 + 5) / 2 = -5

/**
 * @brief  自动扫描更新：无目标时让云台按固定轨迹缓慢搜索。
 *         大/小 yaw 以恒定角速度单向扫描，pitch 做正弦上下摆动。
 * @param  无
 * @return 无
 */
void Gimbal_Scan_Update(void)
{
    static float last_ideal_pitch = 0.0f; // 上一拍的理想 pitch 角（度），用于求摆动增量
    static uint8_t scan_init_flag = 1; // 首次进入扫描模式的标志位

    // 纯累加，dt = 1ms
    float yaw_delta = yaw_speed * 0.001f; // 本拍 yaw 增加角（度）= 速度(度/秒) * 0.001s
    gimbal.top_yaw.targetAngle += yaw_delta; // 小 yaw 目标角持续累加，实现匀速扫描
    gimbal.base_yaw.targetAngle = gimbal.top_yaw.targetAngle; // 大 yaw 目标角跟随小 yaw

    pitch_phase += 2.0f * PI * pitch_freq * 0.001f; // 相位按 2π*f*dt 递增（dt=1ms）
    if (pitch_phase > 2.0f * PI) {
        pitch_phase -= 2.0f * PI; // 相位环绕到 [0,2π)，防止数值无限增大
    }

    float current_ideal_pitch = pitch_amp * arm_sin_f32(pitch_phase) + pitch_offset; // 理想 pitch 角 = 振幅*sin(相位) + 中心偏移（度）

    if (scan_init_flag) {
        last_ideal_pitch = current_ideal_pitch; // 首次进入时初始化，避免第一拍产生巨大增量
        scan_init_flag = 0;
    }
    float pitch_wave_delta = current_ideal_pitch - last_ideal_pitch; // 本拍理想正弦角增量

    float error = current_ideal_pitch - gimbal.pitch.targetAngle; // 理想正弦角 与 当前目标角 的偏差
    float convergence_step = error * 0.02f; // 收敛步长：按比例把目标角拉向理想轨迹

    gimbal.pitch.targetAngle += pitch_wave_delta + convergence_step; // 叠加正弦增量与收敛量，得到平滑的 pitch 扫描目标

    last_ideal_pitch = current_ideal_pitch; // 记录本拍理想角，供下一拍求增量
}

/**
 * @brief  视觉（自瞄）控制：按视觉下发的目标角度设定三轴目标角。
 * @param  无
 * @return 无
 * @note   大 yaw 直接瞄准；小 yaw 只在大 yaw 目标角 ±TOP_YAW_LIMIT 内跟随，
 *         超出的部分留给大 yaw 转；pitch 目标角做上下限幅。
 */
void Gimbal_VisionCtrl()
{
    gimbal.base_yaw.targetAngle = ModularDegreeTowards(vision_target.base_yaw, gimbal.base_yaw.targetAngle);// 大yaw直接瞄就行

    float top_target_unwrap = gimbal.base_yaw.targetAngle + (vision_target.yaw - vision_target.base_yaw); // 小 yaw 的绝对目标角 = 大 yaw 目标角 + 视觉给出的大小 yaw 偏置

    // 视觉控制下的大小yaw和pitch的目标角度
    float delta = top_target_unwrap - gimbal.base_yaw.totalAngle; // 计算相对误差 //计算相对误差
    LIMIT(delta, -TOP_YAW_LIMIT, TOP_YAW_LIMIT); // 把偏差限制在 ±45°，超出部分交由大 yaw 承担

    gimbal.top_yaw.targetAngle = gimbal.base_yaw.totalAngle + delta; // 小 yaw 目标角 = 大 yaw 累计角 + 限幅后偏差（小 yaw 跟随大 yaw）

    gimbal.pitch.targetAngle = vision_target.pitch; // pitch 目标角 = 视觉下发角（度）
    LIMIT(gimbal.pitch.targetAngle, gimbal.pitch.pitchMin, gimbal.pitch.pitchMax); // 俯仰限幅，保护机械
}

/**
 * @brief  摇杆手动控制：用遥控器摇杆增量式微调 yaw 与 pitch 目标角。
 * @param  无
 * @return 无
 * @note   采用「目标角累加」的增量控制：摇杆回中即停止，云台停在当前角度。
 */
void Gimbal_RockerCtrl()
{
    gimbal.top_yaw.targetAngle -= rcInfo.ch1 * 0.3 / 660.0f;    // yaw：右摇杆水平(ch1) -> 小 yaw 目标角增量
    gimbal.base_yaw.targetAngle = gimbal.top_yaw.targetAngle; // 大 yaw 目标角跟随小 yaw
    gimbal.pitch.targetAngle += rcInfo.ch2 * 0.35 / 660.0f; // 旋转云台pitch：右摇杆竖直(ch2) -> pitch 目标角增量
    LIMIT(gimbal.pitch.targetAngle, gimbal.pitch.pitchMin, gimbal.pitch.pitchMax); // 俯仰限幅
}

/**
 * @brief  判断是否强制进入视觉控制（由遥控器拨轮位置决定）。
 * @param  无
 * @return true=拨轮拨到下方（wheel<-400），强制视觉；false=不强制
 */
bool Gimbal_VisionForced()
{
    return rcInfo.wheel < -400; // 遥控器拨轮(wheel)低于 -400 视为强制视觉
}

/**
 * @brief  发射机构手动控制：处理摩擦轮开关与拨弹状态。
 * @param  无
 * @return 无
 * @note   拨轮(wheel)上拨越过阈值一次 -> 翻转摩擦轮开/关；
 *         左侧三档开关打到上档 -> 连续拨弹（非强制视觉时才响应）。
 */
void Shooter_RockerCtrl()
{
    static int lastwheel = 0; // 上一拍拨轮值，用于检测拨轮「上拨越阈」的边沿

    if (rcInfo.wheel > 400 && lastwheel <= 400) // 拨轮从 <=400 上拨到 >400 的上升沿
    {
        shooter.fricOpenFlag = !shooter.fricOpenFlag; // 翻转摩擦轮开/关标志
    }

    if (!Gimbal_VisionForced()) // 非强制视觉时才允许手动控制拨弹
    {
        if (rcInfo.left == 1) // 左侧三档开关打到上档
        {
            shooter.workState = TRIGGER_CONTINUE; // 连续拨弹
        }
        else
        {
            shooter.workState = IDLE; // 空闲，不拨弹
        }
    }

	lastwheel = rcInfo.wheel; // 记录本拍拨轮值（保持原有缩进为一个制表符）
//    LIMIT(p_target,MOTOR_ZERO_POS + MOTOR_ZERO_POS - gimbal.pitch.pitchMax * DEG_TO_RAD,MOTOR_ZERO_POS + gimbal.pitch.pitchMin);
} //MOTOR_ZERO_POS

/**
 * @brief  云台主控制回调（每 1ms 调用一次）：解算目标角、更新角度、计算三轴 PID 输出。
 * @param  无
 * @return 无
 * @note   流程：按底盘模式选择目标角来源（视觉/摇杆/扫描）-> 更新角度 -> 逐轴算 PID 输出。
 */
void Task_Gimbal_Callback()
{
    // visionFindAver=Filter_AverCalc(&gimbal.visionFilter.find,vision.tracking);
    // region 非AI控制
    if (chassis.pattern == Chassis_control) // 手动模式
    {
        if (Gimbal_VisionForced() && vision_target.vaild) // 强制视觉且目标有效
        {
            Gimbal_VisionCtrl(); // 视觉自瞄
        }
        else
        {
            Gimbal_RockerCtrl(); // 否则摇杆手动
        }

        Shooter_RockerCtrl(); // 处理发射机构
    }
    else if (chassis.pattern == Chassis_AI) // AI 模式
    {
        if (rcInfo.left == 1) // 左侧三档开关上档：进入自瞄/扫描
        {
            if (vision_target.vaild) // 有目标
            {
                shooter.fricOpenFlag = true; // 开摩擦轮
                Gimbal_VisionCtrl(); // 视觉自瞄
            }
            else
            {
                shooter.fricOpenFlag = false; // 关摩擦轮
                Gimbal_Scan_Update(); // 无目标则自动扫描
            }
        }
        else
        {
            Gimbal_RockerCtrl(); // 非上档 -> 摇杆手动
        }
    }

    if (chassis.rotate.mode == ChassisMode_Spin) // 小陀螺模式
    {
        gimbal.base_yaw.imuPID.outer.maxIntegral = 6000; // 放大积分限幅，配合持续旋转不掉积分
        gimbal.base_yaw.imuPID.outer.ki = 0.5; // 提高积分系数，增强跟随
    }
    else
    {
        gimbal.base_yaw.imuPID.outer.maxIntegral = 1000; // 底盘跟随模式：常规积分限幅
        gimbal.base_yaw.imuPID.outer.ki = 0.1; // 常规积分系数
    }

    // region AI控制
    // 是否开扫描
    //Gimbal_VisionCtrl();
    Gimbal_UpdataAngle(); // 更新三轴当前角度/角速度与连续角

    //计算小yaw电机输出
    DEPID_CascadeCalc(&gimbal.top_yaw.imuPID,gimbal.top_yaw.targetAngle,gimbal.top_yaw.totalAngle,gimbal.top_yaw.gyro); // 小 yaw 串级 PID：角度环(目标/反馈) + 角速度环
    gimbal.top_yaw.imuPID.output = gimbal.top_yaw.imuPID.output; // 输出直接透传（预留便于后续做缩放/取反）

    // 计算大yaw电机输出
    // DEPID_CascadeCalc(&gimbal.base_yaw.imuPID, gimbal.base_yaw.targetAngle, gimbal.base_yaw.totalAngle, gimbal.base_yaw.gyro);
    // gimbal.base_yaw.imuPID.output = -gimbal.base_yaw.imuPID.output/1000.0f - 2.0f * (gimbal.top_yaw.imuPID.output / 30000.0f) - forwardfeed(gimbal.base_yaw.imuPID.outer.output / 1000.0f);//因为电机倒置 所以输出反向 输出除一千让PID参数乘1000方便调参 再加入前馈
    PID_SingleCalc(&gimbal.base_yaw.imuPID.outer,gimbal.base_yaw.targetAngle,gimbal.base_yaw.totalAngle); // 大 yaw 只用外环 PID(角度环) 计算
    gimbal.base_yaw.imuPID.outer.output = -gimbal.base_yaw.imuPID.outer.output / 1000.0f; // 取负(电机倒置)并除 1000(让 PID 参数可乘 1000 便于调参)

    // 计算pitch电机输出
//        DEPID_CascadeCalc(&gimbal.pitch.imuPID, gimbal.pitch.targetAngle, gimbal.pitch.angle, gimbal.pitch.gyro);
    //		DEPID_CascadeCalc(&gimbal.pitch.imuPID, gimbal.pitch.targetAngle, gimbal.pitch.angle, gimbal.pitch.gyro);
    PID_SingleCalc(&gimbal.pitch.imuPID.outer,gimbal.pitch.targetAngle,gimbal.pitch.angle); // pitch 只用外环 PID(角度环) 计算
    gimbal.pitch.imuPID.output = - gimbal.pitch.imuPID.output/1000.0f  - PITCH_MASS * MASS_G * PITCH_R * arm_cos_f32(gimbal.pitch.angle * PI / 180.0f); ////输出除一千让PID参数乘1000方便调参
    // 说明：再叠加重力补偿力矩 -m*g*R*cos(pitch)，抵消俯仰时重力造成的负载（输出除 1000 同大 yaw）。
//		Gimbal_Follow_IMU();
} //-MASS * G * R * arm_cos_f32(gimbal.pitch.angle * PI / 180.0f)

/*************云台操控任务******************/

/**
 * @brief  云台任务入口（FreeRTOS 任务体）：上电延时后初始化云台，此后每 1ms 执行一次控制回调。
 * @param  argument 任务参数（未使用）
 * @return 无（本函数永不返回）
 */
void OS_GimbalCallback(void const *argument)
{
    osDelay(1500); // 上电等待 1.5s，让 IMU 与外设稳定后再标定/初始化
    Gimbal_Init(); // 初始化云台角度与 PID
    for (;;)
    {
        Task_Gimbal_Callback(); // 执行一次云台控制
        osDelay(1); // 控制周期 1ms
    }
}

