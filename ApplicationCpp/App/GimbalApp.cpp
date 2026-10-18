extern "C"
{
#include "Gimbal.h"
#include "Gimbal_GraphApi.h"
#include "cmsis_os.h"
#include "imu_temp_ctrl.h"
#include "USER_RC.h"
#include "Vision.h"
#include "Chassis.h"
#include "Shooter.h"
#include "degree.h"
#include "Judge.h"
#include "PID.h"
#include "arm_math.h"

#include <math.h>
}

#define DEG_TO_RAD 0.01745329f  // PI / 180
#define TOP_YAW_LIMIT 45.0f
#define BASE_YAW_J 0.04575f

Gimbal_t gimbal;

// region Initialize
static void Gimbal_InitPID(void)
{
    /*pitch由陀螺仪控制*/
    PID_Init(&gimbal.pitch.imuPID.inner, 6, 0, 0.4, 500, 7000);
    DEPID_Init(&gimbal.pitch.imuPID.deOuter, 75, 0.7, 120, 200, 7000, 0.6);//45, 0.7, 50
    PID_Init(&gimbal.pitch.imuPID.outer,-0.8,-0.001,-8,1.3,10);
    PID_Init(&gimbal.pitch.MIT_PID,100,0,2,0,0);
  
    PID_Init(&gimbal.base_yaw.imuPID.inner,5.8,0.02,3.5,1000,7000);
    DEPID_Init(&gimbal.base_yaw.imuPID.deOuter,37,0.03,173,700,2000,0.45);
    PID_Init(&gimbal.base_yaw.imuPID.outer,300,0.5,5000,1000,15000);
    
    PID_Init(&gimbal.top_yaw.imuPID.inner, 50, 0.1, 10, 7000, 30000);//270 0.3  300
    DEPID_Init(&gimbal.top_yaw.imuPID.deOuter, 150, 0.8, 300, 100, 30000, 0.7);
}

// 初始化云台
void Gimbal_Init(void)
{
    gimbal.pitch.pitchMax = 34; // 设定pitch角度限幅
    gimbal.pitch.pitchMin = -20;

    gimbal.pitch.initAngle = 0; // 陀螺仪pitch开机角度 *0表示pitch水平
    gimbal.pitch.targetAngle = gimbal.pitch.initAngle;

    gimbal.base_yaw.initAngle = 0; // 陀螺仪yaw开机角度   *0表示yaw不动
    gimbal.base_yaw.targetAngle = gimbal.base_yaw.initAngle;
    
    gimbal.top_yaw.initAngle = 0;
    gimbal.top_yaw.targetAngle = gimbal.top_yaw.initAngle;

    Gimbal_InitPID();         // 初始化PID参数
}
// endregion

// 数据更新和加弹动作组
static void Gimbal_UpdataAngle(void)
{
    gimbal.top_yaw.gyro = INS.gyro[2];
    gimbal.top_yaw.angle = INS.yaw;
    gimbal.base_yaw.angle = gimbal.top_yaw.angle - (gimbal.top_yawMotor.angle - TOP_YAW_OFFSET) / 8192.0 * 360.0; //得出大yaw的imu角度
    // gimbal.base_yaw.gyro = -gimbal.base_yawMotor.para.vel;   //加负号因为电机倒置 暂时不用 上下板通信不够用 
    gimbal.pitch.gyro = -INS.gyro[0];
    gimbal.pitch.angle = INS.pitch;

    gimbal.top_yaw.totalAngle += ModularDegreeTowards(gimbal.top_yaw.angle - gimbal.top_yaw.lastAngle, 0.0f);
    gimbal.top_yaw.targetAngle = ModularDegreeTowards(gimbal.top_yaw.targetAngle, gimbal.top_yaw.totalAngle);
    
    gimbal.top_yaw.lastAngle = gimbal.top_yaw.angle;
    gimbal.base_yaw.totalAngle = gimbal.top_yaw.totalAngle - (gimbal.top_yawMotor.angle - TOP_YAW_OFFSET) / 8192.0 * 360.0;

    if (chassis.pattern == Chassis_AI && rcInfo.left == 2) // 加弹
    {
        gimbal.base_yaw.targetAngle += 45;
    }
}

// 读遥控器开启视觉控制
static bool Gimbal_VisionForced(void)
{
    return rcInfo.wheel < -400;
}

// 读遥控器开启攻击
static bool Gimbal_AttackAllowed(void)
{
    return rcInfo.left == 1;
}

// 扫描控制
static void Gimbal_Scan(void)
{
    static const float yaw_v = 60.0f;     // 扫描速度（度/秒）

    // 扫描参数配置
    static const float pitch_w = 2.0f * PI;      // 频率 Hz
    static const float pitch_min = -15.0f;
    static const float pitch_max = 5.0f;
    static const float pitch_amp = (pitch_max - pitch_min) / 2;        // 振幅 (5 - (-15)) / 2 = 10

    static float pitch_phase = 0.0f;       // 相位

    switch (USER_SentryCmd.armor_mode)
    {
    case VISION_AI_ARMOR:
        // 纯累加，dt = 1ms
        gimbal.base_yaw.targetAngle = gimbal.top_yaw.targetAngle += yaw_v * 0.001f;

        pitch_phase += pitch_w * 0.001f;

        if (pitch_phase > 2.0f * PI)
        {
            pitch_phase -= 2.0f * PI;
        }
        
        gimbal.pitch.targetAngle += pitch_amp * pitch_w * cos(pitch_phase) * 0.001f;
        LIMIT(gimbal.pitch.targetAngle, pitch_min, pitch_max);
    break;

    case VISION_AI_OUTPOST:
        gimbal.base_yaw.targetAngle = gimbal.top_yaw.targetAngle = USER_SentryCmd.look_yaw;
        gimbal.pitch.targetAngle = USER_SentryCmd.look_pitch + 7.5f;
    break;

    case VISION_AI_RUNE:
        gimbal.base_yaw.targetAngle = gimbal.top_yaw.targetAngle = USER_SentryCmd.look_yaw;
        gimbal.pitch.targetAngle = USER_SentryCmd.look_pitch;
    }
}

// 视觉控制
static bool Gimbal_Shooter_VisionControl(void)
{
    if (vision_target.vaild)
    {
        // base_yaw
        gimbal.base_yaw.targetAngle = ModularDegreeTowards(vision_target.base_yaw, gimbal.base_yaw.targetAngle);// 大yaw直接瞄就行

        // top_yaw//!有点问题吧，我怎么感觉应该无论如何小yaw先摆头到位，之后大yaw跟随路上小yaw反动就好了
        float top_target_unwrap = gimbal.base_yaw.targetAngle + (vision_target.yaw - vision_target.base_yaw);

        float delta = top_target_unwrap - gimbal.base_yaw.totalAngle;//计算相对误差
        LIMIT(delta, -TOP_YAW_LIMIT, TOP_YAW_LIMIT);

        gimbal.top_yaw.targetAngle = gimbal.base_yaw.totalAngle + delta;//小yaw跟随base

        // pitch
        gimbal.pitch.targetAngle = vision_target.pitch;
        LIMIT(gimbal.pitch.targetAngle, gimbal.pitch.pitchMin, gimbal.pitch.pitchMax);

        shooter.idToTrigger = vision_target.idToTrigger;
        shooter.command = vision_target.shooterCommand;
        return true;
    }
    else
    {
        shooter.command = IDLE;
        return false;
    }
}

// region 遥控器控制
static void Shooter_RockerControl(void)
{
    if (Gimbal_AttackAllowed())
    {
        shooter.command = TRIGGER_CONTINUE;
    }
    else
    {
        shooter.command = IDLE;
    }
}

static void Gimbal_RockerControl(void)
{
    gimbal.top_yaw.targetAngle -= rcInfo.ch1 * 0.3 / 660.0f;    // yaw
    gimbal.base_yaw.targetAngle = gimbal.top_yaw.targetAngle;
    gimbal.pitch.targetAngle += rcInfo.ch2 * 0.35 / 660.0f; // 旋转云台pitch
    LIMIT(gimbal.pitch.targetAngle, gimbal.pitch.pitchMin, gimbal.pitch.pitchMax);
}


static void Fric_RockerControl(void)
{
    static int lastwheel = 0;
    
    if (rcInfo.wheel > 400 && lastwheel <= 400)
    {
        shooter.fricOpen = !shooter.fricOpen;
    }
	
	lastwheel = rcInfo.wheel;
}
// endregion

static void Task_Gimbal_Callback(void)
{
// region Target_Node
    if (chassis.pattern == Chassis_AI && Gimbal_AttackAllowed())
    {
        if (Gimbal_Shooter_VisionControl())
        {
            shooter.fricOpen = true;
        }
        else
        {
            Gimbal_Scan();
            shooter.fricOpen = false;
        }
    }
    else
    {
        if (Gimbal_VisionForced())
        {
            if (!Gimbal_Shooter_VisionControl())
            {
                Gimbal_RockerControl();
            }
        }
        else
        {
            Gimbal_RockerControl();
            Shooter_RockerControl();
        }
		
        Fric_RockerControl();
    }

    Gimbal_UpdataAngle();
// endregion

// region Control_Node
    if (chassis.rotate.mode == ChassisMode_Spin)
    {
        gimbal.base_yaw.imuPID.outer.maxIntegral = 6000;
        gimbal.base_yaw.imuPID.outer.ki = 0.5;
    }
    else 
    {
        gimbal.base_yaw.imuPID.outer.maxIntegral = 1000;
        gimbal.base_yaw.imuPID.outer.ki = 0.1;
    }
    
    //计算小yaw电机输出
    DEPID_CascadeCalc(&gimbal.top_yaw.imuPID,gimbal.top_yaw.targetAngle,gimbal.top_yaw.totalAngle,gimbal.top_yaw.gyro);
    gimbal.top_yaw.imuPID.output = gimbal.top_yaw.imuPID.output;

    PID_SingleCalc(&gimbal.base_yaw.imuPID.outer,gimbal.base_yaw.targetAngle,gimbal.base_yaw.totalAngle);
    gimbal.base_yaw.imuPID.outer.output = -gimbal.base_yaw.imuPID.outer.output / 1000.0f;
    
    // 计算pitch电机输出
//        DEPID_CascadeCalc(&gimbal.pitch.imuPID, gimbal.pitch.targetAngle, gimbal.pitch.angle, gimbal.pitch.gyro);
    PID_SingleCalc(&gimbal.pitch.imuPID.outer,gimbal.pitch.targetAngle,gimbal.pitch.angle);
    gimbal.pitch.imuPID.output = - gimbal.pitch.imuPID.output/1000.0f  - PITCH_MASS * MASS_G * PITCH_R * cos(gimbal.pitch.angle * PI / 180.0f); ////输出除一千让PID参数乘1000方便调参
// endregion
}

/*************云台操控任务******************/

extern "C" void OS_GimbalCallback(void const *argument)
{
    osDelay(1500);
    Gimbal_Init();

    // 迁移后：业务由 C++ 插件图执行；旧 Task_Gimbal_Callback 保留作参考，不再调用。
    (void)Task_Gimbal_Callback;
    AppCpp_Gimbal_Init();

    for (;;)
    {
        AppCpp_Gimbal_ProcessFrame();
        osDelay(1);
    }
}

