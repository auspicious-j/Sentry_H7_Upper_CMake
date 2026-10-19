#include "Remote_Control_Node.hpp"

// C 侧头文件没有统一的 extern "C" 包装，这里整体包一层，保证 C++ 引用到的是
// C 符号（全局变量名本来不修饰，函数名则需要 C 链接）。
extern "C"
{
#include "Vision.h"
#include "Shooter.h"
#include "usbd_cdc_if.h"
#include "Chassis.h"
#include "Gimbal.h"
#include "imu_temp_ctrl.h"
#include "Judge.h"
#include "USER_B2B.h"
}

#include "arm_math.h"

#include <stdint.h>
#include <float.h>
#include <math.h>

// 说明：vision_receive / vision_target 由 Application/src/Vision.c 定义，
// Vision.h 中以 extern 声明；本文件不再重复定义，避免链接期重复符号。

// 装甲板解包
static void Vision_UseArmor(void)
{
    vision_target.vaild = true;
    vision_target.base_yaw = vision_receive.armor.base_yaw;
    vision_target.yaw = vision_receive.armor.yaw;
    vision_target.pitch = vision_receive.armor.pitch;

    float a = INS.yaw / 180.0f * PI - vision_receive.armor.yaw / 180 * PI, b = vision_receive.armor.incident_yaw;
    //a是云台角度和目标角度error  b是装甲板的入射角
    float D = vision_receive.armor.distance, r = vision_receive.armor.hit_radius;
    float rcosb = r * cos(b), rsinb = r * sin(b), tana = tan(a);
    
    if (tana < rcosb / (D - rsinb) && tana > -rcosb / (D + rsinb))
    {
        vision_target.shooterCommand = TRIGGER_CONTINUE;
    }
    else
    {
        vision_target.shooterCommand = IDLE;
    }
}

// 打符解包
static void Vision_UseRune(void)
{
    int index = -1;// 当前最优的符id
    bool is_triggering;
    float min_angle_distance = FLT_MAX;

    for (int i = 0; i < 2; ++i)
    {
        int id = vision_receive.rune.id[i];

        if (id != -1)
        {
            if (shooter.triggeringId == id)
            {
                shooter.triggeringHitTick = vision_receive.rune.hit_time[i] * 1000;
                index = i;
                is_triggering = true;
                break;
            }

            if (shooter.triggeringId == -1 && HAL_GetTick() > shooter.hitAtTick[id])
            {
                float angle_distance = hypotf(INS.yaw - vision_receive.rune.yaw[i], INS.pitch - vision_receive.rune.pitch[i]);

                if (angle_distance < min_angle_distance)
                {
                    index = i;
                    is_triggering = false;
                    min_angle_distance = angle_distance;
                }
            }
        }
    }

    if (index != -1)
    {
        vision_target.vaild = true;
        vision_target.base_yaw = vision_receive.rune.base_yaw;
        vision_target.yaw = vision_receive.rune.yaw[index];
        vision_target.pitch = vision_receive.rune.pitch[index];

        if (is_triggering)
        {
            vision_target.shooterCommand = IDLE;
        }
        else
        {
            float a = INS.yaw / 180.0f * PI - vision_receive.rune.yaw[index] / 180 * PI;
            float b = vision_receive.rune.incident_yaw[index];
            float D = vision_receive.rune.distance[index], r = vision_receive.rune.hit_radius;
            float R = D / cos(a - b), y = R * sin(a), x = R * cos(b);

            if (hypotf(y, x * tan(INS.pitch / 180.0f * PI) - D * tan(vision_receive.rune.pitch[index] / 180.0f * PI)) <= r)
            {
                vision_target.idToTrigger = vision_receive.rune.id[index];
                vision_target.shooterCommand = TRIGGER;
            }
            else
            {
                vision_target.shooterCommand = IDLE;
            }
        }
    }
    else
    {
        vision_target.vaild = false;
    }
}

//vision解包
static void Vision_UseData(void)
{
    vx=-vision_receive.linear_y*1000; //ai部分解包 同时转换坐标系 ai坐标系下向前为x 左右为y 电控坐标系下 左右为x 前后为y
    vy=-vision_receive.linear_x*1000;
    vw=vision_receive.angular_z;

    // 解包给底盘
    chassis.rotate.align_yaw = vision_receive.align_yaw;

    // 解包电控侧哨兵自主指令
    USER_SentryCmd.sentry_mode = vision_receive.sentry_mode;
    USER_SentryCmd.armor_mode = vision_receive.armor_mode;
    USER_SentryCmd.look_yaw = vision_receive.look_yaw;
    USER_SentryCmd.look_pitch = vision_receive.look_pitch;
    USER_SentryCmd.energy_activation = vision_receive.energy_activation;
    USER_SentryCmd.buy_projectile = vision_receive.buy_projectile;
    USER_SentryCmd.buy_life = vision_receive.buy_life;
    USER_SentryCmd.remote_buy_bullet = vision_receive.remote_buy_bullet;
    USER_SentryCmd.remote_buy_blood = vision_receive.remote_buy_blood;
    
    switch (vision_receive.mode)
    {
    case VISION_RECEIVE_ARMOR:
        Vision_UseArmor();// 装甲板解包
    break;

    case VISION_RECEIVE_RUNE:
        Vision_UseRune();// 打符解包
    break;

    default:
        vision_target.vaild = false;
    }
}

namespace robot::Gimbal
{
    robot::framework::process_result Remote_Control_Node::process(robot::framework::frame_context& context)
    {
        // M0：节点尚未注册进图；先保留“上游无本帧数据则本帧无输出”的通用骨架。
        // M2 会把输入改成视觉快照信号，并在通过新鲜度检查后调用解包逻辑。
        if ((vision_ == nullptr) || !vision_->is_valid() || (vision_->frame_id() != context.frame_id))
        {
            output_.invalidate();
            return robot::framework::process_result::no_new_data;
        }

        // 业务代码（迁移期仍读取 C 侧解析好的全局结构）
        Vision_UseData();

        // 末尾输出
        output_.publish(context.frame_id, vision_->read());
        return robot::framework::process_result::ok;
    }
}