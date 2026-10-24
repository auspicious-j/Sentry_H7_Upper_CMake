#include "Vision.h"
#include "Shooter.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include "usbd_cdc_if.h"
#include "Chassis.h"
#include "Gimbal.h"
#include "imu_temp_ctrl.h"
#include "Shooter.h"
#include "Judge.h"
#include "USER_B2B.h"
#include "arm_math.h"

#include <stdint.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>

VisionTransmit vision_transmit;
VisionReceive vision_receive;
VisionTarget vision_target;

static void Vision_UseData(void);

//接收来自视觉的信息
void Vision_DataReceive(uint8_t *read_from_usart, uint32_t length)
{
    static int i = -1;

    if (read_from_usart == NULL)
        return;
    
    while (length > 0)
    {
        if (i == -1 && *read_from_usart == VISION_FRAME_HEADER_RX)
        {
            i = 0;
        }

        if (i >= 0)
        {
            int rest = sizeof(vision_receive) - i, size = rest < length ? rest : length;
            memcpy((void*)&vision_receive + i, read_from_usart, size);
            i += size;

            if (i == sizeof(vision_receive))
            {
                i = -1;

                if (vision_receive.end_frame == VISION_FRAME_END_RX)
                {
                    Vision_UseData();
                }
            }
			
            read_from_usart += size;
            length -= size;
        }
        else
        {
            ++read_from_usart;
            --length;
        }
    }
}

// 打装甲板
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

// 打符
static void Vision_UseRune(void)
{
    int index = -1;
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

// 搬运数据
static void Vision_UseData(void)
{
    vx=-vision_receive.linear_y*1000; //ai部分解包 同时转换坐标系 ai坐标系下向前为x 左右为y 电控坐标系下 左右为x 前后为y
    vy=-vision_receive.linear_x*1000;
    vw=vision_receive.angular_z;

    chassis.rotate.align_yaw = vision_receive.align_yaw;

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
        Vision_UseArmor();
    break;

    case VISION_RECEIVE_RUNE:
        Vision_UseRune();
    break;

    default:
        vision_target.vaild = false;
    }
}

// 发送
static void Vision_DataTransmit(void)
{
    vision_transmit.header = VISION_FRAME_HEADER_TX;
    vision_transmit.detect_color = !USER_JudgeData.self_color; // 打红0 打蓝1
    vision_transmit.mode = 0; //0为打装甲板 1为打符
    vision_transmit.top_yaw = gimbal.top_yaw.totalAngle;
    vision_transmit.pitch = INS.pitch;
    vision_transmit.roll = INS.roll;
    vision_transmit.diff_yaw = (gimbal.top_yawMotor.angle - TOP_YAW_OFFSET) / 8192.0f * 360.0f;
    vision_transmit.diff_pitch = (PITCH_MOTOR_ZERO_POS - gimbal.pitchMotor.para.pos) / (2 * PI) * 360.0f;
    vision_transmit.bullet_speed = USER_JudgeData.initial_speed;
    vision_transmit.robo_status = 0xFF;  /*receive_485.Judge_Data.robo_status;*/
    memcpy(&vision_transmit.AI_Judge_data, &USER_JudgeData, sizeof(Judge_Data_e));
    vision_transmit.see_enemy = vision_target.vaild;
    vision_transmit.end_frame = VISION_FRAME_END_TX;
    CDC_Transmit_HS((uint8_t*)&vision_transmit, sizeof(vision_transmit));
}

void OS_VisionCallback(void const * argument)
{
    for(;;)
    {
        Vision_DataTransmit();
        osDelay(1);
    }
}
