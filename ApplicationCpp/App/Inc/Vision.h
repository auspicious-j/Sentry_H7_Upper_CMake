#ifndef _VISION_H_
#define _VISION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stdbool.h"
#include "Shooter.h"

#include <math.h>
#include <stdint.h>

#define VISION_FRAME_HEADER_TX   0x5A
#define VISION_FRAME_END_TX      0xA5
#define VISION_FRAME_HEADER_RX   0xA5
#define VISION_FRAME_END_RX      0x5A

typedef struct __attribute__((packed))
{
    uint8_t game_progress; // 当前比赛状态 0:未开始比赛 1:准备阶段 2:自检阶段 3:五秒倒计时 4:比赛中 5:比赛结算中
    uint16_t remain_time;  // 比赛剩余时间 单位:s
    uint16_t current_hp;   // 当前血量
    uint16_t projectile;    // 哨兵当前剩余允许发弹量

    uint32_t sentry_info;     //bit0-10：除远程兑换外，哨兵机器人成功兑换的允许发弹量，开局为0，在哨兵机器人成功兑换一定允许发弹量后，该值将变为哨兵机器人成功兑换的允许发弹量值
                            //bit11-14：哨兵机器人成功远程兑换允许发弹量的次数，开局为0，在哨兵机器人成功远程兑换允许发弹量后，该值将变为哨兵机器人成功远程兑换允许发弹量的次数
                            //bit15-18：哨兵机器人成功远程兑换血量的次数，开局为0，在哨兵机器人成功远程兑换血量后，该值将变为哨兵机器人成功远程兑换血量的次数
                            //bit19：哨兵机器人当前是否可以确认免费复活，可以确认免费复活时值为1，否则为0
                            //bit20：哨兵机器人当前是否可以兑换立即复活，可以兑换立即复活时值为1，否则为0
                            //bit21-30：哨兵机器人当前若兑换立即复活需要花费的金币数。
                            //bit31 保留
    uint16_t sentry_info_2;    // bit0：哨兵当前是否处于脱战状态，处于脱战状态时为1，否则为0
                            // bit1-11：队伍17mm允许发弹量的剩余可兑换数
                            // bit12-13:哨兵当前姿态，1为进攻姿态，2为防御姿态，3为移动姿态
                            // bit14：己方能量机关是否能够进入正在激活状态，1为当前可激活
                            // bit 15：保留位

    uint8_t sentry_info_3;  // bit 0: 装甲板是否被攻击 0:否 1:是
                            // bit 1: RFID 是否检测到堡垒 0:否 1:是
                            // bit 2: RFID 是否检测到补给区(与兑换站不重叠) 0:否 1:是
                            // bit 3: RFID 是否检测到补给区(与兑换站重叠) 0:否 1:是
                            // bit 4: 当前剩余能量值是否小于30% 0:否 1:是
                            // bit 5：RFID 是否检测到对方前哨站增益点
                            // bit 6：RFID 是否检测到对方堡垒增益点
    uint8_t rune_state;     // bit 0-1:小符的状态 0未激活 1已激活 2正在激活
                            // bit 2-3:大符的状态 0未激活 1已激活 2正在激活
    uint16_t ally_outpost_hp;  // 己方前哨站血量
    uint16_t ally_base_hp;     // 己方基地血量
}Judge_Data_e;
//能否能打符  当前姿态  是否能买活 买活花多少钱
typedef struct __attribute__((packed))
{
    uint8_t header;
    uint8_t detect_color;  // 0-red 1-blue
    uint8_t mode;  //0为打装甲板 1为打符

    float roll;
    float pitch;
    float top_yaw;
    float diff_yaw;//大小yaw之间相差角度
      float diff_pitch;//大小yaw之间pitch差值
    float bullet_speed;
    uint8_t robo_status; //敌方机器人死没死

    Judge_Data_e AI_Judge_data;
    uint8_t see_enemy; //0表示没瞄到 1表示瞄到装甲板 2表示瞄到符

     uint8_t end_frame;
    //uint16_t checksum;
}VisionTransmit;

enum { VISION_RECEIVE_NONE, VISION_RECEIVE_ARMOR, VISION_RECEIVE_RUNE };
enum { VISION_AI_ARMOR, VISION_AI_OUTPOST, VISION_AI_RUNE };

#ifdef __cplusplus
}
#endif

#endif
