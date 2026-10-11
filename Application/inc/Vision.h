/*
 * Vision.h — 视觉通信接口（上位机/视觉端 <-> 哨兵电控）的数据协议与对外函数声明。
 *
 * 【整体说明】
 * 本模块负责哨兵（STM32H7 下位机）与视觉上位机之间的双向串口通信：
 *   - 接收（RX）：视觉端下发瞄准结果、底盘运动指令、战术指令等；
 *   - 发送（TX）：电控端上传云台姿态、比赛判罚数据、弹速等状态。
 *
 * 【帧格式（术语：帧 = 一次完整的数据包）】
 *   每一帧由 3 段组成：帧头(header) + 数据体 + 帧尾(end_frame)，用于界定一帧的边界。
 *   - 发送帧（TX，本机 -> 视觉端）：帧头 = VISION_FRAME_HEADER_TX(0x5A)，帧尾 = VISION_FRAME_END_TX(0xA5)
 *   - 接收帧（RX，视觉端 -> 本机）：帧头 = VISION_FRAME_HEADER_RX(0xA5)，帧尾 = VISION_FRAME_END_RX(0x5A)
 *   注意收发帧头/帧尾的取值是"互换"的，接收解析只认 RX 帧头，发送只填 TX 帧头，避免收发混淆。
 *
 * 【结构体对齐（术语：packed = 紧凑布局，不插入填充字节）】
 *   所有通信结构体均加 __attribute__((packed))，保证结构体内存布局与协议字节流逐字节一致，
 *   从而可以直接对结构体首地址做 memcpy/串口收发，无需手工逐字段打包。
 *
 * 【大小端（术语：字节序）】
 *   收发双方均为小端(little-endian)，多字节字段按低字节在前的顺序在线上传输。
 */

#ifndef _VISION_H_
#define _VISION_H_

#include "main.h"
#include "stdint.h"
#include "stdbool.h"

#include <math.h>
#include <stdint.h>

/* 帧定界符（单字节，取值刻意让收发相反以区分方向） */
#define VISION_FRAME_HEADER_TX   0x5A   // 发送帧帧头（电控 -> 视觉端）
#define VISION_FRAME_END_TX      0xA5   // 发送帧帧尾（电控 -> 视觉端）
#define VISION_FRAME_HEADER_RX   0xA5   // 接收帧帧头（视觉端 -> 电控）
#define VISION_FRAME_END_RX      0x5A   // 接收帧帧尾（视觉端 -> 电控）

/* 比赛判罚数据（裁判系统统一下发的比赛状态子集）。
 * 该结构既是裁判系统数据的本地映射，也是随发送帧转发给视觉端的数据块。
 * 与 USER_JudgeData 保持相同布局，可直接 memcpy 互通。 */
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
/*
 * 发送帧数据结构（电控 -> 视觉端）。
 * 用途：把当前云台姿态、弹速、以及"能否打符 / 当前姿态 / 是否能买活 / 买活花费"等
 *       状态回传给视觉端，供其决策与显示。
 */
typedef struct __attribute__((packed))
{
    uint8_t header;        // 帧头，固定填 VISION_FRAME_HEADER_TX(0x5A)
    uint8_t detect_color;  // 目标颜色 0-red 1-blue（本机识别/需要打击的敌方颜色）
    uint8_t mode; // 视觉工作模式 0为打车(打装甲板) 1为打前哨 2为打符

    float roll;        // 机体横滚角 单位:°（来自 IMU）
    float pitch; // 云台俯仰角 单位:° (INS.pitch)
    float top_yaw;     // 大 yaw 累计角度 单位:°（不限幅，总转角）
    float diff_yaw;//大小yaw之间相差角度 单位:°（大 yaw 与向心/参考方向之差）
      float diff_pitch; // 大小yaw之间pitch差值 单位:°（pitch 电机位置角相对零位的偏差）
    float bullet_speed; // 当前弹丸初速度 单位:m/s
    uint8_t robo_status; // 敌方机器人存活状态（0xFF 表示未知/无效）


    Judge_Data_e AI_Judge_data; // 附带下发的比赛判罚数据副本
    uint8_t see_enemy; // 视觉可见性 0表示没瞄到 1表示瞄到装甲板 2表示瞄到符

     uint8_t end_frame; // 帧尾，固定填 VISION_FRAME_END_TX(0xA5)
    //uint16_t checksum;  // 预留校验字段（当前协议未启用）
}VisionTransmit;

/* 接收帧 mode 字段的取值：表示本帧携带哪种目标信息 */
enum { VISION_RECEIVE_NONE, VISION_RECEIVE_ARMOR, VISION_RECEIVE_RUNE };

/*
 * 接收帧数据结构（视觉端 -> 电控）。
 * 前半部分为瞄准/火控信息（装甲板或打符二选一，由 mode 决定用哪个联合体分支），
 * 后半部分为 AI 战术指令（底盘运动、姿态、买弹买血等）。
 */
typedef struct __attribute__((packed))
{
    uint8_t header;  // 帧头，应等于 VISION_FRAME_HEADER_RX(0xA5)
    /*以下为视觉通信部分*/
    /*打装甲板模式*/
    uint8_t mode;    // 帧类型，取值见 VISION_RECEIVE_xxx 枚举（0无 1装甲板 2打符）

    /* 联合体：同一段内存按 mode 复用为装甲板或打符两种布局（省带宽） */
    union {
        /* 装甲板自瞄分支 */
        struct __attribute__((packed)) {
            float base_yaw; //自瞄目标角度 单位°
            float yaw;      // 目标 yaw 角 单位:°（云台需要指向的角度）
            float pitch;    // 目标 pitch 角 单位:°（云台需要抬到的角度）
            float incident_yaw; //单位弧度 （目标角度和目前角度差值，用于火控）
            float distance; //两车中心距离 单位:m（用于弹道解算与命中判定）
            float hit_radius; // 目标等效命中半径 单位:m
        } armor;

        /* 打符（能量机关）分支 */
        struct __attribute__((packed)) {
            float base_yaw;      // 基准 yaw 角 单位:°
            float hit_radius;    // 符的等效命中半径 单位:m
            float wait_time;     // 单页(扇叶)转过的等待时间 单位:s（命中后需等待的时间窗口）
            float yaw[2];        // 两片待打扇叶的 yaw 角 单位:°（索引 0/1）
            float pitch[2];      // 两片待打扇叶的 pitch 角 单位:°
            float incident_yaw[2]; // 两片扇叶的入射角 单位:弧度
            float distance[2];   // 两片扇叶的距离 单位:m
            float hit_time[2];   // 两片扇叶建议的射击时刻窗口/间隔 单位:s
            int8_t index[2];     // 扇叶编号（-1 表示该槽位无效）
        } rune;
    };

    uint8_t force_shoot;//检测到 强制开一发火 0是不开 1为开火

/***********以下为ai传输内容***********/
    float linear_x;   // AI 坐标系下前后方向速度指令（前为正） 单位:m/s
    float linear_y;   // AI 坐标系下左右方向速度指令（左为正） 单位:m/s
    float angular_z; //旋转速度 单位:rad/s

    float align_yaw; //和起伏路段对齐角度 单位:°（底盘姿态对齐用）
    float rune_yaw; //符的角度 单位:°（预测的符当前朝向）
    float outpost_yaw; //前哨站的角度 单位:°（前哨站目标朝向）

    uint8_t spin_mode;  //0为小陀螺关  1为小陀螺开
    uint8_t sentry_mode; //1为进攻 2为防守 3为移动 默认为3
    uint8_t armor_mode;  //0为打车 1打前哨 2为打符
    uint8_t align_mode; //是否对齐 0为不对齐 1为对齐起伏路段 2为对齐装甲板
    uint8_t energy_activation; //0为不激活 1为激活小符 2为激活大符 激活小还是大和比赛开始时间有关
    uint8_t buy_life;     //0为不买活 1为买活（立即复活）
    uint8_t remote_buy_blood;    //远程买血次数 0到1为远程买一次血 1到2为买一次 依此类推
    uint8_t remote_buy_bullet;  //远程买弹次数 0到1为远程买一次弹 1到2为买一次 依此类推
    uint16_t buy_projectile; //哨兵要买多少发弹 开局为0 修改后烧饼在补血点就能兑换 只能单增 如0->100买100发 100->101买1发 依此类推
    uint8_t end_frame; // 帧尾，应等于 VISION_FRAME_END_RX(0x5A)
}VisionReceive;

/* 解析后统一的目标信息（给云台/火控使用的简化视角）。
 * 无论原始帧是装甲板还是打符，最终都归一化到这里，便于上层统一处理。 */
typedef struct {
    bool vaild;      // 目标是否有效（true = 有可打击目标）
    float base_yaw;  // 基准 yaw 角 单位:°
    float yaw;       // 目标 yaw 角 单位:°
    float pitch;     // 目标 pitch 角 单位:°
} VisionTarget;

/* 模块对外全局变量（定义在 Vision.c） */
extern VisionReceive vision_receive;    // 最近一帧解析出的接收数据
extern VisionTransmit vision_transmit;  // 待发送的发送帧数据
extern VisionTarget vision_target;      // 归一化后的当前目标

void Vision_DataReceive(uint8_t *read_from_usart, uint32_t length); // 串口接收回调：缓存并拆帧，凑满一帧后解析
void Vision_DataTransmit(void);                                     // 组装并发送一帧数据
void Vision_DataUpdate(void);                                       // 用当前传感/判罚状态刷新发送帧各字段
void Vision_Init(void);                                             // 初始化视觉通信（设置发送帧帧头等）
void Vision_ParseData(void);                                        // 解析已接收完整的一帧并更新目标/火控状态

#endif
