/*
 * Vision.c — 视觉通信收发与解析实现。
 *
 * 数据流：
 *   接收：串口中断回调 Vision_DataReceive() 逐字节拼帧 -> 凑满一帧调用 Vision_ParseData()
 *         -> 更新 vision_target / 底盘运动指令 / 火控(shooter.workState)。
 *   发送：定时任务 OS_VisionCallback() 周期调用 Vision_DataTransmit()
 *         = Vision_DataUpdate()(刷新各字段) + CDC_Transmit_HS()(USB-CDC 发送)。
 *
 * 术语：
 *   - 拆帧：从连续字节流中按"帧头 + 定长数据体 + 帧尾"提取出一帧的过程。
 *   - 火控：根据目标角度/距离/入射角判断是否允许开火（命中判定）。
 */

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
#include "arm_math.h"
#include "USER_B2B.h"

#include <stdint.h>
#include <float.h>
#include <stdbool.h>

/* 模块全局变量：分别承载 待发送帧 / 最近接收帧 / 归一化目标 */
VisionTransmit vision_transmit;  // 发送帧缓冲（对应 Vision.h 中 extern 声明）
VisionReceive vision_receive;    // 接收帧缓冲（拆帧时逐字节写入此处）
VisionTarget vision_target;      // 解析后的当前目标（供云台/火控读取）

/*
 * 函数：Vision_Init
 * 用途：视觉通信初始化。目前只把发送帧帧头预置为固定值，其余字段由 Vision_DataUpdate 周期刷新。
 * 参数：无
 * 返回：无
 */
void Vision_Init(void)
{
    vision_transmit.header = VISION_FRAME_HEADER_TX;  // 预置发送帧帧头(0x5A)
}

/*
 * 函数：Vision_DataReceive
 * 用途：串口/USB 接收回调。把不定长字节流按协议拼成一整帧：
 *       先丢弃数据直到匹配帧头(0xA5)，之后逐字节写入 vision_receive，
 *       凑满 sizeof(vision_receive) 且帧尾正确后交给 Vision_ParseData() 解析。
 * 参数：read_from_usart —— 本次收到的数据首地址（串口驱动传入）；
 *       length          —— 本次收到的字节数。
 * 返回：无
 * 说明：用 static 变量 i 跨调用记录拼帧进度：-1=尚未找到帧头；>=0=已缓存 i 字节数据体。
 */
void Vision_DataReceive(uint8_t *read_from_usart, uint32_t length)
{
    static int i = -1;  // 拼帧进度：-1=未对齐帧头；>=0=已缓存的数据体字节数

    if (read_from_usart == NULL)
        return;  // 空指针保护

    while (length > 0)  // 逐段消费本次收到的全部字节
    // 查找帧头
    {
        if (i == -1 && *read_from_usart == VISION_FRAME_HEADER_RX)  // 未对齐且当前字节恰为帧头
        {
            i = 0;  // 对齐成功，开始缓存数据体
        }

        if (i >= 0)  // 已对齐帧头：继续往接收缓冲填充
        {
            int rest = sizeof(vision_receive) - i, size = rest < length ? rest : length;  // 还差多少字节 / 本次最多可拷贝的字节数
            memcpy((void*)&vision_receive + i, read_from_usart, size);  // 按字节追加写入接收缓冲
            i += size;  // 更新拼帧进度

            if (i == sizeof(vision_receive) && vision_receive.end_frame == VISION_FRAME_END_RX)  // 缓冲已满且帧尾匹配
            {
                i = -1;  // 一帧完成，复位进度以接收下一帧
                Vision_ParseData();  // 解析刚收到的完整帧
            }

            read_from_usart += size;  // 前移读指针
            length -= size;  // 扣减剩余长度
        }
        else
        {
            ++read_from_usart;  // 未对齐：跳过当前字节继续寻找帧头
            --length;
        }
    //判断帧头数据是否正确
    //将数据存入接收buffer
    }
}

/*
 * 函数：Vision_DataUpdate
 * 用途：把当前传感/判罚状态刷新到发送帧 vision_transmit 的各个字段，
 *       供 Vision_DataTransmit 直接整块发出。
 * 参数：无
 * 返回：无
 */
 //对发送的数据更新
void Vision_DataUpdate(void)
{
    vision_transmit.header = VISION_FRAME_HEADER_TX;  // 帧头固定 0x5A
    vision_transmit.detect_color = !USER_JudgeData.self_color; // 目标颜色取己方颜色的反色：打红0 打蓝1
    vision_transmit.mode = 0; // 视觉工作模式：0为打装甲板 1为打符
    vision_transmit.top_yaw = gimbal.top_yaw.totalAngle;  // 大 yaw 累计总角度
    vision_transmit.pitch = INS.pitch;                    // 云台俯仰角（来自 IMU）
    vision_transmit.roll = INS.roll;                      // 机体横滚角（来自 IMU）
    vision_transmit.diff_yaw = (gimbal.top_yawMotor.angle - TOP_YAW_OFFSET) / 8192.0f * 360.0f;  // 大 yaw 电机相对零位的角度：编码器计数(8192/圈)折算为 0~360°
    vision_transmit.diff_pitch = (PITCH_MOTOR_ZERO_POS - gimbal.pitchMotor.para.pos) / (2 * PI) * 360.0f;  // pitch 相对零位角：先算弧度偏差再折算为角度
    vision_transmit.bullet_speed = USER_JudgeData.initial_speed;  // 当前弹丸初速度
    vision_transmit.robo_status = 0xFF;  /*receive_485.Judge_Data.robo_status;*/  // 敌方存活状态暂填 0xFF(未知)
    memcpy(&vision_transmit.AI_Judge_data, &USER_JudgeData, sizeof(Judge_Data_e));  // 附带整块转发判罚数据
    vision_transmit.see_enemy = vision_target.vaild;  // 是否可见目标（由解析得到的有效标志驱动）
    vision_transmit.end_frame = VISION_FRAME_END_TX;  // 帧尾固定 0xA5
}

/*
 * 函数：Vision_ParseData
 * 用途：解析已接收完整的一帧 vision_receive，做三件事：
 *       1) 把 AI 战术指令（底盘速度、姿态、买弹买血等）写入对应模块；
 *       2) 按 mode 更新归一化目标 vision_target；
 *       3) 根据目标几何关系做命中判定，设置 shooter.workState（开火/待命）。
 * 参数：无（直接读取全局 vision_receive）
 * 返回：无
 */
void Vision_ParseData(void)
{
    vx=-vision_receive.linear_y*1000; //ai部分解包 同时转换坐标系 ai坐标系下向前为x 左右为y 电控坐标系下 左右为x 前后为y；再×1000 由 m/s 转 mm/s
    vy=-vision_receive.linear_x*1000; // 同上，AI 的 linear_x(前) 映射到电控的 vy(前后)，取负并转 mm/s
    vw=vision_receive.angular_z;       // 底盘旋转角速度，单位 rad/s，直接透传
    chassis.rotate.align_yaw = vision_receive.align_yaw;  // 起伏路段对齐角，供底盘姿态对齐使用
    USER_SentryCmd.sentry_mode = vision_receive.sentry_mode;              // 战术姿态：1进攻 2防守 3移动
    USER_SentryCmd.energy_activation = vision_receive.energy_activation;  // 能量机关激活请求：0不激活 1小符 2大符
    USER_SentryCmd.buy_projectile = vision_receive.buy_projectile;        // 买弹数量（单调递增的累计值）
    USER_SentryCmd.buy_life = vision_receive.buy_life;                    // 买活(立即复活)请求 0/1
    USER_SentryCmd.remote_buy_bullet = vision_receive.remote_buy_bullet;  // 远程买弹次数
    USER_SentryCmd.remote_buy_blood = vision_receive.remote_buy_blood;    // 远程买血次数

    if (vision_receive.mode == VISION_RECEIVE_ARMOR)  // 模式一：打装甲板（自瞄）
    {
        vision_target.vaild = true;  // 标记存在有效目标
        vision_target.base_yaw = vision_receive.armor.base_yaw;
        vision_target.yaw = vision_receive.armor.yaw;
        vision_target.pitch = vision_receive.armor.pitch;

        float a = INS.yaw / 180.0f * PI - vision_receive.armor.yaw / 180 * PI, b = vision_receive.armor.incident_yaw;  // a=云台当前 yaw 与目标 yaw 的角差(弧度)；b=装甲板入射角(弧度)
        //a是云台角度和目标角度error  b是装甲板的入射角
        float D = vision_receive.armor.distance, r = vision_receive.armor.hit_radius;  // D=目标距离(m)；r=命中半径(m)
        float rcosb = r * cos(b), rsinb = r * sin(b), tana = tan(a);  // 预计算：命中圆在视线方向的投影量，以及瞄准角差的正切

        if (tana < rcosb / (D - rsinb) && tana > -rcosb / (D + rsinb))  // 命中判定：瞄准角差落在目标等效张角范围内才允许开火
        {
            shooter.workState = TRIGGER_CONTINUE;  // 连续开火
        }
        else
        {
            shooter.workState = IDLE;  // 未对准，保持待命
        }
    }
    else if (vision_receive.mode == VISION_RECEIVE_RUNE)  // 模式二：打符（能量机关，两片扇叶择机射击）
    {
        static uint32_t last_trigger_tick[5];  // 记录每片扇叶(按编号 index，最多 5 个槽)上次开火的系统滴答，用于计算冷却

        if (shooter.workState != TRIGGER)  // 本分支只在非"单发触发"状态下工作（瞬间的 TRIGGER 由下方判定置位）
        {
            shooter.workState = IDLE;  // 默认待命
        }

        uint32_t current_tick = HAL_GetTick();  // 当前系统毫秒计时（1ms 递增）
        int target = -1;  // 选中的候选扇叶下标(0/1)，-1 表示无候选
        bool can_target_trigger = false;  // 选中的扇叶当前是否处于"冷却结束、允许开火"状态
        float min_angle_distance = FLT_MAX;  // 与候选扇叶的瞄准角偏，取最小者优先

        for (int i = 0; i < 2; ++i)  // 遍历视觉下发的两片待打扇叶
        {
            int j = vision_receive.rune.index[i];  // 该扇叶的物理编号（用于冷却计时索引）

            if (j != -1)  // -1 表示此槽位无效
            {
                float since_last_trigger = 0.001f * (current_tick - last_trigger_tick[j]);  // 距上次开火已过秒数(ms→s)
                bool can_trigger = since_last_trigger >= vision_receive.rune.hit_time[i] + vision_receive.rune.wait_time;  // 冷却(命中时间+等待时间)是否已过

                // 处于"可开火"或"仍在等待窗口(wait_time)内"的扇叶才作为候选
                if (can_trigger || since_last_trigger <= vision_receive.rune.wait_time)
                {
                    float angle_distance = hypotf(INS.yaw - vision_receive.rune.yaw[i], INS.pitch - vision_receive.rune.pitch[i]);  // 云台当前角与扇叶目标的综合角偏(勾股合成)

                    if (angle_distance < min_angle_distance)  // 取角偏最小者作为最优目标
                    {
                        target = i;
                        can_target_trigger = can_trigger;
                        min_angle_distance = angle_distance;
                    }
                }
            }
        }

        if (target != -1)  // 找到有效候选扇叶
        {
            vision_target.vaild = true;
            vision_target.base_yaw = vision_receive.rune.base_yaw;
            vision_target.yaw = vision_receive.rune.yaw[target];
            vision_target.pitch = vision_receive.rune.pitch[target];

            if (can_target_trigger)  // 仅在冷却结束、允许开火时做命中判定
            {
                float a = INS.yaw / 180.0f * PI - vision_receive.rune.yaw[target] / 180 * PI;  // 云台 yaw 与目标扇叶 yaw 的角差(弧度)
                float b = vision_receive.rune.incident_yaw[target];                    // 扇叶入射角(弧度)
                float D = vision_receive.rune.distance[target], r = vision_receive.rune.hit_radius;  // 距离(m) 与 命中半径(m)
                float R = D / cos(a - b), y = R * sin(a), x = R * cos(b);  // 由几何关系求扇叶在命中平面内的横向偏移 y 与纵向 x

                // 命中判据：把云台俯仰与目标俯仰折算到同一平面后，横向偏差 <= 命中半径 即判定命中
                if (hypotf(y, x * tan(INS.pitch / 180.0f * PI) - D * tan(vision_receive.rune.pitch[target] / 180.0f * PI)) <= r)
                {
                    shooter.workState = TRIGGER;  // 单发触发一次
                    last_trigger_tick[vision_receive.rune.index[target]] = current_tick;  // 记录本次开火时刻，用于冷却计算
                }
            }
        }
        else  // 无有效扇叶
        {
            vision_target.vaild = false;
            shooter.workState = IDLE;
        }
    }
    else  // 其他模式：无目标
    {
        vision_target.vaild = false;
        shooter.workState = IDLE;
    }
}

/*
 * 函数：Vision_DataTransmit
 * 用途：刷新发送帧内容并整帧通过 USB-CDC 发出。
 * 参数：无
 * 返回：无
 */
void Vision_DataTransmit(void)
{
    Vision_DataUpdate();  // 先刷新各字段
    CDC_Transmit_HS((uint8_t*)&vision_transmit, sizeof(vision_transmit));  // 把整个结构体按字节发出（packed 布局与协议一致）
}

/*
 * 函数：OS_VisionCallback
 * 用途：视觉通信周期任务（FreeRTOS 线程体）。
 * 参数：argument —— 任务创建参数（未使用）
 * 返回：无（永不返回）
 */
void OS_VisionCallback(void const * argument)
{
    Vision_Init();  // 任务启动时初始化

    for(;;)  // 周期循环
    {
        Vision_DataTransmit();  // 每周期发送一帧
        osDelay(1);  // 延时 1 个 tick，控制发送频率
    }
}
