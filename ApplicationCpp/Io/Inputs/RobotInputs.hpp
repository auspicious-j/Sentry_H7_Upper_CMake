/* ============================================================================
 * 文件：RobotInputs.hpp
 * 作用：把 C 侧各数据源（IMU / 遥控器 / 裁判数据 / 电机与下板反馈）整理成
 *       与平台无关的 POD 快照，供插件图节点在帧内读取。
 * 设计：
 *   - 读侧快照：节点在自己的任务里调用 read_*()，函数内部用极短的关中断
 *     临界区整份拷贝 C 全局量，保证不会读到被中断写了一半的数据；
 *   - 不修改旧链路：C 侧的写方（IMU 任务、RC 任务、CAN 中断、B2B 中断）
 *     完全不动，迁移期可以和新逻辑并存、旁路对比；
 *   - sampled_at_us 由调用方传入（通常用本帧 timestamp_us），表示“这份副本
 *     是何时取的”，用于后续的 TTL/age 判断。
 * 注意：本层只做数据搬运，不做任何业务判断。
 * ========================================================================== */
#ifndef robot_io_inputs_robot_inputs_hpp
#define robot_io_inputs_robot_inputs_hpp

#include <cstdint>

namespace robot::io::inputs {

// 快照新鲜度阈值（供消费者判断“这份输入还能不能用”）。
inline constexpr std::uint64_t k_imu_ttl_us = 20000U;     // IMU 正常 1kHz，20ms 足够宽松。
inline constexpr std::uint64_t k_rc_ttl_us = 100000U;    // 遥控器 15ms 轮询，100ms 超时。
inline constexpr std::uint64_t k_judge_ttl_us = 100000U; // 下板反馈正常 1kHz。

// IMU 姿态快照（对应 C 侧 INS_t）。
struct imu_snapshot {
    float yaw_deg;       // 偏航角，度。
    float pitch_deg;     // 俯仰角，度。
    float roll_deg;      // 横滚角，度。
    float yaw_total_deg; // 偏航累计角，度。
    float gyro_x;        // 原始角速度。
    float gyro_y;
    float gyro_z;
    std::uint64_t sampled_at_us; // 取快照时刻，微秒。
};

// 遥控器快照（对应 C 侧 RC_TypeDef）。
struct rc_snapshot {
    std::int16_t ch1; // 右横（已去中值与死区）。
    std::int16_t ch2; // 右竖。
    std::int16_t ch3; // 左横。
    std::int16_t ch4; // 左竖。
    std::int16_t wheel;
    std::uint8_t left;  // 左三挡开关。
    std::uint8_t right; // 右三挡开关。
    std::uint8_t left_last;
    std::uint8_t right_last;
    std::uint8_t lost;     // 失联标志。
    std::uint8_t failsafe; // 失控保护标志。
    std::uint64_t sampled_at_us;
};

// 裁判系统快照（对应 C 侧 USER_JudgeData，实际由下板通过 B2B 写入）。
struct judge_snapshot {
    std::uint8_t game_progress;
    std::uint16_t remain_time_s;
    std::uint16_t current_hp;
    std::uint16_t projectile_allowance;
    std::uint16_t sentry_info;
    std::uint16_t sentry_info_2;
    std::uint16_t ally_outpost_hp;
    std::uint16_t ally_base_hp;
    std::uint16_t barrel_cooling_value; // 冷却速度。
    std::uint16_t barrel_heat_limit;    // 热量上限。
    std::uint16_t barrel_heat_now;      // 当前枪口热量。
    float initial_speed;                // 弹速。
    std::uint8_t power_gimbal_output;
    std::uint8_t power_chassis_output;
    std::uint8_t power_shooter_output;
    std::uint8_t self_color;
    bool is_valid; // 裁判数据是否可用（对应 JUDGE_IsValid）。
    std::uint64_t sampled_at_us;
};

// 执行机构反馈快照：只保留图里真正消费的字段（触发累计角、小 yaw 编码器、
// 四轮舵角、大 yaw 角度）。其余电机量目前无人读，删掉以免整份拷贝时白白搬运。
struct actuator_feedback {
    std::int32_t trigger_total_angle; // 拨弹电机累计角度（Motor 任务更新）。
    std::int16_t top_yaw_angle;       // 小 yaw 编码器（State 节点解算 base yaw）。
    float wheel_turn_angle_deg[4];    // B2B：四轮舵角。
    float base_yaw_angle_deg;         // B2B：大 yaw 角度。
    std::uint64_t sampled_at_us;
};

// 以下读取函数只能在任务上下文调用；内部是极短的关中断临界区。
imu_snapshot read_imu(std::uint64_t sampled_at_us);
rc_snapshot read_rc(std::uint64_t sampled_at_us);
judge_snapshot read_judge(std::uint64_t sampled_at_us);
actuator_feedback read_actuator_feedback(std::uint64_t sampled_at_us);

} // namespace robot::io::inputs

#endif // robot_io_inputs_robot_inputs_hpp
