// =============================================================================
// ChassisData.hpp —— 底盘板间反馈与观察结果的类型化数据定义
//
// 归属：C++ 插件框架 Nodes/Chassis 模块的公共数据头，只定义数据结构、不含任何逻辑。
//
// 数据来源：下板（底盘板）通过板间串口周期上报 4 组舵角与轮速；经 Platform 适配层
//           （legacy_chassis_feedback_source）转换为本文件中的类型化结构后，供底盘插件
//           节点消费。此处刻意与云台 CAN 电机协议分离，避免两套反馈语义混用。
//
// 单位约定：角度用「度」，转速用「rpm」，时间用「毫秒(ms)」；数组元素默认零初始化，
//           标量均给出显式默认值，读者可依据注释直接判断字段是否有效。
// =============================================================================
#ifndef robot_chassis_data_hpp
#define robot_chassis_data_hpp

#include <cstdint>

namespace robot::chassis {

// 底盘轮组数量，固定为 4。
// 用途：作为 steering_deg / drive_rpm 数组的编译期长度，并与 C 侧宏
//       ROBOT_CHASSIS_FEEDBACK_WHEELS 做 static_assert 校验（见适配器实现）。
// 取值：4（常量，不可改）；下标顺序沿用旧底盘协议里的轮组次序。
constexpr uint8_t k_wheel_count = 4U; // 保持旧底盘轮组顺序。

// 来自下板的最新一帧反馈（一次采样的稳定快照），不与云台 CAN 电机协议混用。
struct chassis_feedback {
    float steering_deg[k_wheel_count]{}; // 单圈舵角，度，未扣零偏。
    // 含义：每个轮组对应舵向电机的单圈角度，单位「度」。
    // 范围：约 [-180, 180]（取决于舵机安装与限位），未做零偏/零点标定扣除。
    // 默认：0.0f；索引次序与 k_wheel_count 一致，本轮无反馈时整数组保持零值。
    int16_t drive_rpm[k_wheel_count]{}; // 原始轮电机转速，不是车体速度。
    // 含义：驱动轮电机的原始转速，单位「rpm」（转/分钟），带符号表示转向。
    // 说明：这是电机侧原始量，不是车体线速度/角速度，需上层另行解算。
    // 默认：0。
    uint32_t receive_count{0U}; // 真实接收次数，不是插件帧号。
    // 含义：自启动以来成功收到并解析的板间帧累计计数（只增不减）。
    // 说明：与插件框架的 frame_id 无关，可用于判断是否有新数据到来。
    // 默认：0（从未收到）。
    uint32_t age_ms{0U}; // 最近帧年龄，仅 is_received=true 时有意义。
    // 含义：最近一帧样本的年龄 = 读取时刻 - 接收时刻，单位「毫秒(ms)」。
    // 有效性：仅当 is_received=true 时有意义；is_received=false 时恒为 0。
    // 默认：0。
    bool is_received{false}; // 是否收到过板间反馈。
    // 含义：是否至少成功收到过一帧板间反馈（链路是否曾经工作过）。
    // 默认：false。
    uint8_t is_lower_feedback{0U}; // 下板使能反馈原值。
    // 含义：下板 FEEDBACK 标志的原始值，未做任何解释（不代表单个电机在线）。
    // 默认：0。
};

// 底盘观察结果：对「底盘反馈 + 链路状态」的诊断性汇总，仅供诊断/调试，
// 不产生任何电机命令，也不直接参与控制。
struct chassis_observation {
    chassis_feedback feedback{}; // 本次消费的板间快照。
    // 含义：本帧从输入端口读到的底盘反馈快照；上游缺失时保持默认零值。
    uint32_t frame_id{0U}; // 消费该快照的插件帧号。
    // 含义：产生本观察结果时所处的插件帧号（context.frame_id）。
    // 用途：与快照做同帧对齐，避免跨帧混用数据。
    bool is_online{false}; // 板间链路是否未超时。
    // 含义：链路在线判定结果 = is_received && (age_ms <= 超时阈值)。
    // 默认：false（离线）。
};

} // namespace robot::chassis
#endif // robot_chassis_data_hpp
