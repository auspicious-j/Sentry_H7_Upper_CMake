/* =============================================================================
 * 文件：MotorData.hpp
 * 作用：定义「电机反馈 / 电机命令」的统一数据结构，作为插件框架中电机相关端口的
 *       数据类型（端口模型见 Framework/Ports.hpp）。
 *
 * 一、为什么需要这一层适配（适配器模式的「目标接口」）
 *   本工程的旧代码直接使用两种电机：DJI 电机（DJI_Motor_t）和达妙 DM 电机
 *   （DM_motor_t），它们的字段命名、量纲、取值范围各不相同。若让上层底盘、云台
 *   节点直接读取这些协议结构体，业务逻辑就会被绑死在具体电机型号上。
 *   因此这里定义一组「与协议无关」的通用结构体作为适配目标：由电机反馈节点
 *   （motor_feedback_node）负责把旧结构体翻译成这里的数据，上层节点只认这组结构体。
 *   将来更换电机型号时，只需替换转换函数，上层逻辑无需改动。
 *
 * 二、这里定义了什么
 *   - motor_feedback      ：单个电机的统一反馈（角度 / 转速 / 转矩 / 温度 / 在线）。
 *   - motor_feedback_frame ：一帧里一组电机反馈（固定 8 通道）。
 *   - motor_online_frame   ：一帧里一组电机在线状态（固定 8 通道）。
 *   - motor_command       ：单个电机的统一控制命令（电流 / 使能）。
 *   - motor_command_frame  ：一帧里一组电机控制命令（固定 8 通道）。
 *   这些结构体只描述「数据长什么样」，不含任何 CAN 收发或控制逻辑。
 *
 * 三、术语提示
 *   「帧（frame）」指插件图被调度执行一轮所对应的数据集合，帧内所有字段属于
 *   同一时刻的采样，配合帧号便于下游判断数据新旧（见 Ports.hpp）。
 * ============================================================================= */

#ifndef robot_motor_data_hpp
#define robot_motor_data_hpp

#include <cstdint>

namespace robot::motor {

// 单个反馈帧里能容纳的最大电机通道数（编译期常量）。
// 用途：下面所有 Frame 结构体都用它作为数组长度，保证结构体大小固定、
//       可按值拷贝进 frame_signal 存储；当前实际使用 5 个通道（见 .cpp）。
constexpr uint8_t k_motor_channel_count = 8U; // 单帧最多支持的电机通道数。

// motor_feedback：单个电机的统一反馈数据。
// 这是「适配器」的输出类型：无论数据来自 DJI 还是 DM 电机，都被翻译成这同一套
// 字段，从而对上层节点隐藏 DJI/DM 协议细节。各字段用默认成员初始化器给出默认值
// 0 / false，保证即使漏赋值，得到的也是一份确定的「空数据」而非随机内存值。
struct motor_feedback {
    uint16_t id{0U}; // 电机协议 ID（如 0x201），用于标识是哪颗电机；默认 0 表示未设置。
    int16_t angle{0}; // 角度反馈：DJI 为编码器原始角度，DM 为位置积分量 p_int；默认 0。
    int16_t speed{0}; // 速度反馈：DJI 为转子转速，DM 为速度 v_int；默认 0。
    int16_t torque{0}; // 转矩 / 电流反馈：DJI 为 actual_current，DM 为扭矩 t_int；默认 0。
    int8_t temperature{0}; // 电机温度：DJI 为 temp，DM 为 MOS 管温度 Tmos（单位摄氏度）；默认 0。
    bool is_online{false}; // 本周期该电机是否在线（由反馈超时判定）；默认 false（离线）。
};

// motor_feedback_frame：一帧电机反馈的集合，是电机反馈端口（output_port）的数据类型。
// 上层底盘、云台等节点通过输入端口一次性读取本结构体，即可拿到全组电机状态。
// 数组尾部未使用的通道保持默认构造的 motor_feedback（id=0、is_online=false）。
struct motor_feedback_frame {
    motor_feedback motor[k_motor_channel_count]{}; // 固定容量反馈数组；下标即通道序号，缺省为默认值。
};

// motor_online_frame：一帧电机在线状态的集合（逐通道的在线 / 离线标志）。
// 与 motor_feedback_frame 分开定义，便于只需要「掉线检测」的上层节点只订阅状态位，
// 而不必搬运整组反馈数据。默认全 false，即未填写时视为全部离线。
struct motor_online_frame {
    bool is_online[k_motor_channel_count]{}; // 固定容量在线状态数组；下标即通道序号，缺省 false（离线）。
};

// motor_command：单个电机的统一控制命令（与具体电机型号解耦）。
// 上层控制器只填写这里的目标量与使能位，由后续的发送节点翻译成具体 CAN 报文。
// 所有字段均带默认值，未显式设置时即为「电流 0、不使能」的安全状态。
struct motor_command {
    uint16_t id{0U}; // 电机协议 ID，指明该命令发给哪颗电机；默认 0 表示未设置。
    int16_t current{0}; // 目标电流（或等价输出量，取决于具体电机协议）；默认 0 表示无输出。
    bool is_enabled{false}; // 是否允许输出：false 表示命令保持停机 / 不下发；默认 false。
};

// motor_command_frame：一帧电机控制命令的集合。
// 说明：目前仅作为「端口协议」定义，用于预留接口，并不直接发送 CAN 报文；
//       真正把命令翻译为 CAN 帧的动作由后续的发送节点负责。
struct motor_command_frame {
    motor_command motor[k_motor_channel_count]{}; // 固定容量命令数组；下标即通道序号，缺省为默认命令。
};

} // namespace robot::motor

#endif /* robot_motor_data_hpp */
