/* =============================================================================
 * 文件：MotorFeedbackNode.cpp
 * 作用：实现「电机反馈节点」——读取旧 C 电机对象并翻译成统一帧，再发布给下游。
 *
 * 数据来源：Gimbal.h / Shooter.h 中定义的全局电机对象
 *   - shooter.fricMotor[0/1]  ：两个摩擦轮（DJI）
 *   - shooter.triggerMotor    ：拨弹电机（DJI）
 *   - gimbal.top_yawMotor     ：小 yaw 电机（DJI）
 *   - gimbal.pitchMotor       ：pitch 电机（达妙 DM）
 * 本文件把它们的字段逐颗翻译成 MotorFeedback，再按固定通道顺序装进 MotorFeedbackFrame。
 *
 * 通道约定（frame.motor 的下标 → 电机）在 process() 中固定：
 *   0=左摩擦轮 1=右摩擦轮 2=拨弹 3=小 yaw 4=pitch，其余通道（5~7）保持默认空值。
 * ============================================================================= */

#include "MotorFeedbackNode.hpp"

extern "C" {
#include "Gimbal.h"
#include "Shooter.h"
}

namespace robot::motor {

// 匿名命名空间：下面这些常量只在本 .cpp 内可见，避免污染其他翻译单元。
namespace {
// 以下常量是各电机写入统一反馈时使用的协议 ID（同时作为帧内的逻辑标识）。
constexpr uint16_t kFrictionMotor0Id = 0x201U; // 左摩擦轮 ID（对应 shooter.fricMotor[0]）。
constexpr uint16_t kFrictionMotor1Id = 0x202U; // 右摩擦轮 ID（对应 shooter.fricMotor[1]）。
constexpr uint16_t kTriggerMotorId = 0x203U; // 拨弹电机 ID（对应 shooter.triggerMotor）。
constexpr uint16_t kTopYawMotorId = 0x205U; // 小 yaw 电机 ID（对应 gimbal.top_yawMotor）。
constexpr uint16_t kPitchMotorId = 0x001U; // DM pitch 电机 ID（对应 gimbal.pitchMotor）。
}

// 构造函数：以节点 ID 20 创建节点（PluginNode 基类构造），
// 并把输出端口 output_ 绑定到本对象内的信号存储 signal_。
// 绑定后，process() 里 output_.publish(...) 写入的就是 signal_。
MotorFeedbackNode::MotorFeedbackNode()
    : PluginNode(20U)
{
    output_.bind(signal_);
}

// 返回本节点的输出端口。组装器用它与下游输入端口对接（端口 → 端口连接）。
robot::framework::OutputPort<MotorFeedbackFrame>& MotorFeedbackNode::output()
{
    return output_;
}

// 返回本节点内部信号存储的只读引用。
// 组装器若需要建立到本节点的输入端口，可用 InputPort::bind(signal()) 绑定此存储。
const robot::framework::FrameSignal<MotorFeedbackFrame>& MotorFeedbackNode::signal() const
{
    return signal_;
}

// 适配一颗 DJI 电机：逐字段从旧结构体拷贝到统一反馈结构。
// 参数 id/channel/motor 含义见头文件声明。
// 返回：已填充的 MotorFeedback（online 由反馈通道超时判定，超时阈值 100 个周期）。
MotorFeedback MotorFeedbackNode::convertDji(uint16_t id, uint8_t channel, const DJI_Motor_t& motor)
{
    MotorFeedback feedback{}; // 当前 DJI 电机的统一反馈；{} 保证字段先被默认初始化。
    feedback.id = id; // 写入逻辑标识。
    feedback.angle = motor.angle; // 角度：直接取 DJI 反馈的编码器原始角度。
    feedback.speed = motor.speed; // 速度：直接取 DJI 反馈的转子转速。
    feedback.torque = motor.torque; // 转矩 / 电流：直接取 DJI 反馈的 actual_current。
    feedback.temperature = motor.temp; // 温度：直接取 DJI 反馈的 temp（摄氏度）。
    feedback.online = BSP_MotorFeedbackIsOnline(channel, 100U); // 在线判定：该通道 100 个周期内是否有新反馈。
    return feedback;
}

// 适配一颗达妙 DM 电机：数据位于 motor.para，且部分字段位宽更大，需显式截断为 int16 / int8。
// 参数 id/channel/motor 含义见头文件声明。
// 返回：已填充的 MotorFeedback（online 判定方式同 DJI，超时阈值 100 个周期）。
MotorFeedback MotorFeedbackNode::convertDm(uint16_t id, uint8_t channel, const DM_motor_t& motor)
{
    MotorFeedback feedback{}; // 当前 DM 电机的统一反馈；{} 保证字段先被默认初始化。
    feedback.id = id; // 写入逻辑标识。
    feedback.angle = static_cast<int16_t>(motor.para.p_int); // 位置：DM 的位置积分量，截断为 16 位有符号。
    feedback.speed = static_cast<int16_t>(motor.para.v_int); // 速度：DM 的速度量，截断为 16 位有符号。
    feedback.torque = static_cast<int16_t>(motor.para.t_int); // 转矩：DM 的扭矩量，截断为 16 位有符号。
    feedback.temperature = static_cast<int8_t>(motor.para.Tmos); // 温度：使用 MOS 管温度 Tmos，截断为 8 位有符号。
    feedback.online = BSP_MotorFeedbackIsOnline(channel, 100U); // 在线判定：该通道 100 个周期内是否有新反馈。
    return feedback;
}

// 每帧业务入口：构造本帧的 MotorFeedbackFrame 快照并发布。
// 参数 context：本帧上下文，其中 frame_id 为本轮调度编号，随发布一起写入以便下游判新旧。
// 返回：发布成功 Ok；端口未绑定等导致发布失败返回 Fault。
robot::framework::ProcessResult MotorFeedbackNode::process(robot::framework::FrameContext& context)
{
    MotorFeedbackFrame frame{}; // 当前帧的统一反馈快照；先整体默认初始化，再逐通道填充。
    // 按固定通道顺序（0~4）翻译各电机；未使用的通道 5~7 保持默认空值。
    frame.motor[0] = convertDji(kFrictionMotor0Id, 0U, shooter.fricMotor[0]); // 通道 0：左摩擦轮。
    frame.motor[1] = convertDji(kFrictionMotor1Id, 1U, shooter.fricMotor[1]); // 通道 1：右摩擦轮。
    frame.motor[2] = convertDji(kTriggerMotorId, 2U, shooter.triggerMotor); // 通道 2：拨弹电机。
    frame.motor[3] = convertDji(kTopYawMotorId, 3U, gimbal.top_yawMotor); // 通道 3：小 yaw 电机。
    frame.motor[4] = convertDm(kPitchMotorId, 4U, gimbal.pitchMotor); // 通道 4：DM pitch 电机。

    // 发布本帧快照；publish 会把 frame_id 一并写入信号，供下游按帧校验数据新鲜度。
    // 若端口未绑定（signal_ 为空），publish 返回 false，此时上报 Fault 让框架感知异常。
    if (!output_.publish(context.frame_id, frame)) {
        return robot::framework::ProcessResult::Fault;
    }
    return robot::framework::ProcessResult::Ok;
}

} // namespace robot::motor
