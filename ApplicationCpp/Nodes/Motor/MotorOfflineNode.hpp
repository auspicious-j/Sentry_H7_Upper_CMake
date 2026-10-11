#ifndef ROBOT_MOTOR_OFFLINE_NODE_HPP
#define ROBOT_MOTOR_OFFLINE_NODE_HPP

// =============================================================================
// 文件：MotorOfflineNode.hpp
// 作用：定义「电机掉线检测节点」MotorOfflineNode。
//
// 背景（CAN 与掉线概念）：
//   1) CAN（Controller Area Network）是机器人上常用的现场总线，多台电机挂在同一条
//      总线上，各电机周期性广播自己的状态反馈帧（角度、速度、电流、温度等）。
//   2) 上位机（本工程）通过 BSP 层接收这些反馈帧，并记录「每个电机通道最后一次
//      收到反馈的时间戳」（HAL tick，单位 ms）。
//   3) 若某电机长时间收不到反馈，就说明它掉线了（断电、线缆断裂、总线故障等）。
//      本节点做的事情就是：在每一帧里逐个检查各通道的反馈是否在阈值时间内到达，
//      把「在线/离线」结果打包成 MotorOnlineFrame，通过输出端口发布给下游使用。
//
// 判断依据：BSP_MotorFeedbackIsOnline(channel, timeout_ms)
//   —— 若该通道从未收到过反馈，或「当前时间 - 最后反馈时间 > 阈值」，则判为离线。
//
// 数据流：
//   电机 --CAN 反馈--> BSP 记录时间戳 --> 本节点按超时判定 --> OutputPort 发布
//   MotorOnlineFrame{online[8]} --> 下游节点（安全逻辑、状态上报等）读取。
// =============================================================================

#include "../../Framework/PluginNode.hpp"
#include "../../Framework/Ports.hpp"
#include "MotorData.hpp"

namespace robot::motor {

// 根据 BSP 接收时间戳发布电机在线状态的节点。
// 说明：这是一个「叶子节点」（没有内部子图），每帧执行一次检测。
//       它不直接读 CAN 报文，只依赖 BSP 层维护的「最后反馈时间戳」来判断在线与否。
//       继承自 robot::framework::PluginNode，遵循框架的生命周期
//       （compose / configure / start / process / stop 等），本节点只重写 process()。
// 线程/时序：由插件图统一调度，process() 在每个执行周期被调用一次。
class MotorOfflineNode final : public robot::framework::PluginNode {
public:
    // 构造函数：创建节点并设置掉线超时阈值。
    // 参数 timeout_ms：判定「在线」的时间窗口，单位毫秒(ms)；默认 100ms。
    //   含义：若某通道距最后一次收到反馈的时间不超过 timeout_ms，就视为在线；
    //         超过则视为离线。100ms 对应「约 10Hz 反馈周期 + 少量余量」的常见取值。
    // 说明：基类构造函数 PluginNode(21U) 传入本节点的唯一 ID = 21。
    explicit MotorOfflineNode(uint32_t timeout_ms = 100U);

    // 返回在线状态输出端口。
    // 用途：供框架/装配代码把该端口与下游节点的输入端口相连，以读取 MotorOnlineFrame。
    // 返回值：本节点内部 output_ 端口的引用（可绑定、可发布）。
    robot::framework::OutputPort<MotorOnlineFrame>& output();

    // 返回在线状态信号存储。
    // 用途：通常供框架或测试代码直接查看本节点已发布的最近一帧在线状态。
    // 返回值：内部 signal_ 的 const 引用（只读，不能通过它写入）。
    const robot::framework::FrameSignal<MotorOnlineFrame>& signal() const;

    // 每帧检查所有电机反馈是否超时。
    // 参数 context：本帧的时间上下文（帧号 frame_id、周期 period_us、时间戳 timestamp_us），
    //               同帧内所有节点共享；本节点用其中的 frame_id 作为发布数据所属的帧号。
    // 返回值：ProcessResult::Ok    表示本帧检测并发布成功；
    //         ProcessResult::Fault 表示输出端口未绑定（publish 失败），节点进入故障。
    // 生命周期时机：由插件图在运行期每个执行周期调用一次。
    robot::framework::ProcessResult process(robot::framework::FrameContext& context) override;

private:
    uint32_t timeout_ms_{100U}; // 反馈超时时间：判定在线的时间窗口，单位 ms；默认 100ms（可由构造函数覆盖）。
    robot::framework::FrameSignal<MotorOnlineFrame> signal_{}; // 在线状态信号：保存本节点最近一帧发布的 MotorOnlineFrame（值 + 帧号 + 有效性）。
    robot::framework::OutputPort<MotorOnlineFrame> output_{}; // 在线状态输出端口：绑定到上面的 signal_，process() 通过它发布数据。
};

} // namespace robot::motor

#endif /* ROBOT_MOTOR_OFFLINE_NODE_HPP */
