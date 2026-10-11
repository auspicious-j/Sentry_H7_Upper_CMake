#ifndef ROBOT_CHASSIS_OBSERVER_NODE_HPP
#define ROBOT_CHASSIS_OBSERVER_NODE_HPP

/*
 * ChassisObserverNode.hpp —— 底盘「观察者」节点（头文件）。
 *
 * 一、这个节点干什么
 *   它是底盘迁移到新插件框架的第一步：只「观察」，不「控制」。
 *   每帧读取下板（底盘板）通过板间链路回传的反馈帧（ChassisFeedback），
 *   判断这条反馈链路当前是否「在线」，并把结论（ChassisObservation）发布出去，
 *   供诊断/上位机显示使用。它不会生成任何轮子电机命令，因此接入它不会改变
 *   底盘运动行为，是安全的旁路节点。
 *
 * 二、三个关键概念
 *   1) 接收计数 receive_count：下板真实收到反馈包的累计次数（不是插件帧号）。
 *      它由下板/通信层维护，持续增长说明链路在动；停滞则说明链路已断。
 *   2) 样本年龄 age_ms：最近一次收到的反馈样本距今的毫秒数。
 *      只有 received == true（曾经收到过）时才有意义；它反映「数据有多旧」。
 *   3) 超时阈值 timeout_ms：构造时传入的一个毫秒门限。当样本年龄超过它，
 *      就认为链路「超时/离线」。判据：online = received && age_ms <= timeout_ms。
 *
 * 三、生命周期时机
 *   本节点是普通叶子节点（无内部子图）：构造后由框架依次调用 compose/configure/
 *   start，进入运行态后每个执行周期触发一次 process()（真正干活的地方）。
 *   process() 要求上游（板间反馈源）已经在本帧发布了同帧号的快照，
 *   否则本帧视为「离线」，且发布一个默认离线结果，避免上一帧的在线状态残留。
 */

#include "Framework/PluginNode.hpp"
#include "Framework/Ports.hpp"
#include "ChassisData.hpp"

namespace robot::chassis {

// 底盘迁移第一步：消费新端口并判断链路在线，不控制轮子。
// 这是一个「只读观察」节点：输入下板反馈，输出在线/离线结论，无任何执行器输出。
class ChassisObserverNode final : public robot::framework::PluginNode {
public:
    // 构造函数。
    // 参数 id         ：本节点在图内的唯一编号（PluginId），由组装层保证全局唯一。
    // 参数 timeout_ms ：板间反馈超时阈值（毫秒）；样本年龄 age_ms 超过它即判为离线。
    // 时机：由组装层在搭图时创建；此时不做任何业务，仅保存参数并接线输出。
    ChassisObserverNode(robot::framework::PluginId id, uint32_t timeout_ms);
    // 返回输入端口（可写句柄），由组装层把下板反馈源绑到这个端口上。
    // 时机：搭图阶段调用一次，用于连接上游信号存储。
    robot::framework::InputPort<ChassisFeedback>& input();
    // 返回只读观察结果（帧信号），供下游节点或调试窗口读取最新在线结论。
    // 内容：本帧的 ChassisObservation（含反馈快照、帧号、online 标志）。
    const robot::framework::FrameSignal<ChassisObservation>& signal() const;
    // 【生命周期第 5 步】每帧业务入口：判断链路在线并发布观察结果。
    // 参数 context：当前帧的时间上下文（含 frame_id 等），本帧所有节点共享。
    // 要求：上游须已提交「同帧号」的反馈快照，否则本帧按离线处理。
    // 返回：发布成功返回 Ok；发布失败（如未绑定输出）返回 Fault。
    robot::framework::ProcessResult process(robot::framework::FrameContext& context) override;

private:
    uint32_t timeout_ms_; // 观测超时阈值，毫秒；判在线的门限（构造时设定）。
    robot::framework::InputPort<ChassisFeedback> input_{}; // 下板反馈输入；由组装层 bind 上游信号。
    robot::framework::FrameSignal<ChassisObservation> signal_{}; // 观察结果存储（值+帧号+有效位）。
    robot::framework::OutputPort<ChassisObservation> output_{}; // 观察结果写端；构造时 bind 到 signal_。
};

} // namespace robot::chassis
#endif // ROBOT_CHASSIS_OBSERVER_NODE_HPP
