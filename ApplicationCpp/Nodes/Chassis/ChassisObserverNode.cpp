#include "ChassisObserverNode.hpp"

namespace robot::chassis {

// 构造函数：保存 id 和 timeout_ms，并把输出端口接到结果存储上。
// 时机：搭图阶段只创建一次；输入端口此时尚未接线，由组装层随后 bind。
// 说明：output_ 绑定到 signal_ 后，process() 才能通过 output_.publish 写入结果。
chassis_observer_node::chassis_observer_node(robot::framework::plugin_id id, uint32_t timeout_ms)
    : plugin_node(id), timeout_ms_(timeout_ms)
{
    output_.bind(signal_);
}

// 暴露输入绑定口：返回可变引用，供组装层把下板反馈源绑上来。
// 时机：搭图阶段调用；运行期不应再改绑定。
// 返回：本节点内部的下板反馈 input_port 引用。
robot::framework::input_port<chassis_feedback>& chassis_observer_node::input()
{
    return input_;
}

// 提供下游只读结果：返回内部结果存储的 const 引用。
// 时机：任意时刻可读，常见于调试窗口或下游诊断节点。
// 返回：保存最新 chassis_observation 的帧信号（只读）。
const robot::framework::frame_signal<chassis_observation>& chassis_observer_node::signal() const
{
    return signal_;
}

// 【每帧执行】判断板间反馈链路在线并发布观察结果。
// 参数 context：当前帧上下文，其中 frame_id 用于校验上游数据是否属于本帧。
// 逻辑：
//   1) 先构造一个默认（离线）观察结果，并把本帧帧号写入；这样即使没有有效上游，
//      也会发布一份「离线」结果，而不是残留上一帧的旧状态。
//   2) 仅当输入端口已绑定且有效、且上游写入的帧号恰为本帧号时，才读取反馈快照，
//      并计算在线状态：is_received（曾收到过反馈）且 age_ms <= timeout_ms_（未超时）。
//   3) 无论在线与否都发布结果，避免上一次的「在线」结果在缺少上游时残留。
// 注意：无反馈与已超时都只是「观察结果」，不会触发旧框架的控制类故障。
// 返回：publish 成功为 ok；若输出端口未绑定（本类构造时已 bind，正常不会发生）则为 fault。
robot::framework::process_result chassis_observer_node::process(robot::framework::frame_context& context)
{
    chassis_observation observation{}; // 本帧默认离线结果（is_online=false，反馈全为默认零值）。
    observation.frame_id = context.frame_id; // 记录本结果对应的插件帧号。
    if (input_.is_valid() && input_.frame_id() == context.frame_id)
    { // 上游已绑有效且提交的是本帧数据。
        observation.feedback = input_.read(); // 取出下板反馈快照。
        observation.is_online = observation.feedback.is_received // 前提：曾经收到过反馈；
            && observation.feedback.age_ms <= timeout_ms_; // 且样本年龄未超过超时阈值（未超时）。
    }
    // 每帧发布，避免前次在线结果在缺少上游时残留。
    return output_.publish(context.frame_id, observation)
        ? robot::framework::process_result::ok : robot::framework::process_result::fault;
}

} // namespace robot::chassis
