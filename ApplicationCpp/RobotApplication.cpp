#include "RobotApplication.hpp"

#include "Platform/DebugSnapshot.hpp"

namespace robot::application {

// 骨架阶段的无硬件节点，用来验证调度、计时和调试快照。
class RobotApplication::HeartbeatNode final : public robot::framework::PluginNode {
public:
    HeartbeatNode() : PluginNode(1U) {}

    // context：当前 1 kHz 帧上下文。
    robot::framework::ProcessResult process(robot::framework::FrameContext&) override
    {
        return robot::framework::ProcessResult::Ok;
    }
};

// 成员对象静态存在，不在运行期 new。
RobotApplication::RobotApplication()
    : profiler_(clock_)
{
    // heartbeat_node：静态心跳节点，不产生硬件输出。
    static HeartbeatNode heartbeat_node;
    heartbeat_ = &heartbeat_node;
}

// 启动阶段依次完成注册、展开、验证、拓扑排序、配置和启动。
robot::framework::PluginStatus RobotApplication::initialize()
{
    if (initialized_) {
        return robot::framework::PluginStatus::InvalidState;
    }

    // status：图组装和启动状态。
    robot::framework::PluginStatus status = graph_.add(*heartbeat_);
    if (status == robot::framework::PluginStatus::Ok) {
        status = graph_.compile();
    }
    if (status == robot::framework::PluginStatus::Ok) {
        status = graph_.configureAll();
    }
    if (status == robot::framework::PluginStatus::Ok) {
        status = graph_.startAll();
    }

    g_robot_debug.graph_node_count = graph_.nodeCount();
    g_robot_debug.graph_edge_count = graph_.edgeCount();
    g_robot_debug.init_error = static_cast<uint32_t>(status);
    g_robot_debug.initialized = (status == robot::framework::PluginStatus::Ok) ? 1U : 0U;
    g_robot_debug.running = (status == robot::framework::PluginStatus::Ok) ? 1U : 0U;

    initialized_ = (status == robot::framework::PluginStatus::Ok);
    return status;
}

// 每次被 FreeRTOS 控制任务唤醒时执行一帧插件图。
robot::framework::ProcessResult RobotApplication::processFrame()
{
    if (!initialized_) {
        return robot::framework::ProcessResult::Fault;
    }

    frame_.frame_id++;
    frame_.timestamp_us = clock_.nowUs();
    profiler_.beginFrame(frame_.frame_id, frame_.timestamp_us);
    // result：本帧插件图汇总结果。
    const robot::framework::ProcessResult result = graph_.processFrame(frame_, &profiler_);
    frame_.timestamp_us = clock_.nowUs();
    profiler_.endFrame(frame_.timestamp_us);
    g_robot_debug.last_result = static_cast<uint8_t>(result);
    return result;
}

} // namespace robot::application
