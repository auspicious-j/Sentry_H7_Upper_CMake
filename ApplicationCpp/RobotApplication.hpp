#ifndef ROBOT_APPLICATION_HPP
#define ROBOT_APPLICATION_HPP

#include "Framework/FrameContext.hpp"
#include "Framework/PluginGraph.hpp"
#include "Framework/PluginNode.hpp"
#include "Nodes/Motor/MotorFeedbackNode.hpp"
#include "Nodes/Motor/MotorOfflineNode.hpp"
#include "Platform/IClock.hpp"
#include "Platform/IProfiler.hpp"
#include "../Platform/STM32/Stm32Clock.hpp"
#include "../Platform/STM32/Stm32Profiler.hpp"

namespace robot::application {

// 组合根：负责创建插件图、初始化服务并向外提供一帧执行入口。
class RobotApplication {
public:
    // 创建应用组合根。
    RobotApplication();

    // 组装并启动插件图。
    robot::framework::PluginStatus initialize();
    // 执行一帧插件业务。
    robot::framework::ProcessResult processFrame();
    // 查询应用是否初始化成功。
    bool initialized() const { return initialized_; }

private:
    class HeartbeatNode;

    // 当前骨架只有心跳节点；后续底盘、云台等节点从这里组装。
    // 根插件图。
    robot::framework::PluginGraph graph_{};
    // 心跳节点指针，实际对象为静态成员。
    HeartbeatNode* heartbeat_{nullptr};
    robot::motor::MotorFeedbackNode motor_feedback_{}; // 旧电机反馈到新端口的适配节点。
    robot::motor::MotorOfflineNode motor_offline_{}; // 电机反馈超时检测节点。
    // STM32 单调时钟。
    robot::platform::stm32::Stm32Clock clock_{};
    // STM32 性能统计器。
    robot::platform::stm32::Stm32Profiler profiler_;
    // 当前帧上下文。
    robot::framework::FrameContext frame_{};
    // 应用是否完成启动。
    bool initialized_{false};
};

} // namespace robot::application

#endif /* ROBOT_APPLICATION_HPP */
