#ifndef ROBOT_APPLICATION_HPP
#define ROBOT_APPLICATION_HPP

#include "Framework/FrameContext.hpp"
#include "Framework/PluginGraph.hpp"
#include "Framework/PluginNode.hpp"
#include "Platform/IClock.hpp"
#include "Platform/IProfiler.hpp"
#include "../Platform/STM32/Stm32Clock.hpp"
#include "../Platform/STM32/Stm32Profiler.hpp"

namespace robot::application {

// 组合根：负责创建插件图、初始化服务并向外提供一帧执行入口。
class RobotApplication {
public:
    RobotApplication();

    robot::framework::PluginStatus initialize();
    robot::framework::ProcessResult processFrame();
    bool initialized() const { return initialized_; }

private:
    class HeartbeatNode;

    // 当前骨架只有心跳节点；后续底盘、云台等节点从这里组装。
    robot::framework::PluginGraph graph_{};
    HeartbeatNode* heartbeat_{nullptr};
    robot::platform::stm32::Stm32Clock clock_{};
    robot::platform::stm32::Stm32Profiler profiler_;
    robot::framework::FrameContext frame_{};
    bool initialized_{false};
};

} // namespace robot::application

#endif /* ROBOT_APPLICATION_HPP */

