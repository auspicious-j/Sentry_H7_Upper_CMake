#ifndef ROBOT_APPLICATION_HPP
#define ROBOT_APPLICATION_HPP

#include "Framework/FrameContext.hpp"
#include "Framework/PluginGraph.hpp"
#include "Framework/PluginNode.hpp"
#include "Nodes/Motor/MotorFeedbackNode.hpp"
#include "Nodes/Motor/MotorOfflineNode.hpp"
#include "robot_build_config.h"
#if ROBOT_ENABLE_PLAN_DEMO
#include "Nodes/Diagnostics/ExecutionPlanDemoNode.hpp"
static_assert(ROBOT_MAX_EXECUTION_PLANS >= 2U, "Plan demo needs two plans");
#endif
#if ROBOT_ENABLE_BRANCH_DEMO
#include "Nodes/Diagnostics/BranchDemoNode.hpp"
#endif
#if ROBOT_ENABLE_CHASSIS_OBSERVER
#include "Nodes/Chassis/ChassisFeedbackNode.hpp"
#include "Nodes/Chassis/ChassisObserverNode.hpp"
#include "../Platform/STM32/LegacyChassisFeedbackSource.hpp"
#endif
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
#if ROBOT_ENABLE_PLAN_DEMO
    // 帧结束复制当前计划与探针执行顺序。
    void updatePlanDebug();
#endif
#if ROBOT_ENABLE_BRANCH_DEMO
    // 同帧执行结束后复制分支状态，保留用户请求值。
    void updateBranchDebug();
#endif
#if ROBOT_ENABLE_CHASSIS_OBSERVER
    // 向 C 调试符号复制观察结果，不在业务节点中引用全局调试对象。
    void updateChassisDebug();
#endif

    // 组装心跳、电机状态和底盘反馈观察节点。
    // 根插件图。
    robot::framework::PluginGraph graph_{};
    // 心跳节点指针，实际对象为静态成员。
    HeartbeatNode* heartbeat_{nullptr};
    robot::motor::MotorFeedbackNode motor_feedback_{}; // 旧电机反馈到新端口的适配节点。
    robot::motor::MotorOfflineNode motor_offline_{}; // 电机反馈超时检测节点。
#if ROBOT_ENABLE_CHASSIS_OBSERVER
    robot::platform::stm32::LegacyChassisFeedbackSource chassis_source_{}; // 先于引用它的节点构造。
    robot::chassis::ChassisFeedbackNode chassis_feedback_{30U, chassis_source_}; // 板间反馈发布节点。
    robot::chassis::ChassisObserverNode chassis_observer_{31U, ROBOT_CHASSIS_FEEDBACK_TIMEOUT_MS}; // 底盘观察节点。
#endif
#if ROBOT_ENABLE_BRANCH_DEMO
    robot::diagnostics::BranchDemoNode branch_demo_{}; // 无硬件输出的分支演示图。
#endif
#if ROBOT_ENABLE_PLAN_DEMO
    robot::diagnostics::ExecutionPlanDemoNode plan_demo_{}; // 预编译顺序演示。
#endif
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
