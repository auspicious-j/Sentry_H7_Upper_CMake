#include "Gimbal_Graph.hpp"
#include "Gimbal_GraphApi.h"
#include "PluginTypes.hpp"
#include "Stm32Profiler.hpp"

namespace robot::Gimbal
{
    Gimbal_Graph::Gimbal_Graph() : profiler_(clock_)
    {
    }

    robot::framework::plugin_status Gimbal_Graph::initialize()
    {
        if (is_initialized())
        {
            return robot::framework::plugin_status::invalid_state;
        }

        // 组装
        input_node_.bind_vision_target(vision_target_node_.output());
        target_node_.bind_inputs(input_node_.output());
        target_node_.bind_state(state_node_.output());
        state_node_.bind_inputs(input_node_.output());
        state_node_.bind_targets(target_node_.output());
        control_node_.bind_inputs(input_node_.output());
        control_node_.bind_state(state_node_.output());
        output_node_.bind_state(state_node_.output());

        robot::framework::plugin_status status_ = robot::framework::plugin_status::ok;

        // 注册节点
        if (status_ == robot::framework::plugin_status::ok)
        {
            status_ = graph_.add(input_node_);
        }
        if (status_ == robot::framework::plugin_status::ok)
        {
            status_ = graph_.add(target_node_);
        }
        if (status_ == robot::framework::plugin_status::ok)
        {
            status_ = graph_.add(state_node_);
        }
        if (status_ == robot::framework::plugin_status::ok)
        {
            status_ = graph_.add(control_node_);
        }
        if (status_ == robot::framework::plugin_status::ok)
        {
            status_ = graph_.add(output_node_);
        }
        if (status_ == robot::framework::plugin_status::ok)
        {
            status_ = graph_.add(vision_target_node_);
        }

        // 依赖边
        if (status_ == robot::framework::plugin_status::ok)
        {
            status_ = graph_.add_dependency(vision_target_node_, input_node_);
        }
        if (status_ == robot::framework::plugin_status::ok)
        {
            status_ = graph_.add_dependency(input_node_, target_node_);
        }
        if (status_ == robot::framework::plugin_status::ok)
        {
            status_ = graph_.add_dependency(target_node_, state_node_);
        }
        if (status_ == robot::framework::plugin_status::ok)
        {
            status_ = graph_.add_dependency(state_node_, control_node_);
        }
        if (status_ == robot::framework::plugin_status::ok)
        {
            status_ = graph_.add_dependency(control_node_, output_node_);
        }

        // 配置与启动
        if (status_ == robot::framework::plugin_status::ok)
        {
            status_ = graph_.set_plan_count(1U);
        }
        if (status_ == robot::framework::plugin_status::ok)
        {
            // compile() 内部会先 compose()（展开子图），再做校验与拓扑排序并冻结图。
            status_ = graph_.compile();
        }
        if (status_ == robot::framework::plugin_status::ok)
        {
            status_ = graph_.configure_all();
        }
        if (status_ == robot::framework::plugin_status::ok)
        {
            status_ = graph_.start_all();
        }

        is_initialized_ = (status_ == robot::framework::plugin_status::ok);

        return status_;
    }

    robot::framework::process_result Gimbal_Graph::process_frame()
    {
        if (!is_initialized())
        {
            return robot::framework::process_result::fault;
        }

        // 帧号与时间戳必须在节点运行前更新：本帧内所有节点看到同一个 frame_id。
        // frame_id 初值是哨兵 k_invalid_frame_id，首次自增后是合法帧号 0。
        frame_context_.frame_id++;
        frame_context_.timestamp_us = clock_.now_us();

        profiler_.begin_frame();
        const robot::framework::process_result result = graph_.process_frame(frame_context_, &profiler_);
        profiler_.end_frame();

        return result;
    }
}

namespace
{
// 图对象独占：C 侧只通过 AppCpp_Gimbal_* 两个函数访问。
robot::Gimbal::Gimbal_Graph g_gimbal_graph;
} // namespace

extern "C" void AppCpp_Gimbal_Init(void)
{
    (void)g_gimbal_graph.initialize();
}

extern "C" void AppCpp_Gimbal_ProcessFrame(void)
{
    (void)g_gimbal_graph.process_frame();
}