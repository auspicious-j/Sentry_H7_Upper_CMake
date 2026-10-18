#include "Chassis_Graph.hpp"

#include "Chassis_GraphApi.h"
#include "PluginTypes.hpp"
#include "Stm32Profiler.hpp"

namespace robot::Chassis
{
    Chassis_Graph::Chassis_Graph() : profiler_(clock_)
    {
    }

    robot::framework::plugin_status Chassis_Graph::initialize()
    {
        if (is_initialized())
        {
            return robot::framework::plugin_status::invalid_state;
        }

        mode_node_.bind_inputs(input_node_.output());
        kinematics_node_.bind_inputs(input_node_.output());
        kinematics_node_.bind_mode(mode_node_.output());

        robot::framework::plugin_status status_ = robot::framework::plugin_status::ok;

        if (status_ == robot::framework::plugin_status::ok)
        {
            status_ = graph_.add(input_node_);
        }
        if (status_ == robot::framework::plugin_status::ok)
        {
            status_ = graph_.add(mode_node_);
        }
        if (status_ == robot::framework::plugin_status::ok)
        {
            status_ = graph_.add(kinematics_node_);
        }

        if (status_ == robot::framework::plugin_status::ok)
        {
            status_ = graph_.add_dependency(input_node_, mode_node_);
        }
        if (status_ == robot::framework::plugin_status::ok)
        {
            status_ = graph_.add_dependency(mode_node_, kinematics_node_);
        }

        if (status_ == robot::framework::plugin_status::ok)
        {
            status_ = graph_.set_plan_count(1U);
        }
        if (status_ == robot::framework::plugin_status::ok)
        {
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

    robot::framework::process_result Chassis_Graph::process_frame()
    {
        if (!is_initialized())
        {
            return robot::framework::process_result::fault;
        }

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
robot::Chassis::Chassis_Graph g_chassis_graph;
} // namespace

extern "C" void AppCpp_Chassis_Init(void)
{
    (void)g_chassis_graph.initialize();
}

extern "C" void AppCpp_Chassis_ProcessFrame(void)
{
    (void)g_chassis_graph.process_frame();
}
