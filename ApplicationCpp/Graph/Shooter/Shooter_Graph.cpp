#include "Shooter_Graph.hpp"

#include "PluginTypes.hpp"
#include "Shooter_GraphApi.h"
#include "Stm32Profiler.hpp"

namespace robot::Shooter
{
    Shooter_Graph::Shooter_Graph() : profiler_(clock_)
    {
    }

    robot::framework::plugin_status Shooter_Graph::initialize()
    {
        if (is_initialized())
        {
            return robot::framework::plugin_status::invalid_state;
        }

        logic_node_.bind_inputs(input_node_.output());

        robot::framework::plugin_status status_ = robot::framework::plugin_status::ok;

        if (status_ == robot::framework::plugin_status::ok)
        {
            status_ = graph_.add(input_node_);
        }
        if (status_ == robot::framework::plugin_status::ok)
        {
            status_ = graph_.add(logic_node_);
        }
        if (status_ == robot::framework::plugin_status::ok)
        {
            status_ = graph_.add_dependency(input_node_, logic_node_);
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

    robot::framework::process_result Shooter_Graph::process_frame()
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
robot::Shooter::Shooter_Graph g_shooter_graph;
} // namespace

extern "C" void AppCpp_Shooter_Init(void)
{
    (void)g_shooter_graph.initialize();
}

extern "C" void AppCpp_Shooter_ProcessFrame(void)
{
    (void)g_shooter_graph.process_frame();
}
