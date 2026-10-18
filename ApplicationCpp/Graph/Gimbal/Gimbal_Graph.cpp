#include "Gimbal_Graph.hpp"
#include "PluginTypes.hpp"
#include "Stm32Profiler.hpp"

namespace robot::Gimbal
{

    robot::framework::plugin_status Gimbal_Graph::initialize()
    {
        if (is_initialized() == 1)
        {
            return robot::framework::plugin_status::invalid_state;
        }

        robot::framework::plugin_status status_ = robot::framework::plugin_status::ok;

        if (status_ == robot::framework::plugin_status::ok)
        {
            status_ = 
        }

        is_initialized_ = (status_ == robot::framework::plugin_status::ok);

        return status_;
    }

    robot::framework::process_result Gimbal_Graph::process_frame()
    {
        if (is_initialized() == 1)
        {
            return robot::framework::process_result::fault;
        }

        profiler_.begin_frame();
        const robot::framework::process_result result = graph_.process_frame(frame_context_, &profiler_);
        profiler_.end_frame();

        frame_context_.frame_id++;

        return result;
    }
}