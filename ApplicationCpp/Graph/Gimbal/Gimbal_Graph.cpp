#include "Gimbal_Graph.hpp"
#include "PluginTypes.hpp"
#include "Stm32Profiler.hpp"

namespace robot::Gimbal
{

    robot::framework::plugin_status Gimbal_Graph::initialize()
    {
        if (is_initialized())
        {
            return robot::framework::plugin_status::invalid_state;
        }

        robot::framework::plugin_status status_ = robot::framework::plugin_status::ok;

        // M2 将在这里注册节点并连边：
        //   graph_.add(input_node_);
        //   graph_.add_dependency(input_node_, state_node_); ...
        // 当前图为空：空图 compile 合法（0 个节点），每帧 process_frame 返回 ok。
        status_ = graph_.set_plan_count(1U);
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