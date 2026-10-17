#include "Stm32Profiler.hpp"

#include "Gimbal_Graph_Config.h"

namespace robot::platform::stm32 
{
    // 构造函数：注入一个单调时钟源（仅保存引用，不拷贝）。
    template <std::size_t Max_Nodes>
    stm32_profiler<Max_Nodes>::stm32_profiler(const robot::platform::i_clock& clock)
        : clock_(clock)
    {
    }

    // 开始记录帧时间：实时读取时钟作为帧起点。
    template <std::size_t Max_Nodes>
    void stm32_profiler<Max_Nodes>::begin_frame()
    {
        frame_start_us_ = clock_.now_us();
    }

    // 开始记录节点时间：实时读取时钟作为节点起点，而不是复用帧起点。
    template <std::size_t Max_Nodes>
    void stm32_profiler<Max_Nodes>::begin_node()
    {
        node_start_us_ = clock_.now_us();
    }

    // 结束记录当前节点：校验配对与时间顺序，耗时按 id 写入逐节点耗时表。
    template <std::size_t Max_Nodes>
    void stm32_profiler<Max_Nodes>::end_node(robot::framework::plugin_id id)
    {
        const uint64_t timestamp_us = clock_.now_us();
        // 配对失败（没有正在计时的节点）或时钟回绕（结束早于开始）时放弃本次统计。
        if (timestamp_us < node_start_us_)
        {
            return;
        }
        // id 越界时不写数据（坚持"不写假值"），但仍结束本次计时会话。
        if (static_cast<std::size_t>(id) < Max_Nodes)
        {
            node_us_[static_cast<std::size_t>(id)] = timestamp_us - node_start_us_;
        }
    }

    // 结束帧计时：与帧起点求差。
    template <std::size_t Max_Nodes>
    void stm32_profiler<Max_Nodes>::end_frame()
    {
        const uint64_t timestamp_us = clock_.now_us();
        // 时钟回绕时放弃本次统计
        if (timestamp_us < frame_start_us_)
        {
            return;
        }
        frame_us_ = timestamp_us - frame_start_us_;
    }

    template class stm32_profiler<ROBOT_MAX_PLUGIN_NODES>;

} // namespace robot::platform::stm32
