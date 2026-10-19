#ifndef _GIMBAL_GRAPH_HPP_
#define _GIMBAL_GRAPH_HPP_

#include "Framework/PluginGraph.hpp"
#include "Framework/PluginNode.hpp"
#include "Framework/PluginTypes.hpp"
#include "STM32/Stm32Clock.hpp"
#include "STM32/Stm32Profiler.hpp"
#include "Gimbal_Graph_Config.h"
#include "Gimbal_Types.hpp"
#include "Gimbal_Input_Node.hpp"
#include "Gimbal_Target_Node.hpp"
#include "Gimbal_State_Node.hpp"
#include "Gimbal_Control_Node.hpp"
#include "Gimbal_Output_Node.hpp"

namespace robot::Gimbal
{
    class Gimbal_Graph
    {
    public:
        Gimbal_Graph();

        robot::framework::plugin_status initialize();

        robot::framework::process_result process_frame();

        bool is_initialized() const
        {
            return is_initialized_;
        }

    private:
        robot::framework::plugin_graph graph_;
        robot::platform::stm32::stm32_clock clock_;
        robot::platform::stm32::stm32_profiler<ROBOT_MAX_PLUGIN_NODES> profiler_;
        robot::framework::frame_context frame_context_;

        // 节点（ID 1..5，图内唯一）。
        Gimbal_Input_Node input_node_{1U};
        Gimbal_Target_Node target_node_{2U};
        Gimbal_State_Node state_node_{3U};
        Gimbal_Control_Node control_node_{4U};
        Gimbal_Output_Node output_node_{5U};

        bool is_initialized_{false};
    };
}

#endif // _GIMBAL_GRAPH_HPP_