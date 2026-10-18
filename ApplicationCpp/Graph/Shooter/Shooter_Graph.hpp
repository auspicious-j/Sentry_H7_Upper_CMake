#ifndef robot_shooter_graph_hpp
#define robot_shooter_graph_hpp

#include "Framework/PluginGraph.hpp"
#include "Framework/PluginNode.hpp"
#include "Framework/PluginTypes.hpp"
#include "STM32/Stm32Clock.hpp"
#include "STM32/Stm32Profiler.hpp"
#include "Gimbal_Graph_Config.h"
#include "Shooter_Types.hpp"
#include "Shooter_Input_Node.hpp"
#include "Shooter_Logic_Node.hpp"

namespace robot::Shooter
{
    class Shooter_Graph
    {
    public:
        Shooter_Graph();

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

        // 节点（ID 21..22，发射图内唯一）。
        Shooter_Input_Node input_node_{21U};
        Shooter_Logic_Node logic_node_{22U};

        bool is_initialized_{false};
    };
}

#endif // robot_shooter_graph_hpp
