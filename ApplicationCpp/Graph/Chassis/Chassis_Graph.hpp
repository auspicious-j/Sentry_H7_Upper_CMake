#ifndef robot_chassis_graph_hpp
#define robot_chassis_graph_hpp

#include "Framework/PluginGraph.hpp"
#include "Framework/PluginNode.hpp"
#include "Framework/PluginTypes.hpp"
#include "STM32/Stm32Clock.hpp"
#include "STM32/Stm32Profiler.hpp"
#include "Gimbal_Graph_Config.h"
#include "Chassis_Types.hpp"
#include "Chassis_Input_Node.hpp"
#include "Chassis_Mode_Node.hpp"
#include "Chassis_Kinematics_Node.hpp"

namespace robot::Chassis
{
    class Chassis_Graph
    {
    public:
        Chassis_Graph();

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

        // 节点（ID 11..13，底盘图内唯一）。
        Chassis_Input_Node input_node_{11U};
        Chassis_Mode_Node mode_node_{12U};
        Chassis_Kinematics_Node kinematics_node_{13U};

        bool is_initialized_{false};
    };
}

#endif // robot_chassis_graph_hpp
