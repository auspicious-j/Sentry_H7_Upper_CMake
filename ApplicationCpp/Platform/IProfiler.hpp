#ifndef ROBOT_PROFILER_HPP
#define ROBOT_PROFILER_HPP

// 平台层只转出框架定义，避免业务节点依赖 STM32 类型。
#include "../Framework/IProfiler.hpp"

namespace robot::platform {
using IProfiler = robot::framework::IProfiler;
}

#endif /* ROBOT_PROFILER_HPP */
