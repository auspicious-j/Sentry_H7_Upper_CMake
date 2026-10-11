#ifndef ROBOT_PROFILER_HPP
#define ROBOT_PROFILER_HPP

// ============================================================================
// IProfiler.hpp（平台层）—— 把框架的性能探针接口“转出”到平台命名空间
// ----------------------------------------------------------------------------
// 作用：本文件不定义任何新类型，只把框架层 Framework/IProfiler.hpp 中定义的
//       IProfiler 重新导出到 robot::platform 命名空间，扮演一个“别名转发/门面”。
//
// 为何这样设计（分层解耦、便于移植）：
//   - 若业务/平台代码直接 #include 框架头文件，就会硬编码依赖“框架的内部目录
//     路径”；一旦框架目录调整，所有包含处都得跟着改。
//   - 这里给上层提供一个稳定的平台入口 robot::platform::IProfiler：上层只认
//     平台层这一处，将来框架文件挪动位置时，只需修改本文件一行即可。
//   - 代价极低：using 是编译期别名，不产生任何运行时代码或对象，只是改了名字。
//
// 术语：
//   - using 别名声明：给一个已存在的类型在其命名空间中另起一个名字
//     （此处是跨命名空间转发，robot::framework 的名字在本命名空间可见）。
//   - 探针 / profiler：负责“计时打点”的对象，用来统计每帧、每个节点的耗时。
//   - IProfiler：框架中负责耗时统计的抽象接口，其完整定义见被包含的框架头文件。
// ============================================================================

// 平台层只转出框架定义，避免业务节点因直接包含框架头而依赖 STM32 相关类型。
#include "../Framework/IProfiler.hpp"

namespace robot::platform {
// 将框架命名空间中的 robot::framework::IProfiler 引入本命名空间：
// 之后在本层写 robot::platform::IProfiler 就等价于 robot::framework::IProfiler。
using IProfiler = robot::framework::IProfiler;
}

#endif /* ROBOT_PROFILER_HPP */
