#ifndef ROBOT_FRAME_PROFILER_HPP
#define ROBOT_FRAME_PROFILER_HPP

#include <cstdint>

#include "PluginTypes.hpp"

namespace robot::framework {

// 性能探针接口。插件图只依赖抽象接口，具体计时器由 STM32 平台实现。
class IProfiler {
public:
    // 虚析构保证接口指针销毁派生对象安全。
    virtual ~IProfiler() = default;
    // 记录一帧开始；frame_id 与业务帧号一致。
    virtual void beginFrame(uint32_t frame_id, uint64_t timestamp_us) = 0;
    // 节点内部实时读取时钟，避免把整帧起始时间误当作节点起始时间。
    virtual void beginNode(PluginId id) = 0;
    // 记录当前节点结束。
    virtual void endNode() = 0;
    // 记录一帧结束。
    virtual void endFrame(uint64_t timestamp_us) = 0;
};

} // namespace robot::framework

#endif /* ROBOT_FRAME_PROFILER_HPP */

