#ifndef ROBOT_FRAME_PROFILER_HPP
#define ROBOT_FRAME_PROFILER_HPP

#include <cstdint>

#include "PluginTypes.hpp"

namespace robot::framework {

class IProfiler {
public:
    virtual ~IProfiler() = default;
    virtual void beginFrame(uint32_t frame_id, uint64_t timestamp_us) = 0;
    virtual void beginNode(PluginId id) = 0;
    virtual void endNode() = 0;
    virtual void endFrame(uint64_t timestamp_us) = 0;
};

} // namespace robot::framework

#endif /* ROBOT_FRAME_PROFILER_HPP */

