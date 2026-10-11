#ifndef ROBOT_FRAME_PROFILER_HPP
#define ROBOT_FRAME_PROFILER_HPP

// ============================================================================
// IProfiler.hpp —— 性能探针（profiler）抽象接口
// ----------------------------------------------------------------------------
// 作用：给插件框架提供“计时打点”能力，用来统计每一帧、每个节点分别耗时多久，
//       便于排查机器人上到底是哪个环节拖慢了控制周期。
//
// 设计要点：本类是一个纯虚类（抽象接口），插件图只依赖这个抽象，不关心具体
//           怎么计时。真正读取硬件时钟的实现由 STM32 平台侧完成，从而把“框架逻辑”
//           和“硬件平台”解耦，方便以后更换平台或做离线回放测试。
//
// 术语：
//   - 探针 / profiler：性能测量工具的统称，这里指负责记录时间点的对象。
//   - 帧（frame）：插件图每执行一轮所有节点，称为一帧。
//   - 节点（node）：插件图中的一个处理单元（如底盘解算、电机控制）。
//
// 时间单位：本接口中所有时间戳参数均为“微秒（us）”。H7 主频约 480MHz，1us 内
//           可执行数百条指令，用微秒做单位分辨率已足够。
// ============================================================================

#include <cstdint>

#include "PluginTypes.hpp"

namespace robot::framework {

// 性能探针接口。插件图只依赖抽象接口，具体计时器由 STM32 平台实现。
class IProfiler {
public:
    // 虚析构保证接口指针销毁派生对象安全（通过基类指针 delete 派生对象时能正确析构）。
    virtual ~IProfiler() = default;
    // 记录一帧开始。
    // 参数 frame_id      ：帧号，与业务帧号一致，用于把统计结果对应到具体某一帧。
    // 参数 timestamp_us  ：本帧开始的时间戳，单位微秒（us）。
    virtual void beginFrame(uint32_t frame_id, uint64_t timestamp_us) = 0;
    // 记录某个节点开始执行。
    // 参数 id：节点 ID，用于区分是哪个节点的耗时。
    // 注意：实现应在此时“实时读取时钟”，而不是复用 beginFrame 传入的时间，
    //       否则会把整帧的起始时间误当作该节点的起始时间，导致统计错误。
    virtual void beginNode(PluginId id) = 0;
    // 记录当前节点结束（与最近一次 beginNode 配对，内部实时读时钟以算出该节点耗时）。
    virtual void endNode() = 0;
    // 记录一帧结束。
    // 参数 timestamp_us：本帧结束的时间戳，单位微秒（us）；与 beginFrame 配合可得到整帧耗时。
    virtual void endFrame(uint64_t timestamp_us) = 0;
};

} // namespace robot::framework

#endif /* ROBOT_FRAME_PROFILER_HPP */

