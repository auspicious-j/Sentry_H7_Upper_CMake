#ifndef robot_stm32_profiler_hpp
#define robot_stm32_profiler_hpp

#include "../../ApplicationCpp/Platform/IClock.hpp"
#include "../../ApplicationCpp/Framework/IProfiler.hpp"

#include <cstddef>
#include <cstdint>

namespace robot::platform::stm32 
{
    // 基于 STM32 单调时钟的性能探针。
    // 模板参数 Max_Nodes：本图节点数上限，决定逐节点耗时表 node_us_ 的容量；
    // 每个 graph 可按自身规模实例化不同大小（如 stm32_profiler<8> / stm32_profiler<32>）。
    template <std::size_t Max_Nodes>
    class stm32_profiler final : public robot::framework::i_profiler 
    {
    public:
        // 构造函数：注入一个单调时钟源（定义见 Stm32Profiler.cpp）。
        // 参数 clock：平台无关的单调时钟接口（STM32 侧由 DWT 实现）；仅保存其引用，不拷贝。
        explicit stm32_profiler(const robot::platform::i_clock& clock);

        // 四个打点函数（定义见 Stm32Profiler.cpp：模板成员定义 + 文件末尾显式实例化）。
        // begin_frame/end_frame：帧起点、终点实时读时钟求差，得到整帧耗时。
        // begin_node/end_node  ：节点起点、终点实时读时钟求差，写入逐节点耗时表 node_us_[id]。
        void begin_frame() override;
        void begin_node() override;
        void end_node(robot::framework::plugin_id id) override;
        void end_frame() override;

    private:
        // 抽象的单调时钟引用：依赖接口而非具体实现，便于将来替换计时后端。
        const robot::platform::i_clock& clock_;
        // 帧开始时间
        uint64_t frame_start_us_{0U};
        // 帧耗时
        uint64_t frame_us_{0U};
        // 节点开始时间
        uint64_t node_start_us_{0U};
        // 节点耗时
        uint64_t node_us_[Max_Nodes]{0U};
    };

} // namespace robot::platform::stm32

#endif /* robot_stm32_profiler_hpp */


