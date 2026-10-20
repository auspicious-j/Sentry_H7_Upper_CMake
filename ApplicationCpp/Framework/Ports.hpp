#ifndef robot_ports_hpp
#define robot_ports_hpp

#include <cstdint>

#include "PluginTypes.hpp" // 帧号哨兵 k_invalid_frame_id 定义于此

// =============================================================================
// 文件：Ports.hpp
// 作用：插件框架（plugin_graph）里"节点之间传递数据"的最小模型（端口/信号）。
//
// 先解释术语：
//   1) 帧（frame）：插件图每被调度执行一轮，就产生一个"帧"。
//      frame_id 是这一轮的编号，用来区分"这是第几轮的数据"。
//   2) 帧信号（frame_signal<t>）：一根"连线"的全部内容——值 + 写入帧号 +
//      有效标志。它是数据真正的存放处，一信号一写者、可多个读者。
//      组合根负责持有/创建信号，并把同一个对象以两种权限分发：
//        - 写方（生产者节点）：frame_signal<t>&（只有它能 publish）；
//        - 读方（消费者节点）：const frame_signal<t>&（只读，无法写）。
//      不再提供 output_port / input_port 句柄类：读写权限直接由
//      非 const 引用 / const 引用表达。
//
// 数据流向：
//   上游节点 --publish--> frame_signal（存放 值 + 帧号 + 有效位） --read--> 下游节点
//
// 为什么必须带帧号：机器人控制是多任务、异步的，下游节点必须能判断
// "我读到的这帧数据是不是本轮刚算出来的"，避免误用上一轮的旧值。
// 帧号与有效位随值一起保存；消费者自行选择严格（帧号相等）或宽松（只看有效位）。
// =============================================================================
namespace robot::framework 
{

    // frame_signal<t>：保存"一个端口在当前帧的最新值"的存储单元。
    // t 为信号的数据类型（例如自定义结构体、浮点数等）。
    // 它同时记录三样东西：值本身、写入时的帧号、是否有效（is_valid）。
    // 节点通过非 const 引用写入、通过 const 引用读出，本类即双方共享的中间存储。
    // 注意：本类不负责线程安全，同一帧内的读写顺序由插件图调度保证。
    template <typename t>
    class frame_signal 
    {
    public:
        // 写入（发布）当前帧的值。
        // 参数 frame_id：本次写入所属的帧号，供下游判断数据新旧。
        // 参数 value   ：要保存的信号值，按值拷贝进内部存储。
        // 副作用：更新 value_ 与 frame_id_，并把 is_valid_ 置为 true。
        void publish(uint32_t frame_id, const t& value)
        {
            value_ = value;
            frame_id_ = frame_id;
            is_valid_ = true;
        }

        // 使当前值失效（把 is_valid_ 置为 false）。
        // 保留原有 value_ 数值便于调试查看，但不允许再当作"当前有效输出"使用；
        // 通常用于本帧该端口没产生数据、或上游断链的情况。无参数、无返回值。
        void invalidate() 
        { 
            is_valid_ = false; 
        }

        // 读取内部保存的值，返回 const 引用（避免拷贝）。
        // 警告：不检查有效性，调用前应先用 is_valid() 判断；
        // 若从未发布过，返回的是 t 的默认构造值（value_{}）。
        const t& read() const
        {
            return value_;
        }

        // 返回该信号是否已有有效值（即 publish 过且之后未被 invalidate）。
        bool is_valid() const
        {
            return is_valid_;
        }

        // 返回最近一次 publish 时写入的帧号；从未发布过时为哨兵 k_invalid_frame_id。
        uint32_t frame_id() const
        {
            return frame_id_;
        }

    private:
        t value_{}; // 信号值本体；用 {} 默认初始化，保证未发布时也有确定初值。
        uint32_t frame_id_{k_invalid_frame_id}; // 最近一次写入对应的帧号；哨兵表示尚未写入。
        bool is_valid_{false}; // 是否已有有效值；false 表示从未写入或已被 invalidate。
    };

} // namespace robot::framework

#endif /* robot_ports_hpp */
