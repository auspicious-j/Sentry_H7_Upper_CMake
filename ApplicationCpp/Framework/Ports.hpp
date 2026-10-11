#ifndef ROBOT_PORTS_HPP
#define ROBOT_PORTS_HPP

#include <cstdint>

// =============================================================================
// 文件：Ports.hpp
// 作用：插件框架（PluginGraph）里"节点之间传递数据"的最小模型（端口/信号）。
//
// 先解释几个新术语：
//   1) 帧（frame）：插件图每被调度执行一轮，就产生一个"帧"。
//      frame_id 是这一轮的编号，用来区分"这是第几轮的数据"。
//   2) 帧信号（FrameSignal<T>）：一块存储，保存"某个端口在本帧的最新值"。
//      它是数据真正的存放处，本身不区分输入还是输出。
//   3) 输出端口（OutputPort<T>）：节点向外发布结果用的句柄，
//      同一时刻只允许一个节点写入（bind 一次、publish 多次）。
//   4) 输入端口（InputPort<T>）：节点读取上游结果用的句柄，只读，
//      同一块信号可以被多个节点同时持有。
//
// 数据流向：
//   上游节点 --publish--> OutputPort --> FrameSignal（存放 值 + 帧号）
//   FrameSignal --被读取--> InputPort --> 下游节点
//
// 为什么必须带帧号：机器人控制是多任务、异步的，下游节点必须能判断
// "我读到的这帧数据是不是本轮刚算出来的"，避免误用上一轮的旧值，
// 因此提供了 validForFrame() 这类按帧号校验的接口。
// =============================================================================
namespace robot::framework {

// FrameSignal<T>：保存"一个端口在当前帧的最新值"的存储单元。
// T 为信号的数据类型（例如自定义结构体、浮点数等）。
// 它同时记录三样东西：值本身、写入时的帧号、是否有效（valid）。
// 节点通过 OutputPort 写入、通过 InputPort 读出，本类即双方共享的中间存储。
// 注意：本类不负责线程安全，同一帧内的读写顺序由插件图调度保证。
template <typename T>
class FrameSignal {
public:
    // 写入（发布）当前帧的值。
    // 参数 frame_id：本次写入所属的帧号，供下游判断数据新旧。
    // 参数 value   ：要保存的信号值，按值拷贝进内部存储。
    // 副作用：更新 value_ 与 frame_id_，并把 valid_ 置为 true。
    void publish(uint32_t frame_id, const T& value)
    {
        value_ = value;
        frame_id_ = frame_id;
        valid_ = true;
    }

    // 使当前值失效（把 valid_ 置为 false）。
    // 保留原有 value_ 数值便于调试查看，但不允许再当作"当前有效输出"使用；
    // 通常用于本帧该端口没产生数据、或上游断链的情况。无参数、无返回值。
    void invalidate() { valid_ = false; }

    // 读取内部保存的值，返回 const 引用（避免拷贝）。
    // 警告：不检查有效性，调用前应先用 valid() 判断；
    // 若从未发布过，返回的是 T 的默认构造值（value_{}）。
    const T& read() const
    {
        return value_;
    }

    // 返回该信号是否已有有效值（即 publish 过且之后未被 invalidate）。
    bool valid() const
    {
        return valid_;
    }

    // 返回最近一次 publish 时写入的帧号；从未发布过时为初值 0。
    uint32_t frameId() const
    {
        return frame_id_;
    }

private:
    T value_{}; // 信号值本体；用 {} 默认初始化，保证未发布时也有确定初值。
    uint32_t frame_id_{0U}; // 最近一次写入对应的帧号；0 表示尚未写入。
    bool valid_{false}; // 是否已有有效值；false 表示从未写入或已被 invalidate。
};

// OutputPort<T>：输出端口句柄，节点用它把本节点的计算结果发布出去。
// 一个输出端口同一时刻只允许一个节点写入（谁 bind 谁写入），
// 这样才能保证一块 FrameSignal 不会被两个节点同时修改。
template <typename T>
class OutputPort {
public:
    // 将输出端口绑定到一块固定的信号存储（FrameSignal）。
    // 参数 signal：目标存储的引用，本端口此后向它 publish。
    // 副作用：把该存储的地址保存到 signal_；重复 bind 会覆盖上一次的绑定。
    void bind(FrameSignal<T>& signal)
    {
        signal_ = &signal;
    }

    // 发布当前帧的输出值。
    // 参数 frame_id：本次发布的帧号；参数 value：要发布的输出值。
    // 返回：绑定成功并写入完成返回 true；若尚未 bind（signal_ 为空）
    //       则返回 false 且不写入任何数据。
    bool publish(uint32_t frame_id, const T& value)
    {
        if (signal_ == nullptr) {
            return false;
        }
        signal_->publish(frame_id, value);
        return true;
    }

private:
    FrameSignal<T>* signal_{nullptr}; // 绑定的输出信号存储地址；nullptr 表示尚未绑定。
};

// InputPort<T>：输入端口句柄，节点用它读取上游节点的输出。
// 只读（内部保存的是 const 指针），因此同一块 FrameSignal 可被多个输入端口同时读取。
template <typename T>
class InputPort {
public:
    // 将输入端口绑定到上游的信号存储。
    // 参数 signal：上游 FrameSignal 的 const 引用（只读）。
    // 副作用：把其地址保存到 signal_；重复 bind 会覆盖上一次的绑定。
    void bind(const FrameSignal<T>& signal)
    {
        signal_ = &signal;
    }

    // 是否已绑定到某个上游信号（只判断"接上了没"，不要求已有数据样本）。
    bool connected() const { return signal_ != nullptr; }

    // 是否"已绑定且当前值有效"（两个条件都需满足）。
    bool valid() const
    {
        return signal_ != nullptr && signal_->valid();
    }

    // 读取输入值，返回 const 引用。
    // 警告：不检查是否已绑定/是否有效，调用前必须先 valid()，
    //       否则可能对空指针解引用。
    const T& read() const
    {
        return signal_->read();
    }

    // 判断读到的输入是否为"指定这一帧"的值。
    // 参数 frame_id：调用方期望的帧号（通常就是本轮调度号）。
    // 返回：已绑定、值有效、且写入帧号恰好等于 frame_id 时为 true；
    //       以此拒绝读取上一轮残留的旧值，保证数据新鲜。
    bool validForFrame(uint32_t frame_id) const
    {
        return valid() && signal_->frameId() == frame_id;
    }

    // 返回输入数据对应的帧号；未绑定时返回 0。
    uint32_t frameId() const
    {
        return signal_ == nullptr ? 0U : signal_->frameId();
    }

private:
    const FrameSignal<T>* signal_{nullptr}; // 上游只读信号地址；nullptr 表示尚未绑定。
};

} // namespace robot::framework

#endif /* ROBOT_PORTS_HPP */
