#ifndef ROBOT_PORTS_HPP
#define ROBOT_PORTS_HPP

#include <cstdint>

namespace robot::framework {

// 保存一个端口在当前帧的最新值。
template <typename T>
class FrameSignal {
public:
    // 写入当前帧的值。
    void publish(uint32_t frame_id, const T& value)
    {
        value_ = value;
        frame_id_ = frame_id;
        valid_ = true;
    }

    // 使旧值失效；保留数值便于调试，不允许当作当前输出使用。
    void invalidate() { valid_ = false; }

    // 读取当前保存的值。
    const T& read() const
    {
        return value_;
    }

    // 判断信号是否已经写入。
    bool valid() const
    {
        return valid_;
    }

    // 返回信号对应的帧号。
    uint32_t frameId() const
    {
        return frame_id_;
    }

private:
    T value_{}; // 信号值。
    uint32_t frame_id_{0U}; // 最近写入帧号。
    bool valid_{false}; // 是否已有有效值。
};

// 输出端口：一个输出端口只允许一个节点写入。
template <typename T>
class OutputPort {
public:
    // 将输出端口绑定到固定信号存储。
    void bind(FrameSignal<T>& signal)
    {
        signal_ = &signal;
    }

    // 发布当前帧输出。
    bool publish(uint32_t frame_id, const T& value)
    {
        if (signal_ == nullptr) {
            return false;
        }
        signal_->publish(frame_id, value);
        return true;
    }

private:
    FrameSignal<T>* signal_{nullptr}; // 输出信号存储地址。
};

// 输入端口：可以被多个节点读取，但不负责修改数据。
template <typename T>
class InputPort {
public:
    // 将输入端口绑定到上游输出信号。
    void bind(const FrameSignal<T>& signal)
    {
        signal_ = &signal;
    }

    // 查询是否已绑定，不要求已有样本。
    bool connected() const { return signal_ != nullptr; }

    // 判断输入是否已连接且有效。
    bool valid() const
    {
        return signal_ != nullptr && signal_->valid();
    }

    // 读取输入值；调用前应先检查 valid()。
    const T& read() const
    {
        return signal_->read();
    }

    // frame_id：要求的帧号；拒绝上轮残留值。
    bool validForFrame(uint32_t frame_id) const
    {
        return valid() && signal_->frameId() == frame_id;
    }

    // 返回输入数据的帧号。
    uint32_t frameId() const
    {
        return signal_ == nullptr ? 0U : signal_->frameId();
    }

private:
    const FrameSignal<T>* signal_{nullptr}; // 上游只读信号地址。
};

} // namespace robot::framework

#endif /* ROBOT_PORTS_HPP */
