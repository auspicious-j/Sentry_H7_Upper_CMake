// =============================================================================
// 文件：motor_offline_node.cpp
// 作用：motor_offline_node（电机掉线检测节点）的实现。
//
// 实现思路：
//   1) 构造时把输出端口 output_ 绑定到内部信号 signal_，之后才能发布数据。
//   2) 每帧 process()：新建一个 motor_online_frame，逐通道调用 BSP 层的
//      BSP_MotorFeedbackIsOnline()，判断该电机是否在阈值时间内收到过 CAN 反馈，
//      将结果写入 is_online.is_online[index]。
//   3) 用 context.frame_id 作为帧号发布该帧数据；发布失败（端口未绑定）即报 fault。
//
// 依赖：BSP 层接口声明在 bsp_can.h（C 语言），故用 extern "C" 包裹包含，
//       避免 C++ 名称修饰（name mangling）导致链接找不到符号。
// =============================================================================

#include "MotorOfflineNode.hpp"

// bsp_can.h 是 C 头文件；用 extern "C" 告诉编译器按 C 的符号规则处理其中的函数，
// 这样 C++ 代码才能正确链接到 BSP 里用 C 编译的 BSP_MotorFeedbackIsOnline() 等。
extern "C" {
#include "bsp_can.h"
}

namespace robot::motor {

// 构造函数实现：
//   参数 timeout_ms 转交给成员 timeout_ms_（掉线判定阈值，单位 ms）；
//   基类 plugin_node(21U) 指定本节点唯一 ID = 21；
//   构造函数体内把输出端口 output_ 绑定到信号存储 signal_，保证 process() 里
//   publish() 能成功写入（未绑定时 publish 会返回 false）。
motor_offline_node::motor_offline_node(uint32_t timeout_ms)
    : plugin_node(21U), timeout_ms_(timeout_ms)
{
    output_.bind(signal_);
}

// 返回在线状态输出端口 output_ 的引用，供外部连接/发布使用。
robot::framework::output_port<motor_online_frame>& motor_offline_node::output()
{
    return output_;
}

// 返回在线状态信号 signal_ 的只读引用，供外部查看最近一帧的在线状态。
const robot::framework::frame_signal<motor_online_frame>& motor_offline_node::signal() const
{
    return signal_;
}

// 每帧执行：检测所有电机通道是否掉线，并发布本帧的在线状态。
// 返回 ok（发布成功）或 fault（输出端口未绑定导致发布失败）。
robot::framework::process_result motor_offline_node::process(robot::framework::frame_context& context)
{
    motor_online_frame is_online{}; // 当前帧的在线状态；用 {} 默认初始化，各通道初值均为 false（离线）。
    // 逐通道判定：index 即电机通道号，范围 [0, k_motor_channel_count)。
    // 注意：k_motor_channel_count = 8，而 BSP 实际只维护 BSP_MOTOR_FEEDBACK_COUNT = 5 个通道的
    //       时间戳，因此 index >= 5 的通道会被 BSP_MotorFeedbackIsOnline() 直接判为离线（false）。
    for (uint8_t index = 0U; index < k_motor_channel_count; ++index)
    {
        // 询问 BSP：该通道是否在 timeout_ms_ 内有反馈。返回 true=在线，false=离线/无反馈。
        is_online.is_online[index] = BSP_MotorFeedbackIsOnline(index, timeout_ms_);
    }

    // 以本帧帧号发布结果；publish 失败通常意味着 output_ 未绑定（构造异常）。
    if (!output_.publish(context.frame_id, is_online))
    {
        return robot::framework::process_result::fault; // 发布失败 -> 报故障。
    }
    return robot::framework::process_result::ok; // 发布成功 -> 本帧正常。
}

} // namespace robot::motor
