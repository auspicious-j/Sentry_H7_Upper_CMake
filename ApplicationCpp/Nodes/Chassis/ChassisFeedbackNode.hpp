/*
 * chassis_feedback_node.hpp —— 底盘反馈节点（插件框架中的叶子节点）。
 *
 * 一、本节点的职责
 *   把「底盘下板反馈」这一路数据源（i_chassis_feedback_source）读取出来，原样发布到
 *   自己的唯一输出端口 output_ 上，供下游节点（底盘观测、诊断等）通过输入端口绑定
 *   读取。节点本身不做任何解算或控制，只做「搬运」：
 *     数据源 --(read)--> 本节点 --(publish)--> 输出信号 --(被读)--> 下游输入端口。
 *
 * 二、它在插件框架中的位置
 *   - 节点（plugin_node 派生类）：执行图上的一个可执行单元，每帧被执行器调用一次
 *     process()。本类用 final 修饰，表示它是最末端实现，不允许再被继承。
 *   - 端口与信号（Ports.hpp）：output_ 是输出端口句柄，signal_ 是真正存放数据的
 *     「帧信号」存储。一个输出端口同一时刻只允许一个节点写入，所以 signal_ 只有
 *     本节点会改，下游只能只读读取。
 *
 * 三、生命周期与调用时机（由框架 plugin_graph 统一驱动）
 *   1) 构造：注入 id 与数据源引用，并把输出端口绑定到内部信号存储。
 *   2) process：每个执行周期调用一次——从数据源读一份稳定快照并发布出去。
 *   本节点没有内部子图，故未重写 compose/configure/start 等钩子，沿用基类默认实现。
 *
 * 四、数据源为何用接口注入
 *   通过 i_chassis_feedback_source 接口隔离本节点与「C 全局对象 / 硬件接收器」，便于
 *   替换实现和单元测试；数据源对象的生命周期须长于本节点，因此以引用方式保存。
 */

#ifndef robot_chassis_feedback_node_hpp
#define robot_chassis_feedback_node_hpp

#include "Framework/PluginNode.hpp"
#include "Framework/Ports.hpp"
#include "IChassisFeedbackSource.hpp"

namespace robot::chassis {

// chassis_feedback_node：把底盘下板反馈数据源读入，并发布到输出端口。
// 继承自 robot::framework::plugin_node；final 表示不可再派生。
// 发布的数据类型为 chassis_feedback（定义见 ChassisData.hpp）。
class chassis_feedback_node final : public robot::framework::plugin_node {
public:
    // 构造函数。
    // 参数 id     ：本节点在图内的唯一编号（plugin_id），由组装层分配，全图不可重复。
    // 参数 source ：底盘反馈数据源，以引用保存；其生命周期必须长于本节点
    //               （由组装层持有，本节点不负责其创建与销毁）。
    // 副作用：把内部输出端口 output_ 绑定到内部信号存储 signal_，令 publish 有落点。
    chassis_feedback_node(robot::framework::plugin_id id, i_chassis_feedback_source& source);
    // 返回本节点发布的只读输出信号（frame_signal<chassis_feedback> 的 const 引用）。
    // 用途：组装层拿到它后，绑定给下游节点的输入端口（input_port::bind）以建立数据流。
    // 返回值：内部信号存储 signal_，下游据此读取本节点每帧发布的最新底盘反馈。
    // 时机：任意时刻可调用；返回的是引用，不发生拷贝。
    const robot::framework::frame_signal<chassis_feedback>& signal() const;
    // 节点的业务入口，每帧由执行器调用一次（重写基类纯虚函数 process）。
    // 参数 context：本次插件帧的上下文（帧号 frame_id、周期、时间戳）。
    // 逻辑：从数据源读取一份稳定快照，再以本帧帧号发布到输出端口。
    // 返回 process_result：发布成功返回 ok；若输出端口未绑定导致发布失败返回 fault。
    // 时机：仅在节点激活、层级可运行且本帧依赖就绪时才会被真正调用。
    robot::framework::process_result process(robot::framework::frame_context& context) override;

private:
    i_chassis_feedback_source& source_; // 外部注入的反馈源；本节点只读它，不拥有其生命周期。
    // 固定信号存储：真正存放本节点输出的内存。用 {} 默认初始化，初始 is_valid=false、
    // frame_id=0，直到首次 publish 才有有效值。
    robot::framework::frame_signal<chassis_feedback> signal_{};
    // 唯一写入端：输出端口句柄，只有本节点向 signal_ 发布，保证单一写入者。
    robot::framework::output_port<chassis_feedback> output_{};
};

} // namespace robot::chassis
#endif // robot_chassis_feedback_node_hpp
