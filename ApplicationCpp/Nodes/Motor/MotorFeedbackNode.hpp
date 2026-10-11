/* =============================================================================
 * 文件：motor_feedback_node.hpp
 * 作用：声明「电机反馈节点」——插件框架中的一个适配器节点（Adapter Node）。
 *
 * 一、它在框架里的位置
 *   插件图（plugin_graph）由若干节点组成，节点之间通过端口传数据（端口模型见
 *   Framework/Ports.hpp）。本节点是「电机反馈」这条数据链的源头：它读取旧 C 代码里
 *   维护的电机对象（shooter / gimbal 下的 DJI、DM 电机结构体），把字段翻译成与协议
 *   无关的 motor_feedback_frame，再通过输出端口发布给下游（底盘、云台等节点）订阅。
 *
 * 二、适配器模式
 *   旧电机结构体（DJI_Motor_t / DM_motor_t）是「被适配者」，motor_feedback_frame 是
 *   「目标接口」，本节点就是「适配器」。上层节点只依赖目标接口，因而与具体电机型号
 *   解耦；将来更换电机时，只需修改本节点内部的转换函数，上层逻辑无需改动。
 *
 * 三、生命周期
 *   本节点只重写 process()（每帧业务入口），其余生命周期回调沿用 plugin_node 的默认
 *   实现。process() 每帧构造一份快照并发布；发布失败（如端口未绑定）时返回 fault。
 * ============================================================================= */

#ifndef robot_motor_feedback_node_hpp
#define robot_motor_feedback_node_hpp

#include "../../Framework/PluginNode.hpp"
#include "../../Framework/Ports.hpp"

extern "C" {
#include "USER_Moto.h"
}
#include "MotorData.hpp"

namespace robot::motor {

// motor_feedback_node：把旧 C 电机对象转换成统一 motor_feedback_frame 的适配器节点。
// 设计要点：本节点只「读取并翻译」电机数据，不修改任何电机状态，对旧代码是只读的。
// final 表示本类不再被继承（它是插件图里的叶子节点）。
class motor_feedback_node final : public robot::framework::plugin_node {
public:
    // 构造函数：以约定的节点 ID 创建节点，并把输出端口绑定到内部信号存储。
    motor_feedback_node();

    // 返回本节点的反馈输出端口。
    // 作用：组装器（composer）取得该端口后，把它连接给下游节点的输入端口，
    //       使底盘、云台等节点得以订阅本节点发布的 motor_feedback_frame。
    robot::framework::output_port<motor_feedback_frame>& output();

    // 返回本节点内部信号存储的只读引用。
    // 作用：组装器需要 input_port 时调用 input_port::bind(signal())，把某块输入绑定到
    //       这块存储；只读返回保证信号内容不会被外部直接改写。
    const robot::framework::frame_signal<motor_feedback_frame>& signal() const;

    // 每帧业务入口（重写 plugin_node 的纯虚函数 process）。
    // 行为：读取旧电机对象 → 翻译成统一反馈 → 通过输出端口发布本帧快照。
    // 返回：发布成功返回 ok；端口未绑定等导致发布失败时返回 fault。
    robot::framework::process_result process(robot::framework::frame_context& context) override;

private:
    // 把一颗 DJI 电机翻译成统一的 motor_feedback。
    // 参数 id      ：写入反馈的电机协议 ID（由调用方给定的逻辑编号）。
    // 参数 channel ：该电机在 CAN 反馈通道表中的通道号，用于查询在线状态。
    // 参数 motor   ：被适配的旧 DJI 电机结构体（只读）。
    // 返回：填充完毕的统一反馈结构。
    static motor_feedback convert_dji(uint16_t id, uint8_t channel, const DJI_Motor_t& motor);

    // 把一颗达妙 DM 电机翻译成统一的 motor_feedback。
    // 参数 id      ：写入反馈的电机协议 ID。
    // 参数 channel ：该电机在 CAN 反馈通道表中的通道号，用于查询在线状态。
    // 参数 motor   ：被适配的旧 DM 电机结构体（只读），实测数据在其 para 成员中。
    // 返回：填充完毕的统一反馈结构（DM 的 para 字段被截断为 int16 / int8）。
    static motor_feedback convert_dm(uint16_t id, uint8_t channel, const DM_motor_t& motor);

    robot::framework::frame_signal<motor_feedback_frame> signal_{}; // 反馈信号存储：保存本帧发布的 motor_feedback_frame，是端口的真正存放处。
    robot::framework::output_port<motor_feedback_frame> output_{}; // 反馈输出端口：对外的发布句柄，构造时 bind 到上面的 signal_。
};

} // namespace robot::motor

#endif /* robot_motor_feedback_node_hpp */
