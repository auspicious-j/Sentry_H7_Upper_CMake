// =============================================================================
// IChassisFeedbackSource.hpp —— 底盘反馈「数据源」抽象接口
//
// 作用：在底盘插件节点与具体的 C 全局对象/硬件接收器之间建立隔离层，使节点只依赖
//       抽象接口，而不直接访问旧固件的 C 全局变量或串口接收器，从而降低耦合。
//
// 概念说明：
//   - 数据源（Source）：能提供「一帧底盘反馈快照」的供应方，例如下板板间串口接收器；
//     插件框架通过本接口读取它，而无需关心其内部如何实现。
//   - 适配器（Adapter）：把某个具体数据源（如 C 侧 static 全局快照）转换成实现本接口
//     的 C++ 对象，例如 Platform 层的 LegacyChassisFeedbackSource。
//   典型用法：插件节点持有一个 IChassisFeedbackSource&（其生命周期长于节点），每帧在
//   process() 中调用 read() 取一份快照。
// =============================================================================
#ifndef ROBOT_CHASSIS_FEEDBACK_SOURCE_HPP
#define ROBOT_CHASSIS_FEEDBACK_SOURCE_HPP

#include "ChassisData.hpp"

namespace robot::chassis {

// 隔离底盘节点与 C 全局对象、硬件接收器的抽象接口（纯虚基类）。
// 上层只依赖此接口，具体实现由 Platform 适配层提供并在组装时注入。
class IChassisFeedbackSource {
public:
    // 虚析构，支持经基类指针/引用析构派生实现；默认实现即可，无特殊资源释放。
    virtual ~IChassisFeedbackSource() = default;
    // 返回一份稳定快照；从未接收时 received=false。
    // 用途：读取当前可用的最新一帧底盘反馈，供上游节点在本帧消费。
    // 参数：无（const 成员函数，调用不应改变数据源的可观察状态）。
    // 返回：robot::chassis::ChassisFeedback，一份「稳定」的完整快照；若从未收到过任何
    //       板间帧，则 received=false，其余字段为零值（默认值）。
    // 时序：设计为可在节点 process() 中逐帧调用，实现内部需自行保证读到的数据不撕裂
    //       （具体由适配器负责，例如在临界区内复制快照）。
    virtual ChassisFeedback read() const = 0;
};

} // namespace robot::chassis
#endif // ROBOT_CHASSIS_FEEDBACK_SOURCE_HPP
