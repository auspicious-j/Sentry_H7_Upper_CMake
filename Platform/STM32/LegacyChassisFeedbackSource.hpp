// =============================================================================
// legacy_chassis_feedback_source.hpp —— 旧底盘反馈到插件框架接口的适配器声明
//
// 作用：把"以 C 全局变量形式存在的旧底盘反馈"桥接到 C++ 插件框架所需的
//       i_chassis_feedback_source 接口上，属于 Platform 适配层的一部分。
//
// 背景：下板（底盘板）通过板间串口周期上报 4 组舵角与轮速；旧固件由 C 代码
//       （USER_B2B.c）解析后写入一个 static 全局快照。新的 C++ 插件框架不希望
//       业务节点直接访问这些 C 全局对象，因此定义了纯接口
//       robot::chassis::i_chassis_feedback_source，再由本类提供 STM32 平台的实现。
//
// 设计模式（适配器模式 Adapter）：本类实现框架接口，内部委托给 C 桥接函数
//       RobotChassisFeedback_Read（声明于 ChassisFeedbackBridge.h），把 C 侧
//       数据结构转换成 C++ 侧带明确单位的类型化结构，从而让上层只依赖抽象接口。
//
// C/C++ 互操作：接口与返回类型均为纯 C++（不含 HAL、无需 extern "C"）；真正
//       跨越语言边界的工作全部交给 Bridge 头里的 extern "C" 函数完成，本类因此
//       对上只暴露 C++ 抽象，对下复用既有的 C 接收逻辑。
// =============================================================================
#ifndef robot_legacy_chassis_feedback_source_hpp
#define robot_legacy_chassis_feedback_source_hpp

#include "Nodes/Chassis/IChassisFeedbackSource.hpp"

namespace robot::platform::stm32 {

// STM32 平台下 i_chassis_feedback_source 的适配器实现。
// - final：标记为最终类，不允许再被继承——它只是一个平台实现，无需再扩展；
// - 业务节点仅依赖基类接口，因此不会引用旧 chassis 全局对象，实现解耦。
class legacy_chassis_feedback_source final : public robot::chassis::i_chassis_feedback_source {
public:
    // 读取旧接收器（板间串口）发布的稳定快照，并转换为与帧号无关的反馈结构。
    // 参数：无（const 成员函数，不修改自身状态）。
    // 返回：robot::chassis::chassis_feedback，各字段含义见 ChassisData.hpp；
    //       若从未收到过板间帧，则 is_received=false，其余字段为零值。
    robot::chassis::chassis_feedback read() const override;
};

} // namespace robot::platform::stm32
#endif // robot_legacy_chassis_feedback_source_hpp
