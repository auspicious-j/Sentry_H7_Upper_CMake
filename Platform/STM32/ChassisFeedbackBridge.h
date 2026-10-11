#ifndef ROBOT_CHASSIS_FEEDBACK_BRIDGE_H
#define ROBOT_CHASSIS_FEEDBACK_BRIDGE_H

#include <stdint.h>

/* ============================================================================
 * 文件：ChassisFeedbackBridge.h
 * 作用：定义“下板（底盘下位机） -> 上位机”板间反馈的纯 C 数据结构与读取入口，
 *       作为 C 语言侧（Application/ 下的 FreeRTOS 任务、中断）与 C++17 插件框架
 *       之间的桥接层。
 * 设计要点：
 *   1. 本文件只使用标准定长整数(uint32_t/int16_t 等)与 float，绝不引入 HAL/CMSIS
 *      等平台类型，保证 C 与 C++ 两种编译单元都能安全包含。
 *   2. 结构体是“快照”语义：读取方拿到的是一份完整拷贝，接收方与读取方通过拷贝
 *      解耦，避免读到跨帧的半更新数据（数据竞争）。
 *   3. 术语：下板 = 底盘下位机；上板 = 本工程 STM32H7 上位机。二者经 CAN/UART
 *      板间协议通信，本结构体缓存其反馈帧的解析结果。
 * 链接约定：导出的符号都置于 extern "C" 内，防止 C++ 名字改编(name mangling)
 *       导致 C 侧按下标/原名找不到符号。
 * ========================================================================== */

/* 常量：下板底盘共 4 个轮组（舵轮/麦轮布局），用于确定下列各数组的长度。 */
#define ROBOT_CHASSIS_FEEDBACK_WHEELS 4U // 下板四个轮组。

// extern "C"：以下声明按 C 语言的链接约定导出符号，实现 C 与 C++ 混合编译时正确链接。
// C++ 编译器默认会对函数名做名字改编(name mangling)，C 侧无法按原名调用到符号；
// 用 extern "C" 关闭改编后，两种编译单元即可共享同一批符号。
#ifdef __cplusplus
extern "C" {
#endif

// C/C++ 共享的板间反馈快照，不含 HAL 类型。
// 字段均为固定宽度类型，跨编译单元 ABI 一致；由接收侧写入，读取侧经拷贝获取。
typedef struct {
    float steering_deg[ROBOT_CHASSIS_FEEDBACK_WHEELS]; // 原始单圈舵角(度)，下标对应轮组；来自下板舵轮反馈，未做零点/方向校正。
    int16_t drive_rpm[ROBOT_CHASSIS_FEEDBACK_WHEELS]; // 下板反馈的原始轮电机转速(RPM)；带符号，正值/负值代表转向，未做比例映射。
    uint32_t received_tick_ms; // 最近一次成功接收该反馈帧时的 HAL tick(毫秒)；用于评估数据新鲜度/超时判断。
    uint32_t sampled_tick_ms; // 本次读取(拷贝)快照时的 HAL tick(毫秒)；与 received_tick_ms 之差即为数据年龄。
    uint32_t receive_count; // 累计成功接收的帧数；单调递增，允许 32 位自然回绕(溢出后从 0 继续)。
    uint8_t received; // 布尔标志：是否至少收到过一帧(0=从未收到，非 0=已收到)。
    uint8_t lower_feedback; // 下板 FEEDBACK 报文的原始值；仅表示报文状态，不代表单个电机在线。
} RobotChassisFeedbackSnapshot;

// 向 destination 复制完整快照；空指针时不操作，只供任务调用。
// 参数 destination：输出缓冲区指针，由调用方提供；为 NULL 时函数直接返回，不做任何写入。
// 返回值：无。
// 调用上下文：任务上下文（非中断）。实现内部会用短临界区完成拷贝，确保读到的是一致快照；
//             禁止在中断里调用，以免长时间关闭中断。
void RobotChassisFeedback_Read(RobotChassisFeedbackSnapshot* destination);

// 关闭 extern "C" 区块（与上方开启处配对）。
#ifdef __cplusplus
}
#endif

#endif // ROBOT_CHASSIS_FEEDBACK_BRIDGE_H
