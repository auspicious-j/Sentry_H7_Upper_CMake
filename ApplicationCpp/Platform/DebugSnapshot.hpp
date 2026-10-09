#ifndef ROBOT_DEBUG_SNAPSHOT_HPP
#define ROBOT_DEBUG_SNAPSHOT_HPP

#include <cstdint>

namespace robot::platform {

// 底盘观察数据；online 仅代表下板通信在线，不代表各轮电机在线。
struct ChassisDebugSnapshot {
    float steering_deg[4]{}; // 下板单圈舵角，度，未扣零偏。
    int16_t drive_rpm[4]{}; // 下板原始轮电机转速。
    uint32_t receive_count{0U}; // 实际板间帧接收次数。
    uint32_t age_ms{0U}; // 最近样本年龄，received=1 时有效。
    uint32_t frame_id{0U}; // 观察节点最后发布的插件帧号。
    uint8_t received{0U}; // 是否收到过下板帧。
    uint8_t online{0U}; // 最近 100 ms 内是否收到下板帧（阈值可配置）。
    uint8_t lower_feedback{0U}; // 下板 FEEDBACK 原值。
};

// 一个演示节点的执行记录。
struct BranchNodeDebugSnapshot {
    uint32_t execution_count{0U}; // 累积执行次数。
    uint32_t branch_skip_count{0U}; // 条件未选的次数。
    uint32_t blocked_count{0U}; // 输入或运行状态阻塞次数。
    uint8_t frame_state{0U}; // NodeFrameState 数值。
    uint16_t frame_order{0xFFFFU}; // 本帧实际执行序号，未执行为65535。
};

// 一条路线的两个内部节点及末端结果。
struct BranchRouteDebugSnapshot {
    BranchNodeDebugSnapshot first{}; // 第一级状态。
    BranchNodeDebugSnapshot second{}; // 第二级状态。
    float value{0.0f}; // 本帧有效结果，无效时置 0。
    uint8_t valid{0U}; // 路线末端是否有本帧结果。
};

// 固定图条件分支观察；只有 requested_route 作为调试输入。
struct BranchDebugSnapshot {
    uint32_t requested_route{0U}; // Watch 可改：0=A、1=B。
    uint32_t active_route{0U}; // 本帧实际采用的路线。
    uint32_t generation{0U}; // 路径切换版本。
    uint32_t switch_count{0U}; // 路线实际切换次数。
    uint32_t rejected_frames{0U}; // 请求非法的帧数，保留上一合法路线。
    uint32_t frame_id{0U}; // 当前调试帧号。
    uint32_t output_frame_id{0U}; // 汇合有效输出帧号。
    float input_value{0.0f}; // 公共输入，预期 10。
    float output_value{0.0f}; // 有效时 A=22、B=27。
    uint8_t output_valid{0U}; // 汇合输出是否为本帧有效数据。
    BranchNodeDebugSnapshot source{}; // 公共节点执行统计。
    BranchNodeDebugSnapshot merge{}; // 汇合节点执行统计。
    BranchRouteDebugSnapshot routes[2]{}; // A/B 两条路线的统计。
};

// 执行计划演示；只有 requested_plan 是调试输入。
struct PlanDebugSnapshot {
    uint32_t requested_plan{0U}; // Watch可写：0=A先B后，1=B先A后。
    uint32_t active_plan{0U}; // 本帧采用的计划。
    uint32_t plan_count{1U}; // 根图声明计划数量。
    uint32_t failed_plan{0xFFFFU}; // 失败计划编号，无具体失败为65535。
    uint32_t switch_count{0U}; // 实际计划切换次数。
    uint32_t generation{0U}; // 计划版本。
    uint32_t rejected_frames{0U}; // 请求非法的帧数。
    uint32_t frame_id{0U}; // 本次观测帧号。
    BranchNodeDebugSnapshot a{}; // 探针A状态和执行序号。
    BranchNodeDebugSnapshot b{}; // 探针B状态和执行序号。
};

// Keil Watch 可直接观察的固定调试快照；业务状态仍封装在节点对象内。
struct RobotDebugSnapshot {
    volatile uint32_t frame_id; // 当前帧号。
    volatile uint32_t last_frame_duration_us; // 最近帧耗时。
    volatile uint32_t max_frame_duration_us; // 最大帧耗时。
    volatile uint32_t last_node_duration_us; // 最近节点耗时。
    volatile uint32_t max_node_duration_us; // 最大节点耗时。
    volatile uint32_t graph_node_count; // 图节点数。
    volatile uint32_t graph_edge_count; // 图边数。
    volatile uint32_t init_error; // 初始化错误码。
    volatile uint32_t fault_flags; // 故障标志。
    volatile uint8_t initialized; // 初始化完成标志。
    volatile uint8_t running; // 运行标志。
    volatile uint8_t last_result; // 最近处理结果。
    volatile uint8_t reserved; // 保留字节。
    ChassisDebugSnapshot chassis{}; // 随全局 volatile 快照一起供 Watch 读取。
    BranchDebugSnapshot branch{}; // 条件分支选择入口及结果。
    PlanDebugSnapshot plan{}; // 执行顺序切换入口及结果。
};

} // namespace robot::platform

// 使用 C 链接名，方便 Keil Watch 直接通过 g_robot_debug 查找。
#ifdef __cplusplus
extern "C" {
#endif
extern volatile robot::platform::RobotDebugSnapshot g_robot_debug;
#ifdef __cplusplus
}
#endif

#endif /* ROBOT_DEBUG_SNAPSHOT_HPP */
