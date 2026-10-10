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
    uint32_t enter_count{0U}; // 节点进入通知次数。
    uint32_t exit_count{0U}; // 节点退出通知次数。
    uint32_t plan_change_count{0U}; // 保持激活时计划切换通知次数。
    uint8_t active{0U}; // 帧边界激活状态，不等同于输入有效。
};

// 一条路线的两个内部节点及末端结果。
struct BranchRouteDebugSnapshot {
    BranchNodeDebugSnapshot first{}; // 第一级状态。
    BranchNodeDebugSnapshot second{}; // 第二级状态。
    uint32_t first_runs_since_enter{0U}; // 第一级本次激活执行次数。
    uint32_t second_runs_since_enter{0U}; // 第二级本次激活执行次数。
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

// 单路算术结果及其输入/状态身份；present=0 时其余字段不表示有效结果。
struct ComparisonSampleDebugSnapshot {
    uint64_t sampled_at_us{0U};
    uint32_t input_sequence{0U};
    uint32_t state_epoch{0U};
    uint32_t step{0U};
    double value[2]{}; // [0] 有界累计值（候选含偏差），[1] 实际推进次数。
    uint8_t present{0U};
};

// 通用算术演示：前五项可写，其余字段仅观察。
struct ComparisonDebugSnapshot {
    int32_t input_value{7}; // Watch 请求，合法范围 -1000..1000。
    int32_t candidate_bias{0}; // Watch 输出偏差请求，合法范围 -1000..1000。
    uint32_t reset_all_request{0U}; // 改为不同数值触发一次共同重置。
    uint32_t reset_candidate_request{0U}; // 改为不同数值触发一次候选重置。
    uint8_t pause_candidate{0U}; // 非 0 暂停候选，不补算漏帧。
    int32_t applied_input_value{7}; // 最近接受的合法输入。
    int32_t applied_candidate_bias{0}; // 最近接受的合法输出偏差。
    uint32_t frame_id{0U};
    ComparisonSampleDebugSnapshot reference{};
    ComparisonSampleDebugSnapshot candidate{};
    uint8_t status{0U}; // Missing=0, Unaligned=1, InvalidTolerance=2, NonFinite=3, Equal=4, Different=5。
    uint8_t difference_valid{0U}; // 只有 Equal/Different 才能解释当前差值。
    double difference[2]{}; // candidate - reference。
    double max_abs_difference{0.0}; // 当前有效比较的最大绝对差。
    double historical_max_abs_difference{0.0}; // 自启动以来，不随状态重置清零。
    uint32_t equal_count{0U};
    uint32_t different_count{0U};
    uint32_t unaligned_count{0U};
    uint32_t invalid_count{0U}; // 缺失/非法结果；调度跳过不计入执行次数。
    uint32_t rejected_frames{0U}; // 输入或偏差非法的帧数，保留上次合法参数。
};

// 异步最新值邮箱观察；序号只在生产者发布时变化。
struct SnapshotDebugSnapshot {
    uint32_t frame_id{0U}; // 最近消费者帧号。
    uint32_t sequence{0U}; // 最近发布序号。
    uint32_t read_count{0U}; // 消费次数。
    uint64_t age_us{0U}; // 最近样本年龄。
    uint8_t status{0U}; // SnapshotStatus 数值。
    uint8_t publish_enabled{1U}; // Watch 可写，0暂停发布。
    uint16_t reserved{0U}; // 对齐保留。
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
    SnapshotDebugSnapshot snapshot{}; // 异步快照年龄和序号。
    ComparisonDebugSnapshot comparison{}; // 独立状态算术比较及请求入口。
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
