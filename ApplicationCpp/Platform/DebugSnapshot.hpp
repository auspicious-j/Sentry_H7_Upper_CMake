/**
 * @file DebugSnapshot.hpp
 * @brief 全局调试快照（Debug Snapshot）：一族供调试器直接观察/改写的状态镜像结构体。
 *
 * 背景：本工程是 RoboMaster 哨兵机器人 STM32H7 上位机（FreeRTOS + C++17 插件框架），
 * 业务状态封装在 C++ 节点对象内部，用 Keil / Ozone / VS Code 的 Watch 窗口很难直接看到。
 * 为此，框架把关心的关键字段定期拷贝到本文件定义的 POD 结构体中，并统一挂在全局变量
 * g_robot_debug 上，让调试器可以按名字定位、逐字段查看。
 *
 * 为什么用 volatile：
 *   这些字段会被业务代码之外的「调试器」随时读、甚至写。volatile 要求编译器每次访问都
 *   真正落到内存，禁止把它缓存进寄存器或当作无用赋值消除；否则 Watch 窗口可能读到过期
 *   值、或人写进去的请求被优化掉。注意：volatile 只保证「不被优化」，不保证原子性，
 *   多字节字段在中断上下文并发写时仍可能被读到「半更新」的值。
 *
 * 读写约定：
 *   - 标注「【Watch 可改】」的字段是调试输入（requested_* 以及 *_request / *_enabled 等），
 *     由人在调试器里改值来施加激励；其余字段为观察输出，只读。
 *   - 观察字段由框架按帧刷新，不追求与业务数据强一致，只求「可观察」。
 *
 * 结构体全部为 POD 类型（可平凡拷贝）；成员用 {} 就地初始化，未显式给值即为 0。
 */
#ifndef robot_debug_snapshot_hpp
#define robot_debug_snapshot_hpp

#include <cstdint>

namespace robot::platform {

// 平台层命名空间：存放与具体硬件解耦的调试快照定义。

// 底盘观察数据：镜像下板（底盘板）回传的底盘状态，供 Watch 查看。
// 注意：is_online 仅代表「上下板通信在线」，并不代表每个轮电机都在线。
struct chassis_debug_snapshot {
    float steering_deg[4]{}; // 下板获取的 4 个舵向单圈角度，单位：度（°）；未扣零偏。默认 0。
    int16_t drive_rpm[4]{}; // 下板原始 4 个轮电机转速，单位：rpm。默认 0。
    uint32_t receive_count{0U}; // 累计实际收到的板间帧数量，只增不减。默认 0。
    uint32_t age_ms{0U}; // 最近一帧样本的年龄，单位：ms；仅当 is_received=1 时有效。默认 0。
    uint32_t frame_id{0U}; // 观察节点最后一次发布时对应的插件帧号。默认 0。
    uint8_t is_received{0U}; // 是否曾收到过下板帧：0=从未收到，1=已收到。默认 0。
    uint8_t is_online{0U}; // 最近 100 ms 内是否收到下板帧（阈值可配置）：0=离线，1=在线。默认 0。
    uint8_t is_lower_feedback{0U}; // 下板 FEEDBACK 标志的原始值（未做任何解释）。默认 0。
};

// 一个演示节点的执行记录：统计某框架节点在本次运行中的调度与激活情况。
struct branch_node_debug_snapshot {
    uint32_t execution_count{0U}; // 该节点被实际执行的累计次数。默认 0。
    uint32_t branch_skip_count{0U}; // 因条件分支未选中而跳过的累计次数。默认 0。
    uint32_t blocked_count{0U}; // 因输入无效或运行状态未就绪而被阻塞的累计次数。默认 0。
    uint8_t frame_state{0U}; // node_frame_state 状态机枚举的数值。默认 0。
    uint16_t frame_order{0xFFFFU}; // 本帧内的实际执行序号；未执行时为 0xFFFF（65535）。默认 0xFFFF。
    uint32_t enter_count{0U}; // 收到「节点进入」通知的累计次数。默认 0。
    uint32_t exit_count{0U}; // 收到「节点退出」通知的累计次数。默认 0。
    uint32_t plan_change_count{0U}; // 保持激活期间发生计划切换的通知次数。默认 0。
    uint8_t is_active{0U}; // 帧边界上的激活状态：0=未激活，1=激活；不等于「输入有效」。默认 0。
};

// 一条分支分支编号的观察：包含该分支编号上的两级内部节点统计与分支编号末端结果。
struct branch_path_debug_snapshot {
    branch_node_debug_snapshot first{}; // 第一级节点的执行统计。
    branch_node_debug_snapshot second{}; // 第二级节点的执行统计。
    uint32_t first_runs_since_enter{0U}; // 第一级节点自本次激活以来的执行次数。默认 0。
    uint32_t second_runs_since_enter{0U}; // 第二级节点自本次激活以来的执行次数。默认 0。
    float value{0.0f}; // 本帧分支编号末端的有效结果；无效时被写为 0。默认 0.0f。
    uint8_t is_valid{0U}; // 分支编号末端本帧是否有有效结果：0=无，1=有。默认 0。
};

// 固定图条件分支（A/B 二选一）的观察快照。
// 约定：仅 requested_id 是调试输入（可在 Watch 中直接改写），其余字段均为观察输出。
struct branch_debug_snapshot {
    uint32_t requested_id{0U}; // 【Watch 可改】请求的分支编号：0=A，1=B；其他值为非法。默认 0。
    uint32_t active_id{0U}; // 本帧实际采用的分支编号：0=A，1=B。默认 0。
    uint32_t switch_count{0U}; // 分支编号实际发生切换的累计次数。默认 0。
    uint32_t rejected_frames{0U}; // 因请求非法而被拒绝的帧数（非法时保留上一合法分支编号）。默认 0。
    uint32_t frame_id{0U}; // 当前调试帧号。默认 0。
    uint32_t output_frame_id{0U}; // 汇合节点产生有效输出时对应的帧号。默认 0。
    float input_value{0.0f}; // 公共输入值；正常运行时预期为 10。默认 0.0f。
    float output_value{0.0f}; // 汇合输出值；有效时 A 分支编号为 22、B 分支编号为 27。默认 0.0f。
    uint8_t is_output_valid{0U}; // 汇合输出本帧是否有效：0=无效，1=有效。默认 0。
    branch_node_debug_snapshot source{}; // 公共源节点的执行统计。
    branch_node_debug_snapshot merge{}; // 汇合节点的执行统计。
    branch_path_debug_snapshot branches[2]{}; // 两条分支编号的统计：branches[0]=A，branches[1]=B。
};

// 单路算术结果及其身份信息（输入序号 / 状态世代），用于判断两路结果是否同源同步。
// is_present=0 时，其余字段不代表有效结果。
struct comparison_sample_debug_snapshot {
    uint64_t sampled_at_us{0U}; // 采样时刻，单位：us（微秒）。默认 0。
    uint32_t input_sequence{0U}; // 本次采样所用输入的序号，用来判断两路是否对齐。默认 0。
    uint32_t state_epoch{0U}; // 状态世代号：每次重置自增，用于识别重置边界。默认 0。
    uint32_t step{0U}; // 该路的推进步序号。默认 0。
    double value[2]{}; // [0] 有界累计值（候选侧含偏差）；[1] 实际推进次数。默认全 0。
    uint8_t is_present{0U}; // 本次采样是否有效：0=无效，1=有效。默认 0。
};

// 通用算术演示快照：比对「参考实现」与「候选实现」的数值差异。
// 约定：前五项为调试输入（可在 Watch 中改写以施加激励），其余字段均为观察输出。
struct comparison_debug_snapshot {
    int32_t input_value{7}; // 【Watch 可改】输入请求值；合法范围 -1000..1000，越界帧会被拒绝。默认 7。
    int32_t candidate_bias{0}; // 【Watch 可改】候选侧输出偏差请求；合法范围 -1000..1000。默认 0。
    uint32_t reset_all_request{0U}; // 【Watch 可改】改成与上次不同的值即触发一次「共同重置」。默认 0。
    uint32_t reset_candidate_request{0U}; // 【Watch 可改】改成不同值即触发一次「候选侧重置」。默认 0。
    uint8_t is_pause_candidate{0U}; // 【Watch 可改】非 0 时暂停候选侧推进；不补算暂停期间漏掉的帧。默认 0。
    int32_t applied_input_value{7}; // 最近一次被接受的合法输入值（实际生效值）。默认 7。
    int32_t applied_candidate_bias{0}; // 最近一次被接受的合法候选偏差（实际生效值）。默认 0。
    uint32_t frame_id{0U}; // 当前调试帧号。默认 0。
    comparison_sample_debug_snapshot reference{}; // 参考侧采样。
    comparison_sample_debug_snapshot candidate{}; // 候选侧采样。
    uint8_t status{0U}; // 比较状态：missing=0, unaligned=1, invalid_tolerance=2, non_finite=3, equal=4, different=5。默认 0。
    uint8_t is_difference_valid{0U}; // 当前差值是否可解释：仅当 status 为 equal/different 时为 1。默认 0。
    double difference[2]{}; // 差值，定义为 candidate - reference。默认全 0。
    double max_abs_difference{0.0}; // 当前有效比较中的最大绝对差（每次比较刷新）。默认 0.0。
    double historical_max_abs_difference{0.0}; // 自启动以来的历史最大绝对差；不随状态重置清零。默认 0.0。
    uint32_t equal_count{0U}; // 判定为 equal 的累计次数。默认 0。
    uint32_t different_count{0U}; // 判定为 different 的累计次数。默认 0。
    uint32_t unaligned_count{0U}; // 判定为 unaligned（两路不同源）的累计次数。默认 0。
    uint32_t invalid_count{0U}; // 结果为缺失/非法的累计次数；因调度跳过不算执行、不计入此值。默认 0。
    uint32_t rejected_frames{0U}; // 因输入或偏差非法而被拒绝的帧数（保留上次合法参数）。默认 0。
};

// 异步「最新值邮箱」（SnapshotMailbox）观察数据：生产者发新值、消费者取最新值。
// 序号只在生产者真正发布时变化，可用来判断消费者是否读到了新数据。
struct snapshot_debug_snapshot {
    uint32_t frame_id{0U}; // 最近一次消费者所在的帧号。默认 0。
    uint32_t sequence{0U}; // 最近一次发布产生的序号（生产者侧）。默认 0。
    uint32_t read_count{0U}; // 消费者读取的累计次数。默认 0。
    uint64_t age_us{0U}; // 最近一帧样本的年龄，单位：us（微秒）。默认 0。
    uint8_t status{0U}; // snapshot_status 枚举的数值。默认 0。
    uint8_t publish_enabled{1U}; // 【Watch 可改】0=暂停发布，非 0=发布；默认 1（上电即发布）。
    uint16_t reserved{0U}; // 对齐填充的保留字段，勿使用。默认 0。
};

// 执行计划演示快照：观察「计划」（一组节点的执行顺序）的切换。
// 约定：仅 requested_id 是调试输入，其余字段均为观察输出。
struct plan_debug_snapshot {
    uint32_t requested_id{0U}; // 【Watch 可改】请求的计划：0=A 先 B 后，1=B 先 A 后。默认 0。
    uint32_t active_id{0U}; // 本帧实际采用的计划编号。默认 0。
    uint32_t plan_count{1U}; // 根图声明的计划总数。默认 1。
    uint32_t failed_id{0xFFFFU}; // 失败的计划编号；没有具体失败时为 0xFFFF（65535）。默认 0xFFFF。
    uint32_t switch_count{0U}; // 计划实际切换的累计次数。默认 0。
    uint32_t rejected_frames{0U}; // 因请求非法而被拒绝的帧数。默认 0。
    uint32_t frame_id{0U}; // 本帧观测到的帧号。默认 0。
    branch_node_debug_snapshot a{}; // 探针 A 的状态与执行序号。
    branch_node_debug_snapshot b{}; // 探针 B 的状态与执行序号。
};

// 顶层调试快照：Keil / Ozone 的 Watch 窗口可直接观察的固定结构。
// 业务状态仍封装在各自的节点对象内部，这里只保存镜像副本。
struct robot_debug_snapshot {
    volatile uint32_t frame_id; // 当前帧号，每帧自增。
    volatile uint32_t last_frame_duration_us; // 最近一帧的耗时，单位：us。
    volatile uint32_t max_frame_duration_us; // 历史最大帧耗时，单位：us。
    volatile uint32_t last_node_duration_us; // 最近一个节点的耗时，单位：us。
    volatile uint32_t max_node_duration_us; // 历史最大单节点耗时，单位：us。
    volatile uint32_t graph_node_count; // 当前插件图的节点数量。
    volatile uint32_t graph_edge_count; // 当前插件图的边（连接）数量。
    volatile uint32_t init_error; // 初始化错误码；0=无错误。
    volatile uint32_t fault_flags; // 故障标志位（按位组合）；0=无故障。
    volatile uint8_t is_initialized; // 初始化完成标志：0=未完成，1=已完成。
    volatile uint8_t running; // 运行标志：0=停止，1=运行中。
    volatile uint8_t last_result; // 最近一次处理的返回结果码。
    volatile uint8_t reserved; // 保留字节（对齐用），勿使用。
    chassis_debug_snapshot chassis{}; // 底盘观察数据；随全局 volatile 快照一起供 Watch 读取。
    branch_debug_snapshot branch{}; // 条件分支选择的调试入口与结果。
    plan_debug_snapshot plan{}; // 执行计划切换的调试入口与结果。
    snapshot_debug_snapshot snapshot{}; // 异步快照的年龄与序号。
    comparison_debug_snapshot comparison{}; // 独立状态的算术比较及请求入口。
};

} // namespace robot::platform

// 全局调试快照实例：用 C 链接名（extern "C"）导出，避免 C++ 名字修饰，
// 使 Keil / Ozone 可以按名字 g_robot_debug 直接定位该符号。
#ifdef __cplusplus
extern "C" {
#endif
extern volatile robot::platform::robot_debug_snapshot g_robot_debug; // 唯一的全局调试快照实例，定义在 DebugSnapshot.cpp。
#ifdef __cplusplus
}
#endif

#endif /* robot_debug_snapshot_hpp */
