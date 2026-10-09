#ifndef ROBOT_BUILD_CONFIG_H
#define ROBOT_BUILD_CONFIG_H

/* Shared by the STM32 target and future NUC/Linux targets. */
// 当前阶段只实现 STM32 + FreeRTOS；NUC/Linux 作为后续移植目标。
#define ROBOT_PLATFORM_STM32 1
#define ROBOT_PLATFORM_NUC   0

// 是否把插件框架编译进目标工程。
#define ROBOT_HAS_PLUGIN_GRAPH 1
// 0：插件框架只观测，不接管旧电机输出；1：迁移阶段再切换到新链路。
#define ROBOT_USE_PLUGIN_GRAPH 0

// 插件执行器的逻辑基础频率；当前对应 1 ms 一帧。
#define ROBOT_BASE_RATE_HZ 1000U
// 静态图容量，运行期不扩容。
#define ROBOT_MAX_PLUGIN_NODES 32U
#define ROBOT_MAX_PLUGIN_EDGES 64U
// 根图最多预存的执行计划数；当前演示使用其中两套。
#define ROBOT_MAX_EXECUTION_PLANS 4U

// 轻量耗时统计；完整 trace 另行控制。
#define ROBOT_ENABLE_PERF_COUNTERS 1
#define ROBOT_ENABLE_PERF_TRACE    0
#define ROBOT_ENABLE_DEBUG_SNAPSHOT 1

// 是否启动底盘板间反馈观察链路；不接管任何控制输出。
#define ROBOT_ENABLE_CHASSIS_OBSERVER 1
// 下板帧在线判断阈值，单位 ms；不代表单电机掉线阈值。
#define ROBOT_CHASSIS_FEEDBACK_TIMEOUT_MS 100U

// 是否组装无硬件输出的条件分支演示图。
#define ROBOT_ENABLE_BRANCH_DEMO 1

// 是否组装无硬件输出的执行顺序切换演示。
#define ROBOT_ENABLE_PLAN_DEMO 1

#endif /* ROBOT_BUILD_CONFIG_H */
