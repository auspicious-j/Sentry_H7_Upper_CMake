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

// 轻量耗时统计；完整 trace 另行控制。
#define ROBOT_ENABLE_PERF_COUNTERS 1
#define ROBOT_ENABLE_PERF_TRACE    0
#define ROBOT_ENABLE_DEBUG_SNAPSHOT 1

#endif /* ROBOT_BUILD_CONFIG_H */
