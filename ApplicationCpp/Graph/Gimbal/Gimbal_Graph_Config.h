#ifndef ROBOT_BUILD_CONFIG_H
#define ROBOT_BUILD_CONFIG_H

/* Shared by the STM32 target and future NUC/Linux targets. */

// 静态图容量，运行期不扩容。
inline constexpr std::size_t ROBOT_MAX_PLUGIN_NODES = 32U;
#define ROBOT_MAX_PLUGIN_EDGES 64U
// 根图最多预存的执行计划数。
#define ROBOT_MAX_EXECUTION_PLANS 4U

#endif /* ROBOT_BUILD_CONFIG_H */
