/* ============================================================================
 * 文件：Gimbal_GraphApi.h
 * 作用：Gimbal 图的 C 接口。供 C 侧任务（Gimbal.c 的 OS_GimbalCallback）调用；
 *       实现位于 ApplicationCpp/Graph/Gimbal/Gimbal_Graph.cpp。
 * 线程约定：两个函数都只能在 Gimbal 任务上下文调用（图不是线程安全的）。
 * ========================================================================== */
#ifndef robot_gimbal_graph_api_h
#define robot_gimbal_graph_api_h

#ifdef __cplusplus
extern "C" {
#endif

// 初始化图：注册节点、连边、编译、配置并启动。初始化失败时图保持不可用，
// 后续 process_frame 会返回 fault。
void AppCpp_Gimbal_Init(void);

// 执行一帧：更新帧号/时间戳，按拓扑顺序运行全部节点。
void AppCpp_Gimbal_ProcessFrame(void);

#ifdef __cplusplus
}
#endif

#endif /* robot_gimbal_graph_api_h */
