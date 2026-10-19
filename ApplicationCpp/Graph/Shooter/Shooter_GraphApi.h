/* ============================================================================
 * 文件：Shooter_GraphApi.h
 * 作用：发射图的 C 接口，供 Shooter.c 的任务回调调用。
 * ========================================================================== */
#ifndef robot_shooter_graph_api_h
#define robot_shooter_graph_api_h

#ifdef __cplusplus
extern "C" {
#endif

void AppCpp_Shooter_Init(void);
void AppCpp_Shooter_ProcessFrame(void);

#ifdef __cplusplus
}
#endif

#endif /* robot_shooter_graph_api_h */
