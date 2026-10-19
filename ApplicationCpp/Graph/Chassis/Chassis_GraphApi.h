/* ============================================================================
 * 文件：Chassis_GraphApi.h
 * 作用：底盘图的 C 接口，供 Chassis.c 的任务回调调用。
 * ========================================================================== */
#ifndef robot_chassis_graph_api_h
#define robot_chassis_graph_api_h

#ifdef __cplusplus
extern "C" {
#endif

void AppCpp_Chassis_Init(void);
void AppCpp_Chassis_ProcessFrame(void);

#ifdef __cplusplus
}
#endif

#endif /* robot_chassis_graph_api_h */
