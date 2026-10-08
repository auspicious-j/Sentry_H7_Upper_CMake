#ifndef ROBOT_APP_CPP_ENTRY_H
#define ROBOT_APP_CPP_ENTRY_H

// 该头文件必须同时能被 CubeMX 的 C 文件和 C++ 文件包含。
#ifdef __cplusplus
extern "C" {
#endif

// C 侧只调用这三个桥接函数，不直接接触 C++ 对象。
// 初始化 C++ 应用。
void AppCpp_Initialize(void);
// 执行一帧 C++ 插件图。
void AppCpp_ProcessFrame(void);
// FreeRTOS 控制任务入口。
void AppCpp_ControlTask(void const* argument);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_APP_CPP_ENTRY_H */
