#ifndef ROBOT_APP_CPP_ENTRY_H
#define ROBOT_APP_CPP_ENTRY_H

// 该头文件必须同时能被 CubeMX 的 C 文件和 C++ 文件包含。
#ifdef __cplusplus
extern "C" {
#endif

// C 侧只调用这三个桥接函数，不直接接触 C++ 对象。
void AppCpp_Initialize(void);
void AppCpp_ProcessFrame(void);
void AppCpp_ControlTask(void const* argument);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_APP_CPP_ENTRY_H */
