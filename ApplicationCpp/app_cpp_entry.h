#ifndef robot_app_cpp_entry_h
#define robot_app_cpp_entry_h

//=============================================================================
// 文件：app_cpp_entry.h
// 所属：应用层 / C 与 C++ 的桥接接口（ApplicationCpp）
//
// 一、文件定位
//   本工程由两部分组成：CubeMX 生成的 C 代码（main.c、freertos.c、各外设驱动）
//   与手写的 C++17 插件框架（ApplicationCpp 目录）。C 代码不认识 C++ 的类与
//   命名空间，C++ 对象也不宜直接暴露给 C 链接。于是用本头文件充当二者之间
//   唯一的“窄接口”：C 侧只看到下面三个普普通通的 C 函数，真正的 C++ 对象
//   全部藏在 app_cpp_entry.cpp 内部。
//
// 二、为什么需要 extern "C"
//   本头文件会同时被 C 源文件（freertos.c、main.c 等）和 C++ 源文件包含。
//   C 编译器不认识 extern "C"，因此用 #ifdef __cplusplus 把它包起来：只有
//   C++ 编译器才展开 extern "C"，令下面三个函数以 C 语言符号名（不做名称
//   修饰 / name mangling）导出；C 编译器则原样看到函数声明，按 C 符号名链接。
//   这样 C 与 C++ 两侧使用完全相同的符号名，链接才能成功。
//
// 三、谁调用这里的函数
//   - AppCpp_Initialize()：由 CubeMX 生成的 main.c 在初始化阶段调用（全部
//     外设初始化完成之后、启动 FreeRTOS 之前），用于装配并启动插件图。
//   - AppCpp_ControlTask()：作为 FreeRTOS 任务的入口函数；任务创建代码位于
//     CubeMX 生成的 freertos.c 中。
//   - AppCpp_ProcessFrame()：由 AppCpp_ControlTask()（定义于
//     ControlExecutorTask.cpp）在每个控制周期内调用，用来驱动一帧插件图。
//=============================================================================

// 该头文件必须同时能被 CubeMX 的 C 文件和 C++ 文件包含。
// 仅在 C++ 编译环境下展开 extern "C"，使下列函数以 C 符号名导出/链接。
#ifdef __cplusplus
extern "C" {
#endif

// C 侧只调用这三个桥接函数，不直接接触 C++ 对象。
// 初始化 C++ 应用：装配插件图并完成启动前的准备工作。
// 调用时机：CubeMX 生成的 main.c，在全部外设初始化完成后调用一次。
// 参数：无。返回值：无（初始化结果保存在 C++ 应用对象内部）。
void AppCpp_Initialize(void);
// 执行一帧 C++ 插件图：采集输入 → 逐节点计算 → 下发输出。
// 调用时机：由 AppCpp_ControlTask() 在每个 1 ms 控制周期内调用一次。
// 参数：无。返回值：无。
void AppCpp_ProcessFrame(void);
// FreeRTOS 控制任务入口：以固定周期循环调用 AppCpp_ProcessFrame()。
// 定义位置：ControlExecutorTask.cpp；由 CubeMX 生成的 freertos.c 创建任务时引用。
// 参数 argument：FreeRTOS 创建任务时传入的通用指针参数，本项目未使用。
// 返回值：无（任务函数内部为死循环，不会返回）。
void AppCpp_ControlTask(void const* argument);

#ifdef __cplusplus
}
#endif

#endif /* robot_app_cpp_entry_h */
