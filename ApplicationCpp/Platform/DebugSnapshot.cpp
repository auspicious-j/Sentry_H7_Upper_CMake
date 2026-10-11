/**
 * @file DebugSnapshot.cpp
 * @brief 定义全局调试快照实例 g_robot_debug。
 *
 * 该实例是整个工程唯一的调试快照载体：框架各节点把关心的状态写进它的子结构体，
 * 调试器（Keil / Ozone / VS Code）通过符号名 g_robot_debug 直接观察或改写其中字段。
 * 这里用 {} 进行值初始化，保证上电后所有字段都具有确定的初值（整体清零）。
 */

#include "DebugSnapshot.hpp"

// 使用未修饰的 C 符号名（extern "C"），避免 C++ 命名空间 / 名字修饰导致调试器无法解析该符号。
// 这里的 volatile 与 DebugSnapshot.hpp 中的类型定义保持一致，确保编译器不会优化掉对该实例的读写。
extern "C" volatile robot::platform::robot_debug_snapshot g_robot_debug{};
