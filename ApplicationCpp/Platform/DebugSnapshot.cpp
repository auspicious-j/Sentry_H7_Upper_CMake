#include "DebugSnapshot.hpp"

// 使用未修饰的 C 符号名，避免 C++ 命名空间导致 Watch 无法解析。
extern "C" volatile robot::platform::RobotDebugSnapshot g_robot_debug{};
