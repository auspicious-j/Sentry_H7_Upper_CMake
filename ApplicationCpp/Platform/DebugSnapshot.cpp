#include "DebugSnapshot.hpp"

namespace robot::platform {

// 只保留一个全局调试快照实例，避免散落的业务全局变量。
volatile RobotDebugSnapshot g_robot_debug{};

} // namespace robot::platform
