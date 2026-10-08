#include "app_cpp_entry.h"

#include "RobotApplication.hpp"

namespace {
robot::application::RobotApplication g_application;
}

extern "C" void AppCpp_Initialize(void)
{
    (void)g_application.initialize();
}

extern "C" void AppCpp_ProcessFrame(void)
{
    (void)g_application.processFrame();
}
