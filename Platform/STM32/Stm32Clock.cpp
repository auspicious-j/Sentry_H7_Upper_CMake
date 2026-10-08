#include "Stm32Clock.hpp"

#include "bsp_dwt.h"

namespace robot::platform::stm32 {

// DWT 用于高精度测量，不用于替代 FreeRTOS 任务调度。
uint64_t Stm32Clock::nowUs() const
{
    return DWT_GetTimeline_us();
}

} // namespace robot::platform::stm32
