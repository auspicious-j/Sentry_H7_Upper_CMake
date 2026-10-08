#include "Stm32Clock.hpp"

#include "bsp_dwt.h"

namespace robot::platform::stm32 {

uint64_t Stm32Clock::nowUs() const
{
    return DWT_GetTimeline_us();
}

} // namespace robot::platform::stm32
