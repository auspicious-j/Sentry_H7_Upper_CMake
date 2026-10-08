#ifndef ROBOT_STM32_CLOCK_HPP
#define ROBOT_STM32_CLOCK_HPP

#include "../../ApplicationCpp/Platform/IClock.hpp"

namespace robot::platform::stm32 {

// STM32 时钟后端；DWT 初始化仍由现有 CubeMX 用户代码负责。
class Stm32Clock final : public robot::platform::IClock {
public:
    // 返回单调微秒时间。
    uint64_t nowUs() const override;
};

} // namespace robot::platform::stm32

#endif /* ROBOT_STM32_CLOCK_HPP */
