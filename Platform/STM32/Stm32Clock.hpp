#ifndef ROBOT_STM32_CLOCK_HPP
#define ROBOT_STM32_CLOCK_HPP

#include "../../ApplicationCpp/Platform/IClock.hpp"

namespace robot::platform::stm32 {

class Stm32Clock final : public robot::platform::IClock {
public:
    uint64_t nowUs() const override;
};

} // namespace robot::platform::stm32

#endif /* ROBOT_STM32_CLOCK_HPP */
