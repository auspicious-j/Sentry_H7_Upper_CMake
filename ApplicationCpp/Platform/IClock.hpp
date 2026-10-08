#ifndef ROBOT_CLOCK_HPP
#define ROBOT_CLOCK_HPP

#include <cstdint>

namespace robot::platform {

// 平台无关的单调时钟接口；STM32 使用 DWT，未来 Linux 可替换后端。
class IClock {
public:
    virtual ~IClock() = default;
    virtual uint64_t nowUs() const = 0;
};

} // namespace robot::platform

#endif /* ROBOT_CLOCK_HPP */
