#ifndef ROBOT_CLOCK_HPP
#define ROBOT_CLOCK_HPP

#include <cstdint>

namespace robot::platform {

class IClock {
public:
    virtual ~IClock() = default;
    virtual uint64_t nowUs() const = 0;
};

} // namespace robot::platform

#endif /* ROBOT_CLOCK_HPP */
