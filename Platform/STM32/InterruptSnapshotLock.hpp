#ifndef ROBOT_STM32_INTERRUPT_SNAPSHOT_LOCK_HPP
#define ROBOT_STM32_INTERRUPT_SNAPSHOT_LOCK_HPP

#if defined(__ARMCC_VERSION)
#include "cmsis_armclang.h"
#else
#include "cmsis_gcc.h"
#endif
#include "../../ApplicationCpp/Framework/SnapshotMailbox.hpp"

namespace robot::platform::stm32 {

// 单核 STM32 的短快照临界区；只保护复制，不做计算或阻塞。
class InterruptSnapshotLock final : public robot::framework::ISnapshotLock {
public:
    // 保存当前中断屏蔽状态并进入临界区。
    void lock() override
    {
        saved_primask_ = __get_PRIMASK();
        __disable_irq();
        __DMB();
    }

    // 恢复进入临界区前的中断状态。
    void unlock() override
    {
        __DMB();
        __set_PRIMASK(saved_primask_);
    }

private:
    uint32_t saved_primask_{0U}; // 进入前的 PRIMASK。
};

} // namespace robot::platform::stm32
#endif // ROBOT_STM32_INTERRUPT_SNAPSHOT_LOCK_HPP

