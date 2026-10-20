/* ============================================================================
 * 文件：interrupt_snapshot_lock.hpp
 * 作用：为单核 STM32 平台提供“短快照”临界区锁，实现框架的 i_snapshot_lock 接口，
 *       供快照邮箱(SnapshotMailbox)在复制共享数据时独占 CPU。
 * 原理：Cortex-M 只有一条执行流，靠 PRIMASK 屏蔽所有可屏蔽中断即可构成临界区。
 *       加锁时保存并屏蔽，解锁时精确恢复保存值，避免破坏外层已有的屏蔽状态。
 * 注意：这是“只保护拷贝”的极短临界区，锁内不得做计算、分配或阻塞，
 *       否则会显著增大中断延迟。
 * 可移植性：下方按编译器选择 CMSIS 头（ARMCC 用 cmsis_armclang.h，其它用
 *       cmsis_gcc.h），该选择逻辑极其关键，请勿改动其 #include。
 * ========================================================================== */

#ifndef robot_stm32_interrupt_snapshot_lock_hpp
#define robot_stm32_interrupt_snapshot_lock_hpp

/* cmsis_gcc.h 中的 __cmsis_start() 使用了“函数内局部类型 + extern 声明”的写法，
   这是合法 C、但非法 C++ 语法（本头文件会被 C++ 源文件包含）。
   在包含 CMSIS 之前把 __PROGRAM_START 定义为空，跳过该函数体即可；
   __PROGRAM_START 只在启动代码里使用，本编译单元不需要它。 */
#ifndef __PROGRAM_START
#define __PROGRAM_START
#endif

#if defined(__ARMCC_VERSION)
#include "cmsis_armclang.h"
#else
#include "cmsis_gcc.h"
#endif
#include "../../ApplicationCpp/Framework/SnapshotMailbox.hpp"

namespace robot::platform::stm32 
{

    // 单核 STM32 的短快照临界区；只保护复制，不做计算或阻塞。
    // 继承 robot::framework::i_snapshot_lock，是框架锁接口的平台实现；
    // final 表示不再被继承，便于编译器内联 lock()/unlock() 降低开销。
    class interrupt_snapshot_lock final : public robot::framework::i_snapshot_lock 
    {
    public:
        // lock()：保存当前 PRIMASK 并进入临界区。
        // PRIMASK 是 Cortex-M 的中断屏蔽位：置 1 后，除 NMI 与 HardFault 之外的所有
        // 可屏蔽异常都被挂起。这里先存档原值再关中断，是为了 unlock() 能精确还原，
        // 而不是无脑开中断（可能外层本就处于关中断状态）。
        // __DMB() 为内存屏障：保证屏障之前的内存访问先于其后访问完成，防止数据访问
        // 被乱序，确保临界区内外观察到的内存顺序一致、写入对他方可见。
        // 调用上下文：任务上下文调用；lock() 之后必须尽快 unlock()，其间禁止阻塞。
        void lock() override
        {
            saved_primask_ = __get_PRIMASK();
            __disable_irq();
            __DMB();
        }

        // unlock()：退出临界区并恢复进入前的中断状态。
        // 先插一条内存屏障，保证临界区内对共享数据的写入有序且对其他执行流可见，
        // 再通过 __set_PRIMASK 精确恢复 lock() 时保存的屏蔽状态（原先若已关中断则
        // 保持关闭，避免误开中断带来的嵌套/重入问题）。
        // 调用上下文：任务上下文，与 lock() 成对使用。
        void unlock() override
        {
            __DMB();
            __set_PRIMASK(saved_primask_);
        }

    private:
        // 进入临界区前 PRIMASK 的原始值(0=中断开启，1=中断屏蔽)；
        // 默认 0U 保证对象零初始化时处于“未加锁/中断开启”的合理初值。
        uint32_t saved_primask_{0U}; // 进入前的 PRIMASK。
    };

} // namespace robot::platform::stm32
#endif // robot_stm32_interrupt_snapshot_lock_hpp

