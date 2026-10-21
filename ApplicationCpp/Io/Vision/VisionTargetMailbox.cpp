/* ============================================================================
 * 文件：VisionTargetMailbox.cpp
 * 作用：VisionTargetMailbox.hpp 的实现。
 * 线程安全：发布者（Gimbal 任务）与读取者（Vision 任务等）跨任务访问，
 *           由 interrupt_snapshot_lock 的短临界区保证整份快照原子读写。
 * ========================================================================== */
#include "VisionTargetMailbox.hpp"

#include "InterruptSnapshotLock.hpp"

namespace 
{

    // 邮箱与同步后端：先声明锁，再声明邮箱（邮箱构造时保存锁的引用）。
    robot::platform::stm32::interrupt_snapshot_lock g_vision_target_lock{};
    robot::framework::latest_snapshot_mailbox<robot::io::vision::vision_target_snapshot>
        g_vision_target_mailbox{g_vision_target_lock};

} // namespace

namespace robot::io::vision 
{

    robot::framework::latest_snapshot_mailbox<vision_target_snapshot>& vision_target_mailbox()
    {
        return g_vision_target_mailbox;
    }

} // namespace robot::io::vision
