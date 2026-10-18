/* ============================================================================
 * 文件：VisionLink.cpp
 * 作用：视觉链路的传输层实现：
 *         USB 中断：memcpy 到 SPSC 环形缓冲 + 任务通知（微秒级，不做解析）
 *         VisionTask：取字节 → 消帧/解码 → 发布到快照邮箱
 * 设计要点：
 *   - 环形缓冲是单生产者(中断)/单消费者(任务)队列，只需 volatile 索引 +
 *     内存屏障，不需要加锁；
 *   - 缓冲满时整包丢弃并计数，保持字节流边界，交给消帧器重新同步；
 *   - 中断里调用 vTaskNotifyGiveFromISR 是安全的：OTG_HS 中断优先级为 5，
 *     等于 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY，允许调用 FromISR API。
 * ========================================================================== */
#include "VisionLink.hpp"
#include "VisionLinkApi.h"

#include "FreeRTOS.h"
#include "task.h"

#include "InterruptSnapshotLock.hpp"
#include "Stm32Clock.hpp"

namespace {

using robot::framework::latest_snapshot_mailbox;
using robot::io::vision::vision_frame_parser;
using robot::io::vision::vision_frame_snapshot;
using robot::platform::stm32::interrupt_snapshot_lock;
using robot::platform::stm32::stm32_clock;

// SPSC 环形缓冲容量：必须是 2 的幂，便于用掩码取模。
// 视觉端约 100Hz × 88B ≈ 8.8KB/s，1KB 足以吸收 USB 包的突发。
constexpr std::uint32_t k_ring_capacity = 1024U;
constexpr std::uint32_t k_ring_mask = k_ring_capacity - 1U;

std::uint8_t g_ring[k_ring_capacity]{};   // 环形缓冲本体。
volatile std::uint32_t g_write_index{0U}; // 只由中断写。
volatile std::uint32_t g_read_index{0U};  // 只由 VisionTask 写。
volatile std::uint32_t g_overflow_count{0U}; // 整包丢弃次数（诊断用）。

// 中断通知目标：VisionTask 首次 Poll 时登记。
volatile TaskHandle_t g_consumer_handle{nullptr};

// 快照邮箱及其同步后端。二者都在任务上下文使用（发布/复制），
// 锁实现为极短的关中断临界区，满足 i_snapshot_lock 的约定。
interrupt_snapshot_lock g_mailbox_lock{};
latest_snapshot_mailbox<vision_frame_snapshot> g_vision_mailbox{g_mailbox_lock};

stm32_clock g_clock{};           // 发布快照时打时间戳（DWT 微秒）。
vision_frame_parser g_parser{};  // 消帧器状态（只在 VisionTask 访问）。

} // namespace

namespace robot::io::vision {

latest_snapshot_mailbox<vision_frame_snapshot>& vision_frame_mailbox()
{
    return g_vision_mailbox;
}

std::uint32_t vision_link_overflow_count()
{
    return g_overflow_count;
}

} // namespace robot::io::vision

extern "C" void VisionLink_OnUsbReceive(const uint8_t* data, uint32_t length)
{
    if ((data == nullptr) || (length == 0U))
    {
        return;
    }

    // 单生产者视角：写索引由本中断推进，读索引由任务推进。
    const std::uint32_t write = g_write_index;
    const std::uint32_t used = write - g_read_index; // 无符号回绕即正确。
    if (length > (k_ring_capacity - used))
    {
        // 空间不足：整包丢弃（而不是截断），避免把半包数据塞进字节流。
        ++g_overflow_count;
        return;
    }

    for (std::uint32_t i = 0U; i < length; ++i)
    {
        g_ring[(write + i) & k_ring_mask] = data[i];
    }

    __DMB(); // 先发布数据，再发布写索引。
    g_write_index = write + length;

    const TaskHandle_t consumer = g_consumer_handle;
    if (consumer != nullptr)
    {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(consumer, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

extern "C" void VisionLink_Poll(void)
{
    // 首次调用（VisionTask 上下文）登记为中断通知目标。
    if (g_consumer_handle == nullptr)
    {
        g_consumer_handle = xTaskGetCurrentTaskHandle();
    }

    std::uint8_t chunk[64];
    for (;;)
    {
        const std::uint32_t write = g_write_index;
        const std::uint32_t used = write - g_read_index;
        if (used == 0U)
        {
            break;
        }

        const std::uint32_t take = (used < sizeof(chunk)) ? used : static_cast<std::uint32_t>(sizeof(chunk));
        const std::uint32_t read = g_read_index;
        for (std::uint32_t i = 0U; i < take; ++i)
        {
            chunk[i] = g_ring[(read + i) & k_ring_mask];
        }

        __DMB(); // 先读完数据，再发布读索引。
        g_read_index = read + take;

        if (g_parser.feed(chunk, take))
        {
            // 解出完整帧：发布最新快照（同块多帧时最新值覆盖，属预期语义）。
            g_vision_mailbox.publish(g_parser.last_frame(), g_clock.now_us());
        }
    }
}

extern "C" void VisionLink_WaitData(uint32_t max_wait_ms)
{
    const TaskHandle_t consumer = g_consumer_handle;
    if (consumer == nullptr)
    {
        return;
    }
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(max_wait_ms));
}
