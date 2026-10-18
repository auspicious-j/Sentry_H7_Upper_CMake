/* ============================================================================
 * 文件：VisionLink.hpp
 * 作用：视觉链路（中断搬运 + 任务解析 + 快照发布）的 C++ 视图。
 *       - 中断入口与任务循环的 C 接口见 VisionLinkApi.h；
 *       - 本头文件只暴露“消费者读取快照”的接口，供图节点使用。
 * 线程模型：
 *   USB 中断（OTG_HS，优先级 5）--memcpy--> 环形缓冲 --通知--> VisionTask
 *   --> 消帧/解码 --> latest_snapshot_mailbox --> 各图节点按帧复制读取。
 * ========================================================================== */
#ifndef robot_io_vision_link_hpp
#define robot_io_vision_link_hpp

#include <cstdint>

#include "SnapshotMailbox.hpp"
#include "VisionProtocol.hpp"

namespace robot::io::vision {

// 视觉链路新鲜度阈值：超过该年龄的快照视为过期（视觉端正常约 100Hz）。
inline constexpr std::uint64_t k_vision_link_ttl_us = 100000U; // 100ms

// 视觉快照邮箱：生产者在 VisionTask 中发布，消费者在各自任务中读取。
// 跨任务访问由邮箱内部的短临界区锁保证一致；读方应“整份复制后锁外判断”，
// 并用 evaluate_snapshot(copy, now_us, k_vision_link_ttl_us) 得到新鲜度。
robot::framework::latest_snapshot_mailbox<vision_frame_snapshot>& vision_frame_mailbox();

// 环形缓冲因空间不足而整包丢弃的累计次数（诊断用）。
std::uint32_t vision_link_overflow_count();

} // namespace robot::io::vision

#endif // robot_io_vision_link_hpp
