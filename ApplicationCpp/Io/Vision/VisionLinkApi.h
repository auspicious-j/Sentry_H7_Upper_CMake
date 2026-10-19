/* ============================================================================
 * 文件：VisionLinkApi.h
 * 作用：视觉接收链路的 C 接口，供 C 侧（USB 中断回调、VisionTask 循环）调用；
 *       实现在 ApplicationCpp/Io/Vision/VisionLink.cpp。
 * 调用约定：
 *   - VisionLink_OnUsbReceive() 只能从中断上下文调用（只搬字节，不解析）；
 *   - VisionLink_Poll() / VisionLink_WaitData() 只能在 VisionTask 中调用。
 * ========================================================================== */
#ifndef robot_io_vision_link_api_h
#define robot_io_vision_link_api_h

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 中断侧入口：把 USB 收到的原始字节搬进环形缓冲，并唤醒 VisionTask。
// 该函数不做任何解析，可在中断上下文安全调用。
void VisionLink_OnUsbReceive(const uint8_t* data, uint32_t length);

// 任务侧入口：取出环形缓冲中的全部字节喂给消帧器，解出完整帧后发布到
// 快照邮箱。首次调用时自动登记当前任务为中断通知目标。
void VisionLink_Poll(void);

// 任务侧等待：阻塞直到收到中断通知或超时（毫秒）。
// 返回后应再次调用 VisionLink_Poll() 处理数据。
void VisionLink_WaitData(uint32_t max_wait_ms);

#ifdef __cplusplus
}
#endif

#endif /* robot_io_vision_link_api_h */
