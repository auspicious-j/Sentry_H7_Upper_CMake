/* ============================================================================
 * 文件：IrqDispatch.hpp
 * 作用：平台层的中断入口路由。全工程只有这里定义 HAL 的「共享弱符号」回调，
 *       它不含任何设备知识，只负责「认实例 → 查表 → 转发」。
 *
 * 为什么需要它：
 *   HAL 的 HAL_xxxCallback 是弱符号，全工程只能定义一份，但一个回调往往服务
 *   多个外设实例（UART5 遥控 / USART2 板间、FDCAN1/2/3）。把「谁被谁调用」
 *   集中在这里、把「收到数据后干什么」留在各自设备模块，新增外设时只需要在
 *   自己的初始化里登记一次，不必再去改别人的文件。
 *
 * 约束（都是硬约束，违反会静默丢数据）：
 *   - 只允许在中断上下文被 HAL 调用；
 *   - 登记必须在对应外设开中断（HAL_UARTEx_ReceiveToIdle_DMA）之前完成，
 *     并且只在外设初始化阶段登记；登记完成后表只读，ISR 内不加锁；
 *   - 表的容量固定，运行期不分配内存；
 *   - 同一个 UART 实例只允许登记一次，重复登记返回 false。
 * ========================================================================== */
#ifndef robot_platform_stm32_irq_dispatch_hpp
#define robot_platform_stm32_irq_dispatch_hpp

#include <cstdint>

/* HAL 是 C 头文件；C++ 里必须整体包在 extern "C" 中，否则弱符号回调的
   链接名会被当成 C++ 名字修饰，与 HAL 库里的 C 符号对不上。 */
extern "C" 
{
   #include "stm32h7xx_hal.h"
}

namespace robot::platform::stm32 
{

   // UART 接收事件处理函数：在 HAL 的 RxEvent（IDLE / RTO / 半传输）上下文调用。
   // 参数 received_size 是本次事件对应的已接收字节数，由 HAL 给出。
   using uart_rx_event_handler = void (*)(std::uint16_t received_size);

   // 把某个 UART 实例的接收事件绑定到处理函数。
   // 返回：true 登记成功；false 表示参数为空、实例重复登记或表已满。
   // 调用时机：对应外设的 *_Init() 中、调用 HAL_UARTEx_ReceiveToIdle_DMA() 之前。
   bool irq_register_uart_rx(UART_HandleTypeDef* huart, uart_rx_event_handler handler);

   // 内部接口：由 IrqDispatch.cpp 中的 HAL 弱符号实现调用，按实例查表转发。
   // 未登记的事件被丢弃（不回调、不计数），保证路由层不引入新的失败路径。
   void irq_dispatch_uart_rx(UART_HandleTypeDef* huart, std::uint16_t received_size);

} // namespace robot::platform::stm32

#endif /* robot_platform_stm32_irq_dispatch_hpp */
