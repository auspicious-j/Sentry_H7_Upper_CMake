/* ============================================================================
 * 文件：IrqDispatch.cpp
 * 作用：HAL 共享中断入口的唯一实现 + 按实例分发的静态路由表。
 *
 * 设计：
 *   - 表是固定容量的静态数组（默认 4 项），运行期零分配；
 *   - 登记只发生在初始化阶段（调度器启动前），之后表只读，ISR 里裸查表，
 *     不需要再加临界区；
 *   - 这里不允许出现任何业务判断：不解析协议、不判断模式、不写 C 全局量。
 *     收到数据该做什么，由各设备模块登记的 handler 决定。
 * ========================================================================== */
#include "IrqDispatch.hpp"

namespace robot::platform::stm32 
{
    namespace 
    {
        
        // 表容量：当前只有 UART5（遥控）与 USART2（板间）在用，留两项余量。
        // 超出后 irq_register_uart_rx() 返回 false，由调用方决定是否继续开中断。
        constexpr std::uint32_t k_uart_rx_capacity = 4U;

        struct uart_rx_binding
        {
            UART_HandleTypeDef* huart;      // 外设实例（查表键）。
            uart_rx_event_handler handler;  // 该实例的接收事件处理函数。
        };

        uart_rx_binding g_uart_rx[k_uart_rx_capacity]{};
        std::uint32_t g_uart_rx_count{0U};

    } // namespace

    // 注册UART
    bool irq_register_uart_rx(UART_HandleTypeDef* huart, uart_rx_event_handler handler)
    {
        if ((huart == nullptr) || (handler == nullptr))
        {
            return false;
        }

        // 检查重复登记与超限
        for (std::uint32_t index = 0U; index < g_uart_rx_count; ++index)
        {
            if (g_uart_rx[index].huart == huart)
            {
                return false;
            }
        }
        if (g_uart_rx_count >= k_uart_rx_capacity)
        {
            return false;
        }

        g_uart_rx[g_uart_rx_count].huart = huart;
        g_uart_rx[g_uart_rx_count].handler = handler;
        ++g_uart_rx_count;
        return true;
    }

    // 查表
    void irq_dispatch_uart_rx(UART_HandleTypeDef* huart, std::uint16_t received_size)
    {
        for (std::uint32_t index = 0U; index < g_uart_rx_count; ++index)
        {
            if (g_uart_rx[index].huart == huart)
            {
                g_uart_rx[index].handler(received_size);
                return;
            }
        }
        // 未登记的实例：直接丢弃。这里刻意不调用任何默认处理，避免掩盖初始化遗漏。
    }

} // namespace robot::platform::stm32

/* ----------------------------------------------------------------------------
 * HAL 共享入口：全工程唯一的 HAL_UARTEx_RxEventCallback 定义。
 * 它只做一件事——把 (实例, 字节数) 交给路由表。设备是谁、要做什么，这里不知道。
 * -------------------------------------------------------------------------- */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    robot::platform::stm32::irq_dispatch_uart_rx(huart, static_cast<std::uint16_t>(Size));
}
