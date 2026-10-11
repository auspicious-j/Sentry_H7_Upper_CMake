// ============================================================================
// 文件：bsp_ws2812.c
// 功能：WS2812（兼容 WS2812B）单总线智能彩灯驱动实现。
// 原理：用 TIM3 通道2 的 PWM 输出表示单总线数据位——逻辑 1 用较长的高电平、
//       逻辑 0 用较短的高电平；再把整帧波形数据交给 DMA，由硬件自动逐个搬进
//       定时器比较寄存器连续发出，全程不占用 CPU。发完保持约 50us 低电平复位。
// 说明：发送缓冲区被显式放到 AXI SRAM（RAM_D1），因为 STM32H7 的 DMA1
//       无法访问位于 DTCMRAM 的默认 .bss 段。
// ============================================================================
#include "bsp_ws2812.h"
#include "main.h"
#include "cmsis_os.h"
#include <string.h>

/*
 * STM32H7 的 DMA1 无法访问 DTCM 内存（DTCM 只与 CPU 内核直连）。
 * 而默认链接脚本把 .bss 段放进了 DTCMRAM，所以这里用 section 属性
 * 把 WS2812 的 PWM DMA 发送缓冲强制放到 AXI SRAM（RAM_D1）中，
 * 这样 DMA 才能正常读取并搬运这些占空比数据。
 */
 
// 将发送缓冲显式放入 RAM_D1（AXI SRAM），并按 32 字节对齐（便于 DMA 搬运）
__attribute__((section(".RAM_D1"), aligned(32)))
uint16_t WS2812_RGB_Buff[LED_1_NUM * DATA_LEN + WS2812_RST_NUM];  // PWM 占空比缓冲：每颗灯珠 DATA_LEN 个 + 末尾 WS2812_RST_NUM 个复位空闲周期

static volatile uint8_t ws2812_dma_busy = 0;   // DMA 忙标志：1=正在发送、0=空闲；volatile 防止编译器把它优化进寄存器而读到过期值

// 等待上一次 DMA 发送结束。若等待超过 2ms 仍未结束，则强制关闭 TIM3 通道2 的
// PWM+DMA 并清忙标志，避免某次传输卡死导致后续颜色永远发不出去。
static void WS2812_WaitReady(void)
{
    uint32_t tickstart = HAL_GetTick();   // 记录开始等待的毫秒时间戳，用于超时判断

    while (ws2812_dma_busy != 0U)
    {
        if ((HAL_GetTick() - tickstart) > 2U)   // 等待超过 2ms 判定为异常
        {
            HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_2);   // 强制停止 TIM3 通道2 的 PWM+DMA 输出
            ws2812_dma_busy = 0U;   // 清忙标志，允许后续重新发起传输
            break;
        }
    }
}

// 启动一次 DMA 发送：先把比较值清 0（输出低电平），再让 TIM3 通道2 的 PWM 由 DMA
// 自动把 WS2812_RGB_Buff 中的数据逐个搬进比较寄存器并输出波形。
static void WS2812_Transmit(void)
{
    WS2812_WaitReady();   // 先确保上一次发送已经结束，避免覆盖正在使用的缓冲

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);   // 起始先输出低电平，保证帧间复位
    ws2812_dma_busy = 1U;   // 置忙标志，表示开始发送
    if (HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_2, (uint32_t *)WS2812_RGB_Buff,
                              sizeof(WS2812_RGB_Buff) / sizeof(WS2812_RGB_Buff[0])) != HAL_OK)
    {
        ws2812_dma_busy = 0U;   // 启动失败则清忙标志，避免一直卡在忙状态
    }
} 


// 设置第 num 颗灯珠的颜色。
// 参数 num：灯珠序号，范围 [0, LED_1_NUM-1]，越界直接返回；当前仅 0 号有效。
// 参数 R/G/B：红/绿/蓝亮度，各 0~255（0 最暗、255 最亮）。
// 说明：WS2812 每颗灯珠的数据按“绿→红→蓝”（GRB）顺序发送，且每个字节高位(MSB)先发。
//       这里把每个字节的 8 个 bit 依次展开成对应占空比（1→WS_H、0→WS_L）填入发送缓冲，
//       最后启动 DMA 把这一帧数据发出去。
void WS2812_Set(uint16_t num, uint8_t R, uint8_t G, uint8_t B)
{
    if (num >= LED_1_NUM)   // 序号越界（当前只支持 1 颗灯珠），直接返回
    {
        return;
    }

    WS2812_WaitReady();   // 等上一帧发完，避免边发边改缓冲导致颜色出错

    uint32_t indexx = num * DATA_LEN;   // 该灯珠在发送缓冲中的起始下标（每颗灯珠占 DATA_LEN 个元素）
    for (uint8_t i = 0; i < 8; i++)   // 逐 bit 处理，i=0 时取到的正是最高位
    {
        WS2812_RGB_Buff[indexx + i]      = ((G << i) & 0x80U) ? WS_H : WS_L;   // 绿色字节第 i 位 → 缓冲第 0~7 槽
        WS2812_RGB_Buff[indexx + i + 8]  = ((R << i) & 0x80U) ? WS_H : WS_L;   // 红色字节第 i 位 → 缓冲第 8~15 槽
        WS2812_RGB_Buff[indexx + i + 16] = ((B << i) & 0x80U) ? WS_H : WS_L;   // 蓝色字节第 i 位 → 缓冲第 16~23 槽
    }

    WS2812_Transmit();   // 把填好的这一帧用 DMA 发出去
}

// DMA 发送完成回调（HAL 在 PWM 数据传输结束时调用）：关闭 DMA 方式的 PWM 输出、
// 把比较值清 0 让灯线保持低电平（形成复位信号），并清忙标志通知等待方可继续。
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)   // 只处理 TIM3（WS2812 所用定时器）的事件
    {
        HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_2);
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 0U);
        ws2812_dma_busy = 0U;   // 本帧发送结束，清忙标志
    }
}

// 初始化 WS2812 驱动：先把整个发送缓冲清零，再调用一次 Set(0,0,0,0) 把 0 号灯珠熄灭，
// 保证上电后灯珠处于熄灭状态。
void WS2812_Init(void)
{
    memset(WS2812_RGB_Buff, 0, sizeof(WS2812_RGB_Buff));   // 缓冲全部填 0（等价于全低电平）
    WS2812_Set(0, 0, 0, 0);
}

// FreeRTOS 任务函数：周期性让 0 号灯珠在绿色(0,50,0)与熄灭之间切换，形成约 1Hz 闪烁，
// 用作系统正常运行的指示灯。参数 argument 为任务参数，此处未使用。
void OS_LedCallback(void const * argument)
{
    WS2812_Init();
    for (;;)
    {
        WS2812_Set(0, 0, 50, 0);   // 点亮：红色 0、绿色 50、蓝色 0（绿色）
        osDelay(500);   // 保持 500ms
        WS2812_Set(0, 0, 0, 0);   // 熄灭
        osDelay(500);   // 保持 500ms
    }
}
