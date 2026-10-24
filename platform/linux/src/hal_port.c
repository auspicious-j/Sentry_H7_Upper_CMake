/**
 ******************************************************************************
 * @file    hal_port.c
 * @brief   [LINUX-PORT] STM32 HAL 在 Ubuntu 上的替代实现
 *
 * 对应关系：
 *   HAL_GetTick()            ->  clock_gettime(CLOCK_MONOTONIC)      （Linux/POSIX）
 *   HAL_Delay()              ->  clock_nanosleep(CLOCK_MONOTONIC)    （Linux/POSIX）
 *   HAL_NVIC_SystemReset()   ->  exit(EXIT_SUCCESS)
 *   FDCAN / UART / TIM / SPI ->  空实现，通信与外设输出暂时停用
 *
 * 后续接入真实硬件通信时，只需要替换本文件中的函数体即可，
 * 业务层与中间件层不需要改动。
 ******************************************************************************
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include <errno.h>
#include <unistd.h>

#include "port_linux.h"
#include "stm32h7xx_hal.h"
#include "usart.h"
#include "tim.h"
#include "spi.h"
#include "dma.h"
#include "fdcan.h"
#include "usbd_cdc_if.h"

/* ---------------- [LINUX-PORT] 虚拟外设句柄定义 ---------------- */
FDCAN_HandleTypeDef hfdcan1 = {0};
FDCAN_HandleTypeDef hfdcan2 = {0};
FDCAN_HandleTypeDef hfdcan3 = {0};

UART_HandleTypeDef huart1 = {0};
UART_HandleTypeDef huart2 = {0};
UART_HandleTypeDef huart5 = {0};

TIM_HandleTypeDef htim3 = {.Instance = TIM3};
TIM_HandleTypeDef htim12 = {.Instance = TIM12};

SPI_HandleTypeDef hspi2 = {0};

DMA_HandleTypeDef hdma_usart1_rx = {0};
DMA_HandleTypeDef hdma_usart2_rx = {0};
DMA_HandleTypeDef hdma_uart5_rx = {0};

static FDCAN_GlobalTypeDef s_fdcan1_instance = {0};
static FDCAN_GlobalTypeDef s_fdcan2_instance = {0};
static FDCAN_GlobalTypeDef s_fdcan3_instance = {0};

/* ---------------- [LINUX-PORT] 日志 ---------------- */
static bool s_verbose = false;

bool port_verbose_enabled(void)
{
    return s_verbose;
}

void port_log(const char *fmt, ...)
{
    if (!s_verbose)
        return;

    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fflush(stderr);
}

void port_init(void)
{
    const char *env = getenv("SENTRY_PORT_VERBOSE");
    s_verbose = (env != NULL) && (env[0] != '\0') && (env[0] != '0');

    /* [LINUX-PORT] 让 CAN/串口句柄的 Instance 指针有合法指向，
     * 供 UserFreertos.c 里的 CLEAR_BIT(hfdcan->Instance->CCCR, ...) 使用 */
    hfdcan1.Instance = &s_fdcan1_instance;
    hfdcan2.Instance = &s_fdcan2_instance;
    hfdcan3.Instance = &s_fdcan3_instance;

    port_log("[port] Linux 移植层初始化完成 (verbose=%d)\n", (int)s_verbose);
}

/* ---------------- [LINUX-PORT] 时间基准 ---------------- */
uint64_t port_monotonic_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

uint32_t port_monotonic_ms(void)
{
    return (uint32_t)(port_monotonic_us() / 1000ULL);
}

void port_sleep_ms(uint32_t ms)
{
    if (ms == 0U)
        return;

    struct timespec req;
    req.tv_sec = (time_t)(ms / 1000U);
    req.tv_nsec = (long)(ms % 1000U) * 1000000L;

    /* 处理 EINTR：保证至少睡够 ms 毫秒 */
    while (clock_nanosleep(CLOCK_MONOTONIC, 0, &req, &req) == EINTR)
    {
    }
}

uint32_t HAL_GetTick(void)
{
    return port_monotonic_ms();
}

void HAL_Delay(uint32_t Delay)
{
    port_sleep_ms(Delay);
}

void HAL_NVIC_SystemReset(void)
{
    /* [LINUX-PORT] Linux 上没有软复位；等价行为是退出进程，由外部脚本/watchdog 重启 */
    port_log("[port] HAL_NVIC_SystemReset() -> 退出进程\n");
    exit(EXIT_SUCCESS);
}

HAL_StatusTypeDef HAL_Init(void)
{
    return HAL_OK;
}

/* ---------------- [LINUX-PORT] GPIO / TIM / UART / SPI / FDCAN 空实现 ---------------- */

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
    (void)GPIOx;
    (void)GPIO_Pin;
    (void)PinState;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    (void)htim;
    (void)Channel;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    (void)htim;
    (void)Channel;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint32_t Length)
{
    (void)htim;
    (void)Channel;
    (void)pData;
    (void)Length;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    (void)htim;
    (void)Channel;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    /* [LINUX-PORT] 串口接收暂时停用。
     * 后续可用 termios + poll() 读取 /dev/ttyUSBx，收到数据后回调
     * HAL_UARTEx_RxEventCallback()，接口保持不变。 */
    (void)huart;
    (void)pData;
    (void)Size;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    /* [LINUX-PORT] 串口发送暂时停用（可用 write() 到 termios 句柄实现） */
    (void)huart;
    (void)pData;
    (void)Size;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData,
                                          uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
{
    (void)hspi;
    (void)pTxData;
    (void)Timeout;
    if (pRxData != NULL)
        memset(pRxData, 0, Size);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *hfdcan, FDCAN_FilterTypeDef *sFilterConfig)
{
    (void)hfdcan;
    (void)sFilterConfig;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *hfdcan, uint32_t NonMatchingStd,
                                               uint32_t NonMatchingExt, uint32_t RejectRemoteStd,
                                               uint32_t RejectRemoteExt)
{
    (void)hfdcan;
    (void)NonMatchingStd;
    (void)NonMatchingExt;
    (void)RejectRemoteStd;
    (void)RejectRemoteExt;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_ConfigFifoWatermark(FDCAN_HandleTypeDef *hfdcan, uint32_t Fifo, uint32_t Watermark)
{
    (void)hfdcan;
    (void)Fifo;
    (void)Watermark;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *hfdcan, uint32_t ActiveITs, uint32_t BufferIndexes)
{
    (void)hfdcan;
    (void)ActiveITs;
    (void)BufferIndexes;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *hfdcan, uint32_t RxLocation,
                                         FDCAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData)
{
    (void)hfdcan;
    (void)RxLocation;
    (void)RxHeader;
    (void)RxData;
    return HAL_ERROR; /* 没有数据 */
}

HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *TxHeader,
                                                uint8_t *TxData)
{
    (void)hfdcan;
    (void)TxHeader;
    (void)TxData;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *hfdcan, FDCAN_ProtocolStatusTypeDef *ProtocolStatus)
{
    (void)hfdcan;
    if (ProtocolStatus != NULL)
        memset(ProtocolStatus, 0, sizeof(*ProtocolStatus));
    return HAL_OK;
}

/* ---------------- [LINUX-PORT] USB CDC ---------------- */
uint8_t CDC_Transmit_HS(uint8_t *Buf, uint16_t Len)
{
    /* [LINUX-PORT] 向上位机发送数据暂时停用。
     * 后续可改成 write() 到串口 / UDP socket / ROS2 话题。 */
    (void)Buf;
    (void)Len;
    return USBD_OK;
}
