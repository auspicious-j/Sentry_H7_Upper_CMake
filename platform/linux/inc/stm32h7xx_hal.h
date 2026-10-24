/**
 ******************************************************************************
 * @file    stm32h7xx_hal.h
 * @brief   [LINUX-PORT] STM32 HAL 的 Linux 兼容层
 *
 * 原工程依赖 STM32Cube HAL（Drivers/STM32H7xx_HAL_Driver）。
 * 移植到 Ubuntu 后没有真实外设，这里用“Linux 等价物”替换：
 *
 *   HAL_GetTick() / HAL_Delay()  ->  clock_gettime(CLOCK_MONOTONIC) + nanosleep()   (platform/linux/src/hal_port.c)
 *   FDCAN / UART / TIM / SPI     ->  桩函数（通信相关代码按要求暂时停用）
 *   GPIO                         ->  无操作
 *
 * 说明：本文件只保留本项目实际用到的类型、常量和函数，不是完整的 HAL。
 *       后续要接入真实通信（SocketCAN / termios 串口）时，直接替换 hal_port.c 中的实现即可。
 ******************************************************************************
 */
#ifndef __STM32H7xx_HAL_H
#define __STM32H7xx_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/* ---------------- [LINUX-PORT] 通用宏（与原 HAL 保持一致） ---------------- */
#ifndef SET_BIT
#define SET_BIT(REG, BIT) ((REG) |= (BIT))
#endif
#ifndef CLEAR_BIT
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))
#endif
#ifndef READ_BIT
#define READ_BIT(REG, BIT) ((REG) & (BIT))
#endif
#ifndef UNUSED
#define UNUSED(X) ((void)(X))
#endif

#ifndef RESET
#define RESET 0U
#endif
#ifndef SET
#define SET 1U
#endif

/* ---------------- [LINUX-PORT] HAL 状态与基础类型 ---------------- */
typedef enum
{
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U,
    HAL_BUSY = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

/* GPIO：Linux 下没有物理引脚，仅保留枚举值让原有代码可以编译 */
typedef enum
{
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET
} GPIO_PinState;

typedef struct
{
    uint32_t dummy;
} GPIO_TypeDef;

#define GPIO_PIN_0 ((uint16_t)0x0001)
#define GPIO_PIN_3 ((uint16_t)0x0008)

/* ---------------- [LINUX-PORT] FDCAN ----------------
 * 原工程通过 bxCAN/FDCAN 与电机通信。Linux 侧推荐的等价方案是 SocketCAN
 * （linux/can.h + PF_CAN 套接字），但按需求通信代码暂时停用，
 * 因此这里只保留数据结构，函数体为空实现（见 hal_port.c）。
 */
#define FDCAN_CCCR_INIT 0x00000001U

#define FDCAN_STANDARD_ID 0x00000000U
#define FDCAN_EXTENDED_ID 0x00000001U

#define FDCAN_DATA_FRAME 0x00000000U
#define FDCAN_REMOTE_FRAME 0x00000001U

#define FDCAN_ESI_ACTIVE 0x00000000U
#define FDCAN_ESI_PASSIVE 0x00000002U

#define FDCAN_BRS_OFF 0x00000000U
#define FDCAN_BRS_ON 0x00000001U

#define FDCAN_CLASSIC_CAN 0x00000000U
#define FDCAN_FD_CAN 0x00000001U

#define FDCAN_NO_TX_EVENTS 0x00000000U

#define FDCAN_FILTER_MASK 0x00000000U
#define FDCAN_FILTER_RANGE 0x00000001U
#define FDCAN_FILTER_DUAL 0x00000002U
#define FDCAN_FILTER_TO_RXFIFO0 0x00000000U
#define FDCAN_FILTER_TO_RXFIFO1 0x00000001U
#define FDCAN_FILTER_REMOTE 0x00000002U
#define FDCAN_REJECT 0x00000000U
#define FDCAN_ACCEPT_IN_RX_FIFO0 0x00000001U
#define FDCAN_ACCEPT_IN_RX_FIFO1 0x00000002U

#define FDCAN_RX_FIFO0 0x00000000U
#define FDCAN_RX_FIFO1 0x00000001U
#define FDCAN_CFG_RX_FIFO0 0x00000000U
#define FDCAN_CFG_RX_FIFO1 0x00000001U

#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 0x00000001U
#define FDCAN_IT_BUS_OFF 0x00000002U

typedef struct
{
    volatile uint32_t CCCR; /* 仅用于兼容原代码中的 CLEAR_BIT(hfdcan->Instance->CCCR, ...) */
} FDCAN_GlobalTypeDef;

typedef struct
{
    FDCAN_GlobalTypeDef *Instance;
    uint32_t dummy;
} FDCAN_HandleTypeDef;

typedef struct
{
    uint32_t IdType;
    uint32_t FilterIndex;
    uint32_t FilterType;
    uint32_t FilterConfig;
    uint32_t FilterID1;
    uint32_t FilterID2;
} FDCAN_FilterTypeDef;

typedef struct
{
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t TxFrameType;
    uint32_t DataLength;
    uint32_t ErrorStateIndicator;
    uint32_t BitRateSwitch;
    uint32_t FDFormat;
    uint32_t TxEventFifoControl;
    uint32_t MessageMarker;
} FDCAN_TxHeaderTypeDef;

typedef struct
{
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t RxFrameType;
    uint32_t DataLength;
    uint32_t ErrorStateIndicator;
    uint32_t BitRateSwitch;
    uint32_t FDFormat;
    uint32_t RxTimestamp;
    uint32_t FilterIndex;
    uint32_t IsFilterMatchingFrame;
} FDCAN_RxHeaderTypeDef;

typedef struct
{
    uint32_t BusOff;
    uint32_t Warning;
    uint32_t ErrorPassive;
    uint32_t LastErrorCode;
    uint32_t DataLastErrorCode;
    uint32_t Activity;
    uint32_t ErrorCntr;
} FDCAN_ProtocolStatusTypeDef;

/* ---------------- [LINUX-PORT] UART ----------------
 * Linux 侧等价方案：termios（/dev/ttyUSBx）或 socket。
 */
typedef struct
{
    uint32_t Instance;
    uint32_t dummy;
} UART_HandleTypeDef;

/* ---------------- [LINUX-PORT] TIM ----------------
 * Linux 侧没有 PWM 外设；蜂鸣器/WS2812 均已桩化。
 */
typedef struct
{
    uint32_t Instance;
    uint32_t dummy;
} TIM_HandleTypeDef;

#define TIM3 ((uint32_t)0x03U)
#define TIM12 ((uint32_t)0x0CU)
#define TIM_CHANNEL_1 0x00000000U
#define TIM_CHANNEL_2 0x00000004U
#define TIM_CHANNEL_3 0x00000008U
#define TIM_CHANNEL_4 0x0000000CU

typedef struct
{
    uint32_t Instance;
} SPI_HandleTypeDef;

typedef struct
{
    uint32_t Instance;
} DMA_HandleTypeDef;

#define DMA_IT_HT 0x00000004U

/* ---------------- [LINUX-PORT] HAL API ---------------- */
HAL_StatusTypeDef HAL_Init(void);
void Error_Handler(void);

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t Delay);
void HAL_NVIC_SystemReset(void);

/* GPIO */
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);

/* TIM (PWM) */
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint32_t Length);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);

#define __HAL_TIM_SET_COMPARE(__HANDLE__, __CHANNEL__, __COMPARE__) \
    do                                                             \
    {                                                              \
        UNUSED(__HANDLE__);                                        \
        UNUSED(__CHANNEL__);                                       \
        UNUSED(__COMPARE__);                                       \
    } while (0)

#define __HAL_TIM_SET_AUTORELOAD(__HANDLE__, __AUTORELOAD__) \
    do                                                       \
    {                                                        \
        UNUSED(__HANDLE__);                                  \
        UNUSED(__AUTORELOAD__);                              \
    } while (0)

/* [LINUX-PORT] 兼容旧版 HAL 命名（Beep.c 使用 PascalCase 形式，H7 HAL 中不存在） */
#define __HAL_TIM_SetAutoreload(__HANDLE__, __AUTORELOAD__) \
    __HAL_TIM_SET_AUTORELOAD(__HANDLE__, __AUTORELOAD__)

#define __HAL_TIM_SetCompare(__HANDLE__, __CHANNEL__, __COMPARE__) \
    __HAL_TIM_SET_COMPARE(__HANDLE__, __CHANNEL__, __COMPARE__)

/* UART */
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

#define __HAL_DMA_DISABLE_IT(__HANDLE__, __INTERRUPT__) \
    do                                                  \
    {                                                   \
        UNUSED(__HANDLE__);                             \
        UNUSED(__INTERRUPT__);                          \
    } while (0)

/* SPI */
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData,
                                          uint8_t *pRxData, uint16_t Size, uint32_t Timeout);

/* FDCAN（全部为空实现） */
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *hfdcan, FDCAN_FilterTypeDef *sFilterConfig);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *hfdcan, uint32_t NonMatchingStd,
                                               uint32_t NonMatchingExt, uint32_t RejectRemoteStd,
                                               uint32_t RejectRemoteExt);
HAL_StatusTypeDef HAL_FDCAN_ConfigFifoWatermark(FDCAN_HandleTypeDef *hfdcan, uint32_t Fifo, uint32_t Watermark);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *hfdcan, uint32_t ActiveITs, uint32_t BufferIndexes);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *hfdcan, uint32_t RxLocation,
                                         FDCAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *TxHeader,
                                                uint8_t *TxData);
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *hfdcan, FDCAN_ProtocolStatusTypeDef *ProtocolStatus);

#ifdef __cplusplus
}
#endif

#endif /* __STM32H7xx_HAL_H */
