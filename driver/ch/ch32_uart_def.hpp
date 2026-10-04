#pragma once

#include "libxr.hpp"
#include DEF2STR(LIBXR_CH32_CONFIG_FILE)

/**
 * @brief CH32 UART 实例编号 / CH32 UART instance identifier
 */
typedef enum
{
#if defined(USART1)
  CH32_USART1,
#endif
#if defined(USART2)
  CH32_USART2,
#endif
#if defined(USART3)
  CH32_USART3,
#endif
#if defined(USART4)
  CH32_USART4,
#endif
#if defined(USART5)
  CH32_USART5,
#endif
#if defined(USART6)
  CH32_USART6,
#endif
#if defined(USART7)
  CH32_USART7,
#endif
#if defined(USART8)
  CH32_USART8,
#endif
#if defined(UART1)
  CH32_UART1,
#endif
#if defined(UART2)
  CH32_UART2,
#endif
#if defined(UART3)
  CH32_UART3,
#endif
#if defined(UART4)
  CH32_UART4,
#endif
#if defined(UART5)
  CH32_UART5,
#endif
#if defined(UART6)
  CH32_UART6,
#endif
#if defined(UART7)
  CH32_UART7,
#endif
#if defined(UART8)
  CH32_UART8,
#endif
  CH32_UART_NUMBER,
  CH32_UART_ID_ERROR
} ch32_uart_id_t;

static constexpr uint8_t CH32_UART_APB_MAP[] = {
#if defined(USART1)
    2,
#endif
#if defined(USART2)
    1,
#endif
#if defined(USART3)
    1,
#endif
#if defined(USART4)
    0,
#endif
#if defined(USART5)
    0,
#endif
#if defined(USART6)
    0,
#endif
#if defined(USART7)
    0,
#endif
#if defined(USART8)
    0,
#endif
#if defined(UART1)
    0,
#endif
#if defined(UART2)
    0,
#endif
#if defined(UART3)
    0,
#endif
#if defined(UART4)
    1,
#endif
#if defined(UART5)
    1,
#endif
#if defined(UART6)
    1,
#endif
#if defined(UART7)
    1,
#endif
#if defined(UART8)
    1,
#endif
};

static constexpr uint32_t CH32_UART_RCC_PERIPH_MAP[] = {
#if defined(USART1)
    RCC_APB2Periph_USART1,
#endif
#if defined(USART2)
    RCC_APB1Periph_USART2,
#endif
#if defined(USART3)
    RCC_APB1Periph_USART3,
#endif
#if defined(USART4)
    0,
#endif
#if defined(USART5)
    0,
#endif
#if defined(USART6)
    0,
#endif
#if defined(USART7)
    0,
#endif
#if defined(USART8)
    0,
#endif
#if defined(UART1)
    0,
#endif
#if defined(UART2)
    0,
#endif
#if defined(UART3)
    0,
#endif
#if defined(UART4)
    RCC_APB1Periph_UART4,
#endif
#if defined(UART5)
    RCC_APB1Periph_UART5,
#endif
#if defined(UART6)
    RCC_APB1Periph_UART6,
#endif
#if defined(UART7)
    RCC_APB1Periph_UART7,
#endif
#if defined(UART8)
    RCC_APB1Periph_UART8,
#endif
};

static constexpr uint32_t CH32_UART_RCC_PERIPH_MAP_DMA[] = {
#if defined(USART1)
    RCC_AHBPeriph_DMA1,
#endif
#if defined(USART2)
    RCC_AHBPeriph_DMA1,
#endif
#if defined(USART3)
    RCC_AHBPeriph_DMA1,
#endif
#if defined(USART4)
    RCC_AHBPeriph_DMA1,
#endif
#if defined(USART5)
    RCC_AHBPeriph_DMA1,
#endif
#if defined(USART6)
    RCC_AHBPeriph_DMA1,
#endif
#if defined(USART7)
    RCC_AHBPeriph_DMA1,
#endif
#if defined(USART8)
    RCC_AHBPeriph_DMA1,
#endif
#if defined(UART1)
    RCC_AHBPeriph_DMA2,
#endif
#if defined(UART2)
    RCC_AHBPeriph_DMA2,
#endif
#if defined(UART3)
    RCC_AHBPeriph_DMA2,
#endif
#if defined(UART4)
#if defined(CH32V20x_D6) || defined(CH32V20x_D8) || defined(CH32V20x_D8W)
    RCC_AHBPeriph_DMA1,
#else
    RCC_AHBPeriph_DMA2,
#endif
#endif
#if defined(UART5)
    RCC_AHBPeriph_DMA2,
#endif
#if defined(UART6)
    RCC_AHBPeriph_DMA2,
#endif
#if defined(UART7)
    RCC_AHBPeriph_DMA2,
#endif
#if defined(UART8)
    RCC_AHBPeriph_DMA2,
#endif
};

static constexpr uint32_t CH32_UART_TX_DMA_IT_MAP[] = {
#if defined(USART1)
    DMA1_IT_TC4,
#endif
#if defined(USART2)
    DMA1_IT_TC7,
#endif
#if defined(USART3)
    DMA1_IT_TC2,
#endif
#if defined(USART4)
    0,
#endif
#if defined(USART5)
    0,
#endif
#if defined(USART6)
    0,
#endif
#if defined(USART7)
    0,
#endif
#if defined(USART8)
    0,
#endif
#if defined(UART1)
    0,
#endif
#if defined(UART2)
    0,
#endif
#if defined(UART3)
    0,
#endif
#if defined(UART4)
#if !(defined(CH32V20x_D6) || defined(CH32V20x_D8) || defined(CH32V20x_D8W)) && \
    defined(DMA2_IT_TC5)
    DMA2_IT_TC5,
#else
    DMA1_IT_TC1,
#endif
#endif
#if defined(UART5)
    DMA2_IT_TC4,
#endif
#if defined(UART6)
    DMA2_IT_TC6,
#endif
#if defined(UART7)
    DMA2_IT_TC8,
#endif
#if defined(UART8)
    DMA2_IT_TC10,
#endif
};

static constexpr uint32_t CH32_UART_RX_DMA_IT_TC_MAP[] = {
#if defined(USART1)
    DMA1_IT_TC5,
#endif
#if defined(USART2)
    DMA1_IT_TC6,
#endif
#if defined(USART3)
    DMA1_IT_TC3,
#endif
#if defined(USART4)
    0,
#endif
#if defined(USART5)
    0,
#endif
#if defined(USART6)
    0,
#endif
#if defined(USART7)
    0,
#endif
#if defined(USART8)
    0,
#endif
#if defined(UART1)
    0,
#endif
#if defined(UART2)
    0,
#endif
#if defined(UART3)
    0,
#endif
#if defined(UART4)
#if !(defined(CH32V20x_D6) || defined(CH32V20x_D8) || defined(CH32V20x_D8W)) && \
    defined(DMA2_IT_TC3)
    DMA2_IT_TC3,
#else
    DMA1_IT_TC8,
#endif
#endif
#if defined(UART5)
    DMA2_IT_TC2,
#endif
#if defined(UART6)
    DMA2_IT_TC7,
#endif
#if defined(UART7)
    DMA2_IT_TC9,
#endif
#if defined(UART8)
    DMA2_IT_TC11,
#endif
};

static constexpr uint32_t CH32_UART_RX_DMA_IT_HT_MAP[] = {
#if defined(USART1)
    DMA1_IT_HT5,
#endif
#if defined(USART2)
    DMA1_IT_HT6,
#endif
#if defined(USART3)
    DMA1_IT_HT3,
#endif
#if defined(USART4)
    0,
#endif
#if defined(USART5)
    0,
#endif
#if defined(USART6)
    0,
#endif
#if defined(USART7)
    0,
#endif
#if defined(USART8)
    0,
#endif
#if defined(UART1)
    0,
#endif
#if defined(UART2)
    0,
#endif
#if defined(UART3)
    0,
#endif
#if defined(UART4)
#if !(defined(CH32V20x_D6) || defined(CH32V20x_D8) || defined(CH32V20x_D8W)) && \
    defined(DMA2_IT_HT3)
    DMA2_IT_HT3,
#else
    DMA1_IT_HT8,
#endif
#endif
#if defined(UART5)
    DMA2_IT_HT2,
#endif
#if defined(UART6)
    DMA2_IT_HT7,
#endif
#if defined(UART7)
    DMA2_IT_HT9,
#endif
#if defined(UART8)
    DMA2_IT_HT11,
#endif
};

static DMA_Channel_TypeDef* const CH32_UART_TX_DMA_CHANNEL_MAP[] = {
#if defined(USART1)
    DMA1_Channel4,
#endif
#if defined(USART2)
    DMA1_Channel7,
#endif
#if defined(USART3)
    DMA1_Channel2,
#endif
#if defined(USART4)
    nullptr,
#endif
#if defined(USART5)
    nullptr,
#endif
#if defined(USART6)
    nullptr,
#endif
#if defined(USART7)
    nullptr,
#endif
#if defined(USART8)
    nullptr,
#endif
#if defined(UART1)
    NULL,
#endif
#if defined(UART2)
    NULL,
#endif
#if defined(UART3)
    NULL,
#endif
#if defined(UART4)
#if !(defined(CH32V20x_D6) || defined(CH32V20x_D8) || defined(CH32V20x_D8W)) && \
    defined(DMA2_Channel5)
    DMA2_Channel5,
#else
    DMA1_Channel1,
#endif
#endif
#if defined(UART5)
    DMA2_Channel4,
#endif
#if defined(UART6)
    DMA2_Channel6,
#endif
#if defined(UART7)
    DMA2_Channel8,
#endif
#if defined(UART8)
    DMA2_Channel10,
#endif
};

static DMA_Channel_TypeDef* const CH32_UART_RX_DMA_CHANNEL_MAP[] = {
#if defined(USART1)
    DMA1_Channel5,
#endif
#if defined(USART2)
    DMA1_Channel6,
#endif
#if defined(USART3)
    DMA1_Channel3,
#endif
#if defined(USART4)
    nullptr,
#endif
#if defined(USART5)
    nullptr,
#endif
#if defined(USART6)
    nullptr,
#endif
#if defined(USART7)
    nullptr,
#endif
#if defined(USART8)
    nullptr,
#endif
#if defined(UART1)
    NULL,
#endif
#if defined(UART2)
    NULL,
#endif
#if defined(UART3)
    NULL,
#endif
#if defined(UART4)
#if !(defined(CH32V20x_D6) || defined(CH32V20x_D8) || defined(CH32V20x_D8W)) && \
    defined(DMA2_Channel3)
    DMA2_Channel3,
#else
    DMA1_Channel8,
#endif
#endif
#if defined(UART5)
    DMA2_Channel2,
#endif
#if defined(UART6)
    DMA2_Channel7,
#endif
#if defined(UART7)
    DMA2_Channel9,
#endif
#if defined(UART8)
    DMA2_Channel11,
#endif
};

static constexpr IRQn_Type CH32_UART_IRQ_MAP[] = {
#if defined(USART1)
    USART1_IRQn,
#endif
#if defined(USART2)
    USART2_IRQn,
#endif
#if defined(USART3)
    USART3_IRQn,
#endif
#if defined(USART4)
    USART4_IRQn,
#endif
#if defined(USART5)
    USART5_IRQn,
#endif
#if defined(USART6)
    USART6_IRQn,
#endif
#if defined(USART7)
    USART7_IRQn,
#endif
#if defined(USART8)
    USART8_IRQn,
#endif
#if defined(UART1)
    UART1_IRQn,
#endif
#if defined(UART2)
    UART2_IRQn,
#endif
#if defined(UART3)
    UART3_IRQn,
#endif
#if defined(UART4)
    UART4_IRQn,
#endif
#if defined(UART5)
    UART5_IRQn,
#endif
#if defined(UART6)
    UART6_IRQn,
#endif
#if defined(UART7)
    UART7_IRQn,
#endif
#if defined(UART8)
    UART8_IRQn,
#endif
};

ch32_uart_id_t ch32_uart_get_id(USART_TypeDef* addr);
USART_TypeDef* ch32_uart_get_instance_id(ch32_uart_id_t);
