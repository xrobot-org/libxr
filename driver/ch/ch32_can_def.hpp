#pragma once

#include <cstdint>

#include "ch32_can_caps.hpp"
#include "libxr.hpp"

/**
 * @brief CH32 CAN 实例编号 / CH32 CAN instance identifier
 *
 * @note 不同型号可能只有 CAN1 或同时支持 CAN1/CAN2。
 *       Some MCUs expose CAN1 only, while others expose both CAN1 and CAN2.
 */
typedef enum
{
#if defined(CAN1)
  CH32_CAN1,
#endif
#if LIBXR_CH32_HAS_CAN2
  CH32_CAN2,
#endif
  CH32_CAN_NUMBER,
  CH32_CAN_ID_ERROR
} ch32_can_id_t;

static_assert(CH32_CAN_NUMBER >= 1, "No CAN instance detected for this MCU");

/**
 * @brief 通过外设地址获取 CAN 编号 / Get CAN ID by peripheral address
 */
ch32_can_id_t CH32_CAN_GetID(CAN_TypeDef* addr);  // NOLINT

/**
 * @brief 通过编号获取外设地址 / Get CAN instance by ID
 */
CAN_TypeDef* CH32_CAN_GetInstanceID(ch32_can_id_t id);  // NOLINT

/**
 * @brief 双 CAN 过滤器分界默认值 / Default filter split for dual-CAN
 *
 * 对于 28 组过滤器，常见分界点为 14。
 * For 28 filter banks, split-at-14 is a common layout.
 */
static constexpr uint8_t CH32_CAN_DEFAULT_SLAVE_START_BANK = 14;

static constexpr uint32_t CH32_CAN_RCC_PERIPH_MAP[] = {
#if defined(CAN1)
    RCC_APB1Periph_CAN1,
#endif
#if LIBXR_CH32_HAS_CAN2
    RCC_APB1Periph_CAN2,
#endif
};

static constexpr IRQn_Type CH32_CAN_TX_IRQ_MAP[] = {
#if defined(CAN1)
    USB_HP_CAN1_TX_IRQn,
#endif
#if LIBXR_CH32_HAS_CAN2
    CAN2_TX_IRQn,
#endif
};

static constexpr IRQn_Type CH32_CAN_RX0_IRQ_MAP[] = {
#if defined(CAN1)
    USB_LP_CAN1_RX0_IRQn,
#endif
#if LIBXR_CH32_HAS_CAN2
    CAN2_RX0_IRQn,
#endif
};

static constexpr IRQn_Type CH32_CAN_RX1_IRQ_MAP[] = {
#if defined(CAN1)
    CAN1_RX1_IRQn,
#endif
#if LIBXR_CH32_HAS_CAN2
    CAN2_RX1_IRQn,
#endif
};

static constexpr IRQn_Type CH32_CAN_SCE_IRQ_MAP[] = {
#if defined(CAN1)
    CAN1_SCE_IRQn,
#endif
#if LIBXR_CH32_HAS_CAN2
    CAN2_SCE_IRQn,
#endif
};
