#pragma once

#include <stdint.h>

#ifdef __cplusplus
#include <atomic>
struct Stm32CanMmioReg
{
  std::atomic<uint32_t> val{0U};
  mutable std::atomic<void (*)()> on_zero_read{nullptr};
  Stm32CanMmioReg() = default;
  Stm32CanMmioReg(const Stm32CanMmioReg& o) noexcept
      : val(o.val.load(std::memory_order_relaxed)),
        on_zero_read(o.on_zero_read.load(std::memory_order_relaxed))
  {
  }
  Stm32CanMmioReg& operator=(const Stm32CanMmioReg& o) noexcept
  {
    val.store(o.val.load(std::memory_order_relaxed), std::memory_order_relaxed);
    on_zero_read.store(o.on_zero_read.load(std::memory_order_relaxed),
                       std::memory_order_relaxed);
    return *this;
  }
  uint32_t operator=(uint32_t v) noexcept
  {
    val.store(v, std::memory_order_release);
    return v;
  }
  operator uint32_t() const noexcept
  {
    const uint32_t v = val.load(std::memory_order_acquire);
    if (v == 0U)
    {
      if (auto* cb = on_zero_read.exchange(nullptr, std::memory_order_acq_rel))
      {
        cb();
      }
    }
    return v;
  }
};
#endif

#define HAL_CAN_MODULE_ENABLED 1

#ifndef SET_BIT
#define SET_BIT(REG, BIT) ((REG) |= (BIT))
#endif

#ifndef CLEAR_BIT
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))
#endif

#define DISABLE 0U
#define ENABLE 1U

typedef enum
{
  HAL_OK = 0x00U,
  HAL_ERROR = 0x01U,
  HAL_BUSY = 0x02U,
  HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

typedef struct Stm32CanRegs
{
  volatile uint32_t MCR;
  volatile uint32_t BTR;
#ifdef __cplusplus
  Stm32CanMmioReg TSR;
#else
  volatile uint32_t TSR;
#endif
  volatile uint32_t ESR;
} CAN_TypeDef;

typedef struct
{
  CAN_TypeDef* Instance;
} CAN_HandleTypeDef;

typedef struct
{
  uint32_t FilterIdHigh;
  uint32_t FilterIdLow;
  uint32_t FilterMaskIdHigh;
  uint32_t FilterMaskIdLow;
  uint32_t FilterFIFOAssignment;
  uint32_t FilterBank;
  uint32_t FilterMode;
  uint32_t FilterScale;
  uint32_t FilterActivation;
  uint32_t SlaveStartFilterBank;
} CAN_FilterTypeDef;

typedef struct
{
  uint32_t StdId;
  uint32_t ExtId;
  uint32_t IDE;
  uint32_t RTR;
  uint32_t DLC;
  uint32_t TransmitGlobalTime;
} CAN_TxHeaderTypeDef;

typedef struct
{
  uint32_t StdId;
  uint32_t ExtId;
  uint32_t IDE;
  uint32_t RTR;
  uint32_t DLC;
} CAN_RxHeaderTypeDef;

#ifdef __cplusplus
extern "C"
{
#endif

  extern CAN_TypeDef g_stm32_can1_regs;
  extern CAN_TypeDef g_stm32_can2_regs;

#define CAN1 (&g_stm32_can1_regs)
#define CAN2 (&g_stm32_can2_regs)

#define CAN_FILTERMODE_IDMASK 0U
#define CAN_FILTERSCALE_32BIT 1U
#define CAN_RX_FIFO0 0U
#define CAN_RX_FIFO1 1U

#define CAN_IT_TX_MAILBOX_EMPTY (1U << 0)
#define CAN_IT_RX_FIFO0_MSG_PENDING (1U << 1)
#define CAN_IT_RX_FIFO1_MSG_PENDING (1U << 4)
#define CAN_IT_ERROR_WARNING (1U << 8)
#define CAN_IT_ERROR_PASSIVE (1U << 9)
#define CAN_IT_BUSOFF (1U << 10)
#define CAN_IT_ERROR (1U << 15)

#define CAN_MCR_NART (1U << 4)

#define CAN_BTR_BRP_Pos 0U
#define CAN_BTR_BRP_Msk (0x3FFU << CAN_BTR_BRP_Pos)
#define CAN_BTR_TS1_Pos 16U
#define CAN_BTR_TS1_Msk (0xFU << CAN_BTR_TS1_Pos)
#define CAN_BTR_TS2_Pos 20U
#define CAN_BTR_TS2_Msk (0x7U << CAN_BTR_TS2_Pos)
#define CAN_BTR_SJW_Pos 24U
#define CAN_BTR_SJW_Msk (0x3U << CAN_BTR_SJW_Pos)
#define CAN_BTR_LBKM (1U << 30)
#define CAN_BTR_SILM (1U << 31)

#define CAN_ID_STD 0U
#define CAN_ID_EXT 4U
#define CAN_RTR_DATA 0U
#define CAN_RTR_REMOTE 2U

#define CAN_TSR_TME0 (1U << 26)
#define CAN_TSR_TME1 (1U << 27)
#define CAN_TSR_TME2 (1U << 28)

#define CAN_ESR_EWGF (1U << 0)
#define CAN_ESR_EPVF (1U << 1)
#define CAN_ESR_BOFF (1U << 2)

  HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef* hcan,
                                         const CAN_FilterTypeDef* sFilterConfig);
  HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef* hcan);
  HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef* hcan);
  HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef* hcan,
                                                 uint32_t ActiveITs);
  HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef* hcan,
                                                   uint32_t InactiveITs);
  HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef* hcan,
                                         const CAN_TxHeaderTypeDef* pHeader,
                                         const uint8_t aData[], uint32_t* pTxMailbox);
  HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef* hcan, uint32_t RxFifo,
                                         CAN_RxHeaderTypeDef* pHeader, uint8_t aData[]);
  HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef* hcan);
  uint32_t HAL_RCC_GetPCLK1Freq(void);

  void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan);
  void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan);
  void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan);
  void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* hcan);
  void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef* hcan);
  void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef* hcan);

#ifdef __cplusplus
}
#endif
