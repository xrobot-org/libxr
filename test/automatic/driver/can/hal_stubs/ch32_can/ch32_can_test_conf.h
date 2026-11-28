#pragma once

#include <stdint.h>

#ifdef __cplusplus
#include <atomic>
struct Ch32CanMmioReg
{
  std::atomic<uint32_t> val{0U};
  mutable std::atomic<void (*)()> on_zero_read{nullptr};
  Ch32CanMmioReg() = default;
  Ch32CanMmioReg(const Ch32CanMmioReg& o) noexcept
      : val(o.val.load(std::memory_order_relaxed)),
        on_zero_read(o.on_zero_read.load(std::memory_order_relaxed))
  {
  }
  Ch32CanMmioReg& operator=(const Ch32CanMmioReg& o) noexcept
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

#define CH32V30x_D8C 1

typedef enum
{
  DISABLE = 0,
  ENABLE = 1
} FunctionalState;

typedef enum
{
  RESET = 0,
  SET = 1
} FlagStatus,
    ITStatus;

typedef enum
{
  USB_HP_CAN1_TX_IRQn = 19,
  USB_LP_CAN1_RX0_IRQn = 20,
  CAN1_RX1_IRQn = 21,
  CAN1_SCE_IRQn = 22,
  CAN2_TX_IRQn = 63,
  CAN2_RX0_IRQn = 64,
  CAN2_RX1_IRQn = 65,
  CAN2_SCE_IRQn = 66
} IRQn_Type;

typedef struct Ch32CanRegs
{
#ifdef __cplusplus
  Ch32CanMmioReg TSTATR;
#else
  volatile uint32_t TSTATR;
#endif
  volatile uint32_t ERRSR;
} CAN_TypeDef;

typedef struct
{
  uint16_t CAN_Prescaler;
  uint8_t CAN_Mode;
  uint8_t CAN_SJW;
  uint8_t CAN_BS1;
  uint8_t CAN_BS2;
  FunctionalState CAN_TTCM;
  FunctionalState CAN_ABOM;
  FunctionalState CAN_AWUM;
  FunctionalState CAN_NART;
  FunctionalState CAN_RFLM;
  FunctionalState CAN_TXFP;
} CAN_InitTypeDef;

typedef struct
{
  uint16_t CAN_FilterIdHigh;
  uint16_t CAN_FilterIdLow;
  uint16_t CAN_FilterMaskIdHigh;
  uint16_t CAN_FilterMaskIdLow;
  uint16_t CAN_FilterFIFOAssignment;
  uint8_t CAN_FilterNumber;
  uint8_t CAN_FilterMode;
  uint8_t CAN_FilterScale;
  FunctionalState CAN_FilterActivation;
} CAN_FilterInitTypeDef;

typedef struct
{
  uint32_t StdId;
  uint32_t ExtId;
  uint8_t IDE;
  uint8_t RTR;
  uint8_t DLC;
  uint8_t Data[8];
} CanTxMsg;

typedef struct
{
  uint32_t StdId;
  uint32_t ExtId;
  uint8_t IDE;
  uint8_t RTR;
  uint8_t DLC;
  uint8_t Data[8];
  uint8_t FMI;
} CanRxMsg;

typedef struct
{
  uint32_t PCLK1_Frequency;
} RCC_ClocksTypeDef;

#ifdef __cplusplus
extern "C"
{
#endif

  extern CAN_TypeDef g_ch32_can1_regs;
  extern CAN_TypeDef g_ch32_can2_regs;

#define CAN1 (&g_ch32_can1_regs)
#define CAN2 (&g_ch32_can2_regs)

#define RCC_APB1Periph_USB (1U << 23)
#define RCC_APB1Periph_CAN1 (1U << 25)
#define RCC_APB1Periph_CAN2 (1U << 26)

#define CAN_Mode_Normal 0x00U
#define CAN_Mode_LoopBack 0x01U
#define CAN_Mode_Silent 0x02U
#define CAN_Mode_Silent_LoopBack 0x03U

#define CAN_OperatingMode_Initialization 0x00U
#define CAN_OperatingMode_Normal 0x01U
#define CAN_ModeStatus_Failed 0x00U
#define CAN_ModeStatus_Success 0x01U
#define CAN_InitStatus_Failed 0x00U
#define CAN_InitStatus_Success 0x01U

#define CAN_FilterFIFO0 0U
#define CAN_FilterFIFO1 1U
#define CAN_FilterMode_IdMask 0U
#define CAN_FilterScale_32bit 1U

#define CAN_IT_TME (1U << 0)
#define CAN_IT_FMP0 (1U << 1)
#define CAN_IT_FMP1 (1U << 4)
#define CAN_IT_EWG (1U << 8)
#define CAN_IT_EPV (1U << 9)
#define CAN_IT_BOF (1U << 10)
#define CAN_IT_LEC (1U << 11)
#define CAN_IT_ERR (1U << 15)

#define CAN_FLAG_EWG (1U << 0)
#define CAN_FLAG_EPV (1U << 1)
#define CAN_FLAG_BOF (1U << 2)

#define CAN_ErrorCode_StuffErr 0x10U
#define CAN_ErrorCode_FormErr 0x20U
#define CAN_ErrorCode_ACKErr 0x30U
#define CAN_ErrorCode_BitRecessiveErr 0x40U
#define CAN_ErrorCode_BitDominantErr 0x50U
#define CAN_ErrorCode_CRCErr 0x60U

#define CAN_TSTATR_TME0 (1U << 26)
#define CAN_TSTATR_TME1 (1U << 27)
#define CAN_TSTATR_TME2 (1U << 28)
#define CAN_TxStatus_NoMailBox 4U

#define CAN_ID_STD 0U
#define CAN_ID_EXT 4U
#define CAN_RTR_DATA 0U
#define CAN_RTR_REMOTE 2U

  void NVIC_EnableIRQ(IRQn_Type IRQn);
  void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
  void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);
  uint8_t CAN_OperatingModeRequest(CAN_TypeDef* CANx, uint8_t CAN_OperatingMode);
  void CAN_SlaveStartBank(uint8_t CAN_BankNumber);
  void CAN_FilterInit(CAN_FilterInitTypeDef* CAN_FilterInitStruct);
  void CAN_ITConfig(CAN_TypeDef* CANx, uint32_t CAN_IT, FunctionalState NewState);
  void CAN_ClearITPendingBit(CAN_TypeDef* CANx, uint32_t CAN_IT);
  ITStatus CAN_GetITStatus(CAN_TypeDef* CANx, uint32_t CAN_IT);
  void CAN_StructInit(CAN_InitTypeDef* CAN_InitStruct);
  uint8_t CAN_Init(CAN_TypeDef* CANx, CAN_InitTypeDef* CAN_InitStruct);
  uint8_t CAN_Transmit(CAN_TypeDef* CANx, CanTxMsg* TxMessage);
  uint8_t CAN_MessagePending(CAN_TypeDef* CANx, uint8_t FIFONumber);
  void CAN_Receive(CAN_TypeDef* CANx, uint8_t FIFONumber, CanRxMsg* RxMessage);
  FlagStatus CAN_GetFlagStatus(CAN_TypeDef* CANx, uint32_t CAN_FLAG);
  uint8_t CAN_GetLastErrorCode(CAN_TypeDef* CANx);
  uint8_t CAN_GetReceiveErrorCounter(CAN_TypeDef* CANx);
  uint8_t CAN_GetLSBTransmitErrorCounter(CAN_TypeDef* CANx);

  void USB_HP_CAN1_TX_IRQHandler(void);
  void USB_LP_CAN1_RX0_IRQHandler(void);

#ifdef __cplusplus
}
#endif

// x86_64/arm64 主机编译器不支持 WCH-Interrupt-fast 带参 interrupt 属性；
// 仅在主机测试桩内将其映射为普通未使用属性，不修改生产源码。
// Host x86_64/arm64 compilers reject 1-arg WCH-Interrupt-fast; neutralize in stub only.
#if !defined(__riscv)
#define interrupt(x) unused
#endif
