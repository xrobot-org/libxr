#pragma once

#include <stdint.h>

#define HAL_FDCAN_MODULE_ENABLED 1

#ifndef UNUSED
#define UNUSED(X) (void)(X)
#endif

#ifndef SET_BIT
#define SET_BIT(REG, BIT) ((REG) |= (BIT))
#endif

#ifndef CLEAR_BIT
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))
#endif

typedef enum
{
  HAL_OK = 0x00U,
  HAL_ERROR = 0x01U,
  HAL_BUSY = 0x02U,
  HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

typedef struct Stm32FdcanRegs
{
  volatile uint32_t CCCR;
  volatile uint32_t TEST;
  volatile uint32_t NBTP;
  volatile uint32_t DBTP;
} FDCAN_GlobalTypeDef;

typedef struct
{
  FDCAN_GlobalTypeDef* Instance;
  struct
  {
    uint32_t dummy;
  } Init;
  uint32_t ErrorCode;
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
} FDCAN_RxHeaderTypeDef;

typedef struct
{
  uint32_t LastErrorCode;
  uint32_t DataLastErrorCode;
  uint32_t ErrorPassive;
  uint32_t Warning;
  uint32_t BusOff;
} FDCAN_ProtocolStatusTypeDef;

typedef struct
{
  uint32_t TxErrorCnt;
  uint32_t RxErrorCnt;
} FDCAN_ErrorCountersTypeDef;

#ifdef __cplusplus
extern "C"
{
#endif

  extern FDCAN_GlobalTypeDef g_stm32_fdcan1_regs;
  extern FDCAN_GlobalTypeDef g_stm32_fdcan2_regs;
  extern FDCAN_GlobalTypeDef g_stm32_fdcan3_regs;

#define FDCAN1 (&g_stm32_fdcan1_regs)
#define FDCAN2 (&g_stm32_fdcan2_regs)
#define FDCAN3 (&g_stm32_fdcan3_regs)

#define FDCAN_STANDARD_ID 0x00000000U
#define FDCAN_EXTENDED_ID 0x40000000U
#define FDCAN_DATA_FRAME 0x00000000U
#define FDCAN_REMOTE_FRAME 0x20000000U

#define FDCAN_DLC_BYTES_0 0U
#define FDCAN_DLC_BYTES_1 1U
#define FDCAN_DLC_BYTES_2 2U
#define FDCAN_DLC_BYTES_3 3U
#define FDCAN_DLC_BYTES_4 4U
#define FDCAN_DLC_BYTES_5 5U
#define FDCAN_DLC_BYTES_6 6U
#define FDCAN_DLC_BYTES_7 7U
#define FDCAN_DLC_BYTES_8 8U
#define FDCAN_DLC_BYTES_12 9U
#define FDCAN_DLC_BYTES_16 10U
#define FDCAN_DLC_BYTES_20 11U
#define FDCAN_DLC_BYTES_24 12U
#define FDCAN_DLC_BYTES_32 13U
#define FDCAN_DLC_BYTES_48 14U
#define FDCAN_DLC_BYTES_64 15U

#define FDCAN_ESI_PASSIVE 1U
#define FDCAN_BRS_OFF 0U
#define FDCAN_BRS_ON 1U
#define FDCAN_CLASSIC_CAN 0U
#define FDCAN_FD_CAN 1U
#define FDCAN_NO_TX_EVENTS 0U

#define FDCAN_FILTER_MASK 0U
#define FDCAN_FILTER_TO_RXFIFO0 1U
#define FDCAN_FILTER_TO_RXFIFO1 2U
#define FDCAN_RX_FIFO0 0U
#define FDCAN_RX_FIFO1 1U

#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE (1U << 0)
#define FDCAN_IT_RX_FIFO1_NEW_MESSAGE (1U << 4)
#define FDCAN_IT_TX_COMPLETE (1U << 9)
#define FDCAN_IT_TX_FIFO_EMPTY (1U << 11)
#define FDCAN_IT_ERROR_PASSIVE (1U << 23)
#define FDCAN_IT_ERROR_WARNING (1U << 24)
#define FDCAN_IT_BUS_OFF (1U << 25)

#define HAL_FDCAN_ERROR_NONE 0U
#define RCC_PERIPHCLK_FDCAN 1U

#define FDCAN_CCCR_INIT (1U << 0)
#define FDCAN_CCCR_CCE (1U << 1)
#define FDCAN_CCCR_MON (1U << 5)
#define FDCAN_CCCR_DAR (1U << 6)
#define FDCAN_CCCR_TEST (1U << 7)
#define FDCAN_TEST_LBCK (1U << 4)

#define FDCAN_NBTP_NTSEG2_Pos 0U
#define FDCAN_NBTP_NTSEG2_Msk (0x7FU << FDCAN_NBTP_NTSEG2_Pos)
#define FDCAN_NBTP_NTSEG1_Pos 8U
#define FDCAN_NBTP_NTSEG1_Msk (0xFFU << FDCAN_NBTP_NTSEG1_Pos)
#define FDCAN_NBTP_NBRP_Pos 16U
#define FDCAN_NBTP_NBRP_Msk (0x1FFU << FDCAN_NBTP_NBRP_Pos)
#define FDCAN_NBTP_NSJW_Pos 25U
#define FDCAN_NBTP_NSJW_Msk (0x7FU << FDCAN_NBTP_NSJW_Pos)

#define FDCAN_DBTP_DSJW_Pos 0U
#define FDCAN_DBTP_DSJW_Msk (0xFU << FDCAN_DBTP_DSJW_Pos)
#define FDCAN_DBTP_DTSEG2_Pos 4U
#define FDCAN_DBTP_DTSEG2_Msk (0xFU << FDCAN_DBTP_DTSEG2_Pos)
#define FDCAN_DBTP_DTSEG1_Pos 8U
#define FDCAN_DBTP_DTSEG1_Msk (0x1FU << FDCAN_DBTP_DTSEG1_Pos)
#define FDCAN_DBTP_DBRP_Pos 16U
#define FDCAN_DBTP_DBRP_Msk (0x1FU << FDCAN_DBTP_DBRP_Pos)

  HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef* hfdcan,
                                           const FDCAN_FilterTypeDef* sFilterConfig);
  HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef* hfdcan);
  HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef* hfdcan);
  HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef* hfdcan,
                                                   uint32_t ActiveITs,
                                                   uint32_t BufferIndexes);
  HAL_StatusTypeDef HAL_FDCAN_DeactivateNotification(FDCAN_HandleTypeDef* hfdcan,
                                                     uint32_t InactiveITs);
  uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef* hfdcan);
  HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef* hfdcan,
                                                  const FDCAN_TxHeaderTypeDef* pTxHeader,
                                                  const uint8_t* pTxData);
  HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef* hfdcan,
                                           uint32_t RxLocation,
                                           FDCAN_RxHeaderTypeDef* pRxHeader,
                                           uint8_t* pRxData);
  uint32_t HAL_FDCAN_GetRxFifoFillLevel(const FDCAN_HandleTypeDef* hfdcan,
                                        uint32_t RxFifo);
  HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(
      const FDCAN_HandleTypeDef* hfdcan, FDCAN_ProtocolStatusTypeDef* ProtocolStatus);
  HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(const FDCAN_HandleTypeDef* hfdcan,
                                               FDCAN_ErrorCountersTypeDef* ErrorCounters);
  uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk);

  void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef* hcan);
  void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan,
                                     uint32_t ErrorStatusITs);
  void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef* hcan,
                                          uint32_t BufferIndexes);
  void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef* hcan);
  void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hcan, uint32_t RxFifo0ITs);
  void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hcan, uint32_t RxFifo1ITs);

#ifdef __cplusplus
}
#endif
