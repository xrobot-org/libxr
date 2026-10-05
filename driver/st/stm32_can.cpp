#include "stm32_can.hpp"

#ifdef HAL_CAN_MODULE_ENABLED

using namespace LibXR;

STM32CAN* STM32CAN::map[STM32_CAN_NUMBER] = {nullptr};

/**
 * @brief 获取 CAN ID / Get CAN ID
 *
 * @param addr CAN外设地址 / CAN device address
 * @return stm32_can_id_t
 */
stm32_can_id_t STM32_CAN_GetID(CAN_TypeDef* addr)
{
  // NOLINTBEGIN
  if (addr == nullptr)
  {  // NOLINT
    return stm32_can_id_t::STM32_CAN_ID_ERROR;
  }
#ifdef CAN1
  else if (addr == CAN1)
  {  // NOLINT
    return stm32_can_id_t::STM32_CAN1;
  }
#endif
#ifdef CAN2
  else if (addr == CAN2)
  {  // NOLINT
    return stm32_can_id_t::STM32_CAN2;
  }
#endif
#ifdef CAN3
  else if (addr == CAN3)
  {  // NOLINT
    return stm32_can_id_t::STM32_CAN3;
  }
#endif
  else
  {
    return stm32_can_id_t::STM32_CAN_ID_ERROR;
  }
  // NOLINTEND
}

STM32CAN::STM32CAN(CAN_HandleTypeDef* hcan, uint32_t queue_size)
    : CAN(), hcan_(hcan), id_(STM32_CAN_GetID(hcan->Instance)), tx_queue_(queue_size)
{
  map[id_] = this;
  Init();
}

ErrorCode STM32CAN::Init(void)
{
  CAN_FilterTypeDef can_filter = {};

  can_filter.FilterIdHigh = 0;
  can_filter.FilterIdLow = 0;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterMaskIdHigh = 0;
  can_filter.FilterMaskIdLow = 0;
  can_filter.FilterActivation = ENABLE;

#ifdef CAN3
  if (id_ == STM32_CAN1)
  {
    can_filter.FilterBank = 0;
    can_filter.SlaveStartFilterBank = 14;
    fifo_ = CAN_RX_FIFO0;
  }
  else if (id_ == STM32_CAN2)
  {
    can_filter.FilterBank = 14;
    can_filter.SlaveStartFilterBank = 14;
    fifo_ = CAN_RX_FIFO0;
  }
  else if (id_ == STM32_CAN3)
  {
    can_filter.FilterBank = 0;
    fifo_ = CAN_RX_FIFO1;
  }
#else
#ifdef CAN2
  if (id_ == STM32_CAN1)
  {
    can_filter.FilterBank = 0;
    can_filter.SlaveStartFilterBank = 14;
    fifo_ = CAN_RX_FIFO0;
  }
  else if (id_ == STM32_CAN2)
  {
    can_filter.FilterBank = 14;
    can_filter.SlaveStartFilterBank = 14;
    fifo_ = CAN_RX_FIFO1;
  }
#else
  if (id_ == STM32_CAN1)
  {
    can_filter.FilterBank = 0;
    can_filter.SlaveStartFilterBank = 14;
    fifo_ = CAN_RX_FIFO0;
  }
#endif
#endif
  else
  {
    ASSERT(false);
  }

  can_filter.FilterFIFOAssignment = fifo_;

  if (HAL_CAN_ConfigFilter(hcan_, &can_filter) != HAL_OK)
  {
    return ErrorCode::FAILED;
  }

  if (HAL_CAN_Start(hcan_) != HAL_OK)
  {
    return ErrorCode::FAILED;
  }

  if (fifo_ == CAN_RX_FIFO0)
  {
    HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO0_MSG_PENDING);
  }
  else
  {
    HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO1_MSG_PENDING);
  }

  HAL_CAN_ActivateNotification(hcan_, CAN_IT_ERROR);
  HAL_CAN_ActivateNotification(hcan_, CAN_IT_BUSOFF);
  HAL_CAN_ActivateNotification(hcan_, CAN_IT_ERROR_PASSIVE);
  HAL_CAN_ActivateNotification(hcan_, CAN_IT_ERROR_WARNING);
  HAL_CAN_ActivateNotification(hcan_, CAN_IT_TX_MAILBOX_EMPTY);

  return ErrorCode::OK;
}

ErrorCode STM32CAN::SetConfig(const CAN::Configuration& cfg)
{
  ASSERT(hcan_ != nullptr && hcan_->Instance != nullptr);

  CAN_TypeDef* can = hcan_->Instance;

  // 先关掉与 Init 对应的中断
  uint32_t it_mask = 0u;

#ifdef CAN_IT_RX_FIFO0_MSG_PENDING
  if (fifo_ == CAN_RX_FIFO0)
  {
    it_mask |= CAN_IT_RX_FIFO0_MSG_PENDING;
  }
#endif

#ifdef CAN_IT_RX_FIFO1_MSG_PENDING
  if (fifo_ == CAN_RX_FIFO1)
  {
    it_mask |= CAN_IT_RX_FIFO1_MSG_PENDING;
  }
#endif

#ifdef CAN_IT_ERROR
  it_mask |= CAN_IT_ERROR;
#endif

#ifdef CAN_IT_TX_MAILBOX_EMPTY
  it_mask |= CAN_IT_TX_MAILBOX_EMPTY;
#endif

  if (it_mask != 0u)
  {
    HAL_CAN_DeactivateNotification(hcan_, it_mask);
  }

  // 停止 CAN，进入配置状态
  if (HAL_CAN_Stop(hcan_) != HAL_OK)
  {
    return ErrorCode::FAILED;
  }

  // 一次发送模式（不自动重发）→ NART
#ifdef CAN_MCR_NART
  if (cfg.mode.one_shot)
  {
    SET_BIT(can->MCR, CAN_MCR_NART);
  }
  else
  {
    CLEAR_BIT(can->MCR, CAN_MCR_NART);
  }
#endif

  const auto& bt = cfg.bit_timing;

  // 参数范围校验（0 表示保留原值）。
  // Derive field maxima from masks instead of hard-coded constants.
  constexpr uint32_t BRP_FIELD_MAX =
      (CAN_BTR_BRP_Msk >> CAN_BTR_BRP_Pos);  // 存的是 brp-1
  constexpr uint32_t TS1_FIELD_MAX =
      (CAN_BTR_TS1_Msk >> CAN_BTR_TS1_Pos);  // 存的是 ts1-1
  constexpr uint32_t TS2_FIELD_MAX =
      (CAN_BTR_TS2_Msk >> CAN_BTR_TS2_Pos);  // 存的是 ts2-1
  constexpr uint32_t SJW_FIELD_MAX =
      (CAN_BTR_SJW_Msk >> CAN_BTR_SJW_Pos);  // 存的是 sjw-1

  // 实际可配置的量（寄存器值 + 1）
  constexpr uint32_t BRP_MAX = BRP_FIELD_MAX + 1u;  // 1..1024
  constexpr uint32_t TS1_MAX = TS1_FIELD_MAX + 1u;  // 1..16
  constexpr uint32_t TS2_MAX = TS2_FIELD_MAX + 1u;  // 1..8
  constexpr uint32_t SJW_MAX = SJW_FIELD_MAX + 1u;  // 1..4

  // brp 范围检查
  if (bt.brp != 0u)
  {
    if (bt.brp < 1u || bt.brp > BRP_MAX)
    {
      return ErrorCode::ARG_ERR;
    }
  }

  // tseg1 = prop_seg + phase_seg1
  uint32_t tseg1 = bt.prop_seg + bt.phase_seg1;
  if (bt.prop_seg != 0u || bt.phase_seg1 != 0u)
  {
    if (tseg1 < 1u || tseg1 > TS1_MAX)
    {
      return ErrorCode::ARG_ERR;
    }
  }

  if (bt.phase_seg2 != 0u)
  {
    if (bt.phase_seg2 < 1u || bt.phase_seg2 > TS2_MAX)
    {
      return ErrorCode::ARG_ERR;
    }
  }

  if (bt.sjw != 0u)
  {
    if (bt.sjw < 1u || bt.sjw > SJW_MAX)
    {
      return ErrorCode::ARG_ERR;
    }

    // 规范上 SJW ≤ TSEG2（只在二者都要更新时检查）
    if (bt.phase_seg2 != 0u && bt.sjw > bt.phase_seg2)
    {
      return ErrorCode::ARG_ERR;
    }
  }

  uint32_t btr_old = can->BTR;
  uint32_t btr_new = btr_old;
  uint32_t btr_mask = 0u;

  // BRP：bt.brp == 0 → 保持原值
  if (bt.brp != 0u)
  {
    uint32_t brp = (bt.brp - 1u) & BRP_FIELD_MAX;
    uint32_t mask = CAN_BTR_BRP_Msk;
    btr_mask |= mask;
    btr_new &= ~mask;
    btr_new |= (brp << CAN_BTR_BRP_Pos);
  }

  // TSEG1 = PROP_SEG + PHASE_SEG1，两者都为 0 → 保持原值
  if (bt.prop_seg != 0u || bt.phase_seg1 != 0u)
  {
    uint32_t ts1 = (tseg1 - 1u) & TS1_FIELD_MAX;
    uint32_t mask = CAN_BTR_TS1_Msk;
    btr_mask |= mask;
    btr_new &= ~mask;
    btr_new |= (ts1 << CAN_BTR_TS1_Pos);
  }

  // TSEG2 = PHASE_SEG2，phase_seg2 == 0 → 保持原值
  if (bt.phase_seg2 != 0u)
  {
    uint32_t ts2 = (bt.phase_seg2 - 1u) & TS2_FIELD_MAX;
    uint32_t mask = CAN_BTR_TS2_Msk;
    btr_mask |= mask;
    btr_new &= ~mask;
    btr_new |= (ts2 << CAN_BTR_TS2_Pos);
  }

  // SJW，sjw == 0 → 保持原值
  if (bt.sjw != 0u)
  {
    uint32_t sjw = (bt.sjw - 1u) & SJW_FIELD_MAX;
    uint32_t mask = CAN_BTR_SJW_Msk;
    btr_mask |= mask;
    btr_new &= ~mask;
    btr_new |= (sjw << CAN_BTR_SJW_Pos);
  }

  // 三采样：只有 HAL 定义了 CAN_BTR_SAM 才动；否则忽略 triple_sampling
#ifdef CAN_BTR_SAM
  {
    uint32_t mask = CAN_BTR_SAM;
    btr_mask |= mask;
    btr_new &= ~mask;
    if (cfg.mode.triple_sampling)
    {
      btr_new |= mask;
    }
  }
#else
  (void)cfg.mode.triple_sampling;
#endif

  // Loopback：bool 两态，强制覆盖
#ifdef CAN_BTR_LBKM
  {
    uint32_t mask = CAN_BTR_LBKM;
    btr_mask |= mask;
    btr_new &= ~mask;
    if (cfg.mode.loopback)
    {
      btr_new |= mask;
    }
  }
#endif

  // Listen-only：bool 两态，强制覆盖
#ifdef CAN_BTR_SILM
  {
    uint32_t mask = CAN_BTR_SILM;
    btr_mask |= mask;
    btr_new &= ~mask;
    if (cfg.mode.listen_only)
    {
      btr_new |= mask;
    }
  }
#endif

  // 只改被 btr_mask 覆盖到的位，其余保持原值
  if (btr_mask != 0u)
  {
    btr_old &= ~btr_mask;
    btr_old |= (btr_new & btr_mask);
    can->BTR = btr_old;
  }

  // 重新启动 CAN
  if (HAL_CAN_Start(hcan_) != HAL_OK)
  {
    return ErrorCode::FAILED;
  }

  // 按 Init 的方式恢复中断
  uint32_t it_rx = 0u;

#ifdef CAN_IT_RX_FIFO0_MSG_PENDING
  if (fifo_ == CAN_RX_FIFO0)
  {
    it_rx = CAN_IT_RX_FIFO0_MSG_PENDING;
  }
#endif

#ifdef CAN_IT_RX_FIFO1_MSG_PENDING
  if (fifo_ == CAN_RX_FIFO1)
  {
    it_rx = CAN_IT_RX_FIFO1_MSG_PENDING;
  }
#endif

  if (it_rx != 0u)
  {
    HAL_CAN_ActivateNotification(hcan_, it_rx);
  }

  HAL_CAN_ActivateNotification(hcan_, CAN_IT_ERROR);
  HAL_CAN_ActivateNotification(hcan_, CAN_IT_BUSOFF);
  HAL_CAN_ActivateNotification(hcan_, CAN_IT_ERROR_PASSIVE);
  HAL_CAN_ActivateNotification(hcan_, CAN_IT_ERROR_WARNING);
  HAL_CAN_ActivateNotification(hcan_, CAN_IT_TX_MAILBOX_EMPTY);

  return ErrorCode::OK;
}

uint32_t STM32CAN::GetClockFreq() const
{
  // 经典 bxCAN 始终挂在 APB1 上
  return HAL_RCC_GetPCLK1Freq();
}

inline void STM32CAN::BuildTxHeader(const ClassicPack& p, CAN_TxHeaderTypeDef& h)
{
  const bool IS_EXT = (p.type == Type::EXTENDED) || (p.type == Type::REMOTE_EXTENDED);
  const bool IS_RTR =
      (p.type == Type::REMOTE_STANDARD) || (p.type == Type::REMOTE_EXTENDED);

  h.DLC = (p.dlc <= 8u) ? p.dlc : 8u;
  h.IDE = IS_EXT ? CAN_ID_EXT : CAN_ID_STD;
  h.RTR = IS_RTR ? CAN_RTR_REMOTE : CAN_RTR_DATA;

  h.StdId = IS_EXT ? 0u : (p.id & 0x7FFu);
  h.ExtId = IS_EXT ? (p.id & 0x1FFFFFFFu) : 0u;

  h.TransmitGlobalTime = DISABLE;
}

void STM32CAN::TxService()
{
  if (hcan_ == nullptr || hcan_->Instance == nullptr)
  {
    return;
  }

  // 标记：需要一次 TX 服务（无论本次是否抢到锁）
  tx_pend_.store(1u, std::memory_order_release);

  // 尝试抢占服务锁：0 -> 1
  uint32_t expected = 0u;
  if (!tx_lock_.compare_exchange_strong(expected, 1u, std::memory_order_acquire,
                                        std::memory_order_relaxed))
  {
    // 有别的上下文在服务；PEND 已置位，它会在结束时看到并再服务
    return;
  }

  constexpr uint32_t TME_MASK = (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2);

  for (;;)
  {
    // 本轮服务开始：消费掉 pend（期间如有新 kick，会再次置 1）
    tx_pend_.store(0u, std::memory_order_release);

    // 尽可能把空 mailbox 填满：一直到队列空或者 AddTxMessage 失败（邮箱满/忙）
    while ((hcan_->Instance->TSR & TME_MASK) != 0u)
    {
      ClassicPack p{};
      if (tx_retry_valid_)
      {
        p = tx_retry_pack_;
      }
      else if (tx_queue_.Pop(p) != ErrorCode::OK)
      {
        break;  // 队列空
      }

      CAN_TxHeaderTypeDef hdr{};
      BuildTxHeader(p, hdr);

      uint32_t mailbox = 0u;
      if (HAL_CAN_AddTxMessage(hcan_, &hdr, p.data, &mailbox) != HAL_OK)
      {
        tx_retry_pack_ = p;
        tx_retry_valid_ = true;
        break;
      }

      tx_retry_valid_ = false;
      txMailbox = mailbox;
    }

    // 先解锁，再检查是否仍有待处理发送请求。
    // Release lock first, then check whether pending TX work remains.
    tx_lock_.store(0u, std::memory_order_release);

    // 再看是否还有 pend：若没有，直接退出
    if (tx_pend_.load(std::memory_order_acquire) == 0u)
    {
      return;
    }

    // 允许出现 PEND=1, LOCK=0 的“无人服务”状态：尝试重新加锁再服务
    expected = 0u;
    if (!tx_lock_.compare_exchange_strong(expected, 1u, std::memory_order_acquire,
                                          std::memory_order_relaxed))
    {
      return;
    }
  }
}

ErrorCode STM32CAN::AddMessage(const ClassicPack& pack)
{
  if (pack.type == Type::ERROR)
  {
    return ErrorCode::ARG_ERR;
  }

  if (tx_queue_.Push(pack) != ErrorCode::OK)
  {
    return ErrorCode::FULL;
  }

  // kick
  TxService();
  return ErrorCode::OK;
}

void STM32CAN::ProcessRxInterrupt()
{
  while (HAL_CAN_GetRxMessage(hcan_, fifo_, &rx_buff_.header, rx_buff_.pack.data) ==
         HAL_OK)
  {
    if (rx_buff_.header.IDE == CAN_ID_STD)
    {
      rx_buff_.pack.id = rx_buff_.header.StdId;
      rx_buff_.pack.type = Type::STANDARD;
    }
    else
    {
      rx_buff_.pack.id = rx_buff_.header.ExtId;
      rx_buff_.pack.type = Type::EXTENDED;
    }

    if (rx_buff_.header.RTR == CAN_RTR_REMOTE)
    {
      if (rx_buff_.pack.type == Type::STANDARD)
      {
        rx_buff_.pack.type = Type::REMOTE_STANDARD;
      }
      else
      {
        rx_buff_.pack.type = Type::REMOTE_EXTENDED;
      }
    }

    uint8_t dlc = (rx_buff_.header.DLC <= 8u) ? rx_buff_.header.DLC : 8u;
    rx_buff_.pack.dlc = dlc;

    OnMessage(rx_buff_.pack, true);
  }
}

void STM32CAN::ProcessErrorInterrupt()
{
  if (hcan_ == nullptr || hcan_->Instance == nullptr)
  {
    return;
  }

  uint32_t esr = hcan_->Instance->ESR;

  CAN::ClassicPack pack{};
  pack.type = CAN::Type::ERROR;
  pack.dlc = 0;

  CAN::ErrorID eid = CAN::ErrorID::CAN_ERROR_ID_GENERIC;

  if (esr & CAN_ESR_BOFF)
  {
    eid = CAN::ErrorID::CAN_ERROR_ID_BUS_OFF;
  }
  else if (esr & CAN_ESR_EPVF)
  {
    eid = CAN::ErrorID::CAN_ERROR_ID_ERROR_PASSIVE;
  }
  else if (esr & CAN_ESR_EWGF)
  {
    eid = CAN::ErrorID::CAN_ERROR_ID_ERROR_WARNING;
  }
  else
  {
    uint32_t lec = (esr >> 4) & 0x7u;

    switch (lec)
    {
      case 0x01:
        eid = CAN::ErrorID::CAN_ERROR_ID_STUFF;
        break;
      case 0x02:
        eid = CAN::ErrorID::CAN_ERROR_ID_FORM;
        break;
      case 0x03:
        eid = CAN::ErrorID::CAN_ERROR_ID_ACK;
        break;
      case 0x04:
        eid = CAN::ErrorID::CAN_ERROR_ID_BIT1;
        break;
      case 0x05:
        eid = CAN::ErrorID::CAN_ERROR_ID_BIT0;
        break;
      case 0x06:
        eid = CAN::ErrorID::CAN_ERROR_ID_CRC;
        break;
      default:
        eid = CAN::ErrorID::CAN_ERROR_ID_OTHER;
        break;
    }
  }

  pack.id = static_cast<uint32_t>(eid);

  // 在中断上下文中分发错误帧
  OnMessage(pack, true);
}

ErrorCode STM32CAN::GetErrorState(CAN::ErrorState& state) const
{
  if (hcan_ == nullptr || hcan_->Instance == nullptr)
  {
    return ErrorCode::ARG_ERR;
  }

  // 直接读取 bxCAN 的错误状态寄存器 ESR
  uint32_t esr = hcan_->Instance->ESR;

  // TEC: bits 23:16, REC: bits 31:24（参考 RM & CAN_ESR 描述）
  state.tx_error_counter = static_cast<uint8_t>((esr >> 16) & 0xFFu);
  state.rx_error_counter = static_cast<uint8_t>((esr >> 24) & 0xFFu);

  // 状态位：BOFF / EPVF / EWGF
  state.bus_off = (esr & CAN_ESR_BOFF) != 0u;
  state.error_passive = (esr & CAN_ESR_EPVF) != 0u;
  state.error_warning = (esr & CAN_ESR_EWGF) != 0u;

  return ErrorCode::OK;
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
  STM32CAN* can = STM32CAN::map[STM32_CAN_GetID(hcan->Instance)];
  if (can)
  {
    can->ProcessRxInterrupt();
  }
}

extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
  STM32CAN* can = STM32CAN::map[STM32_CAN_GetID(hcan->Instance)];
  if (can)
  {
    can->ProcessRxInterrupt();
  }
}

extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan)
{
  STM32CAN* can = STM32CAN::map[STM32_CAN_GetID(hcan->Instance)];
  if (can)
  {
    can->ProcessErrorInterrupt();
  }

  HAL_CAN_ResetError(hcan);
}

extern "C" void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* hcan)
{
  STM32CAN* can = STM32CAN::map[STM32_CAN_GetID(hcan->Instance)];
  if (can)
  {
    can->TxService();
  }
}

extern "C" void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef* hcan)
{
  STM32CAN* can = STM32CAN::map[STM32_CAN_GetID(hcan->Instance)];
  if (can)
  {
    can->TxService();
  }
}

extern "C" void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef* hcan)
{
  STM32CAN* can = STM32CAN::map[STM32_CAN_GetID(hcan->Instance)];
  if (can)
  {
    can->TxService();
  }
}

#endif
