#include "ch32_can.hpp"

#include "ch32_usbcan_shared.hpp"

using namespace LibXR;

CH32CAN* CH32CAN::map[CH32_CAN_NUMBER] = {nullptr};

#if defined(CAN1) && defined(RCC_APB1Periph_USB)
static void can1_rx0_thunk()
{
  if (auto* can = LibXR::CH32CAN::map[CH32_CAN1])
  {
    can->ProcessRxInterrupt();
  }
}

static void can1_tx_thunk()
{
  if (auto* can = LibXR::CH32CAN::map[CH32_CAN1])
  {
    can->ProcessTxInterrupt();
  }
}
#endif

static inline uint8_t ch32_can_mode_macro(const CAN::Mode& m)
{
  if (m.loopback && m.listen_only)
  {
    return CAN_Mode_Silent_LoopBack;
  }
  if (m.loopback)
  {
    return CAN_Mode_LoopBack;
  }
  if (m.listen_only)
  {
    return CAN_Mode_Silent;
  }
  return CAN_Mode_Normal;
}

static inline void ch32_can_enable_nvic(ch32_can_id_t id, uint8_t fifo)
{
  // TX 中断线。
  // TX interrupt line.
  switch (id)
  {
#if defined(CAN1)
    case CH32_CAN1:
      NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
      break;
#endif

#if LIBXR_CH32_HAS_CAN2
    case CH32_CAN2:
      NVIC_EnableIRQ(CAN2_TX_IRQn);
      break;
#endif
    default:
      break;
  }

  // RX 中断线：FIFO0 -> RX0 vector，FIFO1 -> RX1 vector。
  // RX interrupt line: FIFO0 -> RX0 vector, FIFO1 -> RX1 vector.
  if (fifo == 0u)
  {
    switch (id)
    {
#if defined(CAN1)
      case CH32_CAN1:
        NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
        break;
#endif

#if LIBXR_CH32_HAS_CAN2
      case CH32_CAN2:
        NVIC_EnableIRQ(CAN2_RX0_IRQn);
        break;
#endif
      default:
        break;
    }
  }
  else
  {
    switch (id)
    {
#if defined(CAN1)
      case CH32_CAN1:
        NVIC_EnableIRQ(CAN1_RX1_IRQn);
        break;
#endif

#if LIBXR_CH32_HAS_CAN2
      case CH32_CAN2:
        NVIC_EnableIRQ(CAN2_RX1_IRQn);
        break;
#endif

      default:
        break;
    }
  }

  // SCE 中断线。
  // SCE interrupt line.
  switch (id)
  {
#if defined(CAN1)
    case CH32_CAN1:
      NVIC_EnableIRQ(CAN1_SCE_IRQn);
      break;
#endif

#if LIBXR_CH32_HAS_CAN2
    case CH32_CAN2:
      NVIC_EnableIRQ(CAN2_SCE_IRQn);
      break;
#endif

    default:
      break;
  }
}

CH32CAN::CH32CAN(ch32_can_id_t id, uint32_t queue_size)
    : CAN(), instance_(CH32_CAN_GetInstanceID(id)), id_(id), tx_queue_(queue_size)
{
  if constexpr (LibXR::CH32UsbCanShared::usb_can_share_enabled())
  {
    // 端点地址在 Stop 后仍有效，不能在分配 PMA 后扩大 CAN 过滤器占用。
    // Endpoint addresses survive Stop; reserve CAN filters before allocating PMA.
    REQUIRE(!LibXR::CH32UsbCanShared::usb_pma_configured.load(std::memory_order_acquire));
  }

#if LIBXR_CH32_HAS_CAN2
  if (id == CH32_CAN1)
  {
    filter_bank_ = 0;
    fifo_ = CAN_FilterFIFO0;  // 0
  }
  else
  {
    filter_bank_ = 14;
    fifo_ = CAN_FilterFIFO1;  // 1
  }
#else
  filter_bank_ = 0;
  fifo_ = CAN_FilterFIFO0;  // 0
#endif

  ASSERT(id_ < CH32_CAN_NUMBER);
  ASSERT(instance_ != nullptr);

  map[id_] = this;

  // 打开外设时钟。
  // Enable the peripheral clock.
  RCC_APB1PeriphClockCmd(CH32_CAN_RCC_PERIPH_MAP[id_], ENABLE);
#if LIBXR_CH32_HAS_CAN2
  if (id_ == CH32_CAN2)
  {
    // CAN2 的共享过滤器寄存器位于 CAN1，不能依赖应用先创建 CAN1 对象。
    // CAN2 filter registers belong to CAN1, even in a CAN2-only application.
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
  }
#endif

  // 在调用 SetConfig() 之前，保持 CAN 处于初始化模式。
  // Keep CAN in initialization mode until SetConfig() is called.
  (void)CAN_OperatingModeRequest(instance_, CAN_OperatingMode_Initialization);

#if LIBXR_CH32_HAS_CAN2
  // 在双 CAN 变体上，配置默认的共享过滤器分界点。
  // On dual-CAN variants, configure the default shared filter split point.
  CAN_SlaveStartBank(CH32_CAN_DEFAULT_SLAVE_START_BANK);
#endif

  (void)Init();
}

ErrorCode CH32CAN::Init()
{
  if (instance_ == nullptr)
  {
    return ErrorCode::ARG_ERR;
  }

  // 默认全接收过滤器（ID-mask 模式，全 0）。
  // Default accept-all filter (ID-mask mode, all zeros).
  CAN_FilterInitTypeDef f = {};
  f.CAN_FilterIdHigh = 0u;
  f.CAN_FilterIdLow = 0u;
  f.CAN_FilterMaskIdHigh = 0u;
  f.CAN_FilterMaskIdLow = 0u;
  f.CAN_FilterFIFOAssignment = fifo_;
  f.CAN_FilterNumber = filter_bank_;
  f.CAN_FilterMode = CAN_FilterMode_IdMask;
  f.CAN_FilterScale = CAN_FilterScale_32bit;
  f.CAN_FilterActivation = ENABLE;

  CAN_FilterInit(&f);

  // 在放行中断源之前发布回调，防止已有 pending 中断进入空分发器。
  // Publish callbacks before enabling an interrupt source with pending events.
  if constexpr (LibXR::CH32UsbCanShared::usb_can_irq_share_enabled())
  {
#if defined(CAN1) && defined(RCC_APB1Periph_USB)
    if (id_ == CH32_CAN1)
    {
      LibXR::CH32UsbCanShared::register_can1_rx0(&can1_rx0_thunk);
      LibXR::CH32UsbCanShared::register_can1_tx(&can1_tx_thunk);
      LibXR::CH32UsbCanShared::can1_inited.store(true, std::memory_order_release);
    }
#if LIBXR_CH32_HAS_CAN2
    else if (id_ == CH32_CAN2)
    {
      LibXR::CH32UsbCanShared::can2_inited.store(true, std::memory_order_release);
    }
#endif
#endif
  }

  EnableIRQs();
  ch32_can_enable_nvic(id_, fifo_);

  return ErrorCode::OK;
}

void CH32CAN::DisableIRQs()
{
  uint32_t it = 0u;

#ifdef CAN_IT_FMP0
  if (fifo_ == 0u)
  {
    it |= CAN_IT_FMP0;
  }
#endif
#ifdef CAN_IT_FMP1
  if (fifo_ == 1u)
  {
    it |= CAN_IT_FMP1;
  }
#endif

#ifdef CAN_IT_TME
  it |= CAN_IT_TME;
#endif

#ifdef CAN_IT_ERR
  it |= CAN_IT_ERR;
#endif
#ifdef CAN_IT_BOF
  it |= CAN_IT_BOF;
#endif
#ifdef CAN_IT_EPV
  it |= CAN_IT_EPV;
#endif
#ifdef CAN_IT_EWG
  it |= CAN_IT_EWG;
#endif
#ifdef CAN_IT_LEC
  it |= CAN_IT_LEC;
#endif

  if (it != 0u)
  {
    CAN_ITConfig(instance_, it, DISABLE);
  }
}

void CH32CAN::EnableIRQs()
{
  uint32_t it = 0u;

#ifdef CAN_IT_FMP0
  if (fifo_ == 0u)
  {
    it |= CAN_IT_FMP0;
  }
#endif
#ifdef CAN_IT_FMP1
  if (fifo_ == 1u)
  {
    it |= CAN_IT_FMP1;
  }
#endif

#ifdef CAN_IT_TME
  it |= CAN_IT_TME;
#endif

#ifdef CAN_IT_ERR
  it |= CAN_IT_ERR;
#endif
#ifdef CAN_IT_BOF
  it |= CAN_IT_BOF;
#endif
#ifdef CAN_IT_EPV
  it |= CAN_IT_EPV;
#endif
#ifdef CAN_IT_EWG
  it |= CAN_IT_EWG;
#endif
#ifdef CAN_IT_LEC
  it |= CAN_IT_LEC;
#endif

  if (it != 0u)
  {
    // Clear pending interrupt flags before enabling interrupt sources.
#ifdef CAN_IT_TME
    if ((it & CAN_IT_TME) != 0u)
    {
      CAN_ClearITPendingBit(instance_, CAN_IT_TME);
    }
#endif

#ifdef CAN_IT_FMP0
    if (fifo_ == 0u && ((it & CAN_IT_FMP0) != 0u))
    {
      CAN_ClearITPendingBit(instance_, CAN_IT_FMP0);
    }
#endif
#ifdef CAN_IT_FMP1
    if (fifo_ == 1u && ((it & CAN_IT_FMP1) != 0u))
    {
      CAN_ClearITPendingBit(instance_, CAN_IT_FMP1);
    }
#endif

    CAN_ITConfig(instance_, it, ENABLE);
  }
}

static inline bool fill_keep_zero_from_cache(CAN::BitTiming& dst,
                                             const CAN::BitTiming& cache)
{
  auto keep = [&](uint32_t& field, uint32_t cached) -> bool
  {
    if (field == 0u)
    {
      if (cached == 0u)
      {
        return false;
      }
      field = cached;
    }
    return true;
  };

  bool ok = true;
  ok &= keep(dst.brp, cache.brp);
  ok &= keep(dst.prop_seg, cache.prop_seg);
  ok &= keep(dst.phase_seg1, cache.phase_seg1);
  ok &= keep(dst.phase_seg2, cache.phase_seg2);
  ok &= keep(dst.sjw, cache.sjw);
  return ok;
}

ErrorCode CH32CAN::SetConfig(const CAN::Configuration& cfg_in)
{
  if (instance_ == nullptr)
  {
    return ErrorCode::ARG_ERR;
  }

  // 保证所有返回路径都会恢复 IRQ 状态。
  // Ensure IRQ state restoration on all return paths.
  struct IrqGuard
  {
    CH32CAN* self;
    explicit IrqGuard(CH32CAN* s) : self(s) { self->DisableIRQs(); }
    ~IrqGuard() { self->EnableIRQs(); }
  } guard(this);

  ErrorCode ec = ErrorCode::OK;
  bool entered_init = false;

  // 构造实际使用的时序参数；为 0 的字段复用缓存配置。
  // Build effective timing values; zero fields reuse cached configuration.
  CAN::Configuration cfg = cfg_in;

  if (!fill_keep_zero_from_cache(cfg.bit_timing, cfg_cache_.bit_timing))
  {
    return ErrorCode::ARG_ERR;
  }

  // 校验 bxCAN 时序约束。
  // Validate bxCAN timing constraints.
  const CAN::BitTiming& bt = cfg.bit_timing;

  // BRP: 1..1024.
  if (bt.brp < 1u || bt.brp > 1024u)
  {
    ASSERT(false);
    return ErrorCode::ARG_ERR;
  }

  // BS1 = PROP_SEG + PHASE_SEG1: 1..16.
  const uint32_t BS1 = bt.prop_seg + bt.phase_seg1;
  if (BS1 < 1u || BS1 > 16u)
  {
    ASSERT(false);
    return ErrorCode::ARG_ERR;
  }

  // BS2: 1..8.
  if (bt.phase_seg2 < 1u || bt.phase_seg2 > 8u)
  {
    ASSERT(false);
    return ErrorCode::ARG_ERR;
  }

  // SJW: 1..4 and SJW <= BS2.
  if (bt.sjw < 1u || bt.sjw > 4u || bt.sjw > bt.phase_seg2)
  {
    ASSERT(false);
    return ErrorCode::ARG_ERR;
  }

  // 进入初始化模式。
  // Enter initialization mode.
  if (CAN_OperatingModeRequest(instance_, CAN_OperatingMode_Initialization) ==
      CAN_ModeStatus_Failed)
  {
    return ErrorCode::FAILED;
  }
  entered_init = true;

  // 填充并应用初始化结构体。
  // Apply the initialization structure.
  CAN_InitTypeDef init;
  std::memset(&init, 0, sizeof(init));
  CAN_StructInit(&init);

  init.CAN_Prescaler = static_cast<uint16_t>(bt.brp);
  init.CAN_Mode = ch32_can_mode_macro(cfg.mode);

  init.CAN_SJW = static_cast<uint8_t>(bt.sjw - 1u);
  init.CAN_BS1 = static_cast<uint8_t>(BS1 - 1u);
  init.CAN_BS2 = static_cast<uint8_t>(bt.phase_seg2 - 1u);

  // 工作模式映射。
  // Mode mapping.
  init.CAN_NART = cfg.mode.one_shot ? ENABLE : DISABLE;

  // 控制器默认选项。
  // Default controller options.
  init.CAN_TTCM = DISABLE;
  init.CAN_ABOM = ENABLE;   // auto bus-off management
  init.CAN_AWUM = DISABLE;  // auto wake-up
  init.CAN_RFLM = DISABLE;  // FIFO not locked
  init.CAN_TXFP = ENABLE;   // prioritize by TX request order

  if (CAN_Init(instance_, &init) != CAN_InitStatus_Success)
  {
    ec = ErrorCode::FAILED;
  }
  else
  {
    if (CAN_OperatingModeRequest(instance_, CAN_OperatingMode_Normal) ==
        CAN_ModeStatus_Failed)
    {
      ec = ErrorCode::FAILED;
    }
    else
    {
      // 仅在配置成功后更新缓存。
      // Update the cache only on successful configuration.
      cfg_cache_ = cfg;
      ec = ErrorCode::OK;
    }
  }

  // 如果进入 init 模式后配置失败，则请求回到 normal 模式。
  // If configuration fails after entering init mode, request normal mode.
  if (ec != ErrorCode::OK && entered_init)
  {
    (void)CAN_OperatingModeRequest(instance_, CAN_OperatingMode_Normal);
  }

  return ec;
}

uint32_t CH32CAN::GetClockFreq() const
{
  RCC_ClocksTypeDef clocks{};
  RCC_GetClocksFreq(&clocks);
  return clocks.PCLK1_Frequency;
}

inline void CH32CAN::BuildTxMsg(const ClassicPack& p, CanTxMsg& m)
{
  const bool IS_EXT = (p.type == Type::EXTENDED) || (p.type == Type::REMOTE_EXTENDED);
  const bool IS_RTR =
      (p.type == Type::REMOTE_STANDARD) || (p.type == Type::REMOTE_EXTENDED);

  m.DLC = (p.dlc <= 8u) ? static_cast<uint8_t>(p.dlc) : 8u;
  m.IDE = IS_EXT ? CAN_ID_EXT : CAN_ID_STD;
  m.RTR = IS_RTR ? CAN_RTR_REMOTE : CAN_RTR_DATA;

  m.StdId = IS_EXT ? 0u : (p.id & 0x7FFu);
  m.ExtId = IS_EXT ? (p.id & 0x1FFFFFFFu) : 0u;

  std::memcpy(m.Data, p.data, 8);
}

void CH32CAN::TxService()
{
  if (instance_ == nullptr)
  {
    return;
  }

  tx_pend_.store(1u, std::memory_order_release);

  uint32_t expected = 0u;
  if (!tx_lock_.compare_exchange_strong(expected, 1u, std::memory_order_acquire,
                                        std::memory_order_relaxed))
  {
    return;
  }

  constexpr uint32_t TME_MASK = (CAN_TSTATR_TME0 | CAN_TSTATR_TME1 | CAN_TSTATR_TME2);

  for (;;)
  {
    tx_pend_.store(0u, std::memory_order_release);

    while ((instance_->TSTATR & TME_MASK) != 0u)
    {
      ClassicPack p{};
      if (tx_retry_valid_)
      {
        p = tx_retry_pack_;
      }
      else if (tx_queue_.Pop(p) != ErrorCode::OK)
      {
        break;
      }

      BuildTxMsg(p, tx_msg_);

      uint8_t mb = CAN_Transmit(instance_, &tx_msg_);
      if (mb == CAN_TxStatus_NoMailBox)
      {
        tx_retry_pack_ = p;
        tx_retry_valid_ = true;
        break;
      }

      tx_retry_valid_ = false;
    }

    tx_lock_.store(0u, std::memory_order_release);

    if (tx_pend_.load(std::memory_order_acquire) == 0u)
    {
      return;
    }

    expected = 0u;
    if (!tx_lock_.compare_exchange_strong(expected, 1u, std::memory_order_acquire,
                                          std::memory_order_relaxed))
    {
      return;
    }
  }
}

ErrorCode CH32CAN::AddMessage(const ClassicPack& pack)
{
  if (pack.type == Type::ERROR)
  {
    return ErrorCode::ARG_ERR;
  }

  if (tx_queue_.Push(pack) != ErrorCode::OK)
  {
    return ErrorCode::FULL;
  }

  TxService();
  return ErrorCode::OK;
}

void CH32CAN::ProcessTxInterrupt()
{
#ifdef CAN_IT_TME
  if (instance_ == nullptr)
  {
    return;
  }

  if (CAN_GetITStatus(instance_, CAN_IT_TME) != RESET)
  {
    CAN_ClearITPendingBit(instance_, CAN_IT_TME);
    TxService();
  }
#else
  TxService();
#endif
}

void CH32CAN::ProcessRxInterrupt()
{
  if (instance_ == nullptr)
  {
    return;
  }

  // 先排空 RX FIFO，再确认 RX 挂起标志。
  // Drain the RX FIFO first, then acknowledge RX pending flags.
  while (CAN_MessagePending(instance_, fifo_) != 0u)
  {
    CAN_Receive(instance_, fifo_, &rx_msg_);

    ClassicPack p{};
    if (rx_msg_.IDE == CAN_ID_STD)
    {
      p.id = rx_msg_.StdId;
      p.type = Type::STANDARD;
    }
    else
    {
      p.id = rx_msg_.ExtId;
      p.type = Type::EXTENDED;
    }

    if (rx_msg_.RTR == CAN_RTR_REMOTE)
    {
      p.type = (p.type == Type::STANDARD) ? Type::REMOTE_STANDARD : Type::REMOTE_EXTENDED;
    }

    p.dlc = (rx_msg_.DLC <= 8u) ? rx_msg_.DLC : 8u;
    std::memcpy(p.data, rx_msg_.Data, 8);

    OnMessage(p, true);
  }

  // 确认 RX 挂起标志。
  // Acknowledge RX pending flags.
#ifdef CAN_IT_FMP0
  if (fifo_ == 0u)
  {
    if (CAN_GetITStatus(instance_, CAN_IT_FMP0) != RESET)
    {
      CAN_ClearITPendingBit(instance_, CAN_IT_FMP0);
    }
  }
#endif
#ifdef CAN_IT_FMP1
  if (fifo_ == 1u)
  {
    if (CAN_GetITStatus(instance_, CAN_IT_FMP1) != RESET)
    {
      CAN_ClearITPendingBit(instance_, CAN_IT_FMP1);
    }
  }
#endif
}

void CH32CAN::ProcessErrorInterrupt()
{
  if (instance_ == nullptr)
  {
    return;
  }

  // 在清 pending 位前先快照错误标志和 LEC。
  // Snapshot error flags and LEC before clearing pending bits.
  const bool BOF = (CAN_GetFlagStatus(instance_, CAN_FLAG_BOF) != RESET);
  const bool EPV = (CAN_GetFlagStatus(instance_, CAN_FLAG_EPV) != RESET);
  const bool EWG = (CAN_GetFlagStatus(instance_, CAN_FLAG_EWG) != RESET);
  const uint8_t LEC = CAN_GetLastErrorCode(instance_);

#ifdef CAN_IT_LEC
  if (CAN_GetITStatus(instance_, CAN_IT_LEC) != RESET)
  {
    CAN_ClearITPendingBit(instance_, CAN_IT_LEC);
  }
#endif
#ifdef CAN_IT_ERR
  if (CAN_GetITStatus(instance_, CAN_IT_ERR) != RESET)
  {
    CAN_ClearITPendingBit(instance_, CAN_IT_ERR);
  }
#endif
#ifdef CAN_IT_BOF
  if (CAN_GetITStatus(instance_, CAN_IT_BOF) != RESET)
  {
    CAN_ClearITPendingBit(instance_, CAN_IT_BOF);
  }
#endif
#ifdef CAN_IT_EPV
  if (CAN_GetITStatus(instance_, CAN_IT_EPV) != RESET)
  {
    CAN_ClearITPendingBit(instance_, CAN_IT_EPV);
  }
#endif
#ifdef CAN_IT_EWG
  if (CAN_GetITStatus(instance_, CAN_IT_EWG) != RESET)
  {
    CAN_ClearITPendingBit(instance_, CAN_IT_EWG);
  }
#endif

  ClassicPack p{};
  p.type = Type::ERROR;
  p.dlc = 0u;

  CAN::ErrorID eid = CAN::ErrorID::CAN_ERROR_ID_GENERIC;

  if (BOF)
  {
    eid = CAN::ErrorID::CAN_ERROR_ID_BUS_OFF;
  }
  else if (EPV)
  {
    eid = CAN::ErrorID::CAN_ERROR_ID_ERROR_PASSIVE;
  }
  else if (EWG)
  {
    eid = CAN::ErrorID::CAN_ERROR_ID_ERROR_WARNING;
  }
  else
  {
    switch (LEC)
    {
      case CAN_ErrorCode_StuffErr:
        eid = CAN::ErrorID::CAN_ERROR_ID_STUFF;
        break;
      case CAN_ErrorCode_FormErr:
        eid = CAN::ErrorID::CAN_ERROR_ID_FORM;
        break;
      case CAN_ErrorCode_ACKErr:
        eid = CAN::ErrorID::CAN_ERROR_ID_ACK;
        break;
      case CAN_ErrorCode_BitRecessiveErr:
        eid = CAN::ErrorID::CAN_ERROR_ID_BIT1;
        break;
      case CAN_ErrorCode_BitDominantErr:
        eid = CAN::ErrorID::CAN_ERROR_ID_BIT0;
        break;
      case CAN_ErrorCode_CRCErr:
        eid = CAN::ErrorID::CAN_ERROR_ID_CRC;
        break;
      default:
        eid = CAN::ErrorID::CAN_ERROR_ID_OTHER;
        break;
    }
  }

  p.id = static_cast<uint32_t>(eid);
  OnMessage(p, true);
}

ErrorCode CH32CAN::GetErrorState(CAN::ErrorState& state) const
{
  if (instance_ == nullptr)
  {
    return ErrorCode::ARG_ERR;
  }

  state.rx_error_counter = CAN_GetReceiveErrorCounter(instance_);
  state.tx_error_counter = CAN_GetLSBTransmitErrorCounter(instance_);

  state.bus_off = (CAN_GetFlagStatus(instance_, CAN_FLAG_BOF) != RESET);
  state.error_passive = (CAN_GetFlagStatus(instance_, CAN_FLAG_EPV) != RESET);
  state.error_warning = (CAN_GetFlagStatus(instance_, CAN_FLAG_EWG) != RESET);

  return ErrorCode::OK;
}

#if defined(CAN1)
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void CAN1_TX_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void CAN1_TX_IRQHandler(void)
{
  if (auto* can = LibXR::CH32CAN::map[CH32_CAN1])
  {
    can->ProcessTxInterrupt();
  }
}

// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void CAN1_RX1_IRQHandler(void)
    __attribute__((interrupt("WCH-Interrupt-fast")));
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void CAN1_RX1_IRQHandler(void)
{
  if (auto* can = LibXR::CH32CAN::map[CH32_CAN1])
  {
    can->ProcessRxInterrupt();
  }
}

// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void CAN1_SCE_IRQHandler(void)
    __attribute__((interrupt("WCH-Interrupt-fast")));
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void CAN1_SCE_IRQHandler(void)
{
  if (auto* can = LibXR::CH32CAN::map[CH32_CAN1])
  {
    can->ProcessErrorInterrupt();
  }
}
#endif

#if LIBXR_CH32_HAS_CAN2
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void CAN2_TX_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void CAN2_TX_IRQHandler(void)
{
  if (auto* can = LibXR::CH32CAN::map[CH32_CAN2])
  {
    can->ProcessTxInterrupt();
  }
}

// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void CAN2_RX0_IRQHandler(void)
    __attribute__((interrupt("WCH-Interrupt-fast")));
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void CAN2_RX0_IRQHandler(void)
{
  if (auto* can = LibXR::CH32CAN::map[CH32_CAN2])
  {
    can->ProcessRxInterrupt();
  }
}

// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void CAN2_RX1_IRQHandler(void)
    __attribute__((interrupt("WCH-Interrupt-fast")));
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void CAN2_RX1_IRQHandler(void)
{
  if (auto* can = LibXR::CH32CAN::map[CH32_CAN2])
  {
    can->ProcessRxInterrupt();
  }
}

// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void CAN2_SCE_IRQHandler(void)
    __attribute__((interrupt("WCH-Interrupt-fast")));
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void CAN2_SCE_IRQHandler(void)
{
  if (auto* can = LibXR::CH32CAN::map[CH32_CAN2])
  {
    can->ProcessErrorInterrupt();
  }
}
#endif
