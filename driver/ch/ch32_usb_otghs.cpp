// NOLINTBEGIN(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
// ch32_usb_otghs.cpp  (OTG HS)
#include "ch32_usb_dev.hpp"
#include "ch32_usb_endpoint.hpp"
#include "ch32_usb_rcc.hpp"
#include "ep.hpp"

using namespace LibXR;
using namespace LibXR::USB;

#if defined(USBHSD)

namespace
{

using OtgHsEndpointMap = decltype(LibXR::CH32EndpointOtgHs::map_otg_hs_);

constexpr uint8_t OUT_IDX = static_cast<uint8_t>(LibXR::USB::Endpoint::Direction::OUT);
constexpr uint8_t IN_IDX = static_cast<uint8_t>(LibXR::USB::Endpoint::Direction::IN);

static void ResetEp0State(OtgHsEndpointMap& map)
{
  auto* out0 = map[0][OUT_IDX];
  auto* in0 = map[0][IN_IDX];

  DEV_ASSERT_FROM_CALLBACK(out0 != nullptr, true);
  DEV_ASSERT_FROM_CALLBACK(in0 != nullptr, true);

  out0->SetState(LibXR::USB::Endpoint::State::IDLE);
  out0->tog0_ = true;
  out0->tog1_ = false;
  in0->SetState(LibXR::USB::Endpoint::State::IDLE);
  in0->tog0_ = true;
  in0->tog1_ = false;

  // reset/suspend 恢复后，EP0 必须回到等待下一包 setup 的状态。
  // After reset/suspend recovery, EP0 must be ready for the next setup packet.
  USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_NAK;
  USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
}

static bool CompleteEp0InBeforeSetup(OtgHsEndpointMap& map, uint8_t int_flag,
                                     uint8_t int_st)
{
  if ((int_flag & USBHS_UIF_SETUP_ACT) == 0u || (int_flag & USBHS_UIF_TRANSFER) == 0u)
  {
    return false;
  }
  if ((int_st & USBHS_UIS_ENDP_MASK) != 0u ||
      (int_st & USBHS_UIS_TOKEN_MASK) != USBHS_UIS_TOKEN_IN)
  {
    return false;
  }

  auto* in0 = map[0][IN_IDX];
  DEV_ASSERT_FROM_CALLBACK(in0 != nullptr, true);
  if (in0->GetState() != LibXR::USB::Endpoint::State::BUSY)
  {
    return false;
  }

  // 部分 setup 竞争会把“新 setup 到来”和“上一笔 EP0 IN 完成”同时上报；
  // 这里先消费旧的 EP0 IN 完成，再让 setup 重建控制传输状态。
  // Some setup races report "new setup" and "previous EP0 IN complete" together.
  // Consume the old EP0 IN completion first, then let setup rebuild the control state.
  in0->TransferComplete(0u);
  return true;
}

static void PrepareEp0ForSetup(OtgHsEndpointMap& map)
{
  auto* out0 = map[0][OUT_IDX];
  auto* in0 = map[0][IN_IDX];

  DEV_ASSERT_FROM_CALLBACK(out0 != nullptr, true);
  DEV_ASSERT_FROM_CALLBACK(in0 != nullptr, true);

  // 新的 SETUP 会中断前一笔控制传输，因此在分发 setup 包之前，
  // 必须先把 EP0 恢复成默认控制端点形态。
  // A fresh SETUP cancels the previous control transfer, so EP0 must be re-armed
  // to the default control endpoint shape before the setup packet is dispatched.
  out0->SetState(LibXR::USB::Endpoint::State::IDLE);
  in0->SetState(LibXR::USB::Endpoint::State::IDLE);
  out0->tog0_ = true;
  out0->tog1_ = false;
  in0->tog0_ = true;
  in0->tog1_ = false;
  USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_NAK;
  USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_NAK;
}

static void HandleTransferOut(CH32EndpointOtgHs* ep_out, uint8_t ep_num, uint8_t int_st)
{
  if (ep_out == nullptr)
  {
    return;
  }

  const bool is_nak = ((int_st & USBHS_UIS_IS_NAK) != 0u);
  const bool busy = (ep_out->GetState() == LibXR::USB::Endpoint::State::BUSY);
  const uint16_t rx_len = USBHSD->RX_LEN;

  if (ep_num == 0u)
  {
    if (busy && ((rx_len > 0u) || !is_nak))
    {
      ep_out->TransferComplete(rx_len);
    }
    return;
  }

  if (!is_nak)
  {
    ep_out->TransferComplete(rx_len);
    return;
  }
  if (!busy || rx_len == 0u)
  {
    return;
  }

  // 部分 non-EP0 OUT 路径在端点仍处于 BUSY 时，仍可能给出可用的晚到长度快照；
  // 这里只保留这条窄恢复路径。
  // Some non-EP0 OUT paths may still expose a usable late length snapshot while the
  // endpoint stays BUSY. Keep that narrow recovery path here.
  ep_out->TransferComplete(rx_len);
}

static void HandleTransferIn(CH32EndpointOtgHs* ep_in, uint8_t ep_num, uint8_t int_st)
{
  if (ep_in == nullptr)
  {
    return;
  }

  const bool is_nak = ((int_st & USBHS_UIS_IS_NAK) != 0u);
  if (!is_nak)
  {
    ep_in->TransferComplete(0u);
    return;
  }

  const bool busy = (ep_in->GetState() == LibXR::USB::Endpoint::State::BUSY);
  const bool ep0_data_in_complete =
      (ep_num == 0u) && busy && (ep_in->last_transfer_size_ > 0u);
  if (!ep0_data_in_complete)
  {
    return;
  }

  // EP0 的数据 IN 在 setup 仲裁后也可能没有可见 ACK 就结束；
  // 但 status ZLP 仍然必须保持严格，因此这里只允许非零数据阶段走这条路径。
  // EP0 data IN may complete after setup arbitration without a visible ACK.
  // Status ZLP must still stay strict, so only non-zero data stages use this path.
  ep_in->TransferComplete(0u);
}

static void HandleTransferToken(OtgHsEndpointMap& map, uint8_t int_st)
{
  const uint8_t token = int_st & USBHS_UIS_TOKEN_MASK;
  const uint8_t ep_num = int_st & USBHS_UIS_ENDP_MASK;
  auto& ep = map[ep_num];

  switch (token)
  {
    case USBHS_UIS_TOKEN_SETUP:
    {
      // CH32V30x USBHS 通过 SETUP_ACT 上报 SETUP，因此这里的 TOKEN_SETUP
      // 只是信息位，不能再次重复执行 setup 路径。
      // CH32V30x USBHS reports SETUP through SETUP_ACT, so TOKEN_SETUP is
      // informational only here and must not re-run the setup path.
      break;
    }

    case USBHS_UIS_TOKEN_OUT:
      HandleTransferOut(ep[OUT_IDX], ep_num, int_st);
      break;

    case USBHS_UIS_TOKEN_IN:
      HandleTransferIn(ep[IN_IDX], ep_num, int_st);
      break;

    default:
      break;
  }
}

static void ClearPendingOtgHsInterrupts()
{
  while (true)
  {
    const uint16_t INTFGST = *reinterpret_cast<volatile uint16_t*>(
        reinterpret_cast<uintptr_t>(&USBHSD->INT_FG));
    const uint8_t INTFLAG = static_cast<uint8_t>(INTFGST & 0x00FFu);
    if (INTFLAG == 0u)
    {
      break;
    }
    USBHSD->INT_FG = INTFLAG;

    // 这个循环只清理 INT_FG 里已经锁存的位；如果之后又出现新的总线事件，
    // 硬件会重新触发 IRQ，由下一次进入 handler 时再清。
    // This loop drains only the bits already latched in INT_FG. If hardware
    // observes another bus event later, it will assert a new IRQ and the next
    // handler entry will clear it.
  }
}

}  // namespace

// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" __attribute__((interrupt("WCH-Interrupt-fast"))) void USBHS_IRQHandler(void)
{
  auto* usb = LibXR::CH32USBOtgHS::self_;
  if (usb == nullptr || !usb->IsInited())
  {
    ClearPendingOtgHsInterrupts();
    return;
  }

  auto& map = LibXR::CH32EndpointOtgHs::map_otg_hs_;

  // 处理顺序很重要：先恢复总线级事件，再收束 EP0 setup 仲裁，
  // 最后分发普通 token 完成事件。
  // Handle order matters: recover bus-level events first, then settle EP0 setup
  // arbitration, and finally dispatch ordinary token completions.
  while (true)
  {
    const uint16_t INTFGST = *reinterpret_cast<volatile uint16_t*>(
        reinterpret_cast<uintptr_t>(&USBHSD->INT_FG));

    const uint8_t INTFLAG = static_cast<uint8_t>(INTFGST & 0x00FFu);
    const uint8_t INTST = static_cast<uint8_t>((INTFGST >> 8) & 0x00FFu);

    if (INTFLAG == 0)
    {
      break;
    }

    uint8_t clear_mask = 0;

    // 1) 总线级恢复：reset/suspend 必须优先重建 EP0。
    // 1) Bus-level recovery: reset/suspend must rebuild EP0 first.
    if (INTFLAG & USBHS_UIF_BUS_RST)
    {
      USBHSD->DEV_AD = 0;
      usb->Deinit(true);
      usb->Init(true);
      ResetEp0State(map);
      clear_mask |= USBHS_UIF_BUS_RST;
    }

    if (INTFLAG & USBHS_UIF_SUSPEND)
    {
      usb->Deinit(true);
      usb->Init(true);
      ResetEp0State(map);
      clear_mask |= USBHS_UIF_SUSPEND;
    }

    // 2) Setup 仲裁：在新 setup 重置控制传输状态之前，
    // 先消费上一笔 EP0 IN 完成。
    // 2) Setup arbitration: consume the previous EP0 IN completion before the
    // new setup resets the control-transfer state.
    if (CompleteEp0InBeforeSetup(map, INTFLAG, INTST))
    {
      clear_mask |= USBHS_UIF_TRANSFER;
    }

    if (INTFLAG & USBHS_UIF_SETUP_ACT)
    {
      PrepareEp0ForSetup(map);

      auto* out0 = map[0][OUT_IDX];
      DEV_ASSERT_FROM_CALLBACK(out0 != nullptr, true);
      const auto* setup =
          reinterpret_cast<const LibXR::USB::SetupPacket*>(out0->GetBuffer().addr_);
      usb->OnSetupPacket(true, setup);
      clear_mask |= USBHS_UIF_SETUP_ACT;
    }

    // 3) 普通 token 完成：非 setup 传输统一在这里分发。
    // 3) Ordinary token completion: non-setup transfers arrive here.
    if ((INTFLAG & USBHS_UIF_TRANSFER) && ((clear_mask & USBHS_UIF_TRANSFER) == 0u))
    {
      HandleTransferToken(map, INTST);
      clear_mask |= USBHS_UIF_TRANSFER;
    }

    clear_mask |= static_cast<uint8_t>(INTFLAG & ~(clear_mask));
    USBHSD->INT_FG = clear_mask;
  }
}

CH32USBOtgHS::CH32USBOtgHS(
    const std::initializer_list<CH32USBOtgHS::EPConfig> EP_CFGS, uint16_t vid,
    uint16_t pid, uint16_t bcd,
    const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> LANG_LIST,
    const std::initializer_list<const std::initializer_list<USB::ConfigDescriptorItem*>>
        CONFIGS,
    ConstRawData uid)
    : USB::EndpointPool(EP_CFGS.size() * 2),
      USB::DeviceCore(*this, USB::USBSpec::USB_2_1, USB::Speed::HIGH,
                      USB::DeviceDescriptor::PacketSize0::SIZE_64, vid, pid, bcd,
                      LANG_LIST, CONFIGS, uid)
{
  ASSERT(EP_CFGS.size() > 0 && EP_CFGS.size() <= CH32EndpointOtgHs::EP_OTG_HS_MAX_SIZE);

  auto cfgs_itr = EP_CFGS.begin();

  ASSERT(cfgs_itr->buffer_tx.size_ >= 64 && cfgs_itr->buffer_rx.size_ >= 64 &&
         cfgs_itr->double_buffer == false);

  auto ep0_out =
      new CH32EndpointOtgHs(USB::Endpoint::EPNumber::EP0, USB::Endpoint::Direction::OUT,
                            cfgs_itr->buffer_rx, false);
  auto ep0_in =
      new CH32EndpointOtgHs(USB::Endpoint::EPNumber::EP0, USB::Endpoint::Direction::IN,
                            cfgs_itr->buffer_tx, false);

  USB::EndpointPool::SetEndpoint0(ep0_in, ep0_out);

  USB::Endpoint::EPNumber ep_index = USB::Endpoint::EPNumber::EP1;

  for (++cfgs_itr, ep_index = USB::Endpoint::EPNumber::EP1; cfgs_itr != EP_CFGS.end();
       ++cfgs_itr, ep_index = USB::Endpoint::NextEPNumber(ep_index))
  {
    if (!cfgs_itr->double_buffer)
    {
      auto ep_out = new CH32EndpointOtgHs(ep_index, USB::Endpoint::Direction::OUT,
                                          cfgs_itr->buffer_rx, false);
      USB::EndpointPool::Put(ep_out);

      auto ep_in = new CH32EndpointOtgHs(ep_index, USB::Endpoint::Direction::IN,
                                         cfgs_itr->buffer_tx, false);
      USB::EndpointPool::Put(ep_in);
    }
    else
    {
      auto ep = new CH32EndpointOtgHs(
          ep_index,
          cfgs_itr->is_in ? USB::Endpoint::Direction::IN : USB::Endpoint::Direction::OUT,
          cfgs_itr->buffer_tx, true);
      USB::EndpointPool::Put(ep);
    }
  }
}

LibXR::ErrorCode CH32USBOtgHS::SetAddress(uint8_t address,
                                          USB::DeviceCore::Context context)
{
  if (context == USB::DeviceCore::Context::STATUS_IN_COMPLETE)
  {
    USBHSD->DEV_AD = address;
  }
  return LibXR::ErrorCode::OK;
}

void CH32USBOtgHS::Start(bool)
{
  // OTGHS 先选择共享 48 MHz 时钟源，再打开 USBHS 自己的总线时钟。
  // OTGHS selects the shared 48 MHz source first, then enables its own bus clock.
  LibXR::CH32UsbRcc::ConfigureUsb48M();
#if defined(RCC_AHBPeriph_USBHS)
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBHS, ENABLE);
#endif
  USBHSD->CONTROL = USBHS_UC_CLR_ALL | USBHS_UC_RESET_SIE;
  USBHSD->CONTROL &= ~USBHS_UC_RESET_SIE;
  USBHSD->HOST_CTRL = USBHS_UH_PHY_SUSPENDM;
  USBHSD->CONTROL = USBHS_UC_DMA_EN | USBHS_UC_INT_BUSY | USBHS_UC_SPEED_HIGH;
  USBHSD->INT_EN =
      USBHS_UIE_SETUP_ACT | USBHS_UIE_TRANSFER | USBHS_UIE_DETECT | USBHS_UIE_SUSPEND;
  USBHSD->CONTROL |= USBHS_UC_DEV_PU_EN;
  NVIC_EnableIRQ(USBHS_IRQn);
  self_ = this;
}

void CH32USBOtgHS::Stop(bool)
{
  USBHSD->CONTROL = USBHS_UC_CLR_ALL | USBHS_UC_RESET_SIE;
  USBHSD->CONTROL = 0;
  NVIC_DisableIRQ(USBHS_IRQn);
  self_ = nullptr;
}

#endif  // defined(USBHSD)

// NOLINTEND(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
