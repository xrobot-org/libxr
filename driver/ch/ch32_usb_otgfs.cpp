// NOLINTBEGIN(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
// ch32_usb_otgfs.cpp  (OTG FS)
#include "ch32_usb_dev.hpp"
#include "ch32_usb_endpoint.hpp"
#include "ch32_usb_rcc.hpp"
#include "ep.hpp"

using namespace LibXR;
using namespace LibXR::USB;

#if defined(USBFSD)

namespace
{

constexpr uint8_t OTG_FS_CLEARABLE_MASK = USBFS_UIF_FIFO_OV | USBFS_UIF_HST_SOF |
                                          USBFS_UIF_SUSPEND | USBFS_UIF_TRANSFER |
                                          USBFS_UIF_DETECT | USBFS_UIF_BUS_RST;

static void ClearPendingOtgFsInterrupts()
{
  while (true)
  {
    const uint16_t INTFGST = *reinterpret_cast<volatile uint16_t*>(
        reinterpret_cast<uintptr_t>(&USBFSD->INT_FG));
    const uint8_t INTFLAG = static_cast<uint8_t>(INTFGST & 0x00FFu);
    const uint8_t PENDING = static_cast<uint8_t>(INTFLAG & OTG_FS_CLEARABLE_MASK);
    if (PENDING == 0u)
    {
      break;
    }
    USBFSD->INT_FG = PENDING;

    // 这个循环只清当前 INT_FG 快照里可见的 pending 位；主机后续的新事件
    // 会重新锁存成新的中断，并在下一次 IRQ 进入时处理。
    // This loop only drains the pending bits visible in the current INT_FG
    // snapshot. Any later host event will relatch a fresh interrupt and be
    // handled by the next IRQ entry.
  }
}

}  // namespace

// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" __attribute__((interrupt("WCH-Interrupt-fast"))) void USBFS_IRQHandler(void)
{
  auto* usb = LibXR::CH32USBOtgFS::self_;
  if (usb == nullptr || !usb->IsInited())
  {
    ClearPendingOtgFsInterrupts();
    return;
  }

  auto& map = LibXR::CH32EndpointOtgFs::map_otg_fs_;

  constexpr uint8_t OUT_IDX = static_cast<uint8_t>(LibXR::USB::Endpoint::Direction::OUT);
  constexpr uint8_t IN_IDX = static_cast<uint8_t>(LibXR::USB::Endpoint::Direction::IN);
  auto* out0 = map[0][OUT_IDX];
  auto* in0 = map[0][IN_IDX];
  ASSERT(out0 != nullptr);
  ASSERT(in0 != nullptr);

  // 处理顺序与控制传输生命周期保持一致：
  // 1) 总线级恢复
  // 2) token 完成与 setup 分发
  // Handle order matches the control-transfer lifecycle:
  // 1) bus-level recovery
  // 2) token completion / setup dispatch
  while (true)
  {
    const uint16_t INTFGST = *reinterpret_cast<volatile uint16_t*>(
        reinterpret_cast<uintptr_t>(&USBFSD->INT_FG));

    const uint8_t INTFLAG = static_cast<uint8_t>(INTFGST & 0x00FFu);
    const uint8_t INTST = static_cast<uint8_t>((INTFGST >> 8) & 0x00FFu);

    const uint8_t PENDING = static_cast<uint8_t>(INTFLAG & OTG_FS_CLEARABLE_MASK);
    if (PENDING == 0)
    {
      break;
    }

    uint8_t clear_mask = 0;

    // reset 会重建 EP0 状态，并把设备恢复到“等待 setup”的初始形态。
    // Reset rebuilds EP0 state and returns the device to "waiting for setup".
    if (PENDING & USBFS_UIF_BUS_RST)
    {
      USBFSD->DEV_ADDR = 0;

      usb->Deinit(true);
      usb->Init(true);

      out0->SetState(LibXR::USB::Endpoint::State::IDLE);
      in0->SetState(LibXR::USB::Endpoint::State::IDLE);
      out0->tog_ = true;
      in0->tog_ = true;

      USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_RES_NAK;
      USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_RES_NAK;

      clear_mask |= USBFS_UIF_BUS_RST;
    }

    // suspend 走与 reset 相同的 EP0 恢复路径；resume 由后续主机时序体现。
    // Suspend follows the same EP0 recovery path; resume is observed later by the host.
    if (PENDING & USBFS_UIF_SUSPEND)
    {
      usb->Deinit(true);
      usb->Init(true);

      out0->SetState(LibXR::USB::Endpoint::State::IDLE);
      in0->SetState(LibXR::USB::Endpoint::State::IDLE);
      out0->tog_ = true;
      in0->tog_ = true;

      USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_RES_NAK;
      USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_RES_NAK;

      clear_mask |= USBFS_UIF_SUSPEND;
    }

    if (PENDING & USBFS_UIF_TRANSFER)
    {
      const uint8_t TOKEN = INTST & USBFS_UIS_TOKEN_MASK;
      const uint8_t EPNUM = INTST & USBFS_UIS_ENDP_MASK;

      auto& ep = map[EPNUM];

      switch (TOKEN)
      {
        case USBFS_UIS_TOKEN_SETUP:
        {
          // 新的 setup 会中断前一笔 EP0 事务，因此这里在把 setup 包交给 DeviceCore
          // 之前，先把 EP0 的双向状态恢复到 IDLE/TOG0。
          // A fresh setup cancels the previous EP0 transaction, so both directions are
          // reset to IDLE/TOG0 before handing the setup packet to DeviceCore.
          USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_RES_NAK;
          USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_RES_NAK;

          out0->SetState(LibXR::USB::Endpoint::State::IDLE);
          in0->SetState(LibXR::USB::Endpoint::State::IDLE);
          out0->tog_ = true;
          in0->tog_ = true;

          usb->OnSetupPacket(
              true, reinterpret_cast<const SetupPacket*>(out0->GetBuffer().addr_));
          break;
        }

        case USBFS_UIS_TOKEN_OUT:
        {
          // OTGFS 硬件已经给出了本次 token 的完成 RX 长度。
          // OTGFS hardware already reports the completed RX length for this token.
          const uint16_t LEN = USBFSD->RX_LEN;
          if (ep[OUT_IDX])
          {
            ep[OUT_IDX]->TransferComplete(LEN);
          }
          break;
        }

        case USBFS_UIS_TOKEN_IN:
        {
          // IN token 完成不需要额外 payload 长度，事件本身就足够了。
          // IN token completion has no payload length; completion itself is enough.
          if (ep[IN_IDX])
          {
            ep[IN_IDX]->TransferComplete(0);
          }
          break;
        }

        default:
          break;
      }

      clear_mask |= USBFS_UIF_TRANSFER;
    }

    clear_mask |= static_cast<uint8_t>(PENDING & ~clear_mask);
    USBFSD->INT_FG = clear_mask;
  }
}

CH32USBOtgFS::CH32USBOtgFS(
    const std::initializer_list<EPConfig> EP_CFGS,
    USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
    uint16_t bcd,
    const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> LANG_LIST,
    const std::initializer_list<const std::initializer_list<USB::ConfigDescriptorItem*>>
        CONFIGS,
    ConstRawData uid)
    : USB::EndpointPool(EP_CFGS.size() * 2),
      USB::DeviceCore(*this, USB::USBSpec::USB_2_1, USB::Speed::FULL, packet_size, vid,
                      pid, bcd, LANG_LIST, CONFIGS, uid)
{
  ASSERT(EP_CFGS.size() > 0 && EP_CFGS.size() <= CH32EndpointOtgFs::EP_OTG_FS_MAX_SIZE);

  auto cfgs_itr = EP_CFGS.begin();

  auto ep0_out =
      new CH32EndpointOtgFs(USB::Endpoint::EPNumber::EP0, USB::Endpoint::Direction::OUT,
                            cfgs_itr->buffer, false);
  auto ep0_in =
      new CH32EndpointOtgFs(USB::Endpoint::EPNumber::EP0, USB::Endpoint::Direction::IN,
                            cfgs_itr->buffer, false);

  USB::EndpointPool::SetEndpoint0(ep0_in, ep0_out);

  USB::Endpoint::EPNumber ep_index = USB::Endpoint::EPNumber::EP1;

  for (++cfgs_itr, ep_index = USB::Endpoint::EPNumber::EP1; cfgs_itr != EP_CFGS.end();
       ++cfgs_itr, ep_index = USB::Endpoint::NextEPNumber(ep_index))
  {
    if (cfgs_itr->is_in == -1)
    {
      auto ep_out = new CH32EndpointOtgFs(ep_index, USB::Endpoint::Direction::OUT,
                                          cfgs_itr->buffer, false);
      USB::EndpointPool::Put(ep_out);

      auto ep_in = new CH32EndpointOtgFs(ep_index, USB::Endpoint::Direction::IN,
                                         cfgs_itr->buffer, false);
      USB::EndpointPool::Put(ep_in);
    }
    else
    {
      auto ep = new CH32EndpointOtgFs(
          ep_index,
          cfgs_itr->is_in ? USB::Endpoint::Direction::IN : USB::Endpoint::Direction::OUT,
          cfgs_itr->buffer, true);
      USB::EndpointPool::Put(ep);
    }
  }
}

LibXR::ErrorCode CH32USBOtgFS::SetAddress(uint8_t address,
                                          USB::DeviceCore::Context context)
{
  if (context == USB::DeviceCore::Context::STATUS_IN_COMPLETE)
  {
    USBFSD->DEV_ADDR = (USBFSD->DEV_ADDR & USBFS_UDA_GP_BIT) | address;
    USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_RES_NAK;
    USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_RES_ACK;
  }
  return LibXR::ErrorCode::OK;
}

void CH32USBOtgFS::Start(bool)
{
  // OTGFS 与 FSDEV 共用同一套 USB 48 MHz 时钟选择规则。
  // OTGFS uses the same shared USB 48 MHz clock selection as FSDEV.
  LibXR::CH32UsbRcc::ConfigureUsb48M();
#if defined(RCC_USBCLK48MCLKSource_USBPHY) && defined(RCC_AHBPeriph_USBHS)
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBHS, ENABLE);
#endif
#if defined(RCC_AHBPeriph_USBFS)
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBFS, ENABLE);
#elif defined(RCC_AHBPeriph_USBOTGFS)
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBOTGFS, ENABLE);
#endif
  USBFSH->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
  USBFSH->BASE_CTRL = 0x00;
  USBFSD->INT_EN = USBFS_UIE_SUSPEND | USBFS_UIE_BUS_RST | USBFS_UIE_TRANSFER;
  USBFSD->BASE_CTRL = USBFS_UC_DEV_PU_EN | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN;
  USBFSD->UDEV_CTRL = USBFS_UD_PD_DIS | USBFS_UD_PORT_EN;
  NVIC_EnableIRQ(USBFS_IRQn);
  self_ = this;
}

void CH32USBOtgFS::Stop(bool)
{
  USBFSH->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
  USBFSD->BASE_CTRL = 0x00;
  NVIC_DisableIRQ(USBFS_IRQn);
  self_ = nullptr;
}

#endif  // defined(USBFSD)

// NOLINTEND(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
