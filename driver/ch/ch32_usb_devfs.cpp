// NOLINTBEGIN(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
// ch32_usb_devfs.cpp  (classic FSDEV / PMA)
#include "ch32_usb_dev.hpp"
#include "ch32_usb_endpoint.hpp"
#include "ch32_usb_rcc.hpp"
#include "ch32_usbcan_shared.hpp"
#include "ep.hpp"

using namespace LibXR;
using namespace LibXR::USB;

#if defined(RCC_APB1Periph_USB)

#ifdef USB_BASE
static constexpr uintptr_t USBDEV_REG_BASE = USB_BASE;
#else
static constexpr uintptr_t USBDEV_REG_BASE = 0x40005C00UL;
#endif

static inline volatile uint16_t* usbdev_cntr()
{
  // 固定的 MMIO 物理基地址。
  // Fixed MMIO physical base address.
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  return reinterpret_cast<volatile uint16_t*>(USBDEV_REG_BASE + 0x40U);
}
static inline volatile uint16_t* usbdev_istr()
{
  return reinterpret_cast<volatile uint16_t*>(USBDEV_REG_BASE + 0x44U);
}
static inline volatile uint16_t* usbdev_daddr()
{
  return reinterpret_cast<volatile uint16_t*>(USBDEV_REG_BASE + 0x4CU);
}
static inline volatile uint16_t* usbdev_btable()
{
  return reinterpret_cast<volatile uint16_t*>(USBDEV_REG_BASE + 0x50U);
}
static inline volatile uint16_t* usbdev_ep_reg(uint8_t ep)
{
  return reinterpret_cast<volatile uint16_t*>(USBDEV_REG_BASE +
                                              static_cast<uintptr_t>(ep) * 4U);
}

// 兼容 classic USBLIB 风格的宏别名。
// Compatibility aliases for classic USBLIB-style macro names.
#if !defined(USB_ISTR_CTR) && defined(ISTR_CTR)
#define USB_ISTR_CTR ISTR_CTR
#endif
#if !defined(USB_ISTR_RESET) && defined(ISTR_RESET)
#define USB_ISTR_RESET ISTR_RESET
#endif
#if !defined(USB_ISTR_SUSP) && defined(ISTR_SUSP)
#define USB_ISTR_SUSP ISTR_SUSP
#endif
#if !defined(USB_ISTR_WKUP) && defined(ISTR_WKUP)
#define USB_ISTR_WKUP ISTR_WKUP
#endif
#if !defined(USB_ISTR_EP_ID) && defined(ISTR_EP_ID)
#define USB_ISTR_EP_ID ISTR_EP_ID
#endif

#if !defined(USB_CNTR_FRES) && defined(CNTR_FRES)
#define USB_CNTR_FRES CNTR_FRES
#endif
#if !defined(USB_CNTR_CTRM) && defined(CNTR_CTRM)
#define USB_CNTR_CTRM CNTR_CTRM
#endif
#if !defined(USB_CNTR_RESETM) && defined(CNTR_RESETM)
#define USB_CNTR_RESETM CNTR_RESETM
#endif
#if !defined(USB_CNTR_SUSPM) && defined(CNTR_SUSPM)
#define USB_CNTR_SUSPM CNTR_SUSPM
#endif
#if !defined(USB_CNTR_WKUPM) && defined(CNTR_WKUPM)
#define USB_CNTR_WKUPM CNTR_WKUPM
#endif

#if !defined(USB_DADDR_EF) && defined(DADDR_EF)
#define USB_DADDR_EF DADDR_EF
#endif

#if !defined(USB_EP_CTR_TX) && defined(EP_CTR_TX)
#define USB_EP_CTR_TX EP_CTR_TX
#endif
#if !defined(USB_EP_CTR_RX) && defined(EP_CTR_RX)
#define USB_EP_CTR_RX EP_CTR_RX
#endif
#if !defined(USB_EP_SETUP) && defined(EP_SETUP)
#define USB_EP_SETUP EP_SETUP
#endif
#if !defined(USB_EP_KIND) && defined(EP_KIND)
#define USB_EP_KIND EP_KIND
#endif
#if !defined(USB_EP_T_FIELD) && defined(EP_T_FIELD)
#define USB_EP_T_FIELD EP_T_FIELD
#endif
#if !defined(USB_EPADDR_FIELD) && defined(EPADDR_FIELD)
#define USB_EPADDR_FIELD EPADDR_FIELD
#endif

#ifndef USB_ISTR_CTR
#define USB_ISTR_CTR 0x8000u
#endif
#ifndef USB_ISTR_RESET
#define USB_ISTR_RESET 0x0400u
#endif
#ifndef USB_ISTR_SUSP
#define USB_ISTR_SUSP 0x0800u
#endif
#ifndef USB_ISTR_WKUP
#define USB_ISTR_WKUP 0x1000u
#endif
#ifndef USB_ISTR_EP_ID
#define USB_ISTR_EP_ID 0x000Fu
#endif

#ifndef USB_CNTR_FRES
#define USB_CNTR_FRES 0x0001u
#endif
#ifndef USB_CNTR_CTRM
#define USB_CNTR_CTRM 0x8000u
#endif
#ifndef USB_CNTR_RESETM
#define USB_CNTR_RESETM 0x0400u
#endif
#ifndef USB_CNTR_SUSPM
#define USB_CNTR_SUSPM 0x0800u
#endif
#ifndef USB_CNTR_WKUPM
#define USB_CNTR_WKUPM 0x1000u
#endif

#ifndef USB_DADDR_EF
#define USB_DADDR_EF 0x0080u
#endif

#ifndef USB_EP_CTR_RX
#define USB_EP_CTR_RX 0x8000u
#endif
#ifndef USB_EP_CTR_TX
#define USB_EP_CTR_TX 0x0080u
#endif
#ifndef USB_EP_SETUP
#define USB_EP_SETUP 0x0800u
#endif
#ifndef USB_EP_KIND
#define USB_EP_KIND 0x0100u
#endif
#ifndef USB_EP_T_FIELD
#define USB_EP_T_FIELD 0x0600u
#endif
#ifndef USB_EPADDR_FIELD
#define USB_EPADDR_FIELD 0x000Fu
#endif

#ifndef USB_EP_TX_NAK
#define USB_EP_TX_NAK 0x0020u
#endif
#ifndef USB_EP_RX_DIS
#define USB_EP_RX_DIS 0x0000u
#endif
#ifndef USB_EP_RX_STALL
#define USB_EP_RX_STALL 0x1000u
#endif
#ifndef USB_EP_RX_NAK
#define USB_EP_RX_NAK 0x2000u
#endif
#ifndef USB_EP_RX_VALID
#define USB_EP_RX_VALID 0x3000u
#endif

#ifndef USB_EPREG_MASK
#define USB_EPREG_MASK                                                           \
  (USB_EP_CTR_RX | USB_EP_SETUP | USB_EP_T_FIELD | USB_EP_KIND | USB_EP_CTR_TX | \
   USB_EPADDR_FIELD)
#endif

static inline void usbdev_clear_istr(uint16_t mask)
{
  *usbdev_istr() = static_cast<uint16_t>(~mask);
}

static inline void usbdev_set_ep_address(uint8_t ep, uint8_t addr)
{
  volatile uint16_t* reg = usbdev_ep_reg(ep);
  const uint16_t CUR = *reg;

  const uint16_t V = static_cast<uint16_t>(USB_EP_CTR_RX | USB_EP_CTR_TX |
                                           (CUR & (USB_EPREG_MASK & ~USB_EPADDR_FIELD)) |
                                           (addr & USB_EPADDR_FIELD));

  *reg = V;
}

static LibXR::RawData select_buffer_dev_fs(USB::Endpoint::EPNumber ep_num,
                                           USB::Endpoint::Direction dir,
                                           const LibXR::RawData& buffer)
{
  if (ep_num == USB::Endpoint::EPNumber::EP0)
  {
    return buffer;
  }

  const size_t HALF = buffer.size_ / 2;
  if (dir == USB::Endpoint::Direction::OUT)
  {
    return LibXR::RawData(buffer.addr_, HALF);
  }
  return LibXR::RawData(reinterpret_cast<uint8_t*>(buffer.addr_) + HALF, HALF);
}

static void drain_usbdev_fs_pending_irqs()
{
  while (true)
  {
    const uint16_t ISTR = *usbdev_istr();

    if (ISTR & USB_ISTR_RESET)
    {
      usbdev_clear_istr(USB_ISTR_RESET);
      continue;
    }

    if (ISTR & USB_ISTR_SUSP)
    {
      usbdev_clear_istr(USB_ISTR_SUSP);
      continue;
    }

    if (ISTR & USB_ISTR_WKUP)
    {
      usbdev_clear_istr(USB_ISTR_WKUP);
      continue;
    }

    if ((ISTR & USB_ISTR_CTR) == 0)
    {
      break;
    }

    const uint8_t EP_ID = static_cast<uint8_t>(ISTR & USB_ISTR_EP_ID);

    uint16_t epr = *usbdev_ep_reg(EP_ID);
    if (epr & USB_EP_CTR_RX)
    {
      LibXR::CH32EndpointDevFs::ClearEpCtrRx(EP_ID);
    }

    epr = *usbdev_ep_reg(EP_ID);
    if (epr & USB_EP_CTR_TX)
    {
      LibXR::CH32EndpointDevFs::ClearEpCtrTx(EP_ID);
    }

    // 这里只清掉当前已经锁存的 pending 位；如果主机随后又产生了新事件，
    // 硬件会重新拉起 IRQ，下一次进入 handler 时再处理。
    // We only drain the bits that are latched right now. If the host produces a
    // new event after that point, hardware will assert the IRQ again and the next
    // handler entry will observe it.
  }
}

static void usbdev_fs_irqhandler()
{
  auto* usb = LibXR::CH32USBDeviceFS::self_;
  if (usb == nullptr || !usb->IsInited())
  {
    drain_usbdev_fs_pending_irqs();
    return;
  }

  auto& map = LibXR::CH32EndpointDevFs::map_dev_fs_;

  constexpr uint8_t OUT_IDX = static_cast<uint8_t>(LibXR::USB::Endpoint::Direction::OUT);
  constexpr uint8_t IN_IDX = static_cast<uint8_t>(LibXR::USB::Endpoint::Direction::IN);
  auto* out0 = map[0][OUT_IDX];
  auto* in0 = map[0][IN_IDX];
  ASSERT(out0 != nullptr);
  ASSERT(in0 != nullptr);

  while (true)
  {
    const uint16_t ISTR = *usbdev_istr();

    // 总线 reset 需要先重建 PMA 分配器和 EP0 默认状态，
    // 之后才继续处理新的 CTR 事件。
    // Bus reset rebuilds the PMA allocator and EP0 default state before
    // processing new CTR events again.
    if (ISTR & USB_ISTR_RESET)
    {
      usbdev_clear_istr(USB_ISTR_RESET);

      *usbdev_daddr() = USB_DADDR_EF;
      *usbdev_btable() = 0;

      LibXR::CH32EndpointDevFs::ResetPMAAllocator();

      usb->Deinit(true);
      usb->Init(true);

      out0->SetState(LibXR::USB::Endpoint::State::IDLE);
      in0->SetState(LibXR::USB::Endpoint::State::IDLE);

      LibXR::CH32EndpointDevFs::SetEpTxStatus(0, USB_EP_TX_NAK);
      LibXR::CH32EndpointDevFs::SetEpRxStatus(0, USB_EP_RX_VALID);
      continue;
    }

    // suspend 也走同一条控制端点恢复路径，
    // 但会先把 EP0 RX 保持在 NAK，等协议栈重新挂起下一笔传输。
    // Suspend follows the same control-endpoint recovery path, but keeps EP0 RX
    // in NAK until the stack re-arms the next transfer.
    if (ISTR & USB_ISTR_SUSP)
    {
      usbdev_clear_istr(USB_ISTR_SUSP);

      usb->Deinit(true);
      usb->Init(true);

      out0->SetState(LibXR::USB::Endpoint::State::IDLE);
      in0->SetState(LibXR::USB::Endpoint::State::IDLE);

      LibXR::CH32EndpointDevFs::SetEpTxStatus(0, USB_EP_TX_NAK);
      LibXR::CH32EndpointDevFs::SetEpRxStatus(0, USB_EP_RX_NAK);
      continue;
    }

    if (ISTR & USB_ISTR_WKUP)
    {
      usbdev_clear_istr(USB_ISTR_WKUP);
      continue;
    }

    if ((ISTR & USB_ISTR_CTR) == 0)
    {
      break;
    }

    // 经典 FSDEV 通过 CTR 标志一次只上报一个端点。
    // Classic FSDEV reports one endpoint at a time through CTR flags.
    const uint8_t EP_ID = static_cast<uint8_t>(ISTR & USB_ISTR_EP_ID);

    uint16_t epr = *usbdev_ep_reg(EP_ID);

    if (epr & USB_EP_CTR_RX)
    {
      if (EP_ID == 0)
      {
        if (epr & USB_EP_SETUP)
        {
          // 新的 SETUP 会开始一笔全新的控制传输；
          // 这里只清理挂起的 CTR 标志，EP0 RX/TX 由控制传输处理器重新挂起。
          // A new SETUP starts a fresh control transfer; only clear pending CTR
          // flags here, and let the control-transfer handlers re-arm EP0 RX/TX.
          if (epr & USB_EP_CTR_TX)
          {
            LibXR::CH32EndpointDevFs::ClearEpCtrTx(0);
          }
          LibXR::CH32EndpointDevFs::ClearEpCtrRx(0);

          out0->CopyRxDataToBuffer(sizeof(LibXR::USB::SetupPacket));
          usb->OnSetupPacket(true, reinterpret_cast<const LibXR::USB::SetupPacket*>(
                                       out0->GetBuffer().addr_));

          continue;
        }
        else
        {
          // 普通 EP0 OUT 数据/状态阶段完成。
          // Ordinary EP0 OUT data/status completion.
          LibXR::CH32EndpointDevFs::ClearEpCtrRx(0);
          const uint16_t LEN = LibXR::CH32EndpointDevFs::GetRxCount(0);
          out0->TransferComplete(LEN);
        }
      }
      else
      {
        // non-EP0 OUT 完成直接使用硬件锁存的该端点 RX 长度。
        // Non-EP0 OUT completion uses the per-endpoint RX length latched by hardware.
        LibXR::CH32EndpointDevFs::ClearEpCtrRx(EP_ID);
        const uint16_t LEN = LibXR::CH32EndpointDevFs::GetRxCount(EP_ID);
        if (map[EP_ID][OUT_IDX])
        {
          map[EP_ID][OUT_IDX]->TransferComplete(LEN);
        }
      }
    }

    epr = *usbdev_ep_reg(EP_ID);

    if (epr & USB_EP_CTR_TX)
    {
      if (EP_ID == 0)
      {
        // EP0 IN 完成事件本身就足够，不需要额外长度信息。
        // EP0 IN completion is self-sufficient; no extra length bookkeeping is needed.
        LibXR::CH32EndpointDevFs::ClearEpCtrTx(0);
        in0->TransferComplete(0);
      }
      else
      {
        // FSDEV 上的 non-EP0 IN 完成与 EP0 IN 一样，不额外携带长度信息。
        // Non-EP0 IN completion follows the same rule as EP0 IN on FSDEV.
        LibXR::CH32EndpointDevFs::ClearEpCtrTx(EP_ID);
        if (map[EP_ID][IN_IDX])
        {
          map[EP_ID][IN_IDX]->TransferComplete(0);
        }
      }
    }
  }
}

static void usb_irq_thunk() { usbdev_fs_irqhandler(); }

// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" __attribute__((interrupt("WCH-Interrupt-fast"))) void USBWakeUp_IRQHandler(
    void)
{
#if defined(EXTI_Line18)
  EXTI_ClearITPendingBit(EXTI_Line18);
#endif
}

CH32USBDeviceFS::CH32USBDeviceFS(
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
  ASSERT(EP_CFGS.size() > 0 && EP_CFGS.size() <= CH32EndpointDevFs::EP_DEV_FS_MAX_SIZE);

  auto cfgs_itr = EP_CFGS.begin();

  auto ep0_out =
      new CH32EndpointDevFs(USB::Endpoint::EPNumber::EP0, USB::Endpoint::Direction::OUT,
                            cfgs_itr->buffer, false);
  auto ep0_in =
      new CH32EndpointDevFs(USB::Endpoint::EPNumber::EP0, USB::Endpoint::Direction::IN,
                            cfgs_itr->buffer, false);

  USB::EndpointPool::SetEndpoint0(ep0_in, ep0_out);

  USB::Endpoint::EPNumber ep_index = USB::Endpoint::EPNumber::EP1;

  for (++cfgs_itr, ep_index = USB::Endpoint::EPNumber::EP1; cfgs_itr != EP_CFGS.end();
       ++cfgs_itr, ep_index = USB::Endpoint::NextEPNumber(ep_index))
  {
    if (cfgs_itr->is_in == -1)
    {
      auto ep_out = new CH32EndpointDevFs(
          ep_index, USB::Endpoint::Direction::OUT,
          select_buffer_dev_fs(ep_index, USB::Endpoint::Direction::OUT, cfgs_itr->buffer),
          false);
      USB::EndpointPool::Put(ep_out);

      auto ep_in = new CH32EndpointDevFs(
          ep_index, USB::Endpoint::Direction::IN,
          select_buffer_dev_fs(ep_index, USB::Endpoint::Direction::IN, cfgs_itr->buffer),
          false);
      USB::EndpointPool::Put(ep_in);
    }
    else
    {
      auto ep = new CH32EndpointDevFs(
          ep_index,
          cfgs_itr->is_in ? USB::Endpoint::Direction::IN : USB::Endpoint::Direction::OUT,
          cfgs_itr->buffer, true);
      USB::EndpointPool::Put(ep);
    }
  }
}

LibXR::ErrorCode CH32USBDeviceFS::SetAddress(uint8_t address,
                                             USB::DeviceCore::Context context)
{
  if (context == USB::DeviceCore::Context::STATUS_IN_COMPLETE)
  {
    const uint8_t N_EP = static_cast<uint8_t>(CH32EndpointDevFs::EP_DEV_FS_MAX_SIZE);
    for (uint8_t i = 0; i < N_EP; i++)
    {
      usbdev_set_ep_address(i, i);
    }

    *usbdev_daddr() = static_cast<uint16_t>(USB_DADDR_EF | address);

    CH32EndpointDevFs::SetEpTxStatus(0, USB_EP_TX_NAK);
    CH32EndpointDevFs::SetEpRxStatus(0, USB_EP_RX_VALID);
  }
  return LibXR::ErrorCode::OK;
}

void CH32USBDeviceFS::Start(bool)
{
  // FSDEV 使用共享 USB 48 MHz 时钟；如果这个时钟来自 USBHS PHY PLL，
  // 则必须先打开 USBHS 依赖时钟，再打开 USBDEV 本体时钟。
  // FSDEV uses the shared USB 48 MHz clock; when that clock comes from the
  // USBHS PHY PLL, keep the USBHS dependency clock enabled before USBDEV itself.
  LibXR::CH32UsbRcc::ConfigureUsb48M();
#if defined(RCC_USBCLK48MCLKSource_USBPHY) && defined(RCC_AHBPeriph_USBHS)
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBHS, ENABLE);
#endif
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);

#if defined(RCC_APB2Periph_GPIOA) && defined(GPIOA) && defined(GPIO_Pin_11) &&        \
    defined(GPIO_Pin_12) && defined(GPIO_Mode_Out_PP) && defined(GPIO_Speed_50MHz) && \
    defined(GPIO_Mode_IN_FLOATING)
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitTypeDef gpio{};
  gpio.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  gpio.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &gpio);
  GPIO_ResetBits(GPIOA, GPIO_Pin_11 | GPIO_Pin_12);

  gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &gpio);
#endif

  *usbdev_cntr() = USB_CNTR_FRES;
  *usbdev_cntr() = 0;

  usbdev_clear_istr(0xFFFFu);
  *usbdev_btable() = 0;

  *usbdev_cntr() = static_cast<uint16_t>(USB_CNTR_RESETM | USB_CNTR_SUSPM |
                                         USB_CNTR_WKUPM | USB_CNTR_CTRM);

#if defined(EXTEN_USBD_LS)
  EXTEN->EXTEN_CTR &= ~EXTEN_USBD_LS;
#endif

  *usbdev_daddr() = USB_DADDR_EF;

  CH32EndpointDevFs::SetEpTxStatus(0, USB_EP_TX_NAK);
  CH32EndpointDevFs::SetEpRxStatus(0, USB_EP_RX_VALID);

  // DeviceCore::Init() 可能早于 FSDEV reset/BTABLE 初始化就预挂起 OUT 端点；
  // 因此这里在硬件初始化完成后补一次 non-EP0 OUT 端点重装填。
  // DeviceCore::Init() may arm OUT endpoints before FSDEV reset/BTABLE
  // initialization; re-arm non-EP0 OUT endpoints that remain BUSY afterwards.
  auto& ep_map = LibXR::CH32EndpointDevFs::map_dev_fs_;
  constexpr uint8_t OUT_IDX = static_cast<uint8_t>(LibXR::USB::Endpoint::Direction::OUT);
  const uint8_t N_EP = static_cast<uint8_t>(LibXR::CH32EndpointDevFs::EP_DEV_FS_MAX_SIZE);
  for (uint8_t ep = 1; ep < N_EP; ++ep)
  {
    auto* out = ep_map[ep][OUT_IDX];
    if (out == nullptr)
    {
      continue;
    }
    if (out->GetState() != LibXR::USB::Endpoint::State::BUSY)
    {
      continue;
    }
    (void)out->Transfer(out->MaxTransferSize());
  }

  self_ = this;
  LibXR::CH32UsbCanShared::usb_inited.store(true, std::memory_order_release);
  LibXR::CH32UsbCanShared::register_usb_irq(&usb_irq_thunk);

  // 回调与端点就绪后再放行中断和连接主机。
  // Publish the handler and arm endpoints before enabling IRQs or the pull-up.
  NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
  NVIC_EnableIRQ(USBWakeUp_IRQn);
#if defined(EXTEN_USBD_PU_EN)
  EXTEN->EXTEN_CTR |= EXTEN_USBD_PU_EN;
#endif
}

void CH32USBDeviceFS::Stop(bool)
{
  // 先屏蔽 USB 中断源，再解除回调；共享 CAN 中断线可能仍然开启。
  // Mask USB interrupt sources before unregistering from live shared CAN IRQs.
  *usbdev_cntr() = USB_CNTR_FRES;
  LibXR::CH32UsbCanShared::register_usb_irq(nullptr);
  LibXR::CH32UsbCanShared::usb_inited.store(false, std::memory_order_release);
  self_ = nullptr;

#if defined(EXTEN_USBD_PU_EN)
  EXTEN->EXTEN_CTR &= ~EXTEN_USBD_PU_EN;
#endif

  if (!LibXR::CH32UsbCanShared::can1_active())
  {
    NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn);
  }
  NVIC_DisableIRQ(USBWakeUp_IRQn);
}

#endif  // defined(RCC_APB1Periph_USB)

// NOLINTEND(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
