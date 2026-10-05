#include "esp_usb_dev.hpp"

#if SOC_USB_OTG_SUPPORTED && defined(CONFIG_IDF_TARGET_ESP32S3) && \
    CONFIG_IDF_TARGET_ESP32S3

#include <cstring>
#include <new>

#include "esp32s3/rom/usb/cdc_acm.h"
#include "esp32s3/rom/usb/usb_dc.h"
#include "esp32s3/rom/usb/usb_device.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_private/usb_phy.h"
#include "esp_usb.hpp"
#include "esp_usb_ep.hpp"
#include "soc/interrupts.h"

namespace LibXR
{

// 构造时先建立 endpoint 对象映射，再交给上层 USB core 统一驱动 / Build the
// endpoint-object map first, then hand it to the shared USB core.
ESP32USBDevice::ESP32USBDevice(
    const std::initializer_list<EPConfig> ep_cfgs,
    USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
    uint16_t bcd,
    const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> lang_list,
    const std::initializer_list<const std::initializer_list<USB::ConfigDescriptorItem*>>
        configs,
    ConstRawData uid)
    : USB::EndpointPool(ep_cfgs.size() * 2U),
      USB::DeviceCore(*this, USB::USBSpec::USB_2_1, USB::Speed::FULL, packet_size, vid,
                      pid, bcd, lang_list, configs, uid)
{
  ASSERT(ep_cfgs.size() > 0U && ep_cfgs.size() <= ENDPOINT_COUNT);

  auto cfg_it = ep_cfgs.begin();

  auto* ep0_out = new ESP32USBEndpoint(*this, USB::Endpoint::EPNumber::EP0,
                                       USB::Endpoint::Direction::OUT, cfg_it->buffer);
  auto* ep0_in = new ESP32USBEndpoint(*this, USB::Endpoint::EPNumber::EP0,
                                      USB::Endpoint::Direction::IN, cfg_it->buffer);

  SetEndpoint0(ep0_in, ep0_out);
  endpoint_map_.in[0] = ep0_in;
  endpoint_map_.out[0] = ep0_out;

  auto ep_num = USB::Endpoint::EPNumber::EP1;
  for (++cfg_it; cfg_it != ep_cfgs.end();
       ++cfg_it, ep_num = USB::Endpoint::NextEPNumber(ep_num))
  {
    DEV_ASSERT(USB::Endpoint::EPNumberToInt8(ep_num) < ENDPOINT_COUNT);

    if (cfg_it->direction_hint == EPConfig::DirectionHint::BothDirections)
    {
      auto* ep_out = new ESP32USBEndpoint(*this, ep_num, USB::Endpoint::Direction::OUT,
                                          cfg_it->buffer);
      auto* ep_in = new ESP32USBEndpoint(*this, ep_num, USB::Endpoint::Direction::IN,
                                         cfg_it->buffer);
      endpoint_map_.out[USB::Endpoint::EPNumberToInt8(ep_num)] = ep_out;
      endpoint_map_.in[USB::Endpoint::EPNumberToInt8(ep_num)] = ep_in;
      Put(ep_out);
      Put(ep_in);
    }
    else
    {
      const auto direction = (cfg_it->direction_hint == EPConfig::DirectionHint::InOnly)
                                 ? USB::Endpoint::Direction::IN
                                 : USB::Endpoint::Direction::OUT;
      auto* ep = new ESP32USBEndpoint(*this, ep_num, direction, cfg_it->buffer);
      if (direction == USB::Endpoint::Direction::IN)
      {
        endpoint_map_.in[USB::Endpoint::EPNumberToInt8(ep_num)] = ep;
      }
      else
      {
        endpoint_map_.out[USB::Endpoint::EPNumberToInt8(ep_num)] = ep;
      }
      Put(ep);
    }
  }
}

void ESP32USBDevice::Init(bool in_isr)
{
  // 每次重新 Init 前先重置 FIFO 账本，避免复用旧的分配状态 / Reset FIFO bookkeeping
  // before each re-init so stale allocation state is not reused.
  ResetFifoState();
  USB::DeviceCore::Init(in_isr);
}

void ESP32USBDevice::Deinit(bool in_isr) { USB::DeviceCore::Deinit(in_isr); }

// 地址只在 SETUP status 之前写入硬件，保持 control-transfer 时序正确 / Write the device
// address only at the setup-before-status point to preserve control-transfer timing.
ErrorCode ESP32USBDevice::SetAddress(uint8_t address, USB::DeviceCore::Context context)
{
  if (context == USB::DeviceCore::Context::SETUP_BEFORE_STATUS)
  {
    auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
    dev->dcfg_reg.devaddr = address;
  }
  return ErrorCode::OK;
}

void ESP32USBDevice::Start(bool)
{
  // Start 是幂等的；PHY/IRQ/ROM 清理只做一次，core 寄存器则按当前状态重建 / Start is
  // idempotent: PHY/IRQ/ROM cleanup are one-time, while core registers are rebuilt for
  // the current state.
  if (runtime_.started)
  {
    return;
  }

  EnsureRomUsbCleaned();
  [[maybe_unused]] const auto ensure_phy_ready_result = EnsurePhyReady();
  REQUIRE(ensure_phy_ready_result);
  [[maybe_unused]] const auto ensure_interrupt_ready_result = EnsureInterruptReady();
  REQUIRE(ensure_interrupt_ready_result);

  InitializeCore();
  if (IsInited())
  {
    USB::DeviceCore::Deinit(false);
    USB::DeviceCore::Init(false);
  }

  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  dev->dctl_reg.sftdiscon = 0;
  dev->gahbcfg_reg.glbllntrmsk = 1;
  if (runtime_.intr_handle != nullptr)
  {
    esp_intr_enable(runtime_.intr_handle);
  }
  runtime_.started = true;
}

void ESP32USBDevice::Stop(bool)
{
  // Stop 只关中断和软断开，不回收一次性 runtime 资源 / Stop only masks interrupts and
  // soft-disconnects; it does not release one-time runtime resources.
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  dev->gahbcfg_reg.glbllntrmsk = 0;
  dev->gintmsk_reg.val = 0;
  if (runtime_.intr_handle != nullptr)
  {
    esp_intr_disable(runtime_.intr_handle);
  }
  dev->dctl_reg.sftdiscon = 1;
  runtime_.started = false;
}

void IRAM_ATTR ESP32USBDevice::IsrEntry(void* arg)
{
  auto* self = static_cast<ESP32USBDevice*>(arg);
  if (self != nullptr)
  {
    self->HandleInterrupt();
  }
}

bool ESP32USBDevice::EnsurePhyReady()
{
  // 内部 PHY 生命周期按整个设备实例保留，不做反复创建销毁 / Keep the internal PHY for the
  // lifetime of the device instance instead of repeatedly recreating it.
  if (runtime_.phy_ready)
  {
    return true;
  }

  usb_phy_config_t phy_conf = {
      .controller = USB_PHY_CTRL_OTG,
      .target = USB_PHY_TARGET_INT,
      .otg_mode = USB_OTG_MODE_DEVICE,
      .otg_speed = USB_PHY_SPEED_UNDEFINED,
      .ext_io_conf = nullptr,
      .otg_io_conf = nullptr,
  };

  usb_phy_handle_t handle = nullptr;
  if (usb_new_phy(&phy_conf, &handle) != ESP_OK)
  {
    return false;
  }

  runtime_.phy_handle = handle;
  runtime_.phy_ready = true;
  return true;
}

bool ESP32USBDevice::EnsureInterruptReady()
{
  // 中断句柄按实例生命周期保留，只在第一次 Start 前注册一次 / Keep the interrupt handle
  // for the instance lifetime and register it only once before the first start.
  if (runtime_.irq_ready)
  {
    return true;
  }

  if (esp_intr_alloc(ETS_USB_INTR_SOURCE, ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_INTRDISABLED,
                     IsrEntry, this, &runtime_.intr_handle) != ESP_OK)
  {
    runtime_.intr_handle = nullptr;
    return false;
  }

  runtime_.irq_ready = true;
  return true;
}

void ESP32USBDevice::EnsureRomUsbCleaned()
{
  // 先清掉 ROM 默认 USB/CDC ACM 状态，再接管 DWC2 device core / Tear down the
  // ROM-provided default USB/CDC ACM state before taking over the DWC2 device core.
  if (runtime_.rom_usb_cleaned)
  {
    return;
  }

  usb_dev_deinit();
  usb_dw_ctrl_deinit();
  uart_acm_dev = nullptr;
  runtime_.rom_usb_cleaned = true;
}

void ESP32USBDevice::InitializeCore()
{
  // 这里直接重建 DWC2 device-mode 寄存器基线：先停总中断，复位 core，再重建 FIFO/中断
  // mask / Rebuild the DWC2 device-mode register baseline directly: stop global
  // interrupts, reset the core, then rebuild FIFO and interrupt masks.
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);

  dev->gahbcfg_reg.glbllntrmsk = 0;

  while (!dev->grstctl_reg.ahbidle)
  {
  }

  dev->gusbcfg_reg.forcehstmode = 0;
  dev->gusbcfg_reg.forcedevmode = 1;
  dev->gusbcfg_reg.physel = 1;
  dev->gusbcfg_reg.usbtrdtim = 5;
  dev->gusbcfg_reg.toutcal = 7;
  dev->gusbcfg_reg.srpcap = 0;
  dev->gusbcfg_reg.hnpcap = 0;

  dev->grstctl_reg.csftrst = 1;
  while (dev->grstctl_reg.csftrst)
  {
  }
  while (!dev->grstctl_reg.ahbidle)
  {
  }

  dev->pcgcctl_reg.stoppclk = 0;
  dev->pcgcctl_reg.gatehclk = 0;
  dev->pcgcctl_reg.pwrclmp = 0;
  dev->pcgcctl_reg.rstpdwnmodule = 0;
  dev->gotgctl_reg.avalidoven = 0;
  dev->gotgctl_reg.bvalidoven = 1;
  dev->gotgctl_reg.bvalidovval = 1;

  ClearTxFifoRegisters();

  FlushFifos();
  ResetDeviceState();

  dev->gintsts_reg.val = 0xFFFFFFFFU;
  dev->gotgint_reg.val = 0xFFFFFFFFU;
  dev->gintmsk_reg.val = 0U;
  dev->gahbcfg_reg.hbstlen = ESPUSBDetail::DMA_BURST_INCR4;
  dev->gahbcfg_reg.dmaen = DmaEnabled() ? 1U : 0U;
  dev->gahbcfg_reg.nptxfemplvl = 1;
  dev->gahbcfg_reg.ptxfemplvl = 1;
  dev->dcfg_reg.descdma = 0;
  dev->dcfg_reg.nzstsouthshk = 1;
  dev->gintmsk_reg.usbrstmsk = 1;
  dev->gintmsk_reg.enumdonemsk = 1;
  dev->gintmsk_reg.usbsuspmsk = 1;
  dev->gintmsk_reg.wkupintmsk = 1;
  dev->gintmsk_reg.rxflvlmsk = DmaEnabled() ? 0U : 1U;
  dev->gintmsk_reg.oepintmsk = 1;
  dev->gintmsk_reg.iepintmsk = 1;
  dev->gintmsk_reg.otgintmsk = 1;
}

void ESP32USBDevice::ClearTxFifoRegisters()
{
  // 所有 TX FIFO 尺寸寄存器先清零，再按 endpoint 配置重新分配 / Clear all TX-FIFO size
  // registers first, then reallocate them per endpoint configuration.
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  dev->gnptxfsiz_reg.val = 0U;
  const size_t tx_fifo_reg_count =
      sizeof(dev->dieptxf_regs) / sizeof(dev->dieptxf_regs[0]);
  for (size_t i = 0U; i < tx_fifo_reg_count; ++i)
  {
    dev->dieptxf_regs[i].val = 0U;
  }
}

void ESP32USBDevice::FlushFifos()
{
  // 复位后先冲刷所有 FIFO，避免旧 payload / setup 残留 / Flush all FIFOs after reset so
  // no stale payload or setup bytes survive.
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  dev->grstctl_reg.txfnum = ESPUSBDetail::FLUSH_ALL_TX_FIFO;
  dev->grstctl_reg.txfflsh = 1;
  while (dev->grstctl_reg.txfflsh)
  {
  }
  dev->grstctl_reg.rxfflsh = 1;
  while (dev->grstctl_reg.rxfflsh)
  {
  }
}

void ESP32USBDevice::ResetFifoState()
{
  // FIFO 账本完全由当前硬件深度和 DMA 模式重新推导，不依赖旧运行态 / Recompute the FIFO
  // bookkeeping solely from the current hardware depth and DMA mode, without relying on
  // stale runtime state.
  fifo_state_.depth_words = ESPUSBDetail::GetHardwareFifoDepthWords();
  REQUIRE(fifo_state_.depth_words > 0U);
  fifo_state_.rx_words =
      ESPUSBDetail::CalcConfiguredRxFifoWords(64U, ENDPOINT_COUNT, DmaEnabled());
  fifo_state_.tx_next_words = fifo_state_.rx_words;
  std::memset(fifo_state_.tx_words, 0, sizeof(fifo_state_.tx_words));
  std::memset(fifo_state_.tx_bound, 0, sizeof(fifo_state_.tx_bound));
  fifo_state_.allocated_in = 0U;
}

void ESP32USBDevice::ResetDeviceState()
{
  // 总线复位后先清 device 级寄存器，再由 endpoint 层各自重建具体硬件配置 / After bus
  // reset, clear the device-level register state first; each endpoint later rebuilds its
  // own concrete hardware configuration.
  ResetFifoState();
  ResetControlState();

  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  dev->grxfsiz_reg.rxfdep = fifo_state_.rx_words;
  ClearTxFifoRegisters();
}

void ESP32USBDevice::ResetEndpointHardwareState()
{
  // endpoint 对象本身保留，但它们的硬件绑定状态必须全部失效 / Keep the endpoint objects,
  // but invalidate all of their hardware-bound state.
  for (uint8_t i = 0; i < ENDPOINT_COUNT; ++i)
  {
    if (endpoint_map_.in[i] != nullptr)
    {
      static_cast<ESP32USBEndpoint*>(endpoint_map_.in[i])->ResetHardwareState();
    }
    if (endpoint_map_.out[i] != nullptr)
    {
      static_cast<ESP32USBEndpoint*>(endpoint_map_.out[i])->ResetHardwareState();
    }
  }
}

void ESP32USBDevice::ReloadSetupPacketCount()
{
  // EP0 setup 接收窗口在 reset/setup-done 之后都要重新装填 / Reload the EP0 setup receive
  // window after reset and after each setup-done transition.
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  dev->doeptsiz0_reg.val = 0U;
  if (DmaEnabled())
  {
    dev->doeptsiz0_reg.xfersize = 3U * SETUP_PACKET_BYTES;
    dev->doeptsiz0_reg.pktcnt = 1U;
    dev->doeptsiz0_reg.supcnt = 3U;
    const auto cache_sync_dma_buffer_result =
        ESPUSBDetail::CacheSyncDmaBuffer(setup_packet_, SETUP_DMA_BUFFER_BYTES, false);
    REQUIRE(cache_sync_dma_buffer_result);
    dev->doepdma0_reg.dmaaddr =
        static_cast<uint32_t>(reinterpret_cast<uintptr_t>(setup_packet_));
    dev->doepctl0_reg.cnak = 1;
    dev->doepctl0_reg.epena = 1;
    if (endpoint_map_.out[0] != nullptr)
    {
      static_cast<ESP32USBEndpoint*>(endpoint_map_.out[0])->ep0_out_phase_ =
          ESP32USBEndpoint::Ep0OutPhase::SETUP;
    }
  }
  else
  {
    dev->doeptsiz0_reg.pktcnt = 1U;
    dev->doeptsiz0_reg.xfersize = 3U * SETUP_PACKET_BYTES;
    dev->doeptsiz0_reg.supcnt = 3U;
  }
}

void ESP32USBDevice::ResetControlState() { control_ = {}; }

// 只跟踪 setup 请求的 data-stage 方向，供 EP0 OUT/STATUS 判定复用 / Track only the
// setup-request data-stage direction for later EP0 OUT/STATUS decisions.
void ESP32USBDevice::UpdateSetupState(const uint8_t* setup)
{
  control_.setup_direction_out = (setup != nullptr) && ((setup[0] & 0x80U) == 0U);
}

void IRAM_ATTR ESP32USBDevice::HandleInterrupt()
{
  // 统一从 gintsts & gintmsk 取当前批次待处理中断，并在有限 guard 内排空 / Pull one
  // interrupt batch from gintsts & gintmsk and drain it under a bounded guard.
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  for (uint32_t guard = 0; guard < INTERRUPT_DISPATCH_GUARD; ++guard)
  {
    usb_dwc_gintsts_reg_t pending = {};
    pending.val = dev->gintsts_reg.val & dev->gintmsk_reg.val;

    if (pending.val == 0U)
    {
      break;
    }

    if (pending.usbrst)
    {
      dev->gintsts_reg.usbrst = 1;
      HandleBusReset(true);
    }

    if (pending.enumdone)
    {
      dev->gintsts_reg.enumdone = 1;
      const uint32_t enum_speed = dev->dsts_reg.enumspd;
      if (enum_speed != ESPUSBDetail::ENUM_SPEED_FULL_30_TO_60_MHZ &&
          enum_speed != ESPUSBDetail::ENUM_SPEED_FULL_48_MHZ)
      {
        REQUIRE_FROM_CALLBACK(false, true);
      }
    }

    if (pending.usbsusp)
    {
      dev->gintsts_reg.usbsusp = 1;
    }

    if (pending.wkupint)
    {
      dev->gintsts_reg.wkupint = 1;
    }

    if (!DmaEnabled() && pending.rxflvl)
    {
      dev->gintmsk_reg.rxflvlmsk = 0;
      while (dev->gintsts_reg.rxflvl)
      {
        HandleRxFifoLevel();
      }
      dev->gintmsk_reg.rxflvlmsk = 1;
    }

    if (pending.oepint)
    {
      HandleEndpointInterrupt(true, false);
    }

    if (pending.iepint)
    {
      HandleEndpointInterrupt(true, true);
    }

    if (pending.otgint)
    {
      const uint32_t otg_status = dev->gotgint_reg.val;
      dev->gotgint_reg.val = otg_status;
    }
  }
}

void ESP32USBDevice::HandleBusReset(bool in_isr)
{
  // 总线复位路径先停所有 endpoint，再清 mask/FIFO/状态，最后把 setup 接收窗口重新挂回去 /
  // The bus-reset path stops all endpoints first, then clears masks/FIFOs/state, and
  // finally rearms the setup receive window.
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);

  for (uint8_t i = 0; i < ENDPOINT_COUNT; ++i)
  {
    if (i == 0U)
    {
      ESPUSBDetail::DisableOutEndpointAndWait(dev->doepctl0_reg);
    }
    else
    {
      ESPUSBDetail::DisableOutEndpointAndWait(dev->out_eps[i - 1U].doepctl_reg);
    }
  }

  for (uint8_t i = 0; i < ENDPOINT_COUNT; ++i)
  {
    if (i == 0U)
    {
      if (dev->diepctl0_reg.epena)
      {
        dev->diepctl0_reg.snak = 1;
        dev->diepctl0_reg.epdis = 1;
      }
    }
    else if (dev->in_eps[i - 1U].diepctl_reg.epena)
    {
      dev->in_eps[i - 1U].diepctl_reg.snak = 1;
      dev->in_eps[i - 1U].diepctl_reg.epdis = 1;
    }
  }

  dev->daintmsk_reg.val = 0U;
  dev->doepmsk_reg.val = 0U;
  dev->diepmsk_reg.val = 0U;
  dev->diepempmsk_reg.val = 0U;
  dev->doepmsk_reg.setupmsk = 1;
  dev->doepmsk_reg.xfercomplmsk = 1;
  dev->doepmsk_reg.epdisbldmsk = 1;
  dev->doepmsk_reg.ahberrmsk = 1;
  dev->doepmsk_reg.outtknepdismsk = 1;
  dev->doepmsk_reg.stsphsercvdmsk = 1;
  dev->doepmsk_reg.outpkterrmsk = 1;
  dev->doepmsk_reg.bnaoutintrmsk = 1;
  dev->doepmsk_reg.bbleerrmsk = 1;
  dev->doepmsk_reg.nakmsk = 1;
  dev->doepmsk_reg.nyetmsk = 1;
  dev->diepmsk_reg.xfercomplmsk = 1;
  dev->diepmsk_reg.timeoutmsk = 1;

  FlushFifos();
  ResetDeviceState();
  ResetEndpointHardwareState();

  dev->dcfg_reg.devaddr = 0U;

  if (IsInited())
  {
    Deinit(in_isr);
    Init(in_isr);
  }

  ReloadSetupPacketCount();
}

void ESP32USBDevice::HandleEndpointInterrupt(bool in_isr, bool in_dir)
{
  // daint 里只保留当前方向且已使能的 endpoint 位，然后逐个转发给 endpoint 对象 / Keep
  // only enabled endpoint bits for the current direction from daint, then forward them
  // one by one to the endpoint objects.
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  const uint32_t daint = dev->daint_reg.val & dev->daintmsk_reg.val;
  const uint32_t bits = in_dir ? (daint & 0xFFFFU) : ((daint >> 16U) & 0xFFFFU);

  for (uint8_t ep_num = 0; ep_num < ENDPOINT_COUNT; ++ep_num)
  {
    if ((bits & (1UL << ep_num)) == 0U)
    {
      continue;
    }

    USB::Endpoint* ep = in_dir ? endpoint_map_.in[ep_num] : endpoint_map_.out[ep_num];
    if (ep == nullptr)
    {
      continue;
    }

    auto* esp_ep = static_cast<ESP32USBEndpoint*>(ep);
    if (in_dir)
    {
      esp_ep->HandleInInterrupt(in_isr);
    }
    else
    {
      esp_ep->HandleOutInterrupt(in_isr);
    }
  }
}

void ESP32USBDevice::HandleRxFifoLevel()
{
  // 非 DMA 模式下直接消费 grxstsp；DMA 模式下只把状态转发给 OUT endpoint / In non-DMA
  // mode, consume grxstsp directly; in DMA mode, only forward the status to the OUT
  // endpoint.
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  usb_dwc_grxstsp_reg_t status = {};
  status.val = dev->grxstsp_reg.val;
  const uint8_t ep_num = static_cast<uint8_t>(status.chnum);

  if (DmaEnabled())
  {
    if ((ep_num < ENDPOINT_COUNT) && (endpoint_map_.out[ep_num] != nullptr))
    {
      static_cast<ESP32USBEndpoint*>(endpoint_map_.out[ep_num])
          ->ObserveDmaRxStatus(status.pktsts, status.bcnt);
    }
    return;
  }

  switch (status.pktsts)
  {
    case ESPUSBDetail::RX_STATUS_GLOBAL_OUT_NAK:
      break;

    case ESPUSBDetail::RX_STATUS_SETUP_DATA:
      ESPUSBDetail::ReadFifoPacket(ESPUSBDetail::GetEndpointFifo(dev, 0), setup_packet_,
                                   sizeof(setup_packet_));
      UpdateSetupState(setup_packet_);
      break;

    case ESPUSBDetail::RX_STATUS_SETUP_DONE:
      ReloadSetupPacketCount();
      break;

    case ESPUSBDetail::RX_STATUS_DATA:
    {
      USB::Endpoint* ep = (ep_num < ENDPOINT_COUNT) ? endpoint_map_.out[ep_num] : nullptr;
      if (ep != nullptr)
      {
        static_cast<ESP32USBEndpoint*>(ep)->HandleRxData(status.bcnt);
      }
      else
      {
        uint8_t sink[64] = {};
        ESPUSBDetail::ReadFifoPacket(ESPUSBDetail::GetEndpointFifo(dev, 0), sink,
                                     status.bcnt);
      }
      break;
    }

    case ESPUSBDetail::RX_STATUS_TRANSFER_COMPLETE:
    default:
      break;
  }
}

bool ESP32USBDevice::AllocateTxFifo(uint8_t ep_num, uint16_t packet_size, bool is_bulk,
                                    uint16_t& fifo_words)
{
  // TX FIFO 分配保持“前缀连续”布局；若中间某级无法满足，就整体拒绝，避免留下稀疏映射 /
  // Keep TX-FIFO allocation prefix-contiguous; if any intermediate level cannot be
  // satisfied, reject the whole request to avoid sparse mappings.
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  const uint8_t max_in_ep_num =
      static_cast<uint8_t>(sizeof(dev->dieptxf_regs) / sizeof(dev->dieptxf_regs[0]));

  (void)is_bulk;

  if (ep_num > max_in_ep_num)
  {
    return false;
  }

  const uint16_t words = ESPUSBDetail::CalcTxFifoWords(packet_size, DmaEnabled());
  if (!fifo_state_.tx_bound[ep_num] && (fifo_state_.allocated_in >= IN_ENDPOINT_LIMIT))
  {
    return false;
  }

  for (uint8_t index = 0U; index <= ep_num; ++index)
  {
    const uint16_t requested_words =
        (index == ep_num) ? words : ESPUSBDetail::ESP32_SX_FS_MIN_TX_FIFO_WORDS;
    if (fifo_state_.tx_words[index] != 0U)
    {
      if (fifo_state_.tx_words[index] < requested_words)
      {
        return false;
      }
      continue;
    }

    const uint16_t next_words =
        static_cast<uint16_t>(fifo_state_.tx_next_words + requested_words);
    if (next_words > fifo_state_.depth_words)
    {
      return false;
    }

    const uint32_t tx_fifo_reg =
        ESPUSBDetail::PackTxFifoSizeReg(fifo_state_.tx_next_words, requested_words);
    if (index == 0U)
    {
      dev->gnptxfsiz_reg.val = tx_fifo_reg;
    }
    else
    {
      dev->dieptxf_regs[index - 1U].val = tx_fifo_reg;
    }

    fifo_state_.tx_words[index] = requested_words;
    fifo_state_.tx_next_words = next_words;
  }

  if (!fifo_state_.tx_bound[ep_num])
  {
    fifo_state_.tx_bound[ep_num] = true;
    fifo_state_.allocated_in++;
  }
  fifo_words = words;
  return true;
}

bool ESP32USBDevice::EnsureRxFifo(uint16_t packet_size)
{
  // RX FIFO 只允许单调扩张，不做在线收缩，避免影响已经配置的 OUT endpoint / RX FIFO is
  // allowed to grow monotonically only; it is not shrunk online, so already configured
  // OUT endpoints are not disturbed.
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  const uint16_t needed =
      ESPUSBDetail::CalcConfiguredRxFifoWords(packet_size, ENDPOINT_COUNT, DmaEnabled());
  if (needed <= fifo_state_.rx_words)
  {
    return true;
  }

  if (fifo_state_.tx_next_words != fifo_state_.rx_words)
  {
    return false;
  }

  if (needed > fifo_state_.depth_words)
  {
    return false;
  }

  fifo_state_.rx_words = needed;
  fifo_state_.tx_next_words = needed;
  dev->grxfsiz_reg.rxfdep = fifo_state_.rx_words;
  return true;
}

}  // namespace LibXR

#endif
