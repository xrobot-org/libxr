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
#include "esp_rom_sys.h"
#include "esp_usb.hpp"
#include "esp_usb_ep.hpp"
#include "soc/interrupts.h"

namespace LibXR
{

namespace
{
// GINTSTS 的标志位。这些标志写 1 清除，清除时整字写入只含目标位的值：对位域成员赋值会
// 读出整个寄存器再写回，把当时已置位的其他标志一起清掉。
// Flag bits of GINTSTS. They are cleared by writing 1, with a whole-word write that holds
// only the wanted bit: assigning a bit-field member reads the register and writes it
// back, which clears every other flag that is set at that moment.
constexpr uint32_t GINTSTS_SOF = 1UL << 3U;
// GINTMSK 里的屏蔽位在同一位 / The mask bit in GINTMSK is the same bit
constexpr uint32_t GINTSTS_GOUTNAKEFF = 1UL << 7U;
constexpr uint32_t GINTSTS_USBSUSP = 1UL << 11U;
constexpr uint32_t GINTSTS_USBRST = 1UL << 12U;
constexpr uint32_t GINTSTS_ENUMDONE = 1UL << 13U;
constexpr uint32_t GINTSTS_INCOMPISOIN = 1UL << 20U;
constexpr uint32_t GINTSTS_WKUPINT = 1UL << 31U;
}  // namespace

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
      USB::DeviceCore<USB::FullSpeedCapabilities>(*this, USB::USBSpec::USB_2_1,
                                                  USB::Speed::FULL, packet_size, vid, pid,
                                                  bcd, lang_list, configs, uid),
      reset_wait_(ResetCheck, ResetDone, this)
{
  // USB 中断不驻留 IRAM（见 EnsureInterruptReady），flash cache 关闭期间被屏蔽；
  // 守卫只在任务或该中断里、cache 开启时运行，不需要放进 IRAM。
  // The USB interrupt is not IRAM-resident (see EnsureInterruptReady) and is masked
  // while the flash cache is off; the guard runs in task or interrupt context with the
  // cache on, so it does not need IRAM.
  SetHardwareGuard(
      this,
      [](void* context) -> uintptr_t
      {
        auto* self = static_cast<ESP32USBDevice*>(context);
        portENTER_CRITICAL_SAFE(&self->hardware_mux_);
        return 0;
      },
      [](void* context, uintptr_t)
      {
        auto* self = static_cast<ESP32USBDevice*>(context);
        portEXIT_CRITICAL_SAFE(&self->hardware_mux_);
      });
  ResetFifoState();
  REQUIRE(ep_cfgs.size() > 0U && ep_cfgs.size() <= ENDPOINT_COUNT);

  auto cfg_it = ep_cfgs.begin();

  // EP0 的 OUT 与 IN 各占缓冲的一半，各自能放下一个 EP0 包。
  // EP0 OUT and IN each take half of the buffer, and each half holds one EP0 packet.
  const size_t ep0_half = cfg_it->buffer.size_ / 2U;
  REQUIRE(ep0_half >= static_cast<size_t>(packet_size) && ep0_half % 4U == 0U);
  auto* ep0_out = new ESP32USBEndpoint(*this, USB::Endpoint::EPNumber::EP0,
                                       USB::Endpoint::Direction::OUT,
                                       {cfg_it->buffer.addr_, ep0_half});
  auto* ep0_in = new ESP32USBEndpoint(
      *this, USB::Endpoint::EPNumber::EP0, USB::Endpoint::Direction::IN,
      {static_cast<uint8_t*>(cfg_it->buffer.addr_) + ep0_half, ep0_half});

  SetEndpoint0(ep0_in, ep0_out);
  endpoint_map_.in[0] = ep0_in;
  endpoint_map_.out[0] = ep0_out;

  auto ep_num = USB::Endpoint::EPNumber::EP1;
  for (++cfg_it; cfg_it != ep_cfgs.end();
       ++cfg_it, ep_num = USB::Endpoint::NextEPNumber(ep_num))
  {
    // 本控制器只有 EP0 + 4 个 IN 端点（IN 端点号 1..4），能发 IN 的配置项端点号超过它，
    // 会在主机 SET_CONFIGURATION 时才配置失败（主机侧表现为反复枚举）；构造期就拒绝。
    // The controller has EP0 plus four IN endpoints (IN endpoint numbers 1..4). An
    // IN-capable entry above that only fails when the host sends SET_CONFIGURATION (the
    // host then re-enumerates in a loop), so reject it at construction.
    constexpr uint8_t MAX_IN_EP_NUM = static_cast<uint8_t>(
        sizeof(static_cast<usb_dwc_dev_t*>(nullptr)->dieptxf_regs) /
        sizeof(static_cast<usb_dwc_dev_t*>(nullptr)->dieptxf_regs[0]));
    REQUIRE(cfg_it->direction_hint == EPConfig::DirectionHint::OutOnly ||
            USB::Endpoint::EPNumberToInt8(ep_num) <= MAX_IN_EP_NUM);

    if (cfg_it->direction_hint == EPConfig::DirectionHint::BothDirections)
    {
      const size_t half = cfg_it->buffer.size_ / 2U;
      REQUIRE(half >= 8U && (half % 4U) == 0U);
      auto* ep_out = new ESP32USBEndpoint(*this, ep_num, USB::Endpoint::Direction::OUT,
                                          {cfg_it->buffer.addr_, half});
      auto* ep_in = new ESP32USBEndpoint(
          *this, ep_num, USB::Endpoint::Direction::IN,
          {static_cast<uint8_t*>(cfg_it->buffer.addr_) + half, half});
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
  USB::DeviceCore<USB::FullSpeedCapabilities>::Init(in_isr);
}

void ESP32USBDevice::Deinit(bool in_isr)
{
  USB::DeviceCore<USB::FullSpeedCapabilities>::Deinit(in_isr);
}

// 地址只在 SETUP status 之前写入硬件，保持 control-transfer 时序正确 / Write the device
// address only at the setup-before-status point to preserve control-transfer timing.
ErrorCode ESP32USBDevice::SetAddress(uint8_t address, USB::ControlContext context)
{
  if (context == USB::ControlContext::SETUP_BEFORE_STATUS)
  {
    EndpointPool::HardwareScope hardware(this);
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
    USB::DeviceCore<USB::FullSpeedCapabilities>::Deinit(false);
    USB::DeviceCore<USB::FullSpeedCapabilities>::Init(false);
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

void ESP32USBDevice::Stop(bool in_isr)
{
  // Stop 只关中断和软断开，不回收一次性 runtime 资源 / Stop only masks interrupts and
  // soft-disconnects; it does not release one-time runtime resources.
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  {
    // 复位链正在进行时到此为止：它的下一步看到 IDLE 就不再继续，也不会重连。
    // A running reset chain ends here: its next step sees IDLE, goes no further and does
    // not reconnect.
    EndpointPool::HardwareScope hardware(this);
    reset_stage_ = ResetStage::IDLE;
    dev->gahbcfg_reg.glbllntrmsk = 0;
    dev->gintmsk_reg.val = 0;
    dev->dctl_reg.sftdiscon = 1;
  }
  if (runtime_.intr_handle != nullptr)
  {
    esp_intr_disable(runtime_.intr_handle);
  }
  runtime_.started = false;
  // 离开总线、关掉中断之后不再有事件到达端点；确认所有等待中的停止。Start() 会重建
  // 控制器寄存器和 FIFO 记账。
  // Off the bus and with its interrupt off nothing reaches an endpoint any more; confirm
  // every waiting stop. Start() rebuilds the controller registers and the FIFO
  // bookkeeping.
  ForceStopAll(in_isr);
}

void ESP32USBDevice::IsrEntry(void* arg)
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

  // 不带 ESP_INTR_FLAG_IRAM：处理链会进入核心和类代码（在 flash 里），flash 缓存被禁用
  // 期间（写 flash）中断只是被推迟，而不是在缓存关闭时去取 flash 里的指令而崩溃。
  // Without ESP_INTR_FLAG_IRAM: the handler chain reaches the core and class code (in
  // flash), so while the flash cache is disabled (flash writes) the interrupt is only
  // deferred instead of fetching flash instructions with the cache off and crashing.
  if (esp_intr_alloc(ETS_USB_INTR_SOURCE, ESP_INTR_FLAG_INTRDISABLED, IsrEntry, this,
                     &runtime_.intr_handle) != ESP_OK)
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
  REQUIRE(ResetCoreRegisters());
  REQUIRE(RebuildCoreRegisters());
}

bool ESP32USBDevice::ResetCoreRegisters()
{
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);

  dev->gahbcfg_reg.glbllntrmsk = 0;

  if (!ESPUSBDetail::SpinUntil([dev]() { return dev->grstctl_reg.ahbidle != 0U; }))
  {
    return false;
  }

  BeginCoreReset();
  return ESPUSBDetail::SpinUntil([dev]() { return dev->grstctl_reg.csftrst == 0U; }) &&
         ESPUSBDetail::SpinUntil([dev]() { return dev->grstctl_reg.ahbidle != 0U; });
}

void ESP32USBDevice::BeginCoreReset()
{
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  dev->gusbcfg_reg.forcehstmode = 0;
  dev->gusbcfg_reg.forcedevmode = 1;
  dev->gusbcfg_reg.physel = 1;
  dev->gusbcfg_reg.usbtrdtim = 5;
  dev->gusbcfg_reg.toutcal = 7;
  dev->gusbcfg_reg.srpcap = 0;
  dev->gusbcfg_reg.hnpcap = 0;

  dev->grstctl_reg.csftrst = 1;
}

bool ESP32USBDevice::RebuildCoreRegisters()
{
  BeginBaseline();
  const bool flushed = FlushFifos();
  FinishBaseline();
  return flushed;
}

void ESP32USBDevice::BeginBaseline()
{
  reset_enum_pending_ = false;
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);

  dev->pcgcctl_reg.stoppclk = 0;
  dev->pcgcctl_reg.gatehclk = 0;
  dev->pcgcctl_reg.pwrclmp = 0;
  dev->pcgcctl_reg.rstpdwnmodule = 0;
  dev->gotgctl_reg.avalidoven = 0;
  dev->gotgctl_reg.bvalidoven = 1;
  dev->gotgctl_reg.bvalidovval = 1;

  ClearTxFifoRegisters();
}

void ESP32USBDevice::FinishBaseline()
{
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);

  ResetDeviceState();

  dev->gintsts_reg.val = 0xFFFFFFFFU;
  dev->gotgint_reg.val = 0xFFFFFFFFU;
  dev->gintmsk_reg.val = 0U;
  dev->gahbcfg_reg.hbstlen = ESPUSBDetail::DMA_BURST_INCR4;
  dev->gahbcfg_reg.dmaen = 1;
  dev->dcfg_reg.descdma = 0;
  dev->dcfg_reg.nzstsouthshk = 1;
  dev->gintmsk_reg.usbrstmsk = 1;
  dev->gintmsk_reg.enumdonemsk = 1;
  dev->gintmsk_reg.usbsuspmsk = 1;
  dev->gintmsk_reg.wkupintmsk = 1;
  dev->gintmsk_reg.oepintmsk = 1;
  dev->gintmsk_reg.iepintmsk = 1;
  dev->gintmsk_reg.incompisoinmsk = 1;
  dev->gintmsk_reg.otgintmsk = 1;
  dev->gintmsk_reg.sofmsk = 1;
}

void ESP32USBDevice::OnStopTimeout(bool in_isr)
{
  // 停止没有等到硬件确认：控制器状态未知（DMA 仍可能写槽）。离开总线、确认所有等待中的
  // 停止，然后在毫秒节拍上分阶段复位控制器并重连，让主机重新枚举。复位链已经在进行时
  // 不再重新发起，它结束时会重连。
  // The stop never confirmed and the controller is in an unknown state (the DMA may still
  // be writing a slot). Leave the bus, confirm every waiting stop, then reset the
  // controller in stages on the millisecond tick and reconnect so that the host
  // enumerates again. A reset chain that is already running is not started again; it
  // reconnects when it ends.
  {
    EndpointPool::HardwareScope hardware(this);
    auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
    dev->gahbcfg_reg.glbllntrmsk = 0;
    dev->gintmsk_reg.val = 0U;
    dev->dctl_reg.sftdiscon = 1;
  }
  ForceStopAll(in_isr);
  {
    EndpointPool::HardwareScope hardware(this);
    if (reset_stage_ == ResetStage::IDLE) ArmResetStage(ResetStage::AHB_IDLE);
  }
  OnDisconnect(in_isr);
}

void ESP32USBDevice::ArmResetStage(ResetStage stage)
{
  reset_stage_ = stage;
  if (reset_wait_.Start(0U, RESET_STEP_TIMEOUT_MS) != ErrorCode::OK)
  {
    // 等待排不上：保持断开，主机看到设备消失，等应用重新 Start()。
    // The wait could not be armed: stay disconnected, the host sees the device disappear,
    // until the application calls Start() again.
    reset_stage_ = ResetStage::IDLE;
    runtime_.started = false;
  }
}

bool ESP32USBDevice::ResetCheck(void* arg)
{
  auto* self = static_cast<ESP32USBDevice*>(arg);
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  switch (self->reset_stage_)
  {
    case ResetStage::AHB_IDLE:
    case ResetStage::RESET_IDLE:
      return dev->grstctl_reg.ahbidle != 0U;
    case ResetStage::CORE_RESET:
      return dev->grstctl_reg.csftrst == 0U;
    case ResetStage::TX_FLUSH:
      return dev->grstctl_reg.txfflsh == 0U;
    case ResetStage::RX_FLUSH:
      return dev->grstctl_reg.rxfflsh == 0U;
    case ResetStage::IDLE:
      break;
  }
  return true;
}

void ESP32USBDevice::ResetDone(void* arg, ErrorCode result)
{
  auto* self = static_cast<ESP32USBDevice*>(arg);
  EndpointPool::HardwareScope hardware(self);
  const ResetStage stage = self->reset_stage_;
  self->reset_stage_ = ResetStage::IDLE;
  if (stage == ResetStage::IDLE)
  {
    // Stop() 已经结束了这条链。
    // Stop() has ended this chain.
    return;
  }
  if (result != ErrorCode::OK)
  {
    // 一个自清位在限定时间内没有清：控制器不可信，留在总线外，等应用重新 Start()。
    // A self-clearing bit did not clear within its time: the controller cannot be
    // trusted, stay off the bus until the application calls Start() again.
    self->runtime_.started = false;
    return;
  }

  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  switch (stage)
  {
    case ResetStage::AHB_IDLE:
      self->BeginCoreReset();
      self->ArmResetStage(ResetStage::CORE_RESET);
      break;
    case ResetStage::CORE_RESET:
      self->ArmResetStage(ResetStage::RESET_IDLE);
      break;
    case ResetStage::RESET_IDLE:
      self->BeginBaseline();
      self->StartTxFlushAll();
      self->ArmResetStage(ResetStage::TX_FLUSH);
      break;
    case ResetStage::TX_FLUSH:
      self->StartRxFlush();
      self->ArmResetStage(ResetStage::RX_FLUSH);
      break;
    case ResetStage::RX_FLUSH:
      // 基线重建完成后重连并重新打开全局中断：主机重新枚举，端点在下一次 Configure 里
      // 重新分配 FIFO（ForceStop 已清掉记账）。
      // With the baseline rebuilt, reconnect and turn the global interrupt on again: the
      // host enumerates again and the endpoints allocate their FIFOs at the next
      // Configure (ForceStop cleared the bookkeeping).
      self->FinishBaseline();
      dev->dctl_reg.sftdiscon = 0;
      dev->gahbcfg_reg.glbllntrmsk = 1;
      self->runtime_.started = true;
      break;
    case ResetStage::IDLE:
      break;
  }
}

void ESP32USBDevice::AcquireGlobalOutNak()
{
  if (global_out_nak_users_++ != 0U) return;
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  dev->gintmsk_reg.val |= GINTSTS_GOUTNAKEFF;
  dev->dctl_reg.sgoutnak = 1;
}

void ESP32USBDevice::ReleaseGlobalOutNak()
{
  if (global_out_nak_users_ == 0U || --global_out_nak_users_ != 0U) return;
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  dev->gintmsk_reg.val &= ~GINTSTS_GOUTNAKEFF;
  dev->dctl_reg.cgoutnak = 1;
}

void ESP32USBDevice::ForceStopAll(bool in_isr)
{
  global_out_nak_users_ = 0U;
  for (size_t i = 0U; i < ESP32USBDevice::ENDPOINT_COUNT; ++i)
  {
    if (endpoint_map_.in[i] != nullptr)
    {
      static_cast<ESP32USBEndpoint*>(endpoint_map_.in[i])->ForceStop(in_isr);
    }
    if (endpoint_map_.out[i] != nullptr)
    {
      static_cast<ESP32USBEndpoint*>(endpoint_map_.out[i])->ForceStop(in_isr);
    }
  }
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

bool ESP32USBDevice::FlushFifos()
{
  // 复位后先冲刷所有 FIFO，避免旧 payload / setup 残留 / Flush all FIFOs after reset so
  // no stale payload or setup bytes survive.
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  StartTxFlushAll();
  const bool tx_done =
      ESPUSBDetail::SpinUntil([dev]() { return dev->grstctl_reg.txfflsh == 0U; });
  StartRxFlush();
  const bool rx_done =
      ESPUSBDetail::SpinUntil([dev]() { return dev->grstctl_reg.rxfflsh == 0U; });
  return tx_done && rx_done;
}

void ESP32USBDevice::StartTxFlushAll()
{
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  dev->grstctl_reg.txfnum = ESPUSBDetail::FLUSH_ALL_TX_FIFO;
  dev->grstctl_reg.txfflsh = 1;
}

void ESP32USBDevice::StartRxFlush()
{
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  dev->grstctl_reg.rxfflsh = 1;
}

void ESP32USBDevice::ResetFifoState()
{
  // FIFO 账本完全由当前硬件深度重新推导，不依赖旧运行态 / Recompute the FIFO bookkeeping
  // solely from the current hardware depth, without relying on stale runtime state.
  fifo_state_.depth_words = ESPUSBDetail::GetHardwareFifoDepthWords();
  fifo_state_.rx_words = ESPUSBDetail::CalcConfiguredRxFifoWords(64U, ENDPOINT_COUNT);
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
  global_out_nak_users_ = 0U;
  // Init() 在 Start() 打开内核时钟之前也会重置账本，此时读到的深度为 0；
  // 这里内核已经上电。
  // Init() also resets the bookkeeping before Start() clocks the core, when the depth
  // reads as 0; here the core is powered.
  DEV_ASSERT(fifo_state_.depth_words > 0U);
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

void ESP32USBDevice::ReloadSetupPacketCount(bool clear_nak)
{
  // EP0 setup 接收窗口在 reset/setup-done 之后都要重新装填 / Reload the EP0 setup receive
  // window after reset and after each setup-done transition.
  // DMA 模式下核心只在 EP0 OUT 使能时把 SETUP 写进内存，而 SETUP 阶段结束、OUT 传输完成和
  // 主机进入状态阶段都会让核心关掉它：每次都要重新使能。后面还有 OUT 数据要收时保持 NAK，
  // 数据阶段开始时再清。
  // With DMA the core stores a SETUP only while EP0 OUT is enabled, and the end of a
  // SETUP stage, a completed OUT transfer and the host entering the status stage all make
  // the core disable it: it is enabled again every time. While OUT data is still to come
  // the NAK is kept, and the data stage clears it when it starts.
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  dev->doeptsiz0_reg.val = 0U;
  dev->doeptsiz0_reg.xfersize = 3U * SETUP_PACKET_BYTES;
  dev->doeptsiz0_reg.pktcnt = 1U;
  dev->doeptsiz0_reg.supcnt = 3U;
  const auto cache_sync_dma_buffer_result =
      ESPUSBDetail::CacheSyncDmaBuffer(setup_packet_, SETUP_DMA_BUFFER_BYTES, false);
  REQUIRE(cache_sync_dma_buffer_result);
  dev->doepdma0_reg.dmaaddr =
      static_cast<uint32_t>(reinterpret_cast<uintptr_t>(setup_packet_));
  if (clear_nak)
  {
    dev->doepctl0_reg.cnak = 1;
  }
  else
  {
    dev->doepctl0_reg.snak = 1;
  }
  dev->doepctl0_reg.epena = 1;
  if (endpoint_map_.out[0] != nullptr)
  {
    static_cast<ESP32USBEndpoint*>(endpoint_map_.out[0])->ep0_out_phase_ =
        ESP32USBEndpoint::Ep0OutPhase::SETUP;
  }
}

void ESP32USBDevice::ResetControlState() { control_ = {}; }

// 只跟踪 setup 请求的 data-stage 方向，供 EP0 OUT/STATUS 判定复用 / Track only the
// setup-request data-stage direction for later EP0 OUT/STATUS decisions.
void ESP32USBDevice::UpdateSetupState(const uint8_t* setup)
{
  control_.setup_direction_out = (setup != nullptr) && ((setup[0] & 0x80U) == 0U);
}

void ESP32USBDevice::HandleInterrupt()
{
  // InterruptScope 先构造、后析构：它析构时才执行延后的类服务（类服务的时长由类决定，
  // 不能在守卫内运行），那时守卫已经释放。
  // The InterruptScope is constructed first and destroyed last: the deferred class
  // service (its duration belongs to the class, so it must not run under the guard) only
  // runs when it is destroyed, by which time the guard has been released.
  USB::EndpointPool::InterruptScope interrupt_scope(*this);
  // 中断也持有硬件守卫：任务侧的 HardwareScope 在另一个核上时只靠本核关中断挡不住
  // 中断，两边会交错访问寄存器。守卫可重入，中断里的回调再进 HardwareScope 没问题。
  // The interrupt holds the hardware guard too: when a task-side HardwareScope runs on
  // the other core, masking interrupts on its own core does not keep this interrupt out,
  // and both sides would interleave register accesses. The guard is re-entrant, so a
  // callback that enters a HardwareScope inside the interrupt is fine.
  struct IsrGuard
  {
    portMUX_TYPE* mux;
    explicit IsrGuard(portMUX_TYPE* m) : mux(m) { portENTER_CRITICAL_SAFE(mux); }
    ~IsrGuard() { portEXIT_CRITICAL_SAFE(mux); }
  } isr_guard(&hardware_mux_);
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

    if (pending.sof)
    {
      dev->gintsts_reg.val = GINTSTS_SOF;
      OnSof(true, static_cast<uint16_t>((dev->dsts_reg.val >> 8U) & 0x7ffU));
    }
    if (pending.usbrst)
    {
      dev->gintsts_reg.val = GINTSTS_USBRST;
      HandleBusReset(true);
      continue;
    }

    if (pending.enumdone)
    {
      dev->gintsts_reg.val = GINTSTS_ENUMDONE;
      reset_enum_pending_ = false;
      const uint32_t enum_speed = dev->dsts_reg.enumspd;
      if (enum_speed != ESPUSBDetail::ENUM_SPEED_FULL_30_TO_60_MHZ &&
          enum_speed != ESPUSBDetail::ENUM_SPEED_FULL_48_MHZ)
      {
        REQUIRE_FROM_CALLBACK(false, true);
      }
    }

    if (pending.usbsusp)
    {
      dev->gintsts_reg.val = GINTSTS_USBSUSP;
      if (reset_enum_pending_)
      {
        // ESP32-S3 板上实测：总线复位的 SE0 期间核心会在 ENUMDONE 之前报 USBSUSP，随后
        // 线路一直是 SE0，设备不再回到总线；在复位窗口内软断开再连接可恢复枚举。
        // Measured on an ESP32-S3: during the SE0 of a bus reset the core can raise
        // USBSUSP before ENUMDONE, after which the line stays SE0 and the device does not
        // come back; a soft disconnect and connect inside the reset restores enumeration.
        RecoverFalseSuspend();
        continue;
      }
      dev->gintmsk_reg.sofmsk = 0;
      OnSuspend(true);
    }

    if (pending.wkupint)
    {
      dev->gintsts_reg.val = GINTSTS_WKUPINT;
      dev->gintmsk_reg.sofmsk = 1;
      OnResume(true);
    }

    if (pending.oepint)
    {
      HandleEndpointInterrupt(true, false);
    }

    if (pending.goutnakeff)
    {
      // 全局 OUT NAK 期间这个标志一直为 1，通知一次就够：先关掉它的中断，再让等它的
      // OUT 端点往下走。
      // The flag stays set while the global OUT NAK is on and one notification is enough:
      // mask it, then let the OUT endpoints that wait for it go on.
      dev->gintmsk_reg.val &= ~GINTSTS_GOUTNAKEFF;
      for (uint8_t i = 1; i < ENDPOINT_COUNT; ++i)
      {
        if (endpoint_map_.out[i] != nullptr)
        {
          (void)static_cast<ESP32USBEndpoint*>(endpoint_map_.out[i])
              ->AdvanceStop(true, 0U);
        }
      }
    }

    if (pending.iepint)
    {
      HandleEndpointInterrupt(true, true);
    }

    if (pending.incompisoin)
    {
      // 帧号在清标志之前读一次。ESP32-S3 在新一帧的 SOF 到达时才报这个中断，此时 DSTS
      // 里已经是新一帧的帧号（板上实测：中断总是在同一帧的 SOF 之后几微秒到达，读到的
      // 帧号等于那个 SOF 的帧号），刚结束的是前一帧，所以结束帧的奇偶与读到的相反。
      // The frame number is read once, before the flag is cleared. The ESP32-S3 raises
      // this interrupt when the SOF of the new frame arrives, and DSTS already holds the
      // number of the new frame (measured on the board: the interrupt always comes a few
      // microseconds after the SOF of the same frame, and the number read equals that
      // SOF's number). The frame that ended is the previous one, so its parity is the
      // opposite of the one read.
      const bool ended_odd = (dev->dsts_reg.soffn & 1U) == 0U;
      dev->gintsts_reg.val = GINTSTS_INCOMPISOIN;
      for (uint8_t i = 1; i < ENDPOINT_COUNT; ++i)
      {
        if (endpoint_map_.in[i] != nullptr)
        {
          static_cast<ESP32USBEndpoint*>(endpoint_map_.in[i])->OnIsoIncomplete(ended_odd);
        }
      }
    }

    if (pending.otgint)
    {
      const uint32_t otg_status = dev->gotgint_reg.val;
      dev->gotgint_reg.val = otg_status;
    }
  }
}

void ESP32USBDevice::RecoverFalseSuspend()
{
  // 软断开 20 us 再连接，在主机的复位窗口内完成（20 us 是板上试出的取值）。
  // A soft disconnect of 20 us, then connect, inside the host's reset window (20 us is
  // the value tried on the board).
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  reset_enum_pending_ = false;
  dev->dctl_reg.sftdiscon = 1;
  esp_rom_delay_us(20);
  dev->dctl_reg.sftdiscon = 0;
}

void ESP32USBDevice::HandleBusReset(bool in_isr)
{
  // 总线复位路径先停所有 endpoint，再清 mask/FIFO/状态，最后把 setup 接收窗口重新挂回去 /
  // The bus-reset path stops all endpoints first, then clears masks/FIFOs/state, and
  // finally rearms the setup receive window.
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);

  // 复位时硬件要求所有 OUT 端点 NAK；IN 端点请求禁用。这里只发请求，随后做一次有界
  // 确认：硬件在复位期间没有总线事务，使能位应在几个周期内清掉（EP0 OUT 不允许写
  // EPDIS，也不参与确认）。没清掉说明控制器状态不可信，走控制器复位升级。
  // On reset the hardware requires every OUT endpoint to NAK, and IN endpoints get a
  // disable request. Only the requests are made here, followed by one bounded check:
  // there is no bus traffic during a reset, so the enable bits clear within a few
  // cycles (EP0 OUT must not get EPDIS and does not take part in the check). If they
  // do not clear the controller cannot be trusted and the controller-reset escalation
  // takes over.
  // 禁用 OUT 端点之前全局 OUT NAK 必须已经生效。本后端只用内部 DMA，核心自己处理接收
  // FIFO，GOUTNAKEFF 不需要应用去读 FIFO 就会置位，所以这里可以有界地等它。
  // The global OUT NAK must be in effect before an OUT endpoint is disabled. This backend
  // only uses the internal DMA, where the core handles the receive FIFO itself and
  // GOUTNAKEFF is set without the application reading the FIFO, so a bounded wait works.
  bool out_enabled = false;
  for (uint8_t i = 1; i < ENDPOINT_COUNT; ++i)
  {
    out_enabled = out_enabled || (dev->out_eps[i - 1U].doepctl_reg.epena != 0U);
  }
  if (out_enabled)
  {
    if (dev->gintsts_reg.goutnakeff == 0U)
    {
      dev->dctl_reg.sgoutnak = 1;
    }
    if (!ESPUSBDetail::SpinUntil([dev]() { return dev->gintsts_reg.goutnakeff != 0U; }))
    {
      OnStopTimeout(in_isr);
      return;
    }
  }
  ESPUSBDetail::RequestOutDisable(dev->doepctl0_reg, false);
  for (uint8_t i = 1; i < ENDPOINT_COUNT; ++i)
  {
    ESPUSBDetail::RequestOutDisable(dev->out_eps[i - 1U].doepctl_reg, true);
  }
  if (dev->diepctl0_reg.epena)
  {
    dev->diepctl0_reg.snak = 1;
    dev->diepctl0_reg.epdis = 1;
  }
  for (uint8_t i = 1; i < ENDPOINT_COUNT; ++i)
  {
    if (dev->in_eps[i - 1U].diepctl_reg.epena)
    {
      dev->in_eps[i - 1U].diepctl_reg.snak = 1;
      dev->in_eps[i - 1U].diepctl_reg.epdis = 1;
    }
  }
  const bool settled = ESPUSBDetail::SpinUntil(
      [dev]()
      {
        if (dev->diepctl0_reg.epena)
        {
          return false;
        }
        for (uint8_t i = 1; i < ENDPOINT_COUNT; ++i)
        {
          if (dev->in_eps[i - 1U].diepctl_reg.epena ||
              dev->out_eps[i - 1U].doepctl_reg.epena)
          {
            return false;
          }
        }
        return true;
      });
  if (!settled)
  {
    OnStopTimeout(in_isr);
    return;
  }
  // 端点都已禁用：撤销全局 OUT NAK（这里置的，或被复位打断的停止置的）。
  // Every endpoint is disabled: release the global OUT NAK (set here, or by a stop this
  // reset cut short).
  global_out_nak_users_ = 0U;
  dev->gintmsk_reg.val &= ~GINTSTS_GOUTNAKEFF;
  if (dev->dctl_reg.goutnaksts != 0U)
  {
    dev->dctl_reg.cgoutnak = 1;
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

  (void)FlushFifos();

  for (size_t i = 0U; i < ENDPOINT_COUNT; ++i)
  {
    if (endpoint_map_.in[i] != nullptr)
    {
      static_cast<ESP32USBEndpoint*>(endpoint_map_.in[i])->ConfirmStopAfterReset(in_isr);
    }
    if (endpoint_map_.out[i] != nullptr)
    {
      static_cast<ESP32USBEndpoint*>(endpoint_map_.out[i])->ConfirmStopAfterReset(in_isr);
    }
  }
  dev->gintmsk_reg.sofmsk = 1;

  dev->dcfg_reg.devaddr = 0U;

  reset_enum_pending_ = true;
  OnBusReset(in_isr);
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

  const uint16_t words = ESPUSBDetail::CalcTxFifoWords(packet_size);
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
      ESPUSBDetail::CalcConfiguredRxFifoWords(packet_size, ENDPOINT_COUNT);
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

void ESP32USBDevice::OnControllerReset()
{
  ResetDeviceState();
  ResetEndpointHardwareState();
  ResetFifoState();
}

void ESP32USBDevice::OnControlReady() { ReloadSetupPacketCount(); }

}  // namespace LibXR

#endif
