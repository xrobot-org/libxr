// STM32 OTG backend (driver/st/stm32_usb_otg.cpp) against a model of the DWC2 device
// controller. The backend source is compiled unchanged: its register accesses at the
// OTG_HS / OTG_FS addresses land in the model (fake_dwc2.hpp, through mmio_trap.hpp),
// which implements the register, FIFO and interrupt behaviour of the reference manual. A
// host script (dwc2_host.hpp) sends SETUP, OUT and IN tokens, SOFs, resets, suspend and
// resume; the backend is driven only through its interrupt entry, the endpoint API and
// DeviceCore. The cases cover what the device classes rely on: enumeration, control
// transfers, bulk, interrupt and isochronous data, stall, stop and restart,
// reconfiguration, bus reset, disconnect, suspend, the frame interrupt and exactly-once
// completions, without DMA and with the core's internal DMA, on the core revisions the
// backend distinguishes.

#include <unistd.h>

#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "dwc2_host.hpp"
#include "stm32_usb_dev.hpp"
#include "test_assert.hpp"
#include "test_findings.hpp"
#include "uac_mic.hpp"
#include "usb_backend_cases.hpp"
#include "usb_probe_class.hpp"

using namespace LibXR;
using namespace LibXR::USB;
using Dwc2::Resp;
using UsbProbe::Log;
using UsbProbe::ProbeClass;
using Result = Dwc2::Host::Result;
using namespace UsbBackendCases;

extern "C" void libxr_stm32_otg_dispatch(PCD_HandleTypeDef* hpcd);

extern "C" void libxr_fatal_error(const char* file, uint32_t line, bool)
{
  std::fprintf(stderr, "%s:%u: assertion failed in the library\n", file, line);
  std::abort();
}

namespace
{
constexpr auto LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "LibXR", "STM32 OTG tests", "0");

/// 控制器的工作方式 / How the controller is operated
struct Mode
{
  const char* name;
  bool dma = false;
  uint32_t core = Dwc2::SNPSID_330A;
  bool setup_event_early = false;  ///< 见 Controller::setup_packet_event_early
  bool slave_setup_event = false;  ///< 见 Controller::slave_setup_event
  bool high_speed = false;         ///< ULPI 高速 PHY / ULPI high-speed PHY
  /// FIFO 冲刷位写 1 之后仍读到 1 的次数 / Reads that still see a flush bit set
  unsigned flush_reads = 1;
  /// EPDIS 写入后过多少次寄存器访问端点才禁用 / Register accesses until EPDIS acts
  unsigned disable_latency = 0;
};

constexpr Mode SLAVE{"FIFO mode, core 3.30a"};
constexpr Mode SLAVE_281A{"FIFO mode, core 2.81a", false, Dwc2::SNPSID_281A};
constexpr Mode SLAVE_310A{"FIFO mode, core 3.10a", false, Dwc2::SNPSID_310A, false, true};
constexpr Mode DMA{"DMA, core 3.30a", true};
constexpr Mode DMA_EARLY{"DMA, core 3.30a, early SETUP event", true, Dwc2::SNPSID_330A,
                         true};
constexpr Mode DMA_281A{"DMA, core 2.81a", true, Dwc2::SNPSID_281A};
constexpr Mode HIGH_SPEED{
    "FIFO mode, high speed", false, Dwc2::SNPSID_330A, false, false, true};
constexpr Mode HIGH_SPEED_DMA{
    "DMA, high speed", true, Dwc2::SNPSID_330A, false, false, true};
// FIFO 冲刷立即完成，或要到下一次中断才完成 / A FIFO flush that completes at once, or
// only by the next interrupt
constexpr Mode SLAVE_FLUSH_0{
    "FIFO mode, flush done at once", false, Dwc2::SNPSID_330A, false, false, false, 0};
constexpr Mode SLAVE_FLUSH_3{"FIFO mode, slow flush and disable",
                             false,
                             Dwc2::SNPSID_330A,
                             false,
                             false,
                             false,
                             3,
                             6};
constexpr Mode DMA_FLUSH_3{
    "DMA, slow flush and disable", true, Dwc2::SNPSID_330A, false, false, false, 3, 6};

constexpr Mode FULL_SPEED_MODES[] = {SLAVE, SLAVE_281A, SLAVE_310A,
                                     DMA,   DMA_EARLY,  DMA_281A};
constexpr Mode ALL_MODES[] = {SLAVE,         SLAVE_281A,    SLAVE_310A, DMA,
                              DMA_EARLY,     DMA_281A,      HIGH_SPEED, HIGH_SPEED_DMA,
                              SLAVE_FLUSH_0, SLAVE_FLUSH_3, DMA_FLUSH_3};

/// DMA 能到达的内存，位于 STM32H7 的 AXI SRAM 地址 / Memory the DMA reaches, at the
/// AXI SRAM address of an STM32H7
MmioTrap::LowArena& Arena()
{
  static MmioTrap::LowArena arena(0x24000000UL, 1U << 20U);
  return arena;
}

RawData Mem(size_t bytes) { return {Arena().Alloc(bytes, 64U), bytes}; }

/// 控制器模型、主机脚本和中断入口 / Controller model, host script and interrupt entry
struct Rig
{
  Mode mode;
  Dwc2::Controller hw;
  Dwc2::Host host;
  PCD_HandleTypeDef handle;
  unsigned interrupts = 0;
  bool check_on_exit = true;

  explicit Rig(const Mode& selected, bool full_speed_core = false)
      : mode((Arena().Reset(), fake_cortex = FakeCortexState{}, selected)),
        hw(MakeConfig(selected, full_speed_core)),
        host(hw, [this] { Service(); }),
        handle{full_speed_core ? USB_OTG_FS : USB_OTG_HS}
  {
    hw.setup_packet_event_early = selected.setup_event_early;
    hw.slave_setup_event = selected.slave_setup_event;
    hw.self_clear_reads = selected.flush_reads;
    hw.disable_latency = selected.disable_latency;
  }
  ~Rig()
  {
    if (check_on_exit) Check();
  }

  static Dwc2::Config MakeConfig(const Mode& mode, bool full_speed_core)
  {
    Dwc2::Config config;
    config.base = full_speed_core ? USB2_OTG_FS_PERIPH_BASE : USB1_OTG_HS_PERIPH_BASE;
    config.snpsid = mode.core;
    config.endpoints = 9;
    config.fifo_depth = 1024;
    config.hs_phy = mode.high_speed;
    config.dma_memory = &Arena();
    return config;
  }

  STM32USBOtgDevice::HardwareConfig Hardware(bool vbus_sensing = false) const
  {
    STM32USBOtgDevice::HardwareConfig config;
    config.phy = mode.high_speed ? STM32USBOtgDevice::Phy::ULPI
                                 : STM32USBOtgDevice::Phy::EMBEDDED_FS;
    config.dma = mode.dma;
    config.vbus_sensing = vbus_sensing;
    config.high_speed = mode.high_speed;
    if (mode.dma) config.ep0_dma_buffer = Mem(64);
    return config;
  }

  /// 中断线为高时反复进入中断处理；交替走带句柄和只有异常号的两条入口路径 / Enter the
  /// interrupt handler while the line is high, alternating the entry with a handle and
  /// the one that only has the exception number
  void Service()
  {
    for (unsigned guard = 0; hw.IrqLine(); ++guard)
    {
      TEST_ASSERT(guard < 64U);  // 中断线一直为高 / The line never goes low
      ++interrupts;
      fake_cortex.ipsr =
          16U + static_cast<uint32_t>(handle.Instance == USB_OTG_FS ? OTG_FS_IRQn
                                                                    : OTG_HS_IRQn);
      PCD_HandleTypeDef unset{nullptr};
      libxr_stm32_otg_dispatch((interrupts & 1U) != 0U ? &handle : &unset);
      fake_cortex.ipsr = 0U;
      TEST_ASSERT(fake_cortex.primask == 0U);  // 守卫成对 / The guard is balanced
    }
  }

  void Check()
  {
    for (const auto& text : hw.violations)
      std::fprintf(stderr, "[%s] controller rule broken: %s\n", mode.name, text.c_str());
    TEST_ASSERT(hw.violations.empty());
    TEST_ASSERT(host.toggle_errors == 0U);
    TEST_ASSERT(fake_cortex.primask == 0U);
  }
};

/// 探针设备类的设备，已 Init/Start / Device with the probe class, initialized and started
struct Probe : Rig
{
  using ModeType = Mode;
  static constexpr bool EXACT_IN_PROGRESS = true;
  static constexpr bool CLOSE_DEACTIVATES = true;
  static bool HighSpeed(const Mode& mode) { return mode.high_speed; }

  ProbeClass probe;
  STM32USBDeviceOtgHS device;
  uint16_t bulk = 64;

  explicit Probe(const Mode& selected, bool bus_time = false, bool vbus_sensing = false)
      : Rig(selected),
        device((probe.bulk_packet = selected.high_speed ? 512U : 64U,
                probe.wants_bus_time = bus_time, Hardware(vbus_sensing)),
               selected.high_speed ? 2048U : 512U,
               {Mem(64), Mem(selected.high_speed ? 2048U : 512U), Mem(128)},
               {{Mem(64), 64},
                {Mem(selected.high_speed ? 2048U : 512U),
                 selected.high_speed ? 1024U : 128U},
                {Mem(128), 64},
                {Mem(256), 128}},
               DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x0051, 0x0100,
               {&LANGUAGE}, {{&probe}}),
        bulk(selected.high_speed ? 512U : 64U)
  {
    device.Init(false);
    device.Start(false);
  }
  /// 设备对象离开前从中断入口注销 / Unregister from the interrupt entry before the
  /// device object goes away
  ~Probe() { device.Stop(false); }

  /// 复位、读设备描述符、设地址 7、选配置 1 / Reset, device descriptor, address 7,
  /// configuration 1
  void Up()
  {
    TEST_ASSERT(host.Enumerate(7, 1) == Result::OK);
    TEST_ASSERT(device.IsConfigured() && probe.binds == 1U);
    TEST_ASSERT(probe.bulk_in != nullptr && probe.bulk_out != nullptr);
  }

  Result VendorRead(size_t response, uint16_t length, std::vector<uint8_t>& data)
  {
    probe.response_size = response;
    return host.ControlRead(Dwc2::Host::Setup(0xC1, 0x70, 0, 0, length), data);
  }
  Result VendorWrite(const std::vector<uint8_t>& data)
  {
    return host.ControlWrite(
        Dwc2::Host::Setup(0x41, 0x71, 0, 0, static_cast<uint16_t>(data.size())), data);
  }
};

// ---------------------------------------------------------------------------
// Construction, start and enumeration
// ---------------------------------------------------------------------------

/// 构造后控制器不在总线上，FIFO 按配置划分且互不重叠；Start 之后连上总线，只打开驱动
/// 处理的中断，帧中断关闭。Start 之前到来的中断被挡在总中断开关之外。
/// After construction the controller is off the bus and the FIFO RAM is divided as
/// configured without overlap; Start connects it and unmasks only the interrupts the
/// driver handles, with the frame interrupt off. An interrupt before Start is gated at
/// the global interrupt enable.
void TestConstructionAndStart(const Mode& mode)
{
  Rig rig(mode);
  ProbeClass probe;
  STM32USBDeviceOtgHS device(
      rig.Hardware(), 512U, {Mem(64), Mem(512), Mem(128)},
      {{Mem(64), 64}, {Mem(512), 128}, {Mem(128), 64}, {Mem(256), 128}},
      DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x0051, 0x0100, {&LANGUAGE},
      {{&probe}});
  TEST_ASSERT(fake_cortex.msp_inits == 1U);
  TEST_ASSERT(!rig.hw.Connected());
  TEST_ASSERT(rig.hw.DmaEnabled() == mode.dma);
  TEST_ASSERT(rig.hw.Peek(Dwc2::GRXFSIZ) == 128U);
  TEST_ASSERT(rig.hw.Peek(Dwc2::DIEPTXF0) == ((16U << 16U) | 128U));
  TEST_ASSERT(rig.hw.Peek(Dwc2::DIEPTXF1) == ((32U << 16U) | 144U));
  TEST_ASSERT(rig.hw.Peek(Dwc2::DIEPTXF1 + 4U) == ((16U << 16U) | 176U));
  TEST_ASSERT(rig.hw.Peek(Dwc2::DIEPTXF1 + 8U) == ((32U << 16U) | 192U));
  TEST_ASSERT((rig.hw.Peek(Dwc2::GAHBCFG) & Dwc2::AHB_GINT) == 0U);

  // 未 Start：中断入口只把总中断关掉 / Not started: the entry only gates the line
  USB_OTG_HS->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
  libxr_stm32_otg_dispatch(&rig.handle);
  TEST_ASSERT((rig.hw.Peek(Dwc2::GAHBCFG) & Dwc2::AHB_GINT) == 0U);

  device.Init(false);
  device.Start(false);
  TEST_ASSERT(rig.hw.Connected());
  uint32_t expected = Dwc2::GINT_USBRST | Dwc2::GINT_ENUMDNE | Dwc2::GINT_USBSUSP |
                      Dwc2::GINT_WKUP | Dwc2::GINT_IEPINT | Dwc2::GINT_OEPINT |
                      Dwc2::GINT_IISOIXFR;
  if (!mode.dma) expected |= Dwc2::GINT_RXFLVL;
  TEST_ASSERT(rig.hw.Peek(Dwc2::GINTMSK) == expected);
  TEST_ASSERT((rig.hw.Peek(Dwc2::GAHBCFG) & Dwc2::AHB_GINT) != 0U);
  TEST_ASSERT(!rig.hw.IrqLine());
  device.Stop(false);
  TEST_ASSERT(!rig.hw.Connected());
}

/// 断开与重连。VBUS 检测打开时拔线产生会话结束中断：设备类解绑，在途请求结束一次；
/// 重新上电并枚举后正常。应用调用 Stop()/Start() 时设备离开总线再回来。
/// Disconnect and reconnect. With VBUS sensing a removed cable raises the session-end
/// interrupt: the class is unbound and in-flight requests end once; after power returns
/// and the host enumerates, everything works. With Stop() and Start() the device leaves
/// the bus and comes back.
void TestDisconnectReconnect(const Mode& mode)
{
  Probe f(mode, false, true);
  f.hw.HostVbus(false);
  f.hw.HostVbus(true);
  f.Service();
  f.Up();
  std::vector<uint8_t> data;
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, Bytes(f.bulk, 1)) == ErrorCode::OK);
  TEST_ASSERT(f.probe.bulk_out->Receive(f.bulk, false) == ErrorCode::OK);

  f.hw.HostVbus(false);
  f.Service();
  // 没有总线，停止由下一次上电后的帧或 Stop() 结束；这里先重新上电
  // Off the bus the stops end with frames after power returns
  f.hw.HostVbus(true);
  f.Service();
  f.host.Reset();
  f.host.Sof(4);
  TEST_ASSERT(f.probe.unbinds == 1U);
  TEST_ASSERT(f.probe.bulk_in_log.items.size() == 1U &&
              f.probe.bulk_in_log.items[0].result == ErrorCode::ABORTED);
  TEST_ASSERT(f.probe.bulk_out_log.items.size() == 1U &&
              f.probe.bulk_out_log.items[0].result == ErrorCode::ABORTED);
  TEST_ASSERT(f.host.Enumerate(4, 1) == Result::OK);
  const auto bytes = Bytes(9, 2);
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, bytes) == ErrorCode::OK);
  TEST_ASSERT(f.host.Read(1, data) == Result::OK && data == bytes);

  // Stop()：离开总线，在途请求结束 / Stop(): off the bus, in-flight requests end
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, Bytes(f.bulk, 3)) == ErrorCode::OK);
  f.device.Deinit(false);
  f.Service();  // 停止序列由中断推进 / The stop sequence advances on interrupts
  f.device.Stop(false);
  TEST_ASSERT(!f.hw.Connected() && !f.hw.IrqLine());
  TEST_ASSERT(f.probe.bulk_in_log.Count(ErrorCode::ABORTED) == 2U);
  TEST_ASSERT(f.hw.HostSetup(4, Dwc2::Host::Setup(0x80, 6, 0x100, 0, 18).data()) ==
              Resp::TIMEOUT);
  f.device.Init(false);
  f.device.Start(false);
  TEST_ASSERT(f.hw.Connected());
  TEST_ASSERT(f.host.Enumerate(5, 1) == Result::OK);
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, bytes) == ErrorCode::OK);
  TEST_ASSERT(f.host.Read(1, data) == Result::OK && data == bytes);
}

// ---------------------------------------------------------------------------
// UAC1 microphone
// ---------------------------------------------------------------------------

/// UAC1 麦克风设备 / Device with the UAC1 microphone
struct Uac : Rig
{
  using ModeType = Mode;

  UAC1MicrophoneQ<1, 16> mic;
  STM32USBDeviceOtgHS device;

  explicit Uac(const Mode& selected, bool start = true)
      : Rig(selected),
        mic(Endpoint::EPNumber::EP1, 16000),
        device(Hardware(), 512U, {Mem(64), Mem(128)}, {{Mem(64), 64}, {Mem(256), 128}},
               DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x0052, 0x0100,
               {&LANGUAGE}, {{&mic}})
  {
    if (!start) return;
    device.Init(false);
    device.Start(false);
  }
  ~Uac() { device.Stop(false); }

  /// 库外的代码关掉帧中断 / Code outside the library masks the frame interrupt
  void MaskFrameInterrupt() { USB_OTG_HS->GINTMSK &= ~USB_OTG_GINTMSK_SOFM; }

  /// 每帧先发 SOF 再取一个等时 IN 包，返回非空包的个数 / Each frame: SOF, then one
  /// isochronous IN token; returns how many packets carried data
  unsigned Stream(unsigned frames, size_t packet_bytes = 32U)
  {
    unsigned packets = 0;
    for (unsigned i = 0; i < frames; ++i)
    {
      host.Sof();
      const Dwc2::InResult in = hw.HostIn(host.address, 1);
      Service();
      if (in.resp == Resp::DATA && !in.data.empty())
      {
        TEST_ASSERT(in.data.size() == packet_bytes);
        ++packets;
      }
    }
    return packets;
  }
};

// ---------------------------------------------------------------------------
// The second controller instance
// ---------------------------------------------------------------------------

/// OTG_FS 实例：中断按控制器分发，只有异常号时也能找到它；枚举和一次批量来回。
/// The OTG_FS instance: interrupts are dispatched by controller, also when only the
/// exception number identifies it; enumeration and one bulk round trip.
void TestFullSpeedCore()
{
  Rig rig(SLAVE, true);
  ProbeClass probe;
  STM32USBDeviceOtgFS device(
      rig.Hardware(), 512U, {Mem(64), Mem(512), Mem(128)},
      {{Mem(64), 64}, {Mem(512), 128}, {Mem(128), 64}, {Mem(256), 128}},
      DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x0051, 0x0100, {&LANGUAGE},
      {{&probe}});
  device.Init(false);
  device.Start(false);
  TEST_ASSERT(rig.host.Enumerate(2, 1) == Result::OK && probe.binds == 1U);
  const auto bytes = Bytes(70, 1);
  std::vector<uint8_t> data;
  TEST_ASSERT(SubmitBytes(probe.bulk_in, bytes) == ErrorCode::OK);
  TEST_ASSERT(rig.host.ReadTransfer(1, data, 4096, 64) == Result::OK && data == bytes);
  TEST_ASSERT(probe.bulk_out->Receive(128, false) == ErrorCode::OK);
  TEST_ASSERT(rig.host.Write(1, bytes, 64) == Result::OK);
  TEST_ASSERT(probe.bulk_out_log.items.size() == 1U &&
              probe.bulk_out_log.items[0].data == bytes);
  device.Stop(false);
}

void RunAll()
{
  for (const Mode& mode : ALL_MODES)
  {
    if (!mode.high_speed) TestConstructionAndStart(mode);
    TestEnumeration<Probe>(mode);
    TestControlRead<Probe>(mode);
    TestControlWrite<Probe>(mode);
    TestControlStall<Probe>(mode);
    TestControlStallOutData<Probe>(mode);
    TestControlBackToBackSetups<Probe>(mode);
    TestControlSetupDuringInData<Probe>(mode);
    TestControlSetupDuringOutData<Probe>(mode);
    TestControlSetupSkipsStatus<Probe>(mode);
    TestControlSetupAfterWriteData<Probe>(mode);
    TestBulkIn<Probe>(mode);
    TestBulkOut<Probe>(mode);
    TestInterrupt<Probe>(mode);
    TestCallbackRearm<Probe>(mode);
    TestEndpointHalt<Probe>(mode);
    TestStopIn<Probe>(mode);
    TestStopInRace<Probe>(mode);
    TestStopOut<Probe>(mode);
    TestReconfigure<Probe>(mode);
    TestBusResetMidTransfer<Probe>(mode);
    TestSuspendResume<Probe>(mode);
    TestDisconnectReconnect(mode);
    TestFrameTicks<Probe>(mode);
    TestIsochronousFrames<Probe>(mode);
    TestIsochronousMissedFrame<Probe>(mode);
    std::printf("stm32 otg: %s passed\n", mode.name);
    std::fflush(stdout);
  }
  for (const Mode& mode : FULL_SPEED_MODES)
  {
    TestUacStreams<Uac>(mode);
    TestUacFrameInterruptFollowsConfiguration<Uac>(mode);
    TestUacStopsWithoutFrameInterrupt<Uac>(mode);
  }
  TestFullSpeedCore();
}

// ---------------------------------------------------------------------------
// Findings: behaviour the backend does not have yet (not in the default run)
// ---------------------------------------------------------------------------

/// DMA 模式、3.00a 之后的核心：控制写的最后一个数据包的传输完成，和主机进入状态阶段的
/// 通知（STSPHSRX）在同一次中断里被读到时，后端把这次完成当成"不是数据"丢掉，数据阶段
/// 永远不结束。中断延迟超过数据包应答到状态阶段 IN 令牌的间隔时出现。
/// DMA mode on a core after 3.00a: when the transfer completion of the last data packet
/// of a control write and the notice that the host entered the status stage (STSPHSRX)
/// are read by the same interrupt, the backend drops the completion as "not data" and
/// the data stage never ends. It takes an interrupt latency longer than the gap between
/// the data packet handshake and the status stage IN token.
void FindingDmaDataWithStatusPhase()
{
  Probe f(DMA);
  f.Up();
  const auto payload = Bytes(10, 1);
  TEST_ASSERT(f.host.SendSetup(Dwc2::Host::Setup(0x41, 0x71, 0, 0, 10)) == Result::OK);
  // 第一个 OUT 令牌结束 SETUP 阶段并被 NAK / The first OUT token ends the SETUP stage
  TEST_ASSERT(f.hw.HostOut(7, 0, 1, payload.data(), payload.size()) == Resp::NAK);
  f.Service();
  TEST_ASSERT(f.hw.HostOut(7, 0, 1, payload.data(), payload.size()) == Resp::ACK);
  // 中断还没进，主机已经发了状态阶段的 IN 令牌 / The status IN token arrives before
  // the interrupt is taken
  TEST_ASSERT(f.hw.HostIn(7, 0).resp == Resp::NAK);
  f.Service();
  TEST_ASSERT(f.probe.data_count == 1U);
  TEST_ASSERT(f.host.StatusIn() == Result::OK);
}

/// 端点停止序列还没走完（SNAK 已写、EPDIS 未写）时调用 Stop()：停止被直接确认，端点在
/// 硬件里仍是使能的。下一次会话里对它写传输长度寄存器时端点仍处于使能状态，而手册要求
/// 先按"停止传输"的序列禁用端点、且只在端点禁用时改写这个寄存器。
/// Stop() while an endpoint stop sequence is under way (SNAK written, EPDIS not yet):
/// the stop is confirmed at once and the endpoint stays enabled in the hardware. In the
/// next session its transfer size register is written while it is still enabled; the
/// manual requires the endpoint to be disabled by the transfer stop sequence and the
/// register to be written only while it is disabled.
void FindingStopLeavesEndpointEnabled()
{
  Probe f(SLAVE);
  f.Up();
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, Bytes(64, 1)) == ErrorCode::OK);
  f.device.Deinit(false);
  f.device.Stop(false);  // 中间没有进过中断 / No interrupt was taken in between
  TEST_ASSERT(f.probe.bulk_in_log.Count(ErrorCode::ABORTED) == 1U);
  f.device.Init(false);
  f.device.Start(false);
  TEST_ASSERT(f.host.Enumerate(5, 1) == Result::OK);
  TEST_ASSERT((f.hw.InCtl(1) & Dwc2::CTL_EPENA) == 0U);
  const auto bytes = Bytes(9, 2);
  std::vector<uint8_t> data;
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, bytes) == ErrorCode::OK);
  TEST_ASSERT(f.host.Read(1, data) == Result::OK && data == bytes);
}

}  // namespace

int main(int argc, char** argv)
{
  if (argc >= 2 && std::strcmp(argv[1], "--trace-frame-interrupt") == 0)
  {
    Mode mode = SLAVE;
    mode.flush_reads = argc >= 3 ? static_cast<unsigned>(std::atoi(argv[2])) : 1U;
    Uac f(mode, false);
    return TraceFrameInterrupt(f, argc >= 4 && std::atoi(argv[3]) != 0,
                               argc >= 5 && std::atoi(argv[4]) != 0);
  }
  const int handled = TestFindings::Run(
      argc, argv,
      {{"dma_data_with_status_phase", "XFRC read together with STSPHSRX is dropped",
        FindingDmaDataWithStatusPhase},
       {"stop_leaves_endpoint_enabled", "Stop() during an endpoint stop sequence",
        FindingStopLeavesEndpointEnabled}});
  if (handled >= 0) return handled;
  RunAll();
  std::printf("stm32 otg: all tests passed\n");
  return 0;
}
