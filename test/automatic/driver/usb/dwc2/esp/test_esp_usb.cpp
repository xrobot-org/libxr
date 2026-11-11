// ESP32-S2/S3 USB backend (driver/esp/esp_usb*.cpp) against a model of the DWC2 device
// controller. The backend source is compiled unchanged against fake ESP-IDF headers for
// an ESP32-S3; its register accesses at 0x60080000 land in the model (fake_dwc2.hpp,
// through mmio_trap.hpp), which implements the register, FIFO and interrupt behaviour of
// the controller. The backend always uses the internal DMA, so every object the DMA
// reaches (the device object with its SETUP buffer, the endpoint objects and the endpoint
// slots) is placed in a mapping at the address of the internal RAM. The cases are the
// contract cases shared with the STM32 OTG test (usb_backend_cases.hpp) plus the start-up
// and the stop and start of this backend.

#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <vector>

#include "dwc2_host.hpp"
#include "esp_usb.hpp"
#include "esp_usb_dev.hpp"
#include "esp_usb_ep.hpp"
#include "test_assert.hpp"
#include "test_findings.hpp"
#include "uac_mic.hpp"
#include "usb_backend_cases.hpp"

using namespace LibXR;
using namespace LibXR::USB;
using namespace UsbBackendCases;
using UsbProbe::ProbeClass;

extern "C" void libxr_fatal_error(const char* file, uint32_t line, bool)
{
  std::fprintf(stderr, "%s:%u: assertion failed in the library\n", file, line);
  std::abort();
}

namespace
{
/// 内部 RAM：DMA 能到达的全部内存 / The internal RAM: all the memory the DMA reaches
MmioTrap::LowArena& Arena()
{
  static MmioTrap::LowArena arena(SOC_DMA_LOW, SOC_DMA_HIGH - SOC_DMA_LOW);
  return arena;
}

/// 为真时 new 从内部 RAM 分配：后端在构造函数里 new 出端点对象，端点里有 DMA 会写的
/// 缓冲 / While set, new allocates from the internal RAM: the backend creates its
/// endpoint objects with new in its constructor, and an endpoint holds a buffer the DMA
/// writes
bool arena_new = false;

void* Allocate(size_t size, size_t align)
{
  if (arena_new) return Arena().Alloc(size, align < 16U ? 16U : align);
  void* memory = nullptr;
  if (posix_memalign(&memory, align < sizeof(void*) ? sizeof(void*) : align,
                     size == 0U ? 1U : size) != 0)
    std::abort();
  return memory;
}

void Release(void* memory)
{
  if (memory == nullptr || Arena().Contains(reinterpret_cast<uintptr_t>(memory), 1U))
    return;
  std::free(memory);
}
}  // namespace

void* operator new(size_t size) { return Allocate(size, 16U); }
void* operator new[](size_t size) { return Allocate(size, 16U); }
void* operator new(size_t size, std::align_val_t align)
{
  return Allocate(size, static_cast<size_t>(align));
}
void* operator new[](size_t size, std::align_val_t align)
{
  return Allocate(size, static_cast<size_t>(align));
}
void operator delete(void* memory) noexcept { Release(memory); }
void operator delete[](void* memory) noexcept { Release(memory); }
void operator delete(void* memory, size_t) noexcept { Release(memory); }
void operator delete[](void* memory, size_t) noexcept { Release(memory); }
void operator delete(void* memory, std::align_val_t) noexcept { Release(memory); }
void operator delete[](void* memory, std::align_val_t) noexcept { Release(memory); }
void operator delete(void* memory, size_t, std::align_val_t) noexcept { Release(memory); }
void operator delete[](void* memory, size_t, std::align_val_t) noexcept
{
  Release(memory);
}

namespace
{
constexpr auto LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "LibXR", "ESP32 USB tests", "0");

/// 控制器模型的时间参数 / Timing of the controller model
struct Mode
{
  const char* name;
  bool setup_event_early = false;  ///< 见 Controller::setup_packet_event_early
  unsigned flush_reads = 1;        ///< 见 Controller::self_clear_reads
  unsigned disable_latency = 6;    ///< 见 Controller::disable_latency
};

constexpr Mode NORMAL{"SETUP event at the end of the SETUP stage"};
constexpr Mode EARLY{"SETUP event with the packet", true};
constexpr Mode SLOW{"slow flush and disable", false, 3, 12};
constexpr Mode IMMEDIATE{"endpoint disable takes effect at once", false, 1, 0};
constexpr Mode MODES[] = {NORMAL, EARLY, SLOW};

RawData Mem(size_t bytes) { return {Arena().Alloc(bytes, 64U), bytes}; }

/// 这个后端禁用 OUT 端点时不先进入全局 OUT NAK；作为"发现"单独列出
/// This backend disables an OUT endpoint without the global OUT NAK; listed as a finding
constexpr const char* OUT_DISABLE_RULE = "without global OUT NAK";

/// 控制器模型、主机脚本和中断入口 / Controller model, host script and interrupt entry
struct Rig
{
  Mode mode;
  Dwc2::Controller hw;
  Dwc2::Host host;
  unsigned interrupts = 0;
  bool check_on_exit = true;

  explicit Rig(const Mode& selected)
      : mode((Arena().Reset(), fake_esp = FakeEspState{}, selected)),
        hw(MakeConfig()),
        host(hw, [this] { Service(); })
  {
    hw.setup_packet_event_early = selected.setup_event_early;
    hw.self_clear_reads = selected.flush_reads;
    hw.disable_latency = selected.disable_latency;
    hw.tolerated.emplace_back(OUT_DISABLE_RULE);
  }
  ~Rig()
  {
    fake_esp.handler = nullptr;
    if (check_on_exit) Check();
  }

  /// ESP32-S3 的 USB-OTG：EP0 加 6 个端点，256 字的 FIFO，核心 4.00a / USB-OTG of an
  /// ESP32-S3: EP0 and six endpoints, 256 words of FIFO, core 4.00a
  static Dwc2::Config MakeConfig()
  {
    Dwc2::Config config;
    config.base = ESPUSBDetail::DWC2_FS_REG_BASE;
    config.snpsid = Dwc2::SNPSID_400A;
    config.endpoints = 7;
    config.fifo_depth = 256;
    config.ghwcfg[1] = 0x224DD930U;
    config.ghwcfg[2] = 0x010004B5U;
    config.ghwcfg[3] = 0xD3F0A030U;
    config.dma_memory = &Arena();
    return config;
  }

  /// 中断线为高且中断已使能时，运行后端注册的中断处理 / Run the handler the backend
  /// registered while the line is high and the interrupt is enabled
  void Service()
  {
    for (unsigned guard = 0;
         hw.IrqLine() && fake_esp.interrupt_enabled && fake_esp.handler != nullptr;
         ++guard)
    {
      TEST_ASSERT(guard < 64U);  // 中断线一直为高 / The line never goes low
      ++interrupts;
      fake_esp.handler(fake_esp.handler_arg);
      TEST_ASSERT(fake_esp.critical_depth == 0U);
    }
  }

  void Check()
  {
    for (const auto& text : hw.violations)
      std::fprintf(stderr, "[%s] controller rule broken: %s\n", mode.name, text.c_str());
    TEST_ASSERT(hw.violations.empty());
    TEST_ASSERT(host.toggle_errors == 0U);
    TEST_ASSERT(fake_esp.critical_depth == 0U);
  }

  template <typename... Arguments>
  static ESP32USBDevice* Create(Arguments&&... arguments)
  {
    arena_new = true;
    auto* created = new ESP32USBDevice(std::forward<Arguments>(arguments)...);
    arena_new = false;
    return created;
  }
};

using EpConfig = ESP32USBDevice::EPConfig;

/// 探针设备类的设备，已 Init/Start / Device with the probe class, initialized and started
struct Probe : Rig
{
  using ModeType = Mode;
  /// 被停止的 IN 请求报告的长度是取进 FIFO 的字节数，不是主机取走的；作为"发现"单独
  /// 列出 / A stopped IN request reports the bytes fetched into the FIFO, not the bytes
  /// the host took; listed as a finding
  static constexpr bool EXACT_IN_PROGRESS = false;
  /// 关闭端点时没有清掉 USBAEP，端点仍然应答 NAK；作为"发现"单独列出 / Closing an
  /// endpoint leaves USBAEP set and the endpoint keeps answering NAK; listed as a finding
  static constexpr bool CLOSE_DEACTIVATES = false;
  static bool HighSpeed(const Mode&) { return false; }

  ProbeClass probe;
  ESP32USBDevice& device;
  uint16_t bulk = 64;

  explicit Probe(const Mode& selected, bool needs_tick = false)
      : Rig(selected), device(*Make(needs_tick))
  {
    device.Init(false);
    device.Start(false);
  }
  ~Probe() { device.Stop(false); }

  ESP32USBDevice* Make(bool needs_tick)
  {
    probe.needs_frame_tick = needs_tick;
    return Create(
        std::initializer_list<EpConfig>{EpConfig(Mem(128)), EpConfig(Mem(1024)),
                                        EpConfig(Mem(256)), EpConfig(Mem(256), true)},
        DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x0053, 0x0100,
        std::initializer_list<const DescriptorStrings::LanguagePack*>{&LANGUAGE},
        std::initializer_list<const std::initializer_list<ConfigDescriptorItem*>>{
            {&probe}});
  }

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

/// 同一个设备，上面两项也检查 / The same device with both of the above checked
struct ExactProbe : Probe
{
  using Probe::Probe;
  static constexpr bool EXACT_IN_PROGRESS = true;
  static constexpr bool CLOSE_DEACTIVATES = true;
};

/// UAC1 麦克风设备 / Device with the UAC1 microphone
struct Uac : Rig
{
  using ModeType = Mode;

  UAC1MicrophoneQ<1, 16> mic;
  ESP32USBDevice& device;

  explicit Uac(const Mode& selected, bool start = true)
      : Rig(selected),
        mic(Endpoint::EPNumber::EP1, 16000),
        device(*Create(
            std::initializer_list<EpConfig>{EpConfig(Mem(128)), EpConfig(Mem(256), true)},
            DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x0054, 0x0100,
            std::initializer_list<const DescriptorStrings::LanguagePack*>{&LANGUAGE},
            std::initializer_list<const std::initializer_list<ConfigDescriptorItem*>>{
                {&mic}}))
  {
    if (!start) return;
    device.Init(false);
    device.Start(false);
  }
  ~Uac() { device.Stop(false); }

  void MaskFrameInterrupt()
  {
    reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE)->gintmsk_reg.val &=
        ~Dwc2::GINT_SOF;
  }

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
// Start-up, stop and start
// ---------------------------------------------------------------------------

/// Start：ROM 的 USB 被清掉一次、PHY 建一次、中断注册到 USB 中断源并使能；控制器处于
/// 设备模式、DMA 打开、连上总线，只打开驱动处理的中断，帧中断包括在内，接收 FIFO
/// 中断关闭。 枚举后发送 FIFO 接在接收 FIFO 之后依次排开，互不重叠，总量不超过 FIFO RAM。
/// Start: the ROM USB is torn down once, the PHY is created once and the interrupt is
/// registered on the USB source and enabled; the controller is in device mode with DMA
/// on and connected, and only the interrupts the driver handles are unmasked, the frame
/// interrupt included and the receive FIFO interrupt off. After enumeration the transmit
/// FIFOs follow the receive FIFO one after another without overlap, within the FIFO RAM.
void TestStart(const Mode& mode)
{
  Probe f(mode);
  TEST_ASSERT(fake_esp.rom_deinit == 2U && fake_esp.phy_created == 1U);
  TEST_ASSERT(fake_esp.source == ETS_USB_INTR_SOURCE && fake_esp.interrupt_enabled);
  TEST_ASSERT(f.hw.Connected() && f.hw.DmaEnabled());
  TEST_ASSERT((f.hw.Peek(Dwc2::GAHBCFG) & Dwc2::AHB_GINT) != 0U);
  TEST_ASSERT(f.hw.Peek(Dwc2::GINTMSK) ==
              (Dwc2::GINT_USBRST | Dwc2::GINT_ENUMDNE | Dwc2::GINT_USBSUSP |
               Dwc2::GINT_WKUP | Dwc2::GINT_IEPINT | Dwc2::GINT_OEPINT | Dwc2::GINT_OTG |
               Dwc2::GINT_SOF));
  TEST_ASSERT(!f.hw.IrqLine());
  // Start 是幂等的 / Start is idempotent
  f.device.Start(false);
  TEST_ASSERT(fake_esp.phy_created == 1U && fake_esp.rom_deinit == 2U);

  f.Up();
  const uint32_t receive = f.hw.Peek(Dwc2::GRXFSIZ) & 0xFFFFU;
  TEST_ASSERT(receive >= 88U);
  uint32_t next = receive;
  for (unsigned fifo = 0; fifo < 4U; ++fifo)
  {
    const uint32_t value =
        f.hw.Peek(fifo == 0U ? Dwc2::DIEPTXF0 : Dwc2::DIEPTXF1 + 4U * (fifo - 1U));
    TEST_ASSERT((value & 0xFFFFU) == next);
    TEST_ASSERT((value >> 16U) >= (fifo == 3U ? 24U : 16U));
    next += value >> 16U;
  }
  TEST_ASSERT(next <= 256U);
}

/// Stop 和 Start：Stop 之后设备离开总线、中断关闭；再 Start 后重新枚举，数据照常。
/// Stop and Start: after Stop the device is off the bus and its interrupt is disabled;
/// after Start it enumerates again and data flows as before.
void TestStopStart(const Mode& mode)
{
  Probe f(mode);
  f.Up();
  std::vector<uint8_t> data;
  const auto bytes = Bytes(20, 1);
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, bytes) == ErrorCode::OK);
  TEST_ASSERT(f.host.Read(1, data) == Result::OK && data == bytes);

  f.device.Deinit(false);
  f.Service();
  f.device.Stop(false);
  TEST_ASSERT(!f.hw.Connected() && !fake_esp.interrupt_enabled && !f.hw.IrqLine());
  TEST_ASSERT(f.hw.HostSetup(7, Dwc2::Host::Setup(0x80, 6, 0x100, 0, 18).data()) ==
              Resp::TIMEOUT);

  f.device.Init(false);
  f.device.Start(false);
  TEST_ASSERT(f.hw.Connected() && fake_esp.interrupt_enabled);
  TEST_ASSERT(fake_esp.phy_created == 1U);
  TEST_ASSERT(f.host.Enumerate(5, 1) == Result::OK && f.probe.binds == 2U);
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, bytes) == ErrorCode::OK);
  TEST_ASSERT(f.host.Read(1, data) == Result::OK && data == bytes);
  TEST_ASSERT(f.probe.bulk_out->Receive(64, false) == ErrorCode::OK);
  TEST_ASSERT(f.host.Write(1, bytes, 64) == Result::OK);
  TEST_ASSERT(f.probe.bulk_out_log.items.size() == 1U &&
              f.probe.bulk_out_log.items[0].data == bytes);
}

void RunAll()
{
  for (const Mode& mode : MODES)
  {
    TestStart(mode);
    TestEnumeration<Probe>(mode);
    TestControlRead<Probe>(mode);
    TestControlWrite<Probe>(mode);
    TestControlStall<Probe>(mode);
    TestControlSetupDuringOutData<Probe>(mode);
    TestControlSetupSkipsStatus<Probe>(mode);
    TestBulkIn<Probe>(mode);
    TestBulkOut<Probe>(mode);
    TestInterrupt<Probe>(mode);
    TestCallbackRearm<Probe>(mode);
    TestEndpointHalt<Probe>(mode);
    TestStopIn<Probe>(mode);
    TestStopOut<Probe>(mode);
    TestReconfigure<Probe>(mode);
    TestBusResetMidTransfer<Probe>(mode);
    TestSuspendResume<Probe>(mode);
    TestStopStart(mode);
    TestFrameTicks<Probe>(mode);
    TestUacFrameInterruptIgnoresConfiguration<Uac>(mode);
    std::printf("esp usb: %s passed\n", mode.name);
    std::fflush(stdout);
  }
}

// ---------------------------------------------------------------------------
// Findings: behaviour the backend does not have yet (not in the default run)
// ---------------------------------------------------------------------------

void FindingControlStallOutData() { TestControlStallOutData<Probe>(NORMAL); }
void FindingIsochronousFrames() { TestIsochronousFrames<Probe>(NORMAL); }
void FindingIsochronousMissedFrame() { TestIsochronousMissedFrame<Probe>(NORMAL); }
void FindingUacStreams() { TestUacStreams<Uac>(NORMAL); }

/// 中断状态寄存器的位是写 1 清除的。后端用位域赋值清一个位，编译出来是读出整个寄存器、
/// 置位、写回，于是把这一刻挂起的其他中断也一起清掉：处理挂起中断的过程中到来的总线
/// 复位就这样丢了，设备留在旧地址上，主机枚举不到它。
/// The bits of the interrupt status register are cleared by writing 1. The backend
/// clears one with a bit-field assignment, which compiles to reading the whole register,
/// setting the bit and writing it back, and so clears every other interrupt pending at
/// that moment: a bus reset that arrives while the suspend interrupt is being handled is
/// lost, the device stays at its old address and the host cannot enumerate it.
void FindingInterruptClearLosesReset()
{
  Probe f(NORMAL);
  f.Up();
  f.hw.HostSuspend();
  unsigned reads = 0;
  f.hw.before_read = [&](uint32_t offset)
  {
    // 第一次读是取挂起的中断，第二次是清除挂起位的读改写 / The first read takes the
    // pending interrupts, the second is the read-modify-write that clears the suspend bit
    if (offset == Dwc2::GINTSTS && ++reads == 2U) f.hw.HostResetBegin();
  };
  f.Service();
  f.hw.before_read = nullptr;
  f.Service();
  f.hw.HostResetEnd();
  f.Service();
  TEST_ASSERT(f.probe.resets == 2U);
  f.host.address = 0;
  f.host.ResetToggles();
  std::vector<uint8_t> data;
  TEST_ASSERT(f.host.GetDescriptor(1, 0, 18, data) == Result::OK && data.size() == 18U);
}
void FindingStopInImmediateDisable() { TestStopIn<Probe>(IMMEDIATE); }
void FindingStopInProgress() { TestStopIn<ExactProbe>(NORMAL); }
void FindingStopInRace() { TestStopInRace<Probe>(NORMAL); }
void FindingCloseKeepsEndpointActive() { TestReconfigure<ExactProbe>(NORMAL); }
void FindingBackToBackSetups() { TestControlBackToBackSetups<Probe>(NORMAL); }
void FindingSetupDuringInData() { TestControlSetupDuringInData<Probe>(NORMAL); }
void FindingSetupAfterWriteData() { TestControlSetupAfterWriteData<Probe>(NORMAL); }

/// 手册要求先进入全局 OUT NAK 再禁用 OUT 端点；这个后端直接写 EPDIS。
/// The manual requires the global OUT NAK before an OUT endpoint is disabled; this
/// backend writes EPDIS directly.
void FindingOutDisableWithoutGlobalNak()
{
  Probe f(NORMAL);
  f.hw.tolerated.clear();
  f.Up();
  TEST_ASSERT(f.probe.bulk_out->Receive(64, false) == ErrorCode::OK);
  f.probe.bulk_out->Flush(false);
  f.host.Sof(3);
  TEST_ASSERT(f.probe.bulk_out_log.Count(ErrorCode::ABORTED) == 1U);
}

}  // namespace

int main(int argc, char** argv)
{
  const int handled = TestFindings::Run(
      argc, argv,
      {{"control_stall_out_data",
        "EP0 OUT not enabled again after a stalled control write",
        FindingControlStallOutData},
       {"back_to_back_setups", "the first of several back-to-back SETUPs is answered",
        FindingBackToBackSetups},
       {"setup_during_in_data",
        "EP0 IN is not stopped when a SETUP ends an IN data stage",
        FindingSetupDuringInData},
       {"setup_after_write_data", "SETUP not received before a control write status",
        FindingSetupAfterWriteData},
       {"stop_in_progress", "a stopped IN request reports bytes the host did not take",
        FindingStopInProgress},
       {"stop_in_race", "EPDIS written after the transfer completed during the stop",
        FindingStopInRace},
       {"close_keeps_endpoint_active", "a closed endpoint keeps USBAEP and answers NAK",
        FindingCloseKeepsEndpointActive},
       {"isochronous_frames", "isochronous IN is not scheduled for the next frame",
        FindingIsochronousFrames},
       {"isochronous_missed_frame", "a missed isochronous IN frame is never ended",
        FindingIsochronousMissedFrame},
       {"uac_streams", "UAC1 microphone sends in every other frame only",
        FindingUacStreams},
       {"interrupt_clear_loses_reset",
        "bit-field write to GINTSTS clears other interrupts",
        FindingInterruptClearLosesReset},
       {"stop_in_immediate_disable", "EPDISD cleared unhandled when it is set at once",
        FindingStopInImmediateDisable},
       {"out_disable_without_global_nak", "OUT endpoint disabled without global OUT NAK",
        FindingOutDisableWithoutGlobalNak}});
  if (handled >= 0) return handled;
  RunAll();
  std::printf("esp usb: all tests passed\n");
  return 0;
}
