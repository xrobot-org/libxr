/**
 * @file test_dfu.cpp
 * @brief DFU 类的假后端测试 / Fake-backend tests of the DFU classes
 *
 * 用真实的设备核心、组合层、EP0 端点和 DFU 类；假后端设备同 test_cdc.cpp。主机一侧走
 * 真实的 EP0 控制传输，状态阶段由测试决定完成、不完成或被新的 SETUP 取代，用来测 DFU
 * 依赖的控制完成回调。覆盖运行时和 bootloader 的请求、绑定失败、带进行中的传输和
 * 擦写解绑、状态阶段的各条路径和多线程。
 * Uses the real device core, composition, EP0 endpoints and DFU classes; the fake
 * backend device is the one of test_cdc.cpp. The host side runs real EP0 control
 * transfers, and the test decides whether a status stage completes, never completes or
 * is superseded by a new SETUP, which exercises the control completion callbacks DFU
 * depends on. Covered: the runtime and bootloader requests, bind failure, unbind with
 * transfers and flash work in progress, the status-stage paths and threads.
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "crc.hpp"
#include "dev_core.hpp"
#include "dfu_bootloader.hpp"
#include "dfu_runtime.hpp"
#include "ep.hpp"
#include "ep_pool.hpp"
#include "flash.hpp"
#include "test_assert.hpp"
#include "usb_service.hpp"
#include "winusb_msos20.hpp"

namespace
{
using LibXR::ConstRawData;
using LibXR::ErrorCode;
using LibXR::RawData;
using LibXR::USB::DescriptorStrings;
using LibXR::USB::DeviceClass;
using LibXR::USB::DeviceCore;
using LibXR::USB::DeviceDescriptor;
using LibXR::USB::DeviceService;
using LibXR::USB::DfuBootloaderBackend;
using LibXR::USB::DfuBootloaderClassT;
using LibXR::USB::DFUCapabilities;
using LibXR::USB::DFUClass;
using LibXR::USB::DfuRuntimeClass;
using LibXR::USB::DFUState;
using LibXR::USB::DFUStatusCode;
using LibXR::USB::Endpoint;
using LibXR::USB::EndpointPool;
using LibXR::USB::SetupPacket;
using LibXR::USB::Speed;
using LibXR::USB::USBSpec;
using Direction = Endpoint::Direction;
using EPNumber = Endpoint::EPNumber;
using Bytes = std::vector<uint8_t>;
namespace WinUsb = LibXR::USB::WinUsbMsOs20;

static_assert(LibXR::HW_CACHE_LINE_SIZE <= 64U, "test memory is aligned to 64 bytes");

constexpr auto LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "XRobot", "DFU", "0001");

/// 端点内存 / Endpoint memory
struct alignas(64) Memory
{
  uint8_t bytes[64];
};

/// 记录启动、由测试完成的假端点 / Fake endpoint that records starts, completed by tests
class FakeEp final : public Endpoint
{
 public:
  /// 一次 StartHardware / One StartHardware
  struct Start
  {
    size_t length;
    uint8_t* data;
  };

  FakeEp(EPNumber number, Direction direction, RawData memory)
      : Endpoint(number, direction, memory)
  {
  }

  /// 硬件完成在途请求；只在 owner 中 / The hardware completes the in-flight request;
  /// owner only
  void Done(bool in_isr, size_t actual)
  {
    TEST_ASSERT(inflight);
    inflight = false;
    OnHardwareComplete(in_isr, actual, ErrorCode::OK);
  }

  /// 主机发来 OUT 数据；只在 owner 中 / The host sends OUT data; owner only
  void HostSends(bool in_isr, const Bytes& bytes)
  {
    TEST_ASSERT(inflight && bytes.size() <= starts.back().length);
    if (!bytes.empty())
    {
      std::memcpy(starts.back().data, bytes.data(), bytes.size());
    }
    Done(in_isr, bytes.size());
  }

  size_t LastLength() const { return starts.empty() ? SIZE_MAX : starts.back().length; }

  std::vector<Start> starts;
  bool inflight = false;

 protected:
  void ConfigureHardware(const Config&) override {}
  void CloseHardware(bool) override {}

  ErrorCode StartHardware(const HwRequest& request, bool) override
  {
    TEST_ASSERT(!inflight);
    starts.push_back({request.length, request.data});
    inflight = true;
    return ErrorCode::OK;
  }

  StopOutcome StopHardware(bool) override
  {
    inflight = false;
    return {ErrorCode::ABORTED, 0U};
  }

  ErrorCode StallHardware(bool) override { return ErrorCode::OK; }
  ErrorCode ClearStallHardware(bool) override { return ErrorCode::OK; }
  HwLimits HardwareLimits(Direction, Type) const override { return {}; }
};

/// 只有 EP0 的假端点和端点池 / Fake endpoints and pool with EP0 only
struct Hw
{
  Hw() { pool.SetEndpoint0(&in0, &out0); }

  Memory mem[2];
  FakeEp in0{EPNumber::EP0, Direction::IN, {mem[0].bytes, 64U}};
  FakeEp out0{EPNumber::EP0, Direction::OUT, {mem[1].bytes, 64U}};
  EndpointPool pool;
};

using Configs = std::initializer_list<const std::initializer_list<DeviceClass*>>;

/// 假后端设备 / Fake backend device
class FakeUsb final : public DeviceService, public DeviceCore
{
 public:
  FakeUsb(Hw& hw, const Configs& configs, USBSpec spec = USBSpec::USB_2_0)
      : DeviceCore(*this, hw.pool, spec, Speed::FULL,
                   DeviceDescriptor::PacketSize0::SIZE_64, 0x1209U, 0x0002U, 0x0100U,
                   {&LANGUAGE}, configs)
  {
    TEST_ASSERT(Register(hw.in0) == ErrorCode::OK);
    TEST_ASSERT(Register(hw.out0) == ErrorCode::OK);
  }

  /// 排一个硬件事件并进入控制器中断 / Queue one hardware event and enter the
  /// controller interrupt
  void Irq(std::function<void(bool)> event)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      events_.push_back(std::move(event));
    }
    OnControllerInterrupt();
  }

  void Setup(SetupPacket setup)
  {
    Irq([this, setup](bool isr) { OnSetup(isr, setup); });
  }
  void Reset()
  {
    Irq([this](bool isr) { OnBusReset(isr, Speed::FULL); });
  }

  bool Inited() const { return IsInited(); }
  void StartController(bool) override {}
  void StopController(bool) override {}

  std::atomic<size_t> ready{0U};

 protected:
  void ServiceHardware(bool in_isr) override
  {
    while (true)
    {
      std::function<void(bool)> event;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (events_.empty())
        {
          return;
        }
        event = std::move(events_.front());
        events_.pop_front();
      }
      event(in_isr);
    }
  }

  void MaskInterrupt() override {}
  void UnmaskInterrupt() override {}
  void SetAddress(uint8_t, AddressStage) override {}
  void OnControlReady() override { ready.fetch_add(1U, std::memory_order_relaxed); }
  void ResetController(bool) override {}

 private:
  std::mutex mutex_;
  std::deque<std::function<void(bool)>> events_;
};

/// 内存后备的 Flash；hold 时写入在 held 置位后等 hold 清掉 / RAM-backed flash; with
/// hold set, a write sets held and waits until hold is cleared
class RamFlash : public LibXR::Flash
{
 public:
  RamFlash(size_t min_erase, size_t min_write, RawData area)
      : Flash(min_erase, min_write, area), area_(area)
  {
  }

  ErrorCode Erase(size_t offset, size_t size) override
  {
    TEST_ASSERT(offset + size <= area_.size_);
    std::memset(static_cast<uint8_t*>(area_.addr_) + offset, 0xFF, size);
    erases.fetch_add(1U, std::memory_order_relaxed);
    return ErrorCode::OK;
  }

  ErrorCode Write(size_t offset, ConstRawData data) override
  {
    Gate();
    TEST_ASSERT(offset + data.size_ <= area_.size_);
    std::memcpy(static_cast<uint8_t*>(area_.addr_) + offset, data.addr_, data.size_);
    writes.fetch_add(1U, std::memory_order_relaxed);
    return ErrorCode::OK;
  }

  std::atomic<bool> hold{false};
  std::atomic<bool> held{false};
  std::atomic<size_t> erases{0U};
  std::atomic<size_t> writes{0U};

 private:
  void Gate()
  {
    if (!hold.load(std::memory_order_acquire))
    {
      return;
    }
    held.store(true, std::memory_order_release);
    while (hold.load(std::memory_order_acquire))
    {
      std::this_thread::yield();
    }
  }

  RawData area_;
};

std::atomic<unsigned> g_app_jumps{0U};
std::atomic<unsigned> g_boot_jumps{0U};
void JumpToApp(void*) { g_app_jumps.fetch_add(1U, std::memory_order_relaxed); }
void JumpToBootloader(void*) { g_boot_jumps.fetch_add(1U, std::memory_order_relaxed); }

template <typename Condition>
bool WaitUntil(Condition condition, unsigned timeout_ms = 5000U)
{
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (!condition())
  {
    if (std::chrono::steady_clock::now() >= deadline)
    {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  return true;
}

void SleepMs(uint32_t ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

// ---- 主机一侧 / The host side ----

SetupPacket Make(uint8_t type, uint8_t request, uint16_t value, uint16_t index,
                 uint16_t length)
{
  return SetupPacket{type, request, value, index, length};
}

bool Stalled(Hw& hw) { return hw.in0.IsStalled() && hw.out0.IsStalled(); }

void InDone(FakeUsb& dev, FakeEp& ep)
{
  TEST_ASSERT(ep.inflight);
  const size_t length = ep.LastLength();
  dev.Irq([&ep, length](bool isr) { ep.Done(isr, length); });
}

void HostOut(FakeUsb& dev, FakeEp& ep, const Bytes& bytes)
{
  TEST_ASSERT(ep.inflight);
  dev.Irq([&ep, bytes](bool isr) { ep.HostSends(isr, bytes); });
}

void StatusOut(FakeUsb& dev, Hw& hw)
{
  dev.Irq([&hw](bool isr) { hw.out0.HostSends(isr, {}); });
}

/// IN 请求；finish 为 false 时不发状态 OUT / IN request; without finish no status OUT
Bytes ControlIn(FakeUsb& dev, Hw& hw, SetupPacket setup, bool finish = true)
{
  const size_t ready = dev.ready.load();
  dev.Setup(setup);
  TEST_ASSERT(!Stalled(hw));
  Bytes got;
  while (hw.in0.inflight)
  {
    const FakeEp::Start& start = hw.in0.starts.back();
    got.insert(got.end(), start.data, start.data + start.length);
    InDone(dev, hw.in0);
  }
  if (finish)
  {
    StatusOut(dev, hw);
    TEST_ASSERT(dev.ready.load() == ready + 1U);
  }
  return got;
}

/// 无数据阶段的请求；finish 为 false 时状态 IN 留在途 / No-data request; without
/// finish the status IN stays in flight
void ControlNone(FakeUsb& dev, Hw& hw, SetupPacket setup, bool finish = true)
{
  const size_t ready = dev.ready.load();
  dev.Setup(setup);
  TEST_ASSERT(!Stalled(hw));
  TEST_ASSERT(hw.in0.inflight && hw.in0.LastLength() == 0U);
  if (finish)
  {
    InDone(dev, hw.in0);
    TEST_ASSERT(dev.ready.load() == ready + 1U);
  }
}

/**
 * OUT 请求，数据按 64 字节分包；返回 false 表示数据阶段之后被 STALL。finish 为 false
 * 时状态 IN 留在途。
 * OUT request with the data in 64-byte packets; false means it stalled after the data
 * stage. Without finish the status IN stays in flight.
 */
bool ControlOut(FakeUsb& dev, Hw& hw, SetupPacket setup, const Bytes& data,
                bool finish = true)
{
  const size_t ready = dev.ready.load();
  dev.Setup(setup);
  TEST_ASSERT(!Stalled(hw));
  size_t sent = 0;
  while (sent < data.size())
  {
    const size_t n = std::min<size_t>(64U, data.size() - sent);
    HostOut(dev, hw.out0,
            Bytes(data.begin() + static_cast<std::ptrdiff_t>(sent),
                  data.begin() + static_cast<std::ptrdiff_t>(sent + n)));
    sent += n;
  }
  if (Stalled(hw))
  {
    TEST_ASSERT(dev.ready.load() == ready);
    return false;
  }
  TEST_ASSERT(hw.in0.inflight && hw.in0.LastLength() == 0U);
  if (finish)
  {
    InDone(dev, hw.in0);
    TEST_ASSERT(dev.ready.load() == ready + 1U);
  }
  return true;
}

void Refused(FakeUsb& dev, Hw& hw, SetupPacket setup)
{
  const size_t ready = dev.ready.load();
  dev.Setup(setup);
  TEST_ASSERT(Stalled(hw));
  TEST_ASSERT(dev.ready.load() == ready);
}

Bytes GetDescriptor(FakeUsb& dev, Hw& hw, uint8_t type, uint8_t index, uint16_t length,
                    uint16_t lang = 0U)
{
  return ControlIn(
      dev, hw, Make(0x80U, 6U, static_cast<uint16_t>(type << 8U | index), lang, length));
}

void SetConfiguration(FakeUsb& dev, Hw& hw, uint8_t value)
{
  ControlNone(dev, hw, Make(0x00U, 9U, value, 0U, 0U));
}

uint8_t GetConfiguration(FakeUsb& dev, Hw& hw)
{
  const Bytes got = ControlIn(dev, hw, Make(0x80U, 8U, 0U, 0U, 1U));
  TEST_ASSERT(got.size() == 1U);
  return got[0];
}

/// 复位、编址、配置 1 / Reset, address, configuration 1
void Reenumerate(FakeUsb& dev, Hw& hw)
{
  dev.Reset();
  TEST_ASSERT(dev.Inited());
  ControlNone(dev, hw, Make(0x00U, 5U, 7U, 0U, 0U));
  SetConfiguration(dev, hw, 1U);
}

void Enumerate(FakeUsb& dev, Hw& hw)
{
  dev.Init(false);
  Reenumerate(dev, hw);
}

// ---- DFU 请求 / DFU requests ----

constexpr uint8_t DETACH = 0x00U;
constexpr uint8_t DNLOAD = 0x01U;
constexpr uint8_t UPLOAD = 0x02U;
constexpr uint8_t GETSTATUS = 0x03U;
constexpr uint8_t CLRSTATUS = 0x04U;
constexpr uint8_t GETSTATE = 0x05U;
constexpr uint8_t ABORT = 0x06U;
constexpr uint8_t RUN_APP = 0x5AU;

constexpr uint8_t S(DFUState state) { return static_cast<uint8_t>(state); }
constexpr uint8_t C(DFUStatusCode code) { return static_cast<uint8_t>(code); }

/// GETSTATUS 的应答 / A GETSTATUS answer
struct Status
{
  uint8_t status;
  uint32_t poll;
  uint8_t state;
};

Status GetStatus(FakeUsb& dev, Hw& hw, uint16_t itf = 0U, bool finish = true)
{
  const Bytes got = ControlIn(dev, hw, Make(0xA1U, GETSTATUS, 0U, itf, 6U), finish);
  TEST_ASSERT(got.size() == 6U);
  return {got[0], static_cast<uint32_t>(got[1] | got[2] << 8U | got[3] << 16U), got[4]};
}

uint8_t GetState(FakeUsb& dev, Hw& hw, uint16_t itf = 0U)
{
  const Bytes got = ControlIn(dev, hw, Make(0xA1U, GETSTATE, 0U, itf, 1U));
  TEST_ASSERT(got.size() == 1U);
  return got[0];
}

bool Dnload(FakeUsb& dev, Hw& hw, uint16_t block, const Bytes& data, bool finish = true)
{
  const SetupPacket setup =
      Make(0x21U, DNLOAD, block, 0U, static_cast<uint16_t>(data.size()));
  if (data.empty())
  {
    ControlNone(dev, hw, setup, finish);
    return true;
  }
  return ControlOut(dev, hw, setup, data, finish);
}

Bytes Upload(FakeUsb& dev, Hw& hw, uint16_t block, uint16_t length)
{
  return ControlIn(dev, hw, Make(0xA1U, UPLOAD, block, 0U, length));
}

void ClearStatus(FakeUsb& dev, Hw& hw)
{
  ControlNone(dev, hw, Make(0x21U, CLRSTATUS, 0U, 0U, 0U));
}

Bytes Pattern(size_t size, uint8_t seed)
{
  Bytes bytes(size);
  for (size_t i = 0; i < size; ++i)
  {
    bytes[i] = static_cast<uint8_t>(seed + i * 13U);
  }
  return bytes;
}

/// 主机等 bwPollTimeout 再问，直到不再是 busy 状态；每次等之前调用 process /
/// The host waits bwPollTimeout and asks again until the state is no longer busy;
/// process runs before each wait
Status PollUntilIdle(FakeUsb& dev, Hw& hw, Status status,
                     const std::function<void()>& process)
{
  for (size_t guard = 0; status.state == S(DFUState::DFU_DNBUSY) ||
                         status.state == S(DFUState::DFU_MANIFEST);
       ++guard)
  {
    TEST_ASSERT(guard < 1000U);
    process();
    SleepMs(status.poll);
    status = GetStatus(dev, hw);
  }
  return status;
}

/// 按 DFU 1.1 下载整个镜像并 manifest，返回最后一次 GETSTATUS / Download a whole image
/// as DFU 1.1 says and manifest it; returns the last GETSTATUS
Status DownloadImage(FakeUsb& dev, Hw& hw, const Bytes& image, size_t block,
                     const std::function<void()>& process)
{
  uint16_t number = 0;
  for (size_t offset = 0; offset < image.size(); offset += block, ++number)
  {
    const size_t n = std::min(block, image.size() - offset);
    TEST_ASSERT(Dnload(dev, hw, number,
                       Bytes(image.begin() + static_cast<std::ptrdiff_t>(offset),
                             image.begin() + static_cast<std::ptrdiff_t>(offset + n))));
    const Status status = PollUntilIdle(dev, hw, GetStatus(dev, hw), process);
    TEST_ASSERT(status.status == 0U && status.state == S(DFUState::DFU_DNLOAD_IDLE));
  }
  TEST_ASSERT(Dnload(dev, hw, number, {}));
  return PollUntilIdle(dev, hw, GetStatus(dev, hw), process);
}

/// 测试用 bootloader：Flash 4096 字节，擦除 256，写 8；镜像区 2304 字节，seal 在 2048 /
/// Test bootloader: 4096 bytes of flash, erase 256, write 8; image region of 2304 bytes,
/// seal at 2048
using Boot = DfuBootloaderClassT<256U>;

struct BootFixture
{
  explicit BootFixture(bool autorun = false)
      : boot(flash, 0, 2304, 2048, &JumpToApp, nullptr, autorun)
  {
  }

  Bytes flash_mem = Bytes(4096U, 0U);
  RamFlash flash{256U, 8U, {flash_mem.data(), flash_mem.size()}};
  Boot boot;
  Hw hw;
};

/// 可设 DfuSetAlternate 结果的最小后端 / Minimal backend with a settable
/// DfuSetAlternate result
struct FakeBackend
{
  DFUCapabilities GetDfuCapabilities() const { return {}; }
  ErrorCode DfuSetAlternate(uint8_t) { return set_alt_result; }
  void DfuAbort(uint8_t) { ++aborts; }
  void DfuClearStatus(uint8_t) {}
  DFUStatusCode DfuDownload(uint8_t, uint16_t, ConstRawData, uint32_t&)
  {
    return DFUStatusCode::OK;
  }
  DFUStatusCode DfuGetDownloadStatus(uint8_t, bool& busy, uint32_t&)
  {
    busy = false;
    return DFUStatusCode::OK;
  }
  size_t DfuUpload(uint8_t, uint16_t, RawData, DFUStatusCode& status, uint32_t&)
  {
    status = DFUStatusCode::OK;
    return 0U;
  }
  DFUStatusCode DfuManifest(uint8_t, uint32_t&) { return DFUStatusCode::OK; }
  DFUStatusCode DfuGetManifestStatus(uint8_t, bool& busy, uint32_t&)
  {
    busy = false;
    return DFUStatusCode::OK;
  }
  void AllowPendingWork() {}
  void CommitWorkResult() {}
  void DfuCommitManifestWaitReset(uint8_t) {}
  bool RunPendingWork() { return false; }

  ErrorCode set_alt_result = ErrorCode::OK;
  size_t aborts = 0;
};

// ---- 用例 / Cases ----

/// 描述符：bootloader 改写设备描述符；运行时、bootloader 的功能描述符 / Descriptors:
/// the bootloader rewrites the device descriptor; runtime and bootloader functional
/// descriptors
void TestEnumeration()
{
  {
    BootFixture f;
    FakeUsb dev(f.hw, {{&f.boot}});
    dev.Init(false);
    dev.Reset();

    // 单功能 bootloader：设备类 0xFE/1/2 / Single-function bootloader: device class
    // 0xFE/1/2
    const Bytes device = GetDescriptor(dev, f.hw, 1U, 0U, 18U);
    TEST_ASSERT(device[4] == 0xFEU && device[5] == 0x01U && device[6] == 0x02U);

    // 配置 27 字节：接口（0 个端点，0xFE/1/2，字符串 4）+ 功能描述符（下载、上传，不容忍
    // manifest，wTransferSize 截到 256，DFU 1.1）
    // Configuration of 27 bytes: interface (no endpoint, 0xFE/1/2, string 4) and the
    // functional descriptor (download, upload, not manifestation tolerant,
    // wTransferSize clipped to 256, DFU 1.1)
    const Bytes config = GetDescriptor(dev, f.hw, 2U, 0U, 255U);
    TEST_ASSERT(config.size() == 27U && config[2] == 27U && config[4] == 1U);
    TEST_ASSERT(config[9] == 9U && config[10] == 4U && config[11] == 0U &&
                config[13] == 0U);
    TEST_ASSERT(config[14] == 0xFEU && config[15] == 0x01U && config[16] == 0x02U);
    TEST_ASSERT(config[17] == 4U);
    const Bytes functional(config.begin() + 18, config.end());
    TEST_ASSERT(
        (functional == Bytes{9U, 0x21U, 0x03U, 0U, 0U, 0x00U, 0x01U, 0x10U, 0x01U}));
    const Bytes name = GetDescriptor(dev, f.hw, 3U, 4U, 255U, 0x0409U);
    TEST_ASSERT(name.size() == 2U + 2U * 9U && name[2] == 'X' && name[14] == 'D');

    // 未配置时类请求、类描述符都找不到类 / Before configuration neither class requests
    // nor the class descriptor find the class
    Refused(dev, f.hw, Make(0xA1U, GETSTATUS, 0U, 0U, 6U));
    Refused(dev, f.hw, Make(0x81U, 6U, 0x2100U, 0U, 9U));

    ControlNone(dev, f.hw, Make(0x00U, 5U, 3U, 0U, 0U));
    SetConfiguration(dev, f.hw, 1U);
    TEST_ASSERT(ControlIn(dev, f.hw, Make(0x81U, 6U, 0x2100U, 0U, 9U)) == functional);
    // 其他类型的类描述符被拒绝 / Other class descriptor types are refused
    Refused(dev, f.hw, Make(0x81U, 6U, 0x2200U, 0U, 9U));
    // GET_INTERFACE 由核心答 0；SET_INTERFACE 只接受 0 / GET_INTERFACE is answered 0
    // by the core; SET_INTERFACE accepts 0 only
    TEST_ASSERT(ControlIn(dev, f.hw, Make(0x81U, 10U, 0U, 0U, 1U)) == Bytes{0U});
    ControlNone(dev, f.hw, Make(0x01U, 11U, 0U, 0U, 0U));
    Refused(dev, f.hw, Make(0x01U, 11U, 1U, 0U, 0U));
    TEST_ASSERT(GetState(dev, f.hw) == S(DFUState::DFU_IDLE));
  }

  {
    // 运行时：协议 1，bitWillDetach，wDetachTimeOut 为构造值，wTransferSize 0；设备
    // 描述符不改
    // Runtime: protocol 1, bitWillDetach, wDetachTimeOut from the constructor,
    // wTransferSize 0; the device descriptor is not rewritten
    Hw hw;
    DfuRuntimeClass rt(&JumpToBootloader, nullptr, 20U);
    FakeUsb dev(hw, {{&rt}});
    dev.Init(false);
    dev.Reset();
    TEST_ASSERT(GetDescriptor(dev, hw, 1U, 0U, 18U)[4] != 0xFEU);
    const Bytes config = GetDescriptor(dev, hw, 2U, 0U, 255U);
    TEST_ASSERT(config.size() == 27U && config[16] == 0x01U);
    TEST_ASSERT((Bytes(config.begin() + 18, config.end()) ==
                 Bytes{9U, 0x21U, 0x08U, 20U, 0U, 0U, 0U, 0x10U, 0x01U}));
  }

  {
    // 没有跳转函数的运行时不声明 bitWillDetach / A runtime without a jump function
    // does not set bitWillDetach
    Hw hw;
    DfuRuntimeClass rt(nullptr);
    FakeUsb dev(hw, {{&rt}});
    dev.Init(false);
    dev.Reset();
    const Bytes config = GetDescriptor(dev, hw, 2U, 0U, 255U);
    TEST_ASSERT(config[20] == 0U && config[21] == 0U && config[22] == 0U);
  }
}

/// 运行时的请求 / Runtime requests
void TestRuntimeRequests()
{
  Hw hw;
  DfuRuntimeClass rt(&JumpToBootloader, nullptr, 20U);
  FakeUsb dev(hw, {{&rt}});
  Enumerate(dev, hw);

  // 别的接口号：核心找不到类 / Another interface number: the core finds no class
  Refused(dev, hw, Make(0xA1U, GETSTATUS, 0U, 2U, 6U));

  // DETACH 没有数据阶段，核心发状态 IN / DETACH has no data stage; the core sends the
  // status IN
  TEST_ASSERT(GetStatus(dev, hw).state == S(DFUState::APP_IDLE));
  ControlNone(dev, hw, Make(0x21U, DETACH, 0U, 0U, 0U));
  const Status status = GetStatus(dev, hw);
  TEST_ASSERT(status.status == 0U && status.state == S(DFUState::APP_DETACH));
  TEST_ASSERT(status.poll <= 20U);
  TEST_ASSERT(GetState(dev, hw) == S(DFUState::APP_DETACH));
  // APP_DETACH 中再 DETACH 被拒绝 / DETACH again in APP_DETACH is refused
  Refused(dev, hw, Make(0x21U, DETACH, 0U, 0U, 0U));

  // 同一配置再设一次：解绑再绑定，回到 APP_IDLE / The same configuration again: unbind
  // and bind, back to APP_IDLE
  SetConfiguration(dev, hw, 1U);
  TEST_ASSERT(GetState(dev, hw) == S(DFUState::APP_IDLE));

  // 形状不对的请求在 SETUP 阶段被拒绝 / Requests of a wrong shape are refused at SETUP
  Refused(dev, hw, Make(0x21U, DETACH, 0U, 0U, 4U));       // 带数据阶段 / with data
  Refused(dev, hw, Make(0x21U, 0x7FU, 0U, 0U, 0U));        // 未知请求 / unknown
  Refused(dev, hw, Make(0x21U, GETSTATUS, 0U, 0U, 6U));    // 方向 / direction
  Refused(dev, hw, Make(0xA1U, GETSTATUS, 0U, 0U, 5U));    // 长度 / length
  Refused(dev, hw, Make(0xA1U, GETSTATE, 0U, 0U, 2U));     // 长度 / length
  Refused(dev, hw, Make(0x41U, DETACH, 0U, 0U, 0U));       // 厂商 / vendor
  Refused(dev, hw, Make(0x21U, DNLOAD, 0U, 0U, 0U));       // 运行时没有 / not runtime
  Refused(dev, hw, Make(0x21U, DETACH, 0U, 0x0100U, 0U));  // wIndex 高字节 / high byte
  TEST_ASSERT(GetState(dev, hw) == S(DFUState::APP_IDLE));

  // 没有跳转函数时不提供 DETACH / Without a jump function DETACH is not offered
  Hw hw2;
  DfuRuntimeClass plain(nullptr);
  FakeUsb dev2(hw2, {{&plain}});
  Enumerate(dev2, hw2);
  Refused(dev2, hw2, Make(0x21U, DETACH, 0U, 0U, 0U));
  TEST_ASSERT(GetState(dev2, hw2) == S(DFUState::APP_IDLE));
}

/**
 * 运行时的跳转只由 DETACH 的状态阶段完成触发，中止的各条路径不触发。超时本身被测，
 * 所以"刚完成时还没到期"一步测量间隔，被调度出去时换新对象重来。
 * The runtime jump is triggered only by the completed DETACH status stage and by none of
 * the abort paths. The timeout itself is tested, so the "not yet expired right after
 * completion" step measures the gap and starts over with fresh objects when the thread
 * was descheduled.
 */
void TestRuntimeJump()
{
  bool clean = false;
  for (unsigned attempt = 0; attempt < 20U && !clean; ++attempt)
  {
    g_boot_jumps = 0U;
    Hw hw;
    DfuRuntimeClass rt(&JumpToBootloader, nullptr, 20U);
    FakeUsb dev(hw, {{&rt}});
    Enumerate(dev, hw);

    rt.Process();
    TEST_ASSERT(g_boot_jumps == 0U);
    // 状态 IN 留在途 30 ms：超时从状态阶段完成算起，不跳 / The status IN stays in
    // flight for 30 ms: the timeout counts from its completion, no jump
    ControlNone(dev, hw, Make(0x21U, DETACH, 0U, 0U, 0U), false);
    SleepMs(30U);
    rt.Process();
    TEST_ASSERT(g_boot_jumps == 0U);
    const auto begin = std::chrono::steady_clock::now();
    InDone(dev, hw.in0);
    rt.Process();
    if (std::chrono::steady_clock::now() - begin >= std::chrono::milliseconds(10))
    {
      continue;
    }
    clean = true;
    TEST_ASSERT(g_boot_jumps == 0U);
    TEST_ASSERT(WaitUntil(
        [&]()
        {
          rt.Process();
          return g_boot_jumps != 0U;
        }));
    TEST_ASSERT(g_boot_jumps == 1U);
    rt.Process();
    TEST_ASSERT(g_boot_jumps == 1U);
  }
  TEST_ASSERT(clean);

  // wValue 给的超时优先于默认值；GETSTATUS 报剩下的时间 / The timeout in wValue wins
  // over the default; GETSTATUS reports what is left
  {
    g_boot_jumps = 0U;
    Hw hw;
    DfuRuntimeClass rt(&JumpToBootloader, nullptr, 60000U);
    FakeUsb dev(hw, {{&rt}});
    Enumerate(dev, hw);
    ControlNone(dev, hw, Make(0x21U, DETACH, 5U, 0U, 0U));
    TEST_ASSERT(GetStatus(dev, hw).poll <= 5U);
    TEST_ASSERT(WaitUntil(
        [&]()
        {
          rt.Process();
          return g_boot_jumps != 0U;
        },
        2000U));
  }

  // 状态 IN 被新的 SETUP 取代：OnControlAbort，回到 APP_IDLE，永远不跳 / The status IN
  // is superseded by a new SETUP: OnControlAbort, back to APP_IDLE, never a jump
  {
    g_boot_jumps = 0U;
    Hw hw;
    DfuRuntimeClass rt(&JumpToBootloader, nullptr, 5U);
    FakeUsb dev(hw, {{&rt}});
    Enumerate(dev, hw);
    ControlNone(dev, hw, Make(0x21U, DETACH, 0U, 0U, 0U), false);
    TEST_ASSERT(GetState(dev, hw) == S(DFUState::APP_IDLE));
    SleepMs(20U);
    rt.Process();
    TEST_ASSERT(g_boot_jumps == 0U);
    // 再来一次 DETACH 是允许的 / Another DETACH is allowed
    ControlNone(dev, hw, Make(0x21U, DETACH, 0U, 0U, 0U));
    TEST_ASSERT(WaitUntil(
        [&]()
        {
          rt.Process();
          return g_boot_jumps != 0U;
        },
        2000U));
  }

  // DETACH 完成后总线复位：解绑取消跳转 / Bus reset after a completed DETACH: the
  // unbind cancels the jump
  {
    g_boot_jumps = 0U;
    Hw hw;
    DfuRuntimeClass rt(&JumpToBootloader, nullptr, 5U);
    FakeUsb dev(hw, {{&rt}});
    Enumerate(dev, hw);
    ControlNone(dev, hw, Make(0x21U, DETACH, 0U, 0U, 0U));
    Reenumerate(dev, hw);
    SleepMs(20U);
    rt.Process();
    TEST_ASSERT(g_boot_jumps == 0U);
    TEST_ASSERT(GetState(dev, hw) == S(DFUState::APP_IDLE));
  }
}

/// 下载、manifest、上传、RUN_APP 全流程 / The full download, manifestation, upload
/// and RUN_APP flow
void TestDownload()
{
  g_app_jumps = 0U;
  BootFixture f;
  Hw& hw = f.hw;
  FakeUsb dev(hw, {{&f.boot}});
  Enumerate(dev, hw);

  Status status = GetStatus(dev, hw);
  TEST_ASSERT(status.status == 0U && status.state == S(DFUState::DFU_IDLE));

  // 没有镜像时 UPLOAD 被拒绝并锁存错误；CLRSTATUS 回到空闲
  // Without an image UPLOAD is refused and latches the error; CLRSTATUS returns to idle
  Refused(dev, hw, Make(0xA1U, UPLOAD, 0U, 0U, 32U));
  status = GetStatus(dev, hw);
  TEST_ASSERT(status.status == C(DFUStatusCode::ERR_FIRMWARE) &&
              status.state == S(DFUState::DFU_ERROR));
  ClearStatus(dev, hw);
  TEST_ASSERT(GetState(dev, hw) == S(DFUState::DFU_IDLE));
  // 空闲时零长度 DNLOAD 是协议错误 / A zero-length DNLOAD while idle is a protocol error
  TEST_ASSERT(Dnload(dev, hw, 0U, {}));
  TEST_ASSERT(GetState(dev, hw) == S(DFUState::DFU_ERROR));
  status = GetStatus(dev, hw);
  TEST_ASSERT(status.status == C(DFUStatusCode::ERR_NOTDONE) &&
              status.state == S(DFUState::DFU_ERROR));
  ClearStatus(dev, hw);

  // 块 0：256 字节、四个包，核心收齐后交给类 / Block 0: 256 bytes in four packets,
  // handed to the class once the core has them all
  const Bytes image = Pattern(612U, 0x11U);
  const Bytes block0(image.begin(), image.begin() + 256);
  TEST_ASSERT(Dnload(dev, hw, 0U, block0));
  TEST_ASSERT(GetState(dev, hw) == S(DFUState::DFU_DNLOAD_SYNC));

  // 主机确认 GETSTATUS 之前什么都不写；确认之后也只在 Process() 里写
  // Nothing is written before the host acknowledged GETSTATUS; afterwards only inside
  // Process()
  status = GetStatus(dev, hw);
  TEST_ASSERT(status.state == S(DFUState::DFU_DNBUSY) && status.poll == 25U);
  TEST_ASSERT(f.flash.writes == 0U && f.boot.HasPendingWork());
  SleepMs(status.poll);
  status = GetStatus(dev, hw);
  TEST_ASSERT(status.state == S(DFUState::DFU_DNBUSY));
  TEST_ASSERT(std::all_of(f.flash_mem.begin(), f.flash_mem.begin() + 256,
                          [](uint8_t b) { return b == 0U; }));
  f.boot.Process();
  TEST_ASSERT(f.flash.writes == 1U && !f.boot.HasPendingWork());
  SleepMs(status.poll);
  status = GetStatus(dev, hw);
  TEST_ASSERT(status.status == 0U && status.state == S(DFUState::DFU_DNLOAD_IDLE));
  TEST_ASSERT(std::equal(block0.begin(), block0.end(), f.flash_mem.begin()));

  // 其余两块，最后一块短；然后零长度 DNLOAD，manifest 算 CRC、写 seal
  // The other two blocks, the last one short; then the zero-length DNLOAD, and
  // manifestation computes the CRC and writes the seal
  const auto process = [&]() { f.boot.Process(); };
  for (uint16_t block = 1; block < 3U; ++block)
  {
    const size_t offset = block * 256U;
    const size_t n = std::min<size_t>(256U, image.size() - offset);
    TEST_ASSERT(Dnload(dev, hw, block,
                       Bytes(image.begin() + static_cast<std::ptrdiff_t>(offset),
                             image.begin() + static_cast<std::ptrdiff_t>(offset + n))));
    status = PollUntilIdle(dev, hw, GetStatus(dev, hw), process);
    TEST_ASSERT(status.state == S(DFUState::DFU_DNLOAD_IDLE));
  }
  TEST_ASSERT(Dnload(dev, hw, 3U, {}));
  TEST_ASSERT(GetState(dev, hw) == S(DFUState::DFU_MANIFEST_SYNC));
  status = GetStatus(dev, hw);
  TEST_ASSERT(status.state == S(DFUState::DFU_MANIFEST) && status.poll == 50U);
  status = PollUntilIdle(dev, hw, status, process);
  TEST_ASSERT(status.status == 0U &&
              status.state == S(DFUState::DFU_MANIFEST_WAIT_RESET));
  TEST_ASSERT(f.boot.HasValidImage() && f.boot.ImageSize() == image.size());
  TEST_ASSERT(std::equal(image.begin(), image.end(), f.flash_mem.begin()));
  DfuBootloaderBackend::SealRecord seal{};
  std::memcpy(&seal, f.flash_mem.data() + 2048, sizeof(seal));
  uint32_t crc = 0xFFFFFFFFU;
  for (uint8_t b : image)
  {
    crc = LibXR::CRC32::tab_[(crc ^ b) & 0xFFU] ^ (crc >> 8U);
  }
  TEST_ASSERT(seal.magic == DfuBootloaderBackend::SEAL_MAGIC &&
              seal.image_size == image.size() && seal.crc32 == crc &&
              seal.crc32_inv == ~crc);
  // autorun 关闭：WAIT_RESET 不登记启动 / autorun off: WAIT_RESET records no launch
  TEST_ASSERT(!f.boot.TryConsumeAppLaunch(0U) && g_app_jumps == 0U);

  // 复位后上传：两个满块和一个短块，短块结束上传 / Upload after a reset: two full blocks
  // and a short one that ends the upload
  Reenumerate(dev, hw);
  Bytes back;
  for (uint16_t block = 0; block < 3U; ++block)
  {
    const Bytes got = Upload(dev, hw, block, 256U);
    back.insert(back.end(), got.begin(), got.end());
  }
  TEST_ASSERT(back == image);
  TEST_ASSERT(GetState(dev, hw) == S(DFUState::DFU_IDLE));
  // 跳号的 UPLOAD 被拒绝 / An UPLOAD that skips a number is refused
  TEST_ASSERT(Upload(dev, hw, 0U, 64U).size() == 64U);
  Refused(dev, hw, Make(0xA1U, UPLOAD, 5U, 0U, 64U));
  status = GetStatus(dev, hw);
  TEST_ASSERT(status.status == C(DFUStatusCode::ERR_ADDRESS));
  ClearStatus(dev, hw);

  // RUN_APP：状态阶段完成后才登记，应用取走时跳转一次 / RUN_APP: recorded only once the
  // status stage completed; the application jumps once when it takes it
  Refused(dev, hw, Make(0x41U, RUN_APP, 1U, 0U, 0U));
  ControlNone(dev, hw, Make(0x41U, RUN_APP, 0U, 0U, 0U));
  TEST_ASSERT(f.boot.TryConsumeAppLaunch(0U) && g_app_jumps == 1U);
  TEST_ASSERT(!f.boot.TryConsumeAppLaunch(0U) && g_app_jumps == 1U);

  // 其他拒绝 / Other refusals
  Refused(dev, hw, Make(0x21U, DNLOAD, 0U, 0U, 257U));  // 超过 wTransferSize
  ClearStatus(dev, hw);
  Refused(dev, hw, Make(0x21U, DETACH, 0U, 0U, 0U));  // 属于运行时 / runtime's
  ClearStatus(dev, hw);
  Refused(dev, hw, Make(0xA1U, GETSTATUS, 0U, 9U, 6U));  // 别的接口 / other interface
  Refused(dev, hw, Make(0x21U, CLRSTATUS, 0U, 0U, 0U));  // 不在错误中 / not in error
  ClearStatus(dev, hw);
  Refused(dev, hw, Make(0x41U, 0x5BU, 0U, 0U, 0U));  // 别的厂商请求 / other vendor

  // 新构造的后端从 Flash 认出已存镜像 / A newly constructed backend recognizes the stored
  // image from the flash
  DfuBootloaderBackend again(f.flash, 0, 2304, 2048, &JumpToApp);
  TEST_ASSERT(again.HasValidImage() && again.ImageSize() == image.size());
  // 镜像被破坏时不认 / A damaged image is not recognized
  f.flash_mem[100] ^= 0xFFU;
  DfuBootloaderBackend damaged(f.flash, 0, 2304, 2048, &JumpToApp);
  TEST_ASSERT(!damaged.HasValidImage());
}

/// 轮询等待从应答算起 / The poll wait counts from the answer
void TestPollWait()
{
  BootFixture f;
  Hw& hw = f.hw;
  FakeUsb dev(hw, {{&f.boot}});
  Enumerate(dev, hw);
  const Bytes block0(256U, 0xA5U);

  // 块 0 加一个 GETSTATUS；状态阶段留着 / Block 0 and one GETSTATUS; the status stage
  // is left open
  auto download_block0 = [&]()
  {
    TEST_ASSERT(Dnload(dev, hw, 0U, block0));
    const Status status = GetStatus(dev, hw, 0U, false);
    TEST_ASSERT(status.state == S(DFUState::DFU_DNBUSY));
    return status.poll;
  };
  // 状态阶段迟到（像擦写关中断时那样），主机在应答之后正好等 bwPollTimeout 回来：不
  // STALL，已写完
  // The status stage completes late (as when flash work blocks interrupts), then the
  // host comes back exactly bwPollTimeout after the answer: not stalled, and done
  auto late_status_stage = [&](uint32_t reported)
  {
    SleepMs(reported / 2U);
    StatusOut(dev, hw);
    f.boot.Process();
    SleepMs(reported - reported / 2U);
    const Status status = GetStatus(dev, hw);
    TEST_ASSERT(status.state == S(DFUState::DFU_DNLOAD_IDLE));
  };

  // 还没实测时，要擦除的一步报 25 ms；马上回来太早：STALL，进入错误
  // Before any measurement a step that erases reports 25 ms; coming back at once is too
  // early: stalled, in error
  TEST_ASSERT(download_block0() == 25U);
  Refused(dev, hw, Make(0xA1U, GETSTATUS, 0U, 0U, 6U));
  const Status error = GetStatus(dev, hw);
  TEST_ASSERT(error.state == S(DFUState::DFU_ERROR) &&
              error.status == C(DFUStatusCode::ERR_STALLEDPKT));
  ClearStatus(dev, hw);
  // 被 STALL 打断的那次没有放行任何工作 / The interrupted one released no work
  f.boot.Process();
  TEST_ASSERT(f.flash.writes == 0U);

  TEST_ASSERT(download_block0() == 25U);
  late_status_stage(25U);

  // ABORT 后块 0 要重新擦除，报实测值（RAM Flash 瞬间完成，取下限 10 ms）
  // After ABORT block 0 needs its erase again, reported with the measured value
  // (instant on RAM flash, so the 10 ms floor)
  ControlNone(dev, hw, Make(0x21U, ABORT, 0U, 0U, 0U));
  TEST_ASSERT(download_block0() == 10U);
  late_status_stage(10U);
}

/// DFU 依赖的控制完成回调：只有被主机确认的应答才有后果 / The control completion
/// callbacks DFU depends on: only an answer the host acknowledged has consequences
void TestStatusStage()
{
  // (a) 应答 DNBUSY 的 GETSTATUS 的状态阶段被下一个 GETSTATUS 取代：不放行；下一个
  //     被确认后才放行
  // (a) The status stage of a GETSTATUS that answered DNBUSY is superseded by the next
  //     GETSTATUS: nothing is released; only the acknowledged next one releases
  {
    BootFixture f;
    FakeUsb dev(f.hw, {{&f.boot}});
    Enumerate(dev, f.hw);
    TEST_ASSERT(Dnload(dev, f.hw, 0U, Bytes(256U, 0x3CU)));
    Status status = GetStatus(dev, f.hw, 0U, false);
    TEST_ASSERT(status.state == S(DFUState::DFU_DNBUSY));
    f.boot.Process();
    TEST_ASSERT(f.flash.writes == 0U);
    SleepMs(status.poll);
    status = GetStatus(dev, f.hw);
    TEST_ASSERT(status.state == S(DFUState::DFU_DNBUSY) && status.poll == 25U);
    f.boot.Process();
    TEST_ASSERT(f.flash.writes == 1U && f.flash_mem[0] == 0x3CU);
    status = PollUntilIdle(dev, f.hw, status, [] {});
    TEST_ASSERT(status.state == S(DFUState::DFU_DNLOAD_IDLE));
  }

  // (b) 同样的应答被总线复位中止：工作作废，会话清空 / The same answer aborted by a
  //     bus reset: the work is voided, the session cleared
  {
    BootFixture f;
    FakeUsb dev(f.hw, {{&f.boot}});
    Enumerate(dev, f.hw);
    TEST_ASSERT(Dnload(dev, f.hw, 0U, Bytes(256U, 0x3CU)));
    TEST_ASSERT(GetStatus(dev, f.hw, 0U, false).state == S(DFUState::DFU_DNBUSY));
    Reenumerate(dev, f.hw);
    TEST_ASSERT(!f.boot.HasPendingWork());
    f.boot.Process();
    TEST_ASSERT(f.flash.writes == 0U);
    TEST_ASSERT(GetState(dev, f.hw) == S(DFUState::DFU_IDLE));
    // 会话从块 0 重来 / The session restarts from block 0
    TEST_ASSERT(Dnload(dev, f.hw, 1U, Bytes(256U, 0x3CU)));
    const Status status = GetStatus(dev, f.hw);
    TEST_ASSERT(status.status == C(DFUStatusCode::ERR_ADDRESS) &&
                status.state == S(DFUState::DFU_ERROR));
  }

  // (c) autorun：报告 MANIFEST-WAIT-RESET 的 GETSTATUS 被确认才登记启动；登记后的总线
  //     复位不取消它
  // (c) autorun: the launch is recorded only when the GETSTATUS reporting
  //     MANIFEST-WAIT-RESET is acknowledged; a bus reset after that does not cancel it
  {
    g_app_jumps = 0U;
    BootFixture f(true);
    FakeUsb dev(f.hw, {{&f.boot}});
    Enumerate(dev, f.hw);
    const auto process = [&]() { f.boot.Process(); };
    const Bytes image = Pattern(300U, 0x42U);
    TEST_ASSERT(Dnload(dev, f.hw, 0U, Bytes(image.begin(), image.begin() + 256)));
    TEST_ASSERT(PollUntilIdle(dev, f.hw, GetStatus(dev, f.hw), process).state ==
                S(DFUState::DFU_DNLOAD_IDLE));
    TEST_ASSERT(Dnload(dev, f.hw, 1U, Bytes(image.begin() + 256, image.end())));
    TEST_ASSERT(PollUntilIdle(dev, f.hw, GetStatus(dev, f.hw), process).state ==
                S(DFUState::DFU_DNLOAD_IDLE));
    TEST_ASSERT(Dnload(dev, f.hw, 2U, {}));
    Status status = GetStatus(dev, f.hw);
    TEST_ASSERT(status.state == S(DFUState::DFU_MANIFEST));
    process();
    SleepMs(status.poll);
    // 报告 WAIT_RESET 的那次不确认，被 GETSTATE 取代 / The one reporting WAIT_RESET
    // is not acknowledged and is superseded by GETSTATE
    status = GetStatus(dev, f.hw, 0U, false);
    TEST_ASSERT(status.state == S(DFUState::DFU_MANIFEST_WAIT_RESET));
    TEST_ASSERT(GetState(dev, f.hw) == S(DFUState::DFU_MANIFEST_WAIT_RESET));
    TEST_ASSERT(!f.boot.TryConsumeAppLaunch(0U) && g_app_jumps == 0U);
    // 这次确认 / This time it is acknowledged
    status = GetStatus(dev, f.hw, 0U, false);
    TEST_ASSERT(status.state == S(DFUState::DFU_MANIFEST_WAIT_RESET));
    TEST_ASSERT(!f.boot.TryConsumeAppLaunch(0U));
    StatusOut(dev, f.hw);
    // 主机随即复位；应用的循环在复位之后才来取 / The host resets at once; the
    // application loop comes for it only after the reset
    Reenumerate(dev, f.hw);
    TEST_ASSERT(GetState(dev, f.hw) == S(DFUState::DFU_IDLE));
    TEST_ASSERT(f.boot.TryConsumeAppLaunch(0U) && g_app_jumps == 1U);
    // ABORT 也不取消 RUN_APP 登记的启动 / ABORT does not cancel a launch recorded by
    // RUN_APP either
    ControlNone(dev, f.hw, Make(0x41U, RUN_APP, 0U, 0U, 0U));
    ControlNone(dev, f.hw, Make(0x21U, ABORT, 0U, 0U, 0U));
    TEST_ASSERT(f.boot.TryConsumeAppLaunch(0U) && g_app_jumps == 2U);
    // 新的下载会话取消它 / A new download session cancels it
    ControlNone(dev, f.hw, Make(0x41U, RUN_APP, 0U, 0U, 0U));
    TEST_ASSERT(Dnload(dev, f.hw, 0U, Bytes(16U, 0x01U)));
    TEST_ASSERT(!f.boot.HasValidImage() && !f.boot.TryConsumeAppLaunch(0U));
  }

  // (d) RUN_APP 的状态 IN 被取代：不登记 / The RUN_APP status IN is superseded: nothing
  //     recorded
  {
    g_app_jumps = 0U;
    BootFixture f;
    FakeUsb dev(f.hw, {{&f.boot}});
    Enumerate(dev, f.hw);
    TEST_ASSERT(
        DownloadImage(dev, f.hw, Pattern(100U, 7U), 256U, [&]() { f.boot.Process(); })
            .state == S(DFUState::DFU_MANIFEST_WAIT_RESET));
    Reenumerate(dev, f.hw);
    ControlNone(dev, f.hw, Make(0x41U, RUN_APP, 0U, 0U, 0U), false);
    TEST_ASSERT(GetState(dev, f.hw) == S(DFUState::DFU_IDLE));
    TEST_ASSERT(!f.boot.TryConsumeAppLaunch(0U) && g_app_jumps == 0U);
    // 有未完成工作时 RUN_APP 被拒绝 / RUN_APP is refused while work is pending
    TEST_ASSERT(Dnload(dev, f.hw, 0U, Bytes(16U, 0x01U)));
    Refused(dev, f.hw, Make(0x41U, RUN_APP, 0U, 0U, 0U));
  }

  // (e) DNLOAD 数据阶段短了：STALL，错误留给 GETSTATUS；CLRSTATUS 后照常下载
  // (e) A short DNLOAD data stage: stalled, the error is kept for GETSTATUS; after
  //     CLRSTATUS the download works as usual
  {
    BootFixture f;
    FakeUsb dev(f.hw, {{&f.boot}});
    Enumerate(dev, f.hw);
    TEST_ASSERT(
        !ControlOut(dev, f.hw, Make(0x21U, DNLOAD, 0U, 0U, 128U), Bytes(70U, 1U)));
    Status status = GetStatus(dev, f.hw);
    TEST_ASSERT(status.status == C(DFUStatusCode::ERR_STALLEDPKT) &&
                status.state == S(DFUState::DFU_ERROR));
    TEST_ASSERT(!f.boot.HasPendingWork());
    ClearStatus(dev, f.hw);
    TEST_ASSERT(Dnload(dev, f.hw, 0U, Bytes(128U, 2U)));
    status = PollUntilIdle(dev, f.hw, GetStatus(dev, f.hw), [&]() { f.boot.Process(); });
    TEST_ASSERT(status.state == S(DFUState::DFU_DNLOAD_IDLE) && f.flash_mem[127] == 2U);
  }

  // (f) DNLOAD 的状态 IN 被取代：暂存的块作废，回到 DFU_IDLE / The DNLOAD status IN is
  //     superseded: the staged block is voided, back to DFU_IDLE
  {
    BootFixture f;
    FakeUsb dev(f.hw, {{&f.boot}});
    Enumerate(dev, f.hw);
    TEST_ASSERT(Dnload(dev, f.hw, 0U, Bytes(64U, 9U), false));
    TEST_ASSERT(f.boot.HasPendingWork());
    TEST_ASSERT(GetState(dev, f.hw) == S(DFUState::DFU_IDLE));
    TEST_ASSERT(!f.boot.HasPendingWork());
    f.boot.Process();
    TEST_ASSERT(f.flash.writes == 0U);
  }
}

/// 绑定失败 / Bind failure
void TestBindFail()
{
  // 后端不接受 alt 0：SET_CONFIGURATION 被拒绝，失败的类也被解绑；修好后同一配置可用
  // The backend refuses alt 0: SET_CONFIGURATION is refused and the failing class is
  // unbound too; once fixed the same configuration works
  {
    Hw hw;
    FakeBackend backend;
    backend.set_alt_result = ErrorCode::NOT_SUPPORT;
    DFUClass<FakeBackend, 64U> dfu(backend);
    FakeUsb dev(hw, {{&dfu}});
    dev.Init(false);
    dev.Reset();
    ControlNone(dev, hw, Make(0x00U, 5U, 3U, 0U, 0U));
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(backend.aborts == 1U);
    TEST_ASSERT(GetConfiguration(dev, hw) == 0U);
    Refused(dev, hw, Make(0xA1U, GETSTATUS, 0U, 0U, 6U));
    backend.set_alt_result = ErrorCode::OK;
    SetConfiguration(dev, hw, 1U);
    TEST_ASSERT(GetConfiguration(dev, hw) == 1U);
    TEST_ASSERT(GetStatus(dev, hw).state == S(DFUState::DFU_IDLE));
  }

  // 两个配置：配置 2 中 bootloader 绑定失败，先绑好的运行时被解绑，它等着的跳转取消
  // Two configurations: in configuration 2 the bootloader fails to bind, the runtime
  // bound before it is unbound and its pending jump cancelled
  {
    g_boot_jumps = 0U;
    BootFixture f;
    DfuRuntimeClass rt(&JumpToBootloader, nullptr, 5U);
    FakeBackend backend;
    backend.set_alt_result = ErrorCode::FAILED;
    DFUClass<FakeBackend, 64U> failing(backend);
    FakeUsb dev(f.hw, {{&rt, &f.boot}, {&rt, &failing}});
    Enumerate(dev, f.hw);
    ControlNone(dev, f.hw, Make(0x21U, DETACH, 0U, 0U, 0U));
    TEST_ASSERT(GetState(dev, f.hw, 0U) == S(DFUState::APP_DETACH));
    TEST_ASSERT(GetState(dev, f.hw, 1U) == S(DFUState::DFU_IDLE));
    Refused(dev, f.hw, Make(0x00U, 9U, 2U, 0U, 0U));
    TEST_ASSERT(backend.aborts == 1U);
    TEST_ASSERT(GetConfiguration(dev, f.hw) == 0U);
    SleepMs(20U);
    rt.Process();
    TEST_ASSERT(g_boot_jumps == 0U);
    // 配置 1 照常可用 / Configuration 1 still works
    SetConfiguration(dev, f.hw, 1U);
    TEST_ASSERT(GetState(dev, f.hw, 0U) == S(DFUState::APP_IDLE));
    TEST_ASSERT(GetState(dev, f.hw, 1U) == S(DFUState::DFU_IDLE));
  }
}

/// 带进行中的传输和擦写解绑 / Unbind with transfers and flash work in progress
void TestUnbind()
{
  // (a) DNLOAD 数据阶段收到一半时总线复位 / Bus reset halfway through a DNLOAD data
  //     stage
  {
    BootFixture f;
    FakeUsb dev(f.hw, {{&f.boot}});
    Enumerate(dev, f.hw);
    dev.Setup(Make(0x21U, DNLOAD, 0U, 0U, 256U));
    HostOut(dev, f.hw.out0, Bytes(64U, 1U));
    HostOut(dev, f.hw.out0, Bytes(64U, 1U));
    TEST_ASSERT(f.hw.out0.inflight && !f.boot.HasPendingWork());
    Reenumerate(dev, f.hw);
    TEST_ASSERT(GetState(dev, f.hw) == S(DFUState::DFU_IDLE));
    TEST_ASSERT(Dnload(dev, f.hw, 0U, Bytes(256U, 4U)));
    const Status status =
        PollUntilIdle(dev, f.hw, GetStatus(dev, f.hw), [&]() { f.boot.Process(); });
    TEST_ASSERT(status.state == S(DFUState::DFU_DNLOAD_IDLE) && f.flash_mem[0] == 4U);
  }

  // (b) 放行了还没执行的工作，SET_CONFIGURATION(0)：作废，不写 / Work released but not
  //     run, then SET_CONFIGURATION(0): voided, nothing written
  {
    BootFixture f;
    FakeUsb dev(f.hw, {{&f.boot}});
    Enumerate(dev, f.hw);
    TEST_ASSERT(Dnload(dev, f.hw, 0U, Bytes(256U, 5U)));
    TEST_ASSERT(GetStatus(dev, f.hw).state == S(DFUState::DFU_DNBUSY));
    SetConfiguration(dev, f.hw, 0U);
    TEST_ASSERT(!f.boot.HasPendingWork());
    f.boot.Process();
    TEST_ASSERT(f.flash.writes == 0U && f.flash.erases == 0U);
    SetConfiguration(dev, f.hw, 1U);
    TEST_ASSERT(GetState(dev, f.hw) == S(DFUState::DFU_IDLE));
  }

  // (c) 擦写正在 Process() 里进行时 Deinit：结果作废，工作槽在 Process() 请求的服务里
  //     释放；之后重新枚举，会话从块 0 开始
  // (c) Deinit while the flash work runs inside Process(): the result is voided and the
  //     job slot is freed in the service Process() requests; after enumerating again
  //     the session starts from block 0
  {
    BootFixture f;
    FakeUsb dev(f.hw, {{&f.boot}});
    Enumerate(dev, f.hw);
    TEST_ASSERT(Dnload(dev, f.hw, 0U, Bytes(256U, 6U)));
    TEST_ASSERT(GetStatus(dev, f.hw).state == S(DFUState::DFU_DNBUSY));
    f.flash.hold = true;
    std::thread app([&]() { f.boot.Process(); });
    TEST_ASSERT(WaitUntil([&]() { return f.flash.held.load(); }));
    dev.Deinit(false);
    TEST_ASSERT(!dev.Inited() && f.boot.HasPendingWork());
    f.flash.hold = false;
    app.join();
    TEST_ASSERT(!f.boot.HasPendingWork());
    dev.Init(false);
    Reenumerate(dev, f.hw);
    TEST_ASSERT(GetState(dev, f.hw) == S(DFUState::DFU_IDLE));
    TEST_ASSERT(Dnload(dev, f.hw, 1U, Bytes(256U, 6U)));
    TEST_ASSERT(GetStatus(dev, f.hw).status == C(DFUStatusCode::ERR_ADDRESS));
    ClearStatus(dev, f.hw);
    TEST_ASSERT(Dnload(dev, f.hw, 0U, Bytes(256U, 7U)));
    const Status status =
        PollUntilIdle(dev, f.hw, GetStatus(dev, f.hw), [&]() { f.boot.Process(); });
    TEST_ASSERT(status.state == S(DFUState::DFU_DNLOAD_IDLE) && f.flash_mem[0] == 7U);
  }

  // (c2) manifest 正在 Process() 里写 seal 时 Deinit：结果作废，镜像不发布 / Deinit
  //      while Process() writes the seal: the result is voided, no image is published
  {
    BootFixture f;
    FakeUsb dev(f.hw, {{&f.boot}});
    Enumerate(dev, f.hw);
    TEST_ASSERT(Dnload(dev, f.hw, 0U, Bytes(100U, 3U)));
    TEST_ASSERT(
        PollUntilIdle(dev, f.hw, GetStatus(dev, f.hw), [&]() { f.boot.Process(); })
            .state == S(DFUState::DFU_DNLOAD_IDLE));
    TEST_ASSERT(Dnload(dev, f.hw, 1U, {}));
    TEST_ASSERT(GetStatus(dev, f.hw).state == S(DFUState::DFU_MANIFEST));
    f.flash.hold = true;
    std::thread app([&]() { f.boot.Process(); });
    TEST_ASSERT(WaitUntil([&]() { return f.flash.held.load(); }));
    dev.Deinit(false);
    f.flash.hold = false;
    app.join();
    TEST_ASSERT(!f.boot.HasPendingWork() && !f.boot.HasValidImage());
  }

  // (d) UPLOAD 的 IN 数据阶段发了一包时总线复位；之后上传从块 0 重来 / Bus reset after
  //     one packet of an UPLOAD IN data stage; afterwards the upload restarts at block 0
  {
    BootFixture f;
    FakeUsb dev(f.hw, {{&f.boot}});
    Enumerate(dev, f.hw);
    const Bytes image = Pattern(400U, 0x21U);
    TEST_ASSERT(
        DownloadImage(dev, f.hw, image, 256U, [&]() { f.boot.Process(); }).state ==
        S(DFUState::DFU_MANIFEST_WAIT_RESET));
    Reenumerate(dev, f.hw);
    dev.Setup(Make(0xA1U, UPLOAD, 0U, 0U, 256U));
    TEST_ASSERT(f.hw.in0.inflight && f.hw.in0.LastLength() == 64U);
    InDone(dev, f.hw.in0);
    TEST_ASSERT(f.hw.in0.inflight);
    Reenumerate(dev, f.hw);
    Refused(dev, f.hw, Make(0xA1U, UPLOAD, 1U, 0U, 256U));
    ClearStatus(dev, f.hw);
    Bytes back = Upload(dev, f.hw, 0U, 256U);
    const Bytes rest = Upload(dev, f.hw, 1U, 256U);
    back.insert(back.end(), rest.begin(), rest.end());
    TEST_ASSERT(back == image);
  }

  // (e) DNBUSY 中、状态阶段未完成时再设同一配置：解绑再绑定，工作作废，回到 DFU_IDLE
  // (e) The same configuration set again in DNBUSY with the status stage open: unbind
  //     and bind, the work is voided, back to DFU_IDLE
  {
    BootFixture f;
    FakeUsb dev(f.hw, {{&f.boot}});
    Enumerate(dev, f.hw);
    TEST_ASSERT(Dnload(dev, f.hw, 0U, Bytes(256U, 8U)));
    TEST_ASSERT(GetStatus(dev, f.hw, 0U, false).state == S(DFUState::DFU_DNBUSY));
    SetConfiguration(dev, f.hw, 1U);
    TEST_ASSERT(!f.boot.HasPendingWork());
    TEST_ASSERT(GetState(dev, f.hw) == S(DFUState::DFU_IDLE));
    // SET_INTERFACE 在 DNBUSY 中被拒绝 / SET_INTERFACE is refused in DNBUSY
    TEST_ASSERT(Dnload(dev, f.hw, 0U, Bytes(256U, 8U)));
    TEST_ASSERT(GetStatus(dev, f.hw).state == S(DFUState::DFU_DNBUSY));
    Refused(dev, f.hw, Make(0x01U, 11U, 0U, 0U, 0U));
  }
}

/// MS OS 2.0 描述符集来自组合层；WebUSB 能力来自类 / The MS OS 2.0 descriptor set
/// comes from the composition; the WebUSB capability from the class
void TestWinUsb()
{
  using DeviceSet =
      WinUsb::DeviceScopedWinUsbMsOs20DescSet<WinUsb::GUID_MULTI_SZ_UTF16_BYTES>;
  using Property =
      WinUsb::DeviceInterfaceGuidsRegProperty<WinUsb::GUID_MULTI_SZ_UTF16_BYTES>;

  // 读 BOS，找 MS OS 2.0 平台能力的厂商码，再按主机的方式取描述符集 / Read the BOS,
  // find the vendor code of the MS OS 2.0 platform capability and fetch the set the
  // way a host does
  auto read_set = [](FakeUsb& dev, Hw& hw, size_t& caps)
  {
    const Bytes bos = GetDescriptor(dev, hw, 15U, 0U, 255U);
    caps = bos[4];
    size_t offset = 5U;
    uint8_t vendor = 0U;
    while (offset < bos.size())
    {
      if (bos[offset + 2] == 0x05U && bos[offset] == 28U && bos[offset + 4] == 0xDFU)
      {
        vendor = bos[offset + 26];
      }
      offset += bos[offset];
    }
    TEST_ASSERT(vendor != 0U);
    return ControlIn(dev, hw, Make(0xC0U, vendor, 0U, 7U, 0x400U));
  };

  {
    BootFixture f;
    FakeUsb dev(f.hw, {{&f.boot}}, USBSpec::USB_2_1);
    dev.Init(false);
    dev.Reset();
    DeviceSet expected{};
    expected.Init(Boot::DEFAULT_WINUSB_DEVICE_INTERFACE_GUID);
    size_t caps = 0;
    const Bytes set = read_set(dev, f.hw, caps);
    TEST_ASSERT(caps == 2U && set.size() == sizeof(DeviceSet));
    TEST_ASSERT(std::memcmp(set.data(), &expected, sizeof(DeviceSet)) == 0);
  }

  {
    // 运行时与 bootloader 并存：每个功能一个功能子集，运行时在接口 1；运行时带 WebUSB
    // Runtime next to the bootloader: one function subset each, the runtime at
    // interface 1; the runtime carries WebUSB
    BootFixture f;
    DfuRuntimeClass rt(&JumpToBootloader, nullptr, 50U, "rt", "https://example.com/dfu");
    FakeUsb dev(f.hw, {{&f.boot, &rt}}, USBSpec::USB_2_1);
    dev.Init(false);
    dev.Reset();
    constexpr size_t FUNCTION = 8U + 20U + sizeof(Property);
    size_t caps = 0;
    const Bytes set = read_set(dev, f.hw, caps);
    TEST_ASSERT(caps == 3U);
    TEST_ASSERT(set.size() == 10U + 8U + 2U * FUNCTION);
    TEST_ASSERT((set[10 + 2] | set[10 + 3] << 8U) == 0x0001);
    const uint8_t* second = set.data() + 18 + FUNCTION;
    TEST_ASSERT((second[2] | second[3] << 8U) == 0x0002 && second[4] == 1U);
  }
}

/**
 * 多线程：应用线程不停调用 Process()、TryConsumeAppLaunch() 和任何上下文的查询，测试线程
 * 扮控制器中断和主机，按 bwPollTimeout 下载两遍镜像；autorun 在 WAIT_RESET 确认后由应用
 * 线程跳转。
 * Threads: the application thread keeps calling Process(), TryConsumeAppLaunch() and
 * the any-context queries; the test thread plays the controller interrupt and the host
 * and downloads an image twice, honouring bwPollTimeout; with autorun the application
 * thread jumps once WAIT_RESET is acknowledged.
 */
void TestThreads()
{
  g_app_jumps = 0U;
  BootFixture f(true);
  FakeUsb dev(f.hw, {{&f.boot}});
  Enumerate(dev, f.hw);

  std::atomic<bool> stop{false};
  std::atomic<size_t> loops{0U};
  std::thread app(
      [&]()
      {
        while (!stop.load(std::memory_order_acquire))
        {
          f.boot.Process();
          (void)f.boot.TryConsumeAppLaunch(0U);
          (void)f.boot.HasValidImage();
          (void)f.boot.ImageSize();
          loops.fetch_add(1U, std::memory_order_relaxed);
          std::this_thread::yield();
        }
      });

  for (unsigned round = 0; round < 2U; ++round)
  {
    const Bytes image = Pattern(1000U, static_cast<uint8_t>(0x30U + round));
    const Status status = DownloadImage(dev, f.hw, image, 256U, [] {});
    TEST_ASSERT(status.status == 0U &&
                status.state == S(DFUState::DFU_MANIFEST_WAIT_RESET));
    TEST_ASSERT(WaitUntil([&]() { return g_app_jumps.load() == round + 1U; }));
    TEST_ASSERT(f.boot.ImageSize() == image.size());
    TEST_ASSERT(std::equal(image.begin(), image.end(), f.flash_mem.begin()));
    Reenumerate(dev, f.hw);
  }
  stop = true;
  app.join();
  TEST_ASSERT(loops.load() > 0U && g_app_jumps == 2U);
}

}  // namespace

void test_usb_dfu_enum() { TestEnumeration(); }
void test_usb_dfu_runtime() { TestRuntimeRequests(); }
void test_usb_dfu_runtime_jump() { TestRuntimeJump(); }
void test_usb_dfu_download() { TestDownload(); }
void test_usb_dfu_poll() { TestPollWait(); }
void test_usb_dfu_status_stage() { TestStatusStage(); }
void test_usb_dfu_bind_fail() { TestBindFail(); }
void test_usb_dfu_unbind() { TestUnbind(); }
void test_usb_dfu_winusb() { TestWinUsb(); }
void test_usb_dfu_threads() { TestThreads(); }
