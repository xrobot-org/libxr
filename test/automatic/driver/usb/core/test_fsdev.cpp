// Shared FSDEV runtime (src/driver/usb/backend/fsdev.hpp) against a fake controller:
// the fake implements the register semantics (STAT/DTOG toggle on write-1, CTR/SETUP
// keep, ISTR write-0-to-clear) and the PMA layout (half-word stride 2) of a classic
// FSDEV part, and a host side drives SETUP/OUT/IN packets through it. The tests run
// enumeration, control read/write, multi-packet bulk, stops and bus resets through the
// real dispatch/owner machinery of FSDevDevice.

#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <vector>

#include "cdc_uart.hpp"
#include "test_assert.hpp"
#include "usb/backend/fsdev.hpp"

using namespace LibXR;
using namespace LibXR::USB;

extern "C" void libxr_fatal_error(const char* file, uint32_t line, bool)
{
  std::fprintf(stderr, "%s:%u: FSDEV assertion failed\n", file, line);
  std::abort();
}

namespace
{
/// 经典型 FSDEV 的寄存器与 PMA 语义模型 / Register and PMA model of a classic FSDEV
struct FakeHw
{
  static inline uint16_t cntr = 0;
  static inline uint16_t istr = 0;
  static inline uint16_t fnr = 0;
  static inline uint16_t daddr = 0;
  static inline uint32_t ep[8] = {};
  static inline std::array<uint8_t, 2048> pma{};  // APB 视图 / APB view
  static inline bool pull_up = false;
  static inline bool clocks = false;

  static constexpr uint8_t kEndpoints = 8U;
  static constexpr size_t kPmaSize = 512U;
  static constexpr size_t kPmaAlign = 2U;

  static constexpr uint32_t kEpCtrRx = 0x8000U;
  static constexpr uint32_t kEpDtogRx = 0x4000U;
  static constexpr uint32_t kEpStatRx = 0x3000U;
  static constexpr uint32_t kEpStatRxValid = 0x3000U;
  static constexpr uint32_t kEpStatTxValid = 0x0030U;
  static constexpr uint32_t kEpSetup = 0x0800U;
  static constexpr uint32_t kEpType = 0x0600U;
  static constexpr uint32_t kEpKind = 0x0100U;
  static constexpr uint32_t kEpCtrTx = 0x0080U;
  static constexpr uint32_t kEpDtogTx = 0x0040U;
  static constexpr uint32_t kEpStatTx = 0x0030U;
  static constexpr uint32_t kEpAddr = 0x000FU;
  static constexpr uint32_t kEpErrRx = 0U;
  static constexpr uint32_t kEpErrTx = 0U;
  static constexpr uint32_t kEpKeepMask =
      kEpCtrRx | kEpSetup | kEpType | kEpKind | kEpCtrTx | kEpAddr;
  static constexpr uint32_t kEpToggleMask = kEpStatTx | kEpStatRx | kEpDtogTx | kEpDtogRx;
  /// 写 0 清除、写 1 保持的位（CTR 与 ERR 都是 rc_w0）/ rc_w0 bits: write 0 clears
  static constexpr uint32_t kEpWrite0ClearMask =
      kEpCtrRx | kEpCtrTx | kEpErrRx | kEpErrTx;
  /// 普通读写位 / Plain read-write bits
  static constexpr uint32_t kEpRwMask = kEpType | kEpKind | kEpAddr;

  static constexpr uint16_t kIstrCtr = 0x8000U;
  static constexpr uint16_t kIstrReset = 0x0400U;
  static constexpr uint16_t kIstrSof = 0x0200U;
  static constexpr uint16_t kIstrSusp = 0x0800U;
  static constexpr uint16_t kIstrWkup = 0x1000U;
  static constexpr uint16_t kIstrNoise = 0x7100U;
  static constexpr uint16_t kCntrRunMask = 0x8000U | 0x0400U | 0x0800U | 0x1000U;
  static constexpr uint16_t kCntrSofm = 0x0200U;

  /// ISTR.CTR 是所有端点 CTR 的或，EP_ID 指向挂起端点中编号最小的那个（真硬件按
  /// 优先级，EP0 最高）；软件清位后仍有挂起端点时硬件立刻重新置位。
  /// ISTR.CTR is the OR of all endpoint CTRs and EP_ID points at the pending endpoint
  /// with the lowest number; a pending endpoint re-asserts CTR right after the software
  /// clears it.
  static void UpdateIstr()
  {
    bool pending = false;
    uint8_t id = 0;
    for (uint8_t i = 0; i < kEndpoints; ++i)
    {
      if ((ep[i] & (kEpCtrRx | kEpCtrTx)) != 0U && !pending)
      {
        pending = true;
        id = i;
      }
    }
    if (pending)
    {
      istr = static_cast<uint16_t>((istr & 0xFFF0U) | (id & 0x0FU) | kIstrCtr);
    }
    else
    {
      istr &= static_cast<uint16_t>(~kIstrCtr);
    }
  }

  static uint32_t ReadEp(uint8_t num) { return ep[num]; }
  /// 一次性竞态钩子：在下一次 WriteEp 生效前注入一次硬件事务，复现 SetStat 的
  /// 读改写窗口 / One-shot race hook: injects a hardware transaction right before
  /// the next WriteEp lands, reproducing the SetStat read-modify-write window.
  static inline std::function<void()> on_write;
  /// 一次性竞态钩子：在 WriteEp 生效之后、软件读回之前注入硬件事务，复现
  /// "写入已生效且事务已完成"的窗口 / One-shot race hook: injects a hardware
  /// transaction after the write lands but before the software reads back.
  static inline std::function<void()> on_write_post;
  static void WriteEp(uint8_t num, uint32_t value)
  {
    if (on_write)
    {
      std::function<void()> action = std::move(on_write);
      on_write = nullptr;
      action();
    }
    // CTR/ERR 写 0 清除、写 1 保持；STAT/DTOG 写 1 翻转；TYPE/KIND/EA 直接写入；
    // SETUP 由硬件管理，软件写无效。
    // CTR/ERR: write 0 clears, write 1 keeps. STAT/DTOG: write 1 toggles.
    // TYPE/KIND/EA: written. SETUP is hardware-managed, writes have no effect.
    uint32_t next = ep[num];
    next &= value | ~kEpWrite0ClearMask;
    next = (next & ~kEpRwMask) | (value & kEpRwMask);
    next ^= value & kEpToggleMask;
    next = (next & ~kEpSetup) | (ep[num] & kEpSetup);
    ep[num] = next;
    UpdateIstr();
    if (on_write_post)
    {
      std::function<void()> action = std::move(on_write_post);
      on_write_post = nullptr;
      action();
    }
  }
  static uint16_t ReadCntr() { return cntr; }
  static void WriteCntr(uint16_t value) { cntr = value; }
  static uint16_t ReadIstr() { return istr; }
  static void ClearIstr(uint16_t flags)
  {
    istr &= static_cast<uint16_t>(~flags);
    UpdateIstr();
  }
  static uint16_t ReadFnr() { return fnr; }
  static void WriteDaddr(uint16_t value) { daddr = value; }

  static volatile uint16_t* BtField(uint8_t num, unsigned field)
  {
    // 每数据字占 2 个半字地址：字段 m 在 APB 字节 (8n+2m)*2。
    return reinterpret_cast<volatile uint16_t*>(
        reinterpret_cast<uintptr_t>(pma.data()) +
        (static_cast<uintptr_t>(num) * 8U + 2U * field) * 2U);
  }
  static void SetTxBuf(uint8_t num, uint16_t addr)
  {
    *BtField(num, 0U) = static_cast<uint16_t>(addr & 0xFFFEU);
  }
  static void SetTxCount(uint8_t num, uint16_t bytes) { *BtField(num, 1U) = bytes; }
  static void SetRxBuf(uint8_t num, uint16_t addr)
  {
    *BtField(num, 2U) = static_cast<uint16_t>(addr & 0xFFFEU);
  }
  static void SetRxCountEncoded(uint8_t num, uint16_t mps)
  {
    *BtField(num, 3U) = EncodeRxCount(mps);
  }
  static uint16_t GetRxCount(uint8_t num)
  {
    return static_cast<uint16_t>(*BtField(num, 3U) & 0x03FFU);
  }
  static uint16_t EncodeRxCount(uint16_t mps)
  {
    if (mps == 0U)
    {
      return 0x8000U;
    }
    if (mps <= 62U)
    {
      return static_cast<uint16_t>(((mps + 1U) / 2U) << 10U);
    }
    return static_cast<uint16_t>(0x8000U | ((((mps + 31U) / 32U) - 1U) << 10U));
  }

  static void PmaWrite(uint16_t addr, const uint8_t* src, size_t len)
  {
    volatile uint16_t* p = reinterpret_cast<volatile uint16_t*>(
        reinterpret_cast<uintptr_t>(pma.data()) + static_cast<uintptr_t>(addr) * 2U);
    for (size_t i = 0; i < len; i += 2)
    {
      uint16_t word = src[i];
      if (i + 1 < len)
      {
        word |= static_cast<uint16_t>(src[i + 1]) << 8U;
      }
      *p = word;
      p += 2;
    }
  }
  static void PmaRead(uint8_t* dst, uint16_t addr, size_t len)
  {
    const volatile uint16_t* p = reinterpret_cast<const volatile uint16_t*>(
        reinterpret_cast<uintptr_t>(pma.data()) + static_cast<uintptr_t>(addr) * 2U);
    for (size_t i = 0; i < len; i += 2)
    {
      const uint16_t word = *p;
      p += 2;
      dst[i] = static_cast<uint8_t>(word & 0xFFU);
      if (i + 1 < len)
      {
        dst[i + 1] = static_cast<uint8_t>(word >> 8U);
      }
    }
  }

  static void InitRegs()
  {
    // FRES 脉冲复位 USB 逻辑：EPnR、ISTR、地址全部清零（与真硬件一致）。
    WriteCntr(0x0001U);
    WriteCntr(0U);
    istr = 0;
    daddr = 0;
    for (auto& value : ep)
    {
      value = 0;
    }
    UpdateIstr();
  }
  static void SuspendRegs() { cntr |= 0x0008U; }
  static void SuspendClocked() {}
  static void ResumeRegs() { cntr &= static_cast<uint16_t>(~(0x0004U | 0x0008U)); }
  static void Connect(bool on) { pull_up = on; }
  static void MspInit() { clocks = true; }
  static uintptr_t EnterGuard() { return 0U; }
  static void ExitGuard(uintptr_t) {}

  static void SuspendLine() { istr |= kIstrSusp; }
  static void ResumeLine() { istr |= kIstrWkup; }

  // ---- 主机侧（模拟硬件行为，不经过软件写语义）/ Host side (hardware behavior) ----

  static uint16_t TxAddr(uint8_t num)
  {
    return *reinterpret_cast<volatile uint16_t*>(reinterpret_cast<uintptr_t>(pma.data()) +
                                                 static_cast<uintptr_t>(num) * 16U);
  }
  static uint16_t RxAddr(uint8_t num)
  {
    return *reinterpret_cast<volatile uint16_t*>(reinterpret_cast<uintptr_t>(pma.data()) +
                                                 static_cast<uintptr_t>(num) * 16U + 8U);
  }

  /// 硬件收到一个 OUT/SETUP 包：数据进 PMA、计数写实际值、STAT 置 NAK、CTR 置位。
  /// 普通 OUT 包只在端点 VALID 时被收下（NAK 由主机重试）。
  /// The hardware received an OUT or SETUP packet: data in the PMA, the actual count
  /// written, STAT forced NAK and CTR set. A plain OUT packet is only taken while the
  /// endpoint is VALID (the host retries through NAKs).
  static void RxPacket(uint8_t num, const uint8_t* data, size_t len, bool setup)
  {
    if (!setup && (ep[num] & kEpStatRx) != kEpStatRxValid)
    {
      return;  // NAK：主机稍后重试 / NAK, the host retries later
    }
    if (len != 0U)
    {
      PmaWrite(RxAddr(num), data, len);
    }
    *BtField(num, 3U) = static_cast<uint16_t>(len & 0x3FFU);
    ep[num] = (ep[num] & ~kEpStatRx) | kEpCtrRx | 0x2000U;
    // SETUP 位由硬件在收到非 SETUP token 时清除 / cleared by hardware on a non-SETUP
    // token.
    if (setup)
    {
      ep[num] |= kEpSetup;
    }
    else
    {
      ep[num] &= ~kEpSetup;
    }
    UpdateIstr();
  }

  /// 主机取走 IN 包并 ACK：STAT 置 NAK、CTR 置位 / The host took the IN packet and
  /// ACKed it: STAT forced NAK and CTR set.
  static void AckIn(uint8_t num)
  {
    ep[num] = (ep[num] & ~kEpStatTx) | kEpCtrTx | 0x0020U;
    UpdateIstr();
  }

  static void BusReset()
  {
    for (auto& value : ep)
    {
      value = 0;
    }
    daddr = 0;
    istr |= kIstrReset;
    UpdateIstr();
  }

  static void Sof(uint16_t frame)
  {
    fnr = frame;
    istr |= kIstrSof;
  }
};

using Device = FSDevDevice<FakeHw>;

/// 主机 NAK 重试直到设备挂接收：两轮 Dispatch 让 ISR 批与 owner 的续办都跑完。
/// The host retries through NAKs until the device arms the transfer: two rounds of
/// Dispatch let the ISR batch and the owner continuation both run.
void Pump()
{
  Device::Dispatch();
  Device::Dispatch();
}

/// 状态阶段 OUT 零长包：主机重试直到设备把端点置 VALID 再发。
/// The zero-length status OUT: the host retries until the device sets the endpoint
/// VALID, then sends it.
void SendStatusOut(uint8_t num = 0)
{
  for (int i = 0;
       i < 8 && (FakeHw::ReadEp(num) & FakeHw::kEpStatRx) != FakeHw::kEpStatRxValid; ++i)
  {
    Pump();
  }
  FakeHw::RxPacket(num, nullptr, 0U, false);
  Pump();
}

std::vector<uint8_t> TakeInPacket(uint8_t num)
{
  const uint16_t count = *FakeHw::BtField(num, 1U) & 0x03FFU;
  std::vector<uint8_t> data(count);
  if (count != 0U)
  {
    FakeHw::PmaRead(data.data(), FakeHw::TxAddr(num), count);
  }
  FakeHw::AckIn(num);
  return data;
}

/// 主机重试 IN：等设备把端点置 VALID 再取走包 / The host retries IN: waits until the
/// device sets the endpoint VALID before taking the packet.
std::vector<uint8_t> AwaitInPacket(uint8_t num)
{
  for (int i = 0;
       i < 8 && (FakeHw::ReadEp(num) & FakeHw::kEpStatTx) != FakeHw::kEpStatTxValid; ++i)
  {
    Pump();
  }
  return TakeInPacket(num);
}

/// 主机按包长逐包取一个控制 IN 响应，直到收满 total 或遇短包。
/// The host takes a control IN response packet by packet until total bytes or a short
/// packet.
std::vector<uint8_t> AwaitControlIn(uint8_t num, size_t total)
{
  std::vector<uint8_t> data;
  while (data.size() < total)
  {
    auto packet = AwaitInPacket(num);
    if (packet.empty())
    {
      break;
    }
    data.insert(data.end(), packet.begin(), packet.end());
    Pump();
  }
  return data;
}

std::vector<uint8_t> Bytes(size_t count, uint8_t seed)
{
  std::vector<uint8_t> data(count);
  for (size_t i = 0; i < count; ++i)
  {
    data[i] = static_cast<uint8_t>(seed + i);
  }
  return data;
}

SetupPacket MakeSetup(uint8_t type, uint8_t request, uint16_t value, uint16_t index,
                      uint16_t length)
{
  SetupPacket packet{};
  packet.bmRequestType = type;
  packet.bRequest = request;
  packet.wValue = value;
  packet.wIndex = index;
  packet.wLength = length;
  return packet;
}

// ---------------------------------------------------------------------------
// Control class (minimal, from test_endpoint.cpp)
// ---------------------------------------------------------------------------

constexpr auto TEST_LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "LibXR", "FSDEV tests", "0");

struct BasicCapabilities : FullSpeedCapabilities
{
  static constexpr bool BOS = false;
  static constexpr bool BUS_TIME = false;
};

class ControlClass : public DeviceClass
{
 public:
  std::array<uint8_t, 256> input{}, output{};
  size_t response_size = 0, data_count = 0, complete_count = 0, abort_count = 0;
  size_t last_actual = 0;
  bool reject_data = false;
  std::vector<DeviceEvent> events;
  size_t binds = 0;  ///< 非规划模式的绑定次数 / Binds outside planning
  InterfaceDescriptor interface{};
  size_t GetControlReceiveCapacity() const override { return output.size(); }
  size_t GetInterfaceCount() override { return 1; }
  size_t GetMaxConfigSize() override { return sizeof(interface); }
  bool HasIAD() override { return false; }
  void BindEndpoints(EndpointPool& pool, uint8_t number, bool) override
  {
    if (!pool.IsPlanning()) ++binds;
    interface = {9, 4, number, 0, 0, 0xff, 0, 0, 0};
    SetData({&interface, sizeof(interface)});
  }
  void UnbindEndpoints(EndpointPool&, bool) override {}
  void OnDeviceEvent(bool, DeviceEvent event, uint8_t) override
  {
    events.push_back(event);
  }

 protected:
  ErrorCode OnControlRequest(bool, const SetupPacket& setup,
                             ControlTransferResult& result) override
  {
    if (setup.bRequest == 0x70 && (setup.bmRequestType & 0x80U))
    {
      result.InData() = {input.data(), response_size};
    }
    else if (setup.bRequest == 0x71 && !(setup.bmRequestType & 0x80U))
    {
      return ErrorCode::OK;
    }
    else if (setup.bRequest != 0x72 || setup.wLength != 0U)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    return ErrorCode::OK;
  }
  ErrorCode OnControlData(bool, const SetupPacket& setup, ConstRawData& data) override
  {
    if (!(setup.bmRequestType & 0x80U))
    {
      ++data_count;
      last_actual = data.size_;
      std::memcpy(output.data(), data.addr_, data.size_);
    }
    return reject_data ? ErrorCode::ARG_ERR : ErrorCode::OK;
  }
  void OnControlComplete(bool, const SetupPacket&) override { ++complete_count; }
  void OnControlAbort(bool, const SetupPacket&) override { ++abort_count; }
};

template <size_t N>
struct alignas(64) Block
{
  std::array<uint8_t, N> bytes{};
  RawData Raw() { return {bytes.data(), bytes.size()}; }
};

/// 枚举并配置后的通用底座 / Common base with the device enumerated and configured
struct Fixture
{
  Block<64> ep0_out, ep0_in;
  Block<128> ep1_out, ep1_in;
  Block<128> ep2_out;
  Block<128> ep3_in;
  ControlClass klass;
  Device device;

  explicit Fixture(uint8_t slot_count = 0)
      : device({{ep0_out.Raw(), 64}, {ep1_out.Raw(), 64}, {ep2_out.Raw(), 64}},
               {{ep0_in.Raw(), 64}, {ep1_in.Raw(), 64}, {ep3_in.Raw(), 32}},
               DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x4321, 0x0100,
               {&TEST_LANGUAGE}, {{&klass}}, {}, 0, slot_count)
  {
    device.Init(false);
    device.Start(false);
    TEST_ASSERT(FakeHw::pull_up && FakeHw::clocks);
    FakeHw::BusReset();
    Pump();
    TEST_ASSERT(klass.events.size() == 1 && klass.events.back() == DeviceEvent::RESET);
    FakeHw::RxPacket(0, SetupRaw(MakeSetup(0, 9, 1, 0, 0)).data(), 8U, true);
    Pump();
    SendStatusOut();  // 状态阶段 OUT ZLP / status OUT ZLP

    TEST_ASSERT(klass.events.size() == 1 && klass.binds == 1);
  }

  static std::array<uint8_t, 8> SetupRaw(const SetupPacket& packet)
  {
    std::array<uint8_t, 8> raw{};
    std::memcpy(raw.data(), &packet, sizeof(raw));
    return raw;
  }

  void Setup(const SetupPacket& packet)
  {
    FakeHw::RxPacket(0, SetupRaw(packet).data(), 8U, true);
    Pump();
  }

  void StatusOut() { SendStatusOut(); }
};

/// 借一个 IN 槽、拷入并提交 / Borrow an IN slot, copy the bytes in and submit
ErrorCode SubmitBytes(Endpoint* in, const std::vector<uint8_t>& bytes)
{
  Endpoint::Slot slot;
  const ErrorCode acquire = in->AcquireSlot(slot, false);
  if (acquire != ErrorCode::OK) return acquire;
  std::memcpy(slot.data.addr_, bytes.data(), bytes.size());
  return in->Submit(slot, bytes.size(), false, false);
}

struct Done
{
  size_t actual = 0;
  ErrorCode result = ErrorCode::OK;
  unsigned count = 0;
  Endpoint::Slot slot;  ///< OUT 租约，测试读完数据后归还 / the OUT lease to return
  static void Record(bool, Done* ctx, const Endpoint::Completion& done)
  {
    TEST_ASSERT(done.result == ErrorCode::OK || done.result == ErrorCode::ABORTED);
    ctx->actual = done.actual;
    ctx->result = done.result;
    ctx->slot = done.slot;
    ++ctx->count;
  }
  Callback<const Endpoint::Completion&> CallbackFor()
  {
    return Callback<const Endpoint::Completion&>::Create(&Done::Record, this);
  }
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

void TestEnumerationAndControlRead()
{
  Fixture f;
  f.klass.response_size = 18;
  for (unsigned i = 0; i < 18; ++i)
  {
    f.klass.input[i] = static_cast<uint8_t>(0xF0 + i);
  }
  f.Setup(MakeSetup(0xA1, 0x70, 0, 0, 18));
  auto packet = AwaitInPacket(0);
  Pump();
  TEST_ASSERT(packet.size() == 18 && packet[0] == 0xF0 &&
              packet[17] == static_cast<uint8_t>(0xF0 + 17));
  f.StatusOut();
  TEST_ASSERT(f.klass.complete_count == 1);
  TEST_ASSERT(FakeHw::daddr == 0x0080U);  // 地址在状态完成时生效 / set on completion

  // 类拒绝的请求以 STALL 回应 / A rejected request answers STALL.
  f.Setup(MakeSetup(0x40, 0x77, 0, 0, 0));
  Pump();
  TEST_ASSERT((FakeHw::ReadEp(0) & FakeHw::kEpStatTx) == 0x0010U);
}

void TestControlWrite()
{
  Fixture f;
  const auto payload = Bytes(150, 0x40);
  f.Setup(MakeSetup(0x21, 0x71, 0, 0, 150));
  FakeHw::RxPacket(0, payload.data(), 64U, false);
  Pump();
  TEST_ASSERT(f.klass.data_count == 0);
  FakeHw::RxPacket(0, payload.data() + 64, 64U, false);
  Pump();
  FakeHw::RxPacket(0, payload.data() + 128, 22U, false);
  Pump();
  TEST_ASSERT(f.klass.data_count == 1 && f.klass.last_actual == 150);
  TEST_ASSERT(std::equal(payload.begin(), payload.end(), f.klass.output.begin()));
  auto status = AwaitInPacket(0);  // 状态阶段 IN ZLP / status IN ZLP
  Pump();
  TEST_ASSERT(status.empty());
  f.StatusOut();
  TEST_ASSERT(f.klass.complete_count == 1);

  // 中途新 SETUP：未完成数据作废 / A new SETUP cancels the unfinished data stage.
  f.Setup(MakeSetup(0x21, 0x71, 0, 0, 150));
  FakeHw::RxPacket(0, payload.data(), 64U, false);
  Pump();
  f.Setup(MakeSetup(0x21, 0x72, 0, 0, 0));
  TEST_ASSERT(f.klass.abort_count == 1 && f.klass.data_count == 1);
  auto in = AwaitInPacket(0);  // 0x72 无数据阶段，直接状态 IN / straight to status
  Pump();
  TEST_ASSERT(in.empty());
  f.StatusOut();
}

void TestBulkInMultiPacket()
{
  // 单槽 128 B：一个请求跨两个包 / One 128 B slot, one request over two packets
  Fixture f(1);
  Endpoint* in = nullptr;
  TEST_ASSERT(f.device.Get(in, Endpoint::Direction::IN, Endpoint::EPNumber::EP1) ==
              ErrorCode::OK);
  Done done;
  in->Configure({Endpoint::Direction::IN, Endpoint::Type::BULK, 64}, false);
  in->SetOnComplete(done.CallbackFor());
  const auto bytes = Bytes(100, 0x11);
  TEST_ASSERT(SubmitBytes(in, bytes) == ErrorCode::OK);
  auto packet = TakeInPacket(1);
  Pump();
  TEST_ASSERT(packet == Bytes(64, 0x11));
  packet = TakeInPacket(1);
  Pump();
  TEST_ASSERT(packet == Bytes(36, 0x11 + 64));
  Pump();  // owner 投递完成 / the owner delivers the completion
  TEST_ASSERT(done.count == 1 && done.actual == 100 && done.result == ErrorCode::OK);
}

void TestBulkOutMultiPacket()
{
  // 单槽 128 B：一个请求跨两个包 / One 128 B slot, one request over two packets
  Fixture f(1);
  Endpoint* out = nullptr;
  TEST_ASSERT(f.device.Get(out, Endpoint::Direction::OUT, Endpoint::EPNumber::EP1) ==
              ErrorCode::OK);
  Done done;
  out->Configure({Endpoint::Direction::OUT, Endpoint::Type::BULK, 64}, false);
  out->SetOnComplete(done.CallbackFor());
  const auto data = Bytes(128, 0x22);
  TEST_ASSERT(out->Receive(128, false) == ErrorCode::OK);
  TEST_ASSERT((FakeHw::ReadEp(1) & FakeHw::kEpStatRx) == 0x3000U);  // VALID

  FakeHw::RxPacket(1, data.data(), 64U, false);
  Pump();
  TEST_ASSERT(done.count == 0 && (FakeHw::ReadEp(1) & FakeHw::kEpStatRx) == 0x3000U);
  FakeHw::RxPacket(1, data.data() + 64, 64U, false);
  Pump();
  Pump();
  TEST_ASSERT(done.count == 1 && done.actual == 128 && done.result == ErrorCode::OK);
  // 单槽端点：租约必须归还才能再挂接收 / One slot: return the lease before rearming
  TEST_ASSERT(done.slot.Valid());
  out->ReleaseSlot(done.slot, false);

  // 短包提前结束 / A short packet ends the transfer early
  TEST_ASSERT(out->Receive(128, false) == ErrorCode::OK);
  FakeHw::RxPacket(1, data.data(), 40U, false);
  Pump();
  Pump();
  TEST_ASSERT(done.count == 2 && done.actual == 40);
  out->ReleaseSlot(done.slot, false);
}

void TestStopCountsLatePacket()
{
  Fixture f(1);
  Endpoint* out = nullptr;
  TEST_ASSERT(f.device.Get(out, Endpoint::Direction::OUT, Endpoint::EPNumber::EP1) ==
              ErrorCode::OK);
  Done done;
  out->Configure({Endpoint::Direction::OUT, Endpoint::Type::BULK, 64}, false);
  out->SetOnComplete(done.CallbackFor());
  TEST_ASSERT(out->Receive(128, false) == ErrorCode::OK);
  // 包已收、CTR 未处理时停止：那个包计入进度 / The packet arrived; the stop counts it.
  FakeHw::RxPacket(1, Bytes(64, 3).data(), 64U, false);
  out->Flush(false);
  Pump();
  TEST_ASSERT(done.count == 1 && done.result == ErrorCode::ABORTED && done.actual == 64);
  TEST_ASSERT((FakeHw::ReadEp(1) & FakeHw::kEpStatRx) == 0x2000U);  // NAK

  // 停止后重新挂接收 / Rearm after the flush
  TEST_ASSERT(out->Receive(128, false) == ErrorCode::OK);
  TEST_ASSERT((FakeHw::ReadEp(1) & FakeHw::kEpStatRx) == 0x3000U);
}

void TestBusResetEndsTransfers()
{
  Fixture f;
  Endpoint* in = nullptr;
  TEST_ASSERT(f.device.Get(in, Endpoint::Direction::IN, Endpoint::EPNumber::EP1) ==
              ErrorCode::OK);
  in->Configure({Endpoint::Direction::IN, Endpoint::Type::BULK, 64}, false);
  const auto bytes = Bytes(64, 0x33);
  TEST_ASSERT(SubmitBytes(in, bytes) == ErrorCode::OK);
  TEST_ASSERT((FakeHw::ReadEp(1) & FakeHw::kEpStatTx) == 0x0030U);

  FakeHw::BusReset();
  Pump();
  TEST_ASSERT(FakeHw::daddr == 0x0080U);
  TEST_ASSERT(f.klass.events.back() == DeviceEvent::RESET);
  // 重新枚举：复位后必须先选配置，类的接口才绑定（未配置时类请求会被 STALL）
  // Enumerate again: the configuration must be selected again after a reset, the
  // class interfaces are only bound then (a class request STALLs otherwise).
  const size_t binds = f.klass.binds;
  f.Setup(MakeSetup(0, 9, 1, 0, 0));
  f.StatusOut();
  TEST_ASSERT(f.klass.binds == binds + 1);
  f.klass.response_size = 18;
  FakeHw::RxPacket(0, Fixture::SetupRaw(MakeSetup(0xA1, 0x70, 0, 0, 18)).data(), 8U,
                   true);
  Pump();
  auto packet = AwaitInPacket(0);
  Pump();
  TEST_ASSERT(packet.size() == 18);
  f.StatusOut();
  TEST_ASSERT((FakeHw::ReadEp(1) & FakeHw::kEpStatTx) != 0x0030U);  // 旧会话不再发送
}

void TestSuspendResume()
{
  Fixture f;
  FakeHw::RxPacket(0, nullptr, 0U, false);  // 无在途控制传输时的噪声包，读掉丢弃
  Pump();
  FakeHw::SuspendLine();
  Pump();
  TEST_ASSERT((FakeHw::ReadCntr() & 0x0008U) != 0U);  // FSUSP 置位
  FakeHw::ResumeLine();
  Pump();
  TEST_ASSERT((FakeHw::ReadCntr() & 0x0008U) == 0U);
}

void TestIsochronousRejected()
{
  Fixture f;
  Endpoint* in = nullptr;
  TEST_ASSERT(f.device.Get(in, Endpoint::Direction::IN, Endpoint::EPNumber::EP1) ==
              ErrorCode::OK);
  TEST_ASSERT(in->Configure({Endpoint::Direction::IN, Endpoint::Type::ISOCHRONOUS, 64},
                            false) == ErrorCode::NOT_SUPPORT);
}

void TestPacketSizeLimit()
{
  Fixture f;
  Endpoint* in = nullptr;
  Endpoint* out = nullptr;
  Endpoint* out2 = nullptr;
  TEST_ASSERT(f.device.Get(in, Endpoint::Direction::IN, Endpoint::EPNumber::EP1) ==
              ErrorCode::OK);
  TEST_ASSERT(f.device.Get(out, Endpoint::Direction::OUT, Endpoint::EPNumber::EP1) ==
              ErrorCode::OK);
  TEST_ASSERT(f.device.Get(out2, Endpoint::Direction::OUT, Endpoint::EPNumber::EP2) ==
              ErrorCode::OK);
  TEST_ASSERT(in->LargestPacketSize(Endpoint::Type::BULK) == 64U);
  TEST_ASSERT(out->LargestPacketSize(Endpoint::Type::BULK) == 64U);
  // 同一端点号两方向类型不一致 / Both directions of one number share the type.
  TEST_ASSERT(in->Configure({Endpoint::Direction::IN, Endpoint::Type::BULK, 64}, false) ==
              ErrorCode::OK);
  TEST_ASSERT(out->Configure({Endpoint::Direction::OUT, Endpoint::Type::INTERRUPT, 64},
                             false) == ErrorCode::FAILED);
  // 超过 PMA 缓冲的包长；被拒的端点会停在 ERROR，故用另一个端点检查
  // A packet larger than the PMA buffer; a rejected endpoint stays in ERROR, so the
  // check uses another endpoint.
  TEST_ASSERT(out2->Configure({Endpoint::Direction::OUT, Endpoint::Type::BULK, 128},
                              false) == ErrorCode::ARG_ERR);
}

void TestCdcEcho()
{
  Block<64> ep0_out, ep0_in;
  // FSDEV 的 IN/OUT 可以共用端点号：CDC 数据 IN 与数据 OUT 都是 EP1，通知是 EP2。
  // On FSDEV one number may serve both directions: the CDC data IN and data OUT share
  // EP1 and the notification endpoint is EP2.
  Block<128> ep1_out, ep1_in;
  Block<128> ep2_in;
  CDCUart cdc{Endpoint::EPNumber::EP1,
              Endpoint::EPNumber::EP1,
              Endpoint::EPNumber::EP2,
              128,
              1024,
              32};
  Device device({{ep0_out.Raw(), 64}, {ep1_out.Raw(), 64}},
                {{ep0_in.Raw(), 64}, {ep1_in.Raw(), 64}, {ep2_in.Raw(), 32}},
                DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x4321, 0x0100,
                {&TEST_LANGUAGE}, {{&cdc}});
  device.Init(false);
  device.Start(false);
  FakeHw::BusReset();
  Pump();
  FakeHw::RxPacket(0, Fixture::SetupRaw(MakeSetup(0, 9, 1, 0, 0)).data(), 8U, true);
  Pump();
  SendStatusOut();

  TEST_ASSERT(FakeHw::pull_up);

  // SET_LINE_CODING 115200 8N1
  const uint8_t coding[7] = {0x00, 0xC2, 0x01, 0x00, 0, 0, 8};
  FakeHw::RxPacket(0, Fixture::SetupRaw(MakeSetup(0x21, 0x20, 0, 0, 7)).data(), 8U, true);
  Pump();
  FakeHw::RxPacket(0, coding, 7U, false);
  Pump();
  auto status = AwaitInPacket(0);
  Pump();
  TEST_ASSERT(status.empty());
  SendStatusOut();

  // CDC 数据端点已绑定并挂接收 / The CDC data endpoints are bound and armed
  TEST_ASSERT((FakeHw::ReadEp(1) & FakeHw::kEpStatRx) == 0x3000U);

  // 主机发数据 → CDC 读 / Host sends data, the CDC reader takes it
  const auto packet = Bytes(64, 0x55);
  FakeHw::RxPacket(1, packet.data(), 64U, false);
  Pump();
  ReadOperation read;
  std::array<uint8_t, 64> received{};
  TEST_ASSERT(cdc.Read({received.data(), received.size()}, read) == ErrorCode::OK);
  TEST_ASSERT(std::vector<uint8_t>(received.begin(), received.end()) == packet);

  // CDC 写 → 多包 IN / A CDC write goes out as packets
  const auto payload = Bytes(150, 0x66);
  WriteOperation write;
  TEST_ASSERT(cdc.Write({payload.data(), payload.size()}, write) == ErrorCode::OK);
  auto wire = AwaitInPacket(1);
  Pump();
  TEST_ASSERT(wire == Bytes(64, 0x66));
  wire = TakeInPacket(1);
  Pump();
  TEST_ASSERT(wire == Bytes(64, 0x66 + 64));
  wire = TakeInPacket(1);
  Pump();
  TEST_ASSERT(wire == Bytes(22, 0x66 + 128));
}

/// SetStat 的读改写窗口里硬件自己完成一次事务：旧实现按旧值算翻转增量会把
/// STAT 翻回 VALID，包被静默丢弃且不计进度；写后回读必须兜住。
/// The hardware completes a transaction inside SetStat's read-modify-write
/// window: computing the toggle delta from the stale value flipped STAT back to
/// VALID (packets silently dropped, progress lost). The read-back must catch it.
void TestStopRaceWithTransaction()
{
  // OUT：停止时硬件收走一包 / OUT: the hardware takes a packet during the stop
  {
    Fixture f(1);
    Endpoint* out = nullptr;
    TEST_ASSERT(f.device.Get(out, Endpoint::Direction::OUT, Endpoint::EPNumber::EP1) ==
                ErrorCode::OK);
    Done done;
    out->Configure({Endpoint::Direction::OUT, Endpoint::Type::BULK, 64}, false);
    out->SetOnComplete(done.CallbackFor());
    TEST_ASSERT(out->Receive(128, false) == ErrorCode::OK);
    TEST_ASSERT((FakeHw::ReadEp(1) & FakeHw::kEpStatRx) == 0x3000U);  // VALID

    const auto late = Bytes(64, 7);
    FakeHw::on_write = [&late] { FakeHw::RxPacket(1, late.data(), late.size(), false); };
    out->Flush(false);
    Pump();
    // NAK 而不是 VALID / NAK, not VALID
    TEST_ASSERT((FakeHw::ReadEp(1) & FakeHw::kEpStatRx) == 0x2000U);
    TEST_ASSERT(done.count == 1 && done.result == ErrorCode::ABORTED &&
                done.actual == 64);
    // 停止后再来的包不再被收下（端点保持 NAK）/ A later packet is not taken
    FakeHw::RxPacket(1, Bytes(64, 8).data(), 64U, false);
    TEST_ASSERT((FakeHw::ReadEp(1) & FakeHw::kEpStatRx) == 0x2000U);
    TEST_ASSERT(done.count == 1 && done.actual == 64);
  }

  // IN：停止时主机 ACK 走最后一包 / IN: the host ACKs the last packet during the stop
  {
    Fixture f(1);
    Endpoint* in = nullptr;
    TEST_ASSERT(f.device.Get(in, Endpoint::Direction::IN, Endpoint::EPNumber::EP1) ==
                ErrorCode::OK);
    Done done;
    in->Configure({Endpoint::Direction::IN, Endpoint::Type::BULK, 64}, false);
    in->SetOnComplete(done.CallbackFor());
    const auto bytes = Bytes(64, 9);
    TEST_ASSERT(SubmitBytes(in, bytes) == ErrorCode::OK);
    TEST_ASSERT((FakeHw::ReadEp(1) & FakeHw::kEpStatTx) == 0x0030U);  // VALID

    FakeHw::on_write = [] { FakeHw::AckIn(1); };
    in->Flush(false);
    Pump();
    TEST_ASSERT((FakeHw::ReadEp(1) & FakeHw::kEpStatTx) == 0x0020U);  // NAK
    TEST_ASSERT(done.count == 1 && done.result == ErrorCode::ABORTED &&
                done.actual == 64);
  }
}

/// 构造期 PMA 检查：缓冲不对齐 / 超出 pma_limit 必须在构造时 REQUIRE 失败，
/// 而不是静默截断地址让缓冲重叠。REQUIRE 走 libxr_fatal_error → abort，所以
/// 在 fork 出的子进程里构造，父进程检查异常退出。
/// Construction-time PMA checks: a misaligned buffer or a layout past pma_limit
/// must REQUIRE-fail at construction instead of silently truncating addresses and
/// overlapping buffers. REQUIRE aborts via libxr_fatal_error, so construct in a
/// forked child and expect it to die.
void TestPmaLayoutChecks()
{
  auto expect_fatal = [](bool odd_size, bool tiny_limit)
  {
    pid_t pid = fork();
    TEST_ASSERT(pid >= 0);
    if (pid == 0)
    {
      // 子进程：坏配置直接 abort / the child aborts on the bad config
      Block<64> ep0_out, ep0_in, ep1_out, ep1_in;
      ControlClass klass;
      const uint16_t data_size = odd_size ? 63U : 64U;
      const size_t limit = tiny_limit ? 24U : 0U;
      Device bad_device({{ep0_out.Raw(), 64}, {ep1_out.Raw(), data_size}},
                        {{ep0_in.Raw(), 64}, {ep1_in.Raw(), 64}},
                        DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x4321, 0x0100,
                        {&TEST_LANGUAGE}, {{&klass}}, {}, limit, 0);
      bad_device.Init(false);
      _exit(0);  // 构造成功 = 检查缺失 / construction succeeded = check missing
    }
    int status = 0;
    TEST_ASSERT(waitpid(pid, &status, 0) == pid);
    TEST_ASSERT(WIFSIGNALED(status) && WTERMSIG(status) == SIGABRT);
  };
  expect_fatal(true, false);  // 奇数 pma_size：地址静默截断 / odd size
  expect_fatal(false, true);  // 布局超出 pma_limit / layout past the limit
}

/// 装填 VALID 后、硬件读回之前事务就完成了（写入生效、STAT 自动回 NAK、CTR
/// 锁存）：重试逻辑不得把端点再次翻成 VALID——IN 会用同一份 PMA 数据重发一
/// 包，OUT 会多收一包。正确行为是把这次完成交给 CTR 处理。
/// The transaction completes after the VALID write lands but before the
/// read-back (write effective, STAT auto-NAK, CTR latched): the retry logic
/// must not flip the endpoint back to VALID — that re-arms the same PMA data
/// (IN) or takes an extra packet (OUT). The completion goes through CTR.
void TestArmNotRearmedByRace()
{
  // IN：装填即被 ACK / IN: the arm is ACKed inside the window
  {
    Fixture f(1);
    Endpoint* in = nullptr;
    TEST_ASSERT(f.device.Get(in, Endpoint::Direction::IN, Endpoint::EPNumber::EP1) ==
                ErrorCode::OK);
    Done done;
    in->Configure({Endpoint::Direction::IN, Endpoint::Type::BULK, 64}, false);
    in->SetOnComplete(done.CallbackFor());
    const auto bytes = Bytes(64, 0x44);
    FakeHw::on_write_post = [] { FakeHw::AckIn(1); };
    TEST_ASSERT(SubmitBytes(in, bytes) == ErrorCode::OK);
    // 必须停在 NAK（等 CTR 处理），不能被重试翻回 VALID
    TEST_ASSERT((FakeHw::ReadEp(1) & FakeHw::kEpStatTx) == 0x0020U);
    Pump();
    TEST_ASSERT(done.count == 1 && done.actual == 64 && done.result == ErrorCode::OK);
  }

  // OUT：装填即被收包 / OUT: the arm is taken inside the window
  {
    Fixture f(1);
    Endpoint* out = nullptr;
    TEST_ASSERT(f.device.Get(out, Endpoint::Direction::OUT, Endpoint::EPNumber::EP1) ==
                ErrorCode::OK);
    Done done;
    out->Configure({Endpoint::Direction::OUT, Endpoint::Type::BULK, 64}, false);
    out->SetOnComplete(done.CallbackFor());
    const auto data = Bytes(64, 0x45);
    FakeHw::on_write_post = [&data]
    { FakeHw::RxPacket(1, data.data(), data.size(), false); };
    TEST_ASSERT(out->Receive(64, false) == ErrorCode::OK);
    TEST_ASSERT((FakeHw::ReadEp(1) & FakeHw::kEpStatRx) == 0x2000U);
    Pump();
    TEST_ASSERT(done.count == 1 && done.actual == 64 && done.result == ErrorCode::OK);
    out->ReleaseSlot(done.slot, false);
  }
}

}  // namespace

int main()
{
  TestEnumerationAndControlRead();
  TestControlWrite();
  TestBulkInMultiPacket();
  TestBulkOutMultiPacket();
  TestStopCountsLatePacket();
  TestStopRaceWithTransaction();
  TestArmNotRearmedByRace();
  TestPmaLayoutChecks();
  TestBusResetEndsTransfers();
  TestSuspendResume();
  TestIsochronousRejected();
  TestPacketSizeLimit();
  TestCdcEcho();
  std::printf("fsdev: all tests passed\n");
  return 0;
}
