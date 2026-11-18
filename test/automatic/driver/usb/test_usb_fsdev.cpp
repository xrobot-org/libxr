/**
 * @file test_usb_fsdev.cpp
 * @brief FSDEV 后端在寄存器级假控制器上的测试 / Tests of the FSDEV backend on a
 *        register-level fake controller
 *
 * 假控制器实现经典型 FSDEV 的寄存器语义（STAT、DTOG 写 1 翻转，CTR 写 0 清除，SETUP
 * 只读，收到 SETUP 时两方向置 NAK），PMA 按半字间隔 2 访问。被测设备从 FSDevDevice
 * 派生，只把寄存器读写接到假控制器，BTABLE 和 PMA 用后端自己的代码。主机一侧直接改假
 * 控制器：发 SETUP、OUT 包，取走 IN 包。钩子在读或写寄存器时注入硬件事件，用来复现
 * 停止和装填的竞态窗口。
 * The fake controller implements the register semantics of a classic FSDEV (STAT and
 * DTOG toggle on a written 1, CTR clears on a written 0, SETUP is read-only, a SETUP
 * sets both directions to NAK), with a PMA at half-word stride 2. The device under test
 * derives from FSDevDevice and only routes register reads and writes to the fake; the
 * BTABLE and the PMA go through the backend's own code. The host side changes the fake
 * directly: sends SETUP and OUT packets and takes IN packets. Hooks on register reads and
 * writes inject hardware events to reproduce the race windows of stops and arms.
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "cdc_uart.hpp"
#include "dev_core.hpp"
#include "device_class.hpp"
#include "ep.hpp"
#include "test_assert.hpp"
#include "usb/backend/fsdev.hpp"

namespace
{
using LibXR::Callback;
using LibXR::ConstRawData;
using LibXR::ErrorCode;
using LibXR::RawData;
using LibXR::USB::DescriptorStrings;
using LibXR::USB::DeviceClass;
using LibXR::USB::DeviceDescriptor;
using LibXR::USB::Endpoint;
using LibXR::USB::EndpointPool;
using LibXR::USB::FSDevDevice;
using LibXR::USB::SetupPacket;
using LibXR::USB::Speed;
using Completion = Endpoint::Completion;
using Direction = Endpoint::Direction;
using EPNumber = Endpoint::EPNumber;
using Type = Endpoint::Type;
using Bytes = std::vector<uint8_t>;

constexpr uint32_t CTR_RX = 0x8000U;
constexpr uint32_t DTOG_RX = 0x4000U;
constexpr uint32_t STAT_RX = 0x3000U;
constexpr uint32_t SETUP = 0x0800U;
constexpr uint32_t CTR_TX = 0x0080U;
constexpr uint32_t DTOG_TX = 0x0040U;
constexpr uint32_t STAT_TX = 0x0030U;
constexpr uint32_t RX_NAK = 0x2000U;
constexpr uint32_t RX_VALID = 0x3000U;
constexpr uint32_t TX_NAK = 0x0020U;
constexpr uint32_t TX_VALID = 0x0030U;

/// 经典型 FSDEV 的寄存器与 PMA 模型 / Register and PMA model of a classic FSDEV
struct Ctl
{
  static inline uint16_t cntr = 0;
  static inline uint16_t istr = 0;
  static inline uint16_t fnr = 0;
  static inline unsigned fnr_reads = 0;  ///< 读 FNR 的次数 / Reads of FNR
  static inline uint16_t daddr = 0;
  static inline uint32_t ep[8] = {};
  alignas(4) static inline uint8_t pma[2048] = {};
  static inline bool pull_up = false;
  static inline unsigned connects = 0;  ///< Connect(true) 的次数 / Connect(true) calls
  /// 写入被忽略 STAT 位的端点（模拟写不进）/ Endpoint whose STAT writes are ignored
  static inline int stuck = -1;
  /// 总线复位的 SE0 期间：EPnR 读作 0、写不进，RESET 标志还没有来 / During the SE0 of
  /// a bus reset: EPnR reads 0 and ignores writes, the RESET flag has not come yet
  static inline bool in_reset = false;
  /// EP0 有未处理 SETUP 时 STAT_TX 被写成 VALID 的次数 / Writes that made EP0 STAT_TX
  /// VALID while a SETUP was unhandled
  static inline unsigned setup_violations = 0;
  /// 因 CTR_RX 未清而没被收下的 SETUP / SETUPs not taken because CTR_RX was set
  static inline unsigned setups_refused = 0;
  /// 被测设备是否打开 EP0 快速确认 / Whether the device under test enables the fast
  /// acknowledgement of EP0
  static inline bool fast_ack = true;
  /// 被测设备是否打开状态 OUT 的晚放行 / Whether the device under test enables the late
  /// release of the status OUT
  static inline bool late_status = true;
  /// 被测设备是否声明 EP0 的 OUT 包可能早到 / Whether the device under test declares
  /// that an OUT packet of EP0 may come early
  static inline bool early_out = false;
  /// 因超过接收区大小而没被收下的包 / Packets not taken because they exceed the receive
  /// size
  static inline unsigned rx_overflows = 0;
  /// 每次读 EPnR 之前调用 / Called before every EPnR read
  static inline std::function<void(uint8_t)> before_read;
  /// 每次写 EPnR 之前调用（参数：号、写入值）/ Called before every EPnR write
  static inline std::function<void(uint8_t, uint32_t)> before_write;
  /// 每次写接收区大小之前调用 / Called before every write of a receive size
  static inline std::function<void(uint8_t)> before_rx_size;

  static constexpr uint32_t KEEP = 0x0F0FU;  ///< 类型、KIND、地址 / Type, KIND, address
  static constexpr uint32_t TOGGLE = STAT_TX | STAT_RX | DTOG_TX | DTOG_RX;

  static void Reset()
  {
    cntr = istr = fnr = daddr = 0;
    fnr_reads = 0;
    std::memset(ep, 0, sizeof(ep));
    std::memset(pma, 0, sizeof(pma));
    pull_up = false;
    connects = 0;
    stuck = -1;
    in_reset = false;
    setup_violations = 0;
    setups_refused = 0;
    rx_overflows = 0;
    before_read = nullptr;
    before_write = nullptr;
    before_rx_size = nullptr;
  }

  /// ISTR.CTR 是各端点 CTR 的或，EP_ID 为编号最小者 / ISTR.CTR is the OR of the
  /// endpoint CTRs, EP_ID the lowest such number
  static void UpdateIstr()
  {
    for (uint8_t i = 0; i < 8U; ++i)
    {
      if ((ep[i] & (CTR_RX | CTR_TX)) != 0U)
      {
        istr = static_cast<uint16_t>((istr & 0x7FF0U) | 0x8000U | i);
        return;
      }
    }
    istr &= 0x7FF0U;
  }

  static uint32_t ReadEp(uint8_t num)
  {
    if (before_read)
    {
      before_read(num);
    }
    return in_reset ? 0U : ep[num];
  }

  static void WriteEp(uint8_t num, uint32_t value)
  {
    if (before_write)
    {
      before_write(num, value);
    }
    if (in_reset)
    {
      return;
    }
    const uint32_t old = ep[num];
    uint32_t next = old & (value | ~(CTR_RX | CTR_TX));
    next = (next & ~KEEP) | (value & KEEP);
    uint32_t toggle = value & TOGGLE;
    if (static_cast<int>(num) == stuck)
    {
      toggle &= ~(STAT_TX | STAT_RX);
    }
    next ^= toggle;
    next = (next & ~SETUP) | (old & SETUP);
    if (num == 0U && (old & (CTR_RX | SETUP)) == (CTR_RX | SETUP) &&
        (next & STAT_TX) == TX_VALID && (old & STAT_TX) != TX_VALID)
    {
      ++setup_violations;
    }
    ep[num] = next;
    UpdateIstr();
  }

  static uint32_t ReadRegister(uint32_t offset)
  {
    if (offset < 32U)
    {
      return ReadEp(static_cast<uint8_t>(offset / 4U));
    }
    switch (offset)
    {
      case LibXR::USB::FSDEV_REG_CNTR:
        return cntr;
      case LibXR::USB::FSDEV_REG_ISTR:
        return istr;
      case LibXR::USB::FSDEV_REG_FNR:
        ++fnr_reads;
        return fnr;
      case LibXR::USB::FSDEV_REG_DADDR:
        return daddr;
      default:
        return 0U;
    }
  }

  static void WriteRegister(uint32_t offset, uint32_t value)
  {
    if (offset < 32U)
    {
      WriteEp(static_cast<uint8_t>(offset / 4U), value & 0xFFFFU);
      return;
    }
    const auto word = static_cast<uint16_t>(value);
    switch (offset)
    {
      case LibXR::USB::FSDEV_REG_CNTR:
        if ((word & 0x0001U) != 0U)
        {
          // FRES 复位 USB 逻辑：EPnR、ISTR、地址清零 / FRES resets the USB logic
          istr = 0;
          daddr = 0;
          std::memset(ep, 0, sizeof(ep));
          stuck = -1;
        }
        cntr = word;
        UpdateIstr();
        break;
      case LibXR::USB::FSDEV_REG_ISTR:
        istr &= word;
        UpdateIstr();
        break;
      case LibXR::USB::FSDEV_REG_DADDR:
        daddr = word;
        break;
      default:
        break;
    }
  }

  /// 字段 m 在 APB 字节 (8n+2m)*2 / Field m sits at APB byte (8n+2m)*2
  static uint16_t& Field(uint8_t num, unsigned field)
  {
    return *reinterpret_cast<uint16_t*>(pma + (num * 8U + 2U * field) * 2U);
  }

  static void Write(uint16_t addr, const uint8_t* src, size_t len)
  {
    // 数据字 k 在 APB 字节 (addr+2k)*2 / Data word k sits at APB byte (addr+2k)*2
    for (size_t i = 0; i < len; ++i)
    {
      uint8_t* word = pma + (static_cast<size_t>(addr) + (i & ~size_t{1})) * 2U;
      word[i & 1U] = src[i];
    }
  }

  static void Read(uint8_t* dst, uint16_t addr, size_t len)
  {
    for (size_t i = 0; i < len; ++i)
    {
      const uint8_t* word = pma + (static_cast<size_t>(addr) + (i & ~size_t{1})) * 2U;
      dst[i] = word[i & 1U];
    }
  }

  /// COUNT_RX 的 BL_SIZE、NUM_BLOCK 给出的接收区字节数 / Receive size in bytes from
  /// BL_SIZE and NUM_BLOCK of COUNT_RX
  static size_t RxRoom(uint8_t num)
  {
    const uint16_t field = Field(num, 3U);
    const size_t blocks = (field >> 10U) & 0x1FU;
    return (field & 0x8000U) != 0U ? (blocks + 1U) * 32U : blocks * 2U;
  }

  // ---- 主机一侧 / Host side ----

  /**
   * @brief 硬件收到一个 OUT 或 SETUP 包 / The hardware received an OUT or SETUP packet
   * @param force 不看 STAT：写 NAK 时已开始的事务照样完成 / Ignore STAT: a transaction
   *        already started when NAK was written completes anyway
   * @return 是否收下 / Whether it was taken
   */
  static bool RxPacket(uint8_t num, const uint8_t* data, size_t len, bool setup,
                       bool force = false)
  {
    if (!setup && !force && ((ep[num] & STAT_RX) != RX_VALID || !Answers(num)))
    {
      return false;
    }
    // CTR_RX 没清之前硬件不收新的 SETUP（STM32F302 实测）/ The hardware takes no new
    // SETUP while CTR_RX is set (measured on an STM32F302)
    if (setup && ((ep[num] & CTR_RX) != 0U || !Answers(num)))
    {
      ++setups_refused;
      return false;
    }
    // 超过 BL_SIZE、NUM_BLOCK 给出的接收区：不收（硬件回 STALL 并置 PMAOVR）
    // Beyond the receive size BL_SIZE and NUM_BLOCK give: not taken (the hardware
    // answers STALL and sets PMAOVR)
    if (len > RxRoom(num))
    {
      ++rx_overflows;
      return false;
    }
    if (len != 0U)
    {
      Write(Field(num, 2U), data, len);
    }
    Field(num, 3U) = static_cast<uint16_t>((Field(num, 3U) & 0xFC00U) | (len & 0x3FFU));
    ep[num] = (ep[num] & ~(STAT_RX | SETUP)) | CTR_RX | RX_NAK;
    if (setup)
    {
      // RM0008 p.630：DTOG_TX=1、DTOG_RX=0，两方向 NAK / both directions NAK
      ep[num] = (ep[num] & ~(STAT_TX | DTOG_RX)) | SETUP | TX_NAK | DTOG_TX;
    }
    UpdateIstr();
    return true;
  }

  /// 端点 num 的地址字段与它的号相符：控制器才理会发给它的令牌 / The address field of
  /// endpoint num equals its number: only then the controller heeds a token for it
  static bool Answers(uint8_t num) { return (ep[num] & 0x000FU) == num; }

  /// 主机 ACK 了在途的 IN 包 / The host ACKed the IN packet on the wire
  static void AckIn(uint8_t num)
  {
    ep[num] = (ep[num] & ~STAT_TX) | CTR_TX | TX_NAK;
    UpdateIstr();
  }

  /// 端点 num 当前装填的 IN 包 / The IN packet endpoint num has armed
  static Bytes ArmedIn(uint8_t num)
  {
    Bytes data(Field(num, 1U) & 0x3FFU);
    Read(data.data(), Field(num, 0U), data.size());
    return data;
  }

  static void BusReset()
  {
    in_reset = false;
    std::memset(ep, 0, sizeof(ep));
    daddr = 0;
    istr |= 0x0400U;
    UpdateIstr();
  }
};

/// 被测设备：寄存器接到假控制器 / Device under test: registers go to the fake
class TestDevice final : public FSDevDevice
{
 public:
  TestDevice(const std::initializer_list<EPConfig>& rx,
             const std::initializer_list<EPConfig>& tx,
             const std::initializer_list<const std::initializer_list<DeviceClass*>>& cfgs,
             size_t pma_limit = 0)
      : FSDevDevice({0U, reinterpret_cast<uintptr_t>(Ctl::pma), 512U, 2U, 2U}, rx, tx,
                    DeviceDescriptor::PacketSize0::SIZE_64, 0x1209U, 0x0001U, 0x0100U,
                    {&LANGUAGE}, cfgs, {nullptr, 0}, pma_limit)
  {
    if (Ctl::fast_ack)
    {
      EnableEp0FastAck();
    }
    if (Ctl::late_status)
    {
      EnableEp0LateStatus();
    }
    if (Ctl::early_out)
    {
      EnableEp0EarlyOut();
    }
    InitController();
  }

  std::vector<std::string> log;  ///< SETUP 读取与 IN 装填的先后 / Order of SETUP reads
                                 ///< and IN stagings
  bool masked = false;
  unsigned masks = 0;  ///< owner 进来的次数 / Times the owner came in

  static constexpr auto LANGUAGE = DescriptorStrings::MakeLanguagePack(
      DescriptorStrings::Language::EN_US, "XRobot", "FSDEV", "0001");

 protected:
  void Connect(bool on) override
  {
    Ctl::pull_up = on;
    Ctl::connects += on ? 1U : 0U;
  }
  uint32_t ReadRegister(uint32_t offset) override { return Ctl::ReadRegister(offset); }
  void WriteRegister(uint32_t offset, uint32_t value) override
  {
    Ctl::WriteRegister(offset, value);
  }
  void SetRxCountEncoded(uint8_t num, uint16_t mps) override
  {
    if (Ctl::before_rx_size)
    {
      Ctl::before_rx_size(num);
    }
    FSDevDevice::SetRxCountEncoded(num, mps);
  }
  void PmaWrite(uint16_t addr, const uint8_t* src, size_t len) override
  {
    log.push_back("in" + std::to_string(len));
    FSDevDevice::PmaWrite(addr, src, len);
  }
  void PmaRead(uint8_t* dst, uint16_t addr, size_t len) override
  {
    if (len == 8U && addr == Ctl::Field(0, 2U))
    {
      log.push_back("setup");
    }
    FSDevDevice::PmaRead(dst, addr, len);
  }
  void MaskInterrupt() override
  {
    masked = true;
    ++masks;
  }
  void UnmaskInterrupt() override { masked = false; }
};

constexpr uint8_t REQ_IN = 0x01U;
constexpr uint8_t REQ_OUT = 0x02U;

/// 一个接口，批量 IN 与批量 OUT 共用一个端点号 / One interface, bulk IN and bulk OUT
/// sharing one endpoint number
class BulkClass final : public DeviceClass
{
 public:
  explicit BulkClass(EPNumber num = EPNumber::EP1) : num_(num)
  {
    in_cb_ =
        Callback<const Completion&>::Create([](bool, BulkClass* self, const Completion& c)
                                            { self->in_done.push_back(c); }, this);
    out_cb_ = Callback<const Completion&>::Create(
        [](bool, BulkClass* self, const Completion& c)
        {
          self->out_done.push_back(c);
          if (c.slot.Valid())
          {
            const auto* p = static_cast<const uint8_t*>(c.slot.data.addr_);
            self->received.insert(self->received.end(), p, p + c.slot.data.size_);
            (void)self->out->ReleaseSlot(c.slot);
          }
        },
        this);
  }

  ErrorCode Send(const Bytes& bytes)
  {
    Endpoint::Slot slot;
    ErrorCode result = in->AcquireSlot(slot);
    if (result != ErrorCode::OK)
    {
      return result;
    }
    if (!bytes.empty())
    {
      std::memcpy(slot.data.addr_, bytes.data(), bytes.size());
    }
    return in->Submit(slot, bytes.size(), false, false);
  }

  Endpoint* in = nullptr;
  Endpoint* out = nullptr;
  bool arm_out = false;
  Bytes response;
  Bytes data_seen;
  Bytes received;
  size_t completes = 0;
  size_t aborts = 0;
  std::vector<Completion> in_done;
  std::vector<Completion> out_done;

 protected:
  size_t GetInterfaceCount() const override { return 1U; }
  bool HasIAD() const override { return false; }
  size_t GetMaxConfigSize() const override { return sizeof(desc_); }

  ErrorCode WriteDescriptors(const EndpointPool& pool, Speed speed,
                             uint8_t first_interface, ConstRawData& block) override
  {
    const uint16_t size =
        pool.PacketSizeFor(num_, {Direction::IN, Type::BULK, 64U}, speed);
    const auto n = static_cast<uint8_t>(num_);
    const uint8_t init[sizeof(desc_)] = {9U,
                                         4U,
                                         first_interface,
                                         0U,
                                         2U,
                                         0xffU,
                                         0U,
                                         0U,
                                         0U,
                                         7U,
                                         5U,
                                         static_cast<uint8_t>(0x80U | n),
                                         2U,
                                         static_cast<uint8_t>(size),
                                         0U,
                                         0U,
                                         7U,
                                         5U,
                                         n,
                                         2U,
                                         static_cast<uint8_t>(size),
                                         0U,
                                         0U};
    std::memcpy(desc_, init, sizeof(desc_));
    block = {desc_, sizeof(desc_)};
    return ErrorCode::OK;
  }

  ErrorCode BindEndpoints(EndpointPool& pool, uint8_t, bool in_isr) override
  {
    if (pool.Get(in, Direction::IN, num_) != ErrorCode::OK ||
        pool.Get(out, Direction::OUT, num_) != ErrorCode::OK)
    {
      return ErrorCode::NOT_FOUND;
    }
    in->SetOnComplete(in_cb_);
    out->SetOnComplete(out_cb_);
    ErrorCode result = in->Configure({Direction::IN, Type::BULK, 64U}, in_isr);
    if (result == ErrorCode::OK)
    {
      result = out->Configure({Direction::OUT, Type::BULK, 64U}, in_isr);
    }
    if (result == ErrorCode::OK && arm_out)
    {
      result = out->Receive(out->MaxTransferSize(), in_isr);
    }
    return result;
  }

  void UnbindEndpoints(EndpointPool& pool, bool in_isr) override
  {
    for (Endpoint** ep : {&in, &out})
    {
      if (*ep != nullptr)
      {
        (*ep)->Close(in_isr);
        (void)pool.Release(*ep);
        *ep = nullptr;
      }
    }
  }

  bool OwnsEndpoint(uint8_t ep_addr) const override
  {
    return (ep_addr & 0x7FU) == static_cast<uint8_t>(num_);
  }

  size_t GetControlReceiveCapacity() const override { return 256U; }

  ErrorCode OnControlRequest(bool, const SetupPacket& setup,
                             ConstRawData& in_data) override
  {
    if (setup.bRequest == REQ_IN)
    {
      in_data = {response.data(), response.size()};
      return ErrorCode::OK;
    }
    return setup.bRequest == REQ_OUT ? ErrorCode::OK : ErrorCode::NOT_SUPPORT;
  }

  ErrorCode OnControlData(bool, const SetupPacket&, ConstRawData data) override
  {
    const auto* p = static_cast<const uint8_t*>(data.addr_);
    data_seen.assign(p, p + data.size_);
    return ErrorCode::OK;
  }

  void OnControlComplete(bool, const SetupPacket&) override { ++completes; }
  void OnControlAbort(bool, const SetupPacket&) override { ++aborts; }

 private:
  EPNumber num_;
  uint8_t desc_[23] = {};
  Callback<const Completion&> in_cb_;
  Callback<const Completion&> out_cb_;
};

/// 端点内存 / Endpoint memory
struct alignas(64) Block
{
  uint8_t bytes[256];
  RawData Raw(size_t size = 256U) { return {bytes, size}; }
};

Bytes Pattern(size_t count, uint8_t seed)
{
  Bytes data(count);
  for (size_t i = 0; i < count; ++i)
  {
    data[i] = static_cast<uint8_t>(seed + i);
  }
  return data;
}

SetupPacket Make(uint8_t type, uint8_t request, uint16_t value, uint16_t index,
                 uint16_t length)
{
  return SetupPacket{type, request, value, index, length};
}

/// 控制器中断 / The controller interrupt
void Irq() { FSDevDevice::Dispatch(); }

void SendSetup(const SetupPacket& setup)
{
  uint8_t raw[8];
  std::memcpy(raw, &setup, sizeof(raw));
  Ctl::RxPacket(0, raw, sizeof(raw), true);
  TEST_ASSERT(Ctl::rx_overflows == 0U);
}

/// 主机取走装填的 IN 包并 ACK / The host takes the armed IN packet and ACKs it
Bytes TakeIn(uint8_t num)
{
  TEST_ASSERT((Ctl::ep[num] & STAT_TX) == TX_VALID);
  Bytes data = Ctl::ArmedIn(num);
  Ctl::AckIn(num);
  Irq();
  return data;
}

/// 主机发 OUT 包，端点须已 VALID / The host sends an OUT packet; the endpoint is VALID
void SendOut(uint8_t num, const Bytes& data)
{
  TEST_ASSERT(Ctl::RxPacket(num, data.data(), data.size(), false));
  Irq();
}

/// 一个完整的 IN 控制传输 / One complete IN control transfer
Bytes ControlIn(const SetupPacket& setup)
{
  SendSetup(setup);
  Irq();
  Bytes got;
  while (true)
  {
    const Bytes packet = TakeIn(0);
    got.insert(got.end(), packet.begin(), packet.end());
    if (packet.size() < 64U)
    {
      break;
    }
  }
  SendOut(0, {});
  return got;
}

/// 无数据控制传输：状态 IN / A no-data control transfer: the status IN
void ControlNone(const SetupPacket& setup)
{
  SendSetup(setup);
  Irq();
  TEST_ASSERT(TakeIn(0).empty());
}

/// 两个数据端点号：EP1（IN/OUT 各 64）、EP2（IN 40、OUT 96）/ Two data numbers
struct Fixture
{
  explicit Fixture(bool arm_out = false)
      : device({{ep0_out.Raw(64), 64}, {ep1_out.Raw(), 64}, {ep2_out.Raw(), 96}},
               {{ep0_in.Raw(64), 64}, {ep1_in.Raw(128), 64}, {ep2_in.Raw(), 40}},
               {{&klass}})
  {
    klass.arm_out = arm_out;
    device.Init(false);
    device.Start(false);
    TEST_ASSERT(Ctl::pull_up && (Ctl::cntr & 0x8000U) != 0U);
    Ctl::BusReset();
    Irq();
    TEST_ASSERT((Ctl::ep[0] & 0x0600U) == 0x0200U);  // EP0 为 CONTROL / EP0 is CONTROL
    ControlNone(Make(0x00U, 5U, 9U, 0U, 0U));        // SET_ADDRESS 9
    TEST_ASSERT(Ctl::daddr == 0x0089U);
    ControlNone(Make(0x00U, 9U, 1U, 0U, 0U));  // SET_CONFIGURATION 1
    TEST_ASSERT(klass.in != nullptr && klass.out != nullptr);
  }

  // 成员顺序即构造顺序：先清假控制器 / Member order is construction order
  struct Clean
  {
    Clean() { Ctl::Reset(); }
  } clean;
  Block ep0_out, ep0_in, ep1_out, ep1_in, ep2_out, ep2_in;
  BulkClass klass;
  TestDevice device;
};

/// 写 NAK 生效后第 polls 次读该端点时执行 action / Run action at the polls-th read of
/// the endpoint after NAK has landed
void AfterNak(uint8_t num, bool in, unsigned polls, std::function<void()> action)
{
  auto reads = std::make_shared<unsigned>(0U);
  auto armed = std::make_shared<bool>(false);
  Ctl::before_write = [num, in, armed](uint8_t n, uint32_t value)
  {
    const uint32_t mask = in ? STAT_TX : STAT_RX;
    const uint32_t nak = in ? TX_NAK : RX_NAK;
    if (n == num && ((Ctl::ep[n] ^ value) & mask) == nak)
    {
      *armed = true;
    }
  };
  Ctl::before_read = [num, polls, reads, armed, action](uint8_t n)
  {
    // 只触发一次；钩子留到下一个 Fixture 清掉（在钩子里清会销毁正在运行的 lambda）
    // Fires once; the hook stays until the next Fixture clears it (clearing it here
    // would destroy the running lambda)
    if (n == num && *armed && ++*reads == polls)
    {
      action();
    }
  };
}

// ---------------------------------------------------------------------------
// 枚举与控制传输 / Enumeration and control transfers
// ---------------------------------------------------------------------------

void TestEnumeration()
{
  Fixture f;
  const Bytes device = ControlIn(Make(0x80U, 6U, 0x0100U, 0U, 18U));
  TEST_ASSERT(device.size() == 18U && device[0] == 18U && device[1] == 1U);
  TEST_ASSERT(device[7] == 64U);  // bMaxPacketSize0
  // 配置描述符 9 + 接口 9 + 两个端点 7 / Configuration 9 + interface 9 + endpoints 7
  const Bytes config = ControlIn(Make(0x80U, 6U, 0x0200U, 0U, 255U));
  TEST_ASSERT(config.size() == 32U && config[22] == 64U && config[29] == 64U);
  // 类拒绝的请求两方向 STALL / A refused request stalls both directions
  SendSetup(Make(0x41U, 0x77U, 0U, 0U, 0U));
  Irq();
  TEST_ASSERT((Ctl::ep[0] & STAT_TX) == 0x0010U && (Ctl::ep[0] & STAT_RX) == 0x1000U);
  // 下一个 SETUP 解除 / The next SETUP clears it
  f.klass.response = Pattern(100, 0x30);
  TEST_ASSERT(ControlIn(Make(0xC1U, REQ_IN, 0U, 0U, 100U)) == Pattern(100, 0x30));
  TEST_ASSERT(f.klass.completes == 1U);
}

void TestControlWrite()
{
  Fixture f;
  const Bytes payload = Pattern(150, 0x40);
  SendSetup(Make(0x41U, REQ_OUT, 0U, 0U, 150U));
  Irq();
  SendOut(0, Bytes(payload.begin(), payload.begin() + 64));
  SendOut(0, Bytes(payload.begin() + 64, payload.begin() + 128));
  SendOut(0, Bytes(payload.begin() + 128, payload.end()));
  TEST_ASSERT(f.klass.data_seen == payload);
  TEST_ASSERT(TakeIn(0).empty());  // 状态 IN / status IN
  TEST_ASSERT(f.klass.completes == 1U);
}

// ---------------------------------------------------------------------------
// 批量传输与配置 / Bulk transfers and configuration
// ---------------------------------------------------------------------------

void TestBulk()
{
  Fixture f;
  BulkClass& k = f.klass;
  // EP1 IN 槽 64（两槽平分 128）：一次请求一个包 / EP1 IN slots of 64: one packet
  TEST_ASSERT(k.in->MaxTransferSize() == 64U);
  TEST_ASSERT(k.Send(Pattern(64, 1)) == ErrorCode::OK);
  TEST_ASSERT(k.Send(Pattern(10, 2)) == ErrorCode::OK);
  TEST_ASSERT(TakeIn(1) == Pattern(64, 1));
  TEST_ASSERT(TakeIn(1) == Pattern(10, 2));
  TEST_ASSERT(k.in_done.size() == 2U && k.in_done[0].actual == 64U &&
              k.in_done[1].actual == 10U && k.in_done[1].result == ErrorCode::OK);

  // EP1 OUT 单个 256 槽里两槽各 128：两个包一次请求 / Two 128-byte slots: two packets
  TEST_ASSERT(k.out->MaxTransferSize() == 128U);
  TEST_ASSERT(k.out->Receive(128U, false) == ErrorCode::OK);
  TEST_ASSERT((Ctl::ep[1] & STAT_RX) == RX_VALID);
  SendOut(1, Pattern(64, 3));
  TEST_ASSERT(k.out_done.empty() && (Ctl::ep[1] & STAT_RX) == RX_VALID);
  SendOut(1, Pattern(64, 3 + 64));
  TEST_ASSERT(k.out_done.size() == 1U && k.out_done[0].actual == 128U);
  TEST_ASSERT(k.received == Pattern(128, 3));
  // 短包提前结束 / A short packet ends the request early
  TEST_ASSERT(k.out->Receive(128U, false) == ErrorCode::OK);
  SendOut(1, Pattern(20, 9));
  TEST_ASSERT(k.out_done.size() == 2U && k.out_done[1].actual == 20U);
  // 两个方向共用 EP1 的类型字段 / Both directions share EP1's type field
  TEST_ASSERT((Ctl::ep[1] & 0x0600U) == 0x0000U && (Ctl::ep[1] & 0x000FU) == 1U);
}

void TestConfigure()
{
  Fixture f;
  Endpoint* in2 = nullptr;
  Endpoint* out2 = nullptr;
  TEST_ASSERT(f.device.Get(in2, Direction::IN, EPNumber::EP2) == ErrorCode::OK);
  TEST_ASSERT(f.device.Get(out2, Direction::OUT, EPNumber::EP2) == ErrorCode::OK);
  // 包长由 PMA 缓冲决定：IN 40、OUT 96 → 64（速度上限）/ From the PMA buffer
  TEST_ASSERT(f.device.PacketSizeFor(EPNumber::EP2, {Direction::IN, Type::INTR, 64U},
                                     Speed::FULL) == 40U);
  TEST_ASSERT(f.device.PacketSizeFor(EPNumber::EP2, {Direction::OUT, Type::INTR, 64U},
                                     Speed::FULL) == 64U);
  TEST_ASSERT(f.device.PacketSizeFor(EPNumber::EP2, {Direction::IN, Type::BULK, 64U},
                                     Speed::FULL) == 32U);
  // 等时不支持：包长上限为 0，Configure 当场拒绝 / Isochronous is not supported: the
  // packet limit is 0 and Configure refuses at once
  TEST_ASSERT(f.device.PacketSizeFor(EPNumber::EP2,
                                     {Direction::IN, Type::ISOCHRONOUS, 40U},
                                     Speed::FULL) == 0U);
  TEST_ASSERT(in2->Configure({Direction::IN, Type::ISOCHRONOUS, 40U}, false) ==
              ErrorCode::NO_BUFF);
  TEST_ASSERT(in2->Configure({Direction::IN, Type::INTR, 64U}, false) == ErrorCode::OK);
  TEST_ASSERT(in2->MaxPacketSize() == 40U);
  // 同号另一方向类型不同 / The other half of the number with another type
  TEST_ASSERT(out2->Configure({Direction::OUT, Type::BULK, 64U}, false) ==
              ErrorCode::FAILED);
  TEST_ASSERT(out2->Configure({Direction::OUT, Type::INTR, 64U}, false) == ErrorCode::OK);
  // 配置 OUT 不动 IN 的 DTOG / Configuring OUT leaves IN's DTOG alone
  Ctl::ep[2] |= DTOG_TX;
  out2->Close(false);
  TEST_ASSERT(out2->Configure({Direction::OUT, Type::INTR, 64U}, false) == ErrorCode::OK);
  TEST_ASSERT((Ctl::ep[2] & DTOG_TX) != 0U);
}

// ---------------------------------------------------------------------------
// 停止 / Stops
// ---------------------------------------------------------------------------

//// 主机发出下一个帧起始：帧号加一，SOF 中断 / The host sends the next start of frame:
/// the frame number advances and the SOF interrupt fires
void Sof()
{
  Ctl::fnr = static_cast<uint16_t>((Ctl::fnr + 1U) & 0x07FFU);
  Ctl::istr |= 0x0200U;
  Irq();
}

/// 已装填的端点被停时不等：请求当场 ABORTED，停止由本方向的完成或下一个帧起始结束，
/// 期间完成的包不进槽，排队的请求到那时才装填
/// Stopping an armed endpoint does not wait: the request is ABORTED at once, the stop
/// ends on this direction's completion or on the next start of frame, a packet
/// completing meanwhile stays out of the slot, and a queued request is armed only then
void TestStopPending()
{
  // IN：停止后主机 ACK 了线上的那一包 → 完成中断结束停止
  // IN: the host ACKs the packet on the wire after the stop → the completion interrupt
  // ends the stop
  {
    Fixture f;
    BulkClass& k = f.klass;
    TEST_ASSERT(k.Send(Pattern(64, 5)) == ErrorCode::OK);
    k.in->Flush(false);
    TEST_ASSERT(k.in_done.size() == 1U && k.in_done[0].result == ErrorCode::ABORTED &&
                k.in_done[0].actual == 0U);
    TEST_ASSERT((Ctl::ep[1] & STAT_TX) == TX_NAK);
    // 被停的槽已放回：两个槽都能再用 / The stopped slot was given back: both slots
    // can be used again
    TEST_ASSERT(k.Send(Pattern(10, 2)) == ErrorCode::OK);
    TEST_ASSERT(k.Send(Pattern(12, 3)) == ErrorCode::OK);
    Irq();  // 同一帧、没有完成：停止仍未结束 / same frame, no completion: still pending
    TEST_ASSERT((Ctl::ep[1] & STAT_TX) == TX_NAK);
    Ctl::AckIn(1);
    Irq();
    TEST_ASSERT(k.in_done.size() == 1U);
    TEST_ASSERT((Ctl::ep[1] & (CTR_TX | STAT_TX)) == TX_VALID);
    TEST_ASSERT(TakeIn(1) == Pattern(10, 2));
    TEST_ASSERT(TakeIn(1) == Pattern(12, 3));
    TEST_ASSERT(k.in_done.size() == 3U && k.in_done[1].actual == 10U);
    // 没有停止未结束时不再读帧号 / The frame number is not read with no stop pending
    const unsigned reads = Ctl::fnr_reads;
    Irq();
    TEST_ASSERT(Ctl::fnr_reads == reads);
  }
  // OUT：停止后线上的包照样写进 PMA → 只清 CTR，不拷进已放回的槽
  // OUT: the packet on the wire still lands in the PMA after the stop → only its CTR is
  // cleared and nothing is copied into the slot already given back
  {
    Fixture f;
    BulkClass& k = f.klass;
    TEST_ASSERT(k.out->Receive(128U, false) == ErrorCode::OK);
    k.out->Flush(false);
    TEST_ASSERT(k.out_done.size() == 1U && k.out_done[0].result == ErrorCode::ABORTED &&
                k.out_done[0].actual == 0U);
    TEST_ASSERT((Ctl::ep[1] & STAT_RX) == RX_NAK);
    std::memset(f.ep1_out.bytes, 0xEE, sizeof(f.ep1_out.bytes));
    TEST_ASSERT(k.out->Receive(128U, false) == ErrorCode::OK);
    const Bytes late = Pattern(64, 7);
    TEST_ASSERT(Ctl::RxPacket(1, late.data(), late.size(), false, true));
    Irq();
    TEST_ASSERT(k.out_done.size() == 1U);
    TEST_ASSERT(f.ep1_out.bytes[0] == 0xEE && f.ep1_out.bytes[128] == 0xEE);
    // 排队的接收这时才装填，照常收包 / The queued receive is armed only now and works
    TEST_ASSERT((Ctl::ep[1] & (CTR_RX | STAT_RX)) == RX_VALID);
    SendOut(1, Pattern(20, 9));
    TEST_ASSERT(k.out_done.size() == 2U && k.out_done[1].result == ErrorCode::OK &&
                k.out_done[1].actual == 20U);
    TEST_ASSERT(k.received == Pattern(20, 9));
  }
  // 线上没有事务：下一个帧起始结束停止；同一帧里的中断不结束
  // No transaction on the wire: the next start of frame ends the stop; an interrupt in
  // the same frame does not
  {
    Fixture f;
    BulkClass& k = f.klass;
    TEST_ASSERT(k.Send(Pattern(64, 5)) == ErrorCode::OK);
    Ctl::fnr = 5U;
    k.in->Flush(false);
    TEST_ASSERT(k.in_done.size() == 1U);
    TEST_ASSERT(k.Send(Pattern(10, 2)) == ErrorCode::OK);
    Ctl::istr |= 0x0200U;  // 停止之前那一帧的 SOF 标志 / SOF flag of the frame before
    Irq();
    TEST_ASSERT((Ctl::ep[1] & STAT_TX) == TX_NAK);
    Sof();
    TEST_ASSERT((Ctl::ep[1] & STAT_TX) == TX_VALID && Ctl::ArmedIn(1) == Pattern(10, 2));
    TEST_ASSERT(k.in_done.size() == 1U);
  }
  // 帧起始和线上那一包的完成在同一轮读到：CTR 被清，不算进下一个请求
  // The start of frame and the completion of the packet on the wire are read in one
  // pass: the CTR is cleared and does not count for the next request
  {
    Fixture f;
    BulkClass& k = f.klass;
    TEST_ASSERT(k.Send(Pattern(64, 5)) == ErrorCode::OK);
    k.in->Flush(false);
    Ctl::ep[1] = (Ctl::ep[1] & ~STAT_TX) | CTR_TX | TX_NAK;  // 不更新 ISTR.CTR
    Ctl::fnr = 7U;
    Ctl::istr |= 0x0200U;
    Irq();
    TEST_ASSERT((Ctl::ep[1] & (CTR_TX | STAT_TX)) == TX_NAK);
    TEST_ASSERT(k.in_done.size() == 1U);
    TEST_ASSERT(k.Send(Pattern(10, 2)) == ErrorCode::OK);
    TEST_ASSERT(TakeIn(1) == Pattern(10, 2));
    TEST_ASSERT(k.in_done.size() == 2U);
  }
}

/// 已锁存的最后一包和零长度包算完成 / A latched last packet and a latched
/// zero-length packet count as done
void TestStopLatchedCompletion()
{
  Fixture f;
  TEST_ASSERT(f.klass.Send(Pattern(64, 1)) == ErrorCode::OK);
  Ctl::AckIn(1);  // ISR 还没跑 / the ISR has not run yet
  f.klass.in->Flush(false);
  TEST_ASSERT(f.klass.in_done.size() == 1U &&
              f.klass.in_done[0].result == ErrorCode::OK &&
              f.klass.in_done[0].actual == 64U);
  TEST_ASSERT(f.klass.Send({}) == ErrorCode::OK);
  TEST_ASSERT((Ctl::ep[1] & STAT_TX) == TX_VALID && Ctl::ArmedIn(1).empty());
  Ctl::AckIn(1);
  f.klass.in->Flush(false);
  TEST_ASSERT(f.klass.in_done.size() == 2U &&
              f.klass.in_done[1].result == ErrorCode::OK &&
              f.klass.in_done[1].actual == 0U);
  // 第一包已收、第二包未装填（CTR 已锁存）：STAT 不是 VALID，当场给结果，下一个请求
  // 立即装填
  // First packet latched, second not armed: STAT is not VALID, the outcome comes at once
  // and the next request is armed immediately
  TEST_ASSERT(f.klass.out->Receive(128U, false) == ErrorCode::OK);
  TEST_ASSERT(Ctl::RxPacket(1, Pattern(64, 1).data(), 64U, false));
  f.klass.out->Flush(false);
  TEST_ASSERT(f.klass.out_done.size() == 1U &&
              f.klass.out_done[0].result == ErrorCode::ABORTED &&
              f.klass.out_done[0].actual == 64U);
  TEST_ASSERT(f.klass.out->Receive(128U, false) == ErrorCode::OK);
  TEST_ASSERT((Ctl::ep[1] & (CTR_RX | STAT_RX)) == RX_VALID);
}

/// 写 NAK 的过程中事务完成：CTR 已锁存，当场收下 / A transaction completing while NAK is
/// being written: its CTR is latched and collected at once
void TestStopCompletesInWrite()
{
  Fixture f;
  TEST_ASSERT(f.klass.Send(Pattern(64, 5)) == ErrorCode::OK);
  AfterNak(1, true, 1, [] { Ctl::AckIn(1); });
  f.klass.in->Flush(false);
  TEST_ASSERT(f.klass.in_done.size() == 1U &&
              f.klass.in_done[0].result == ErrorCode::OK &&
              f.klass.in_done[0].actual == 64U);
  TEST_ASSERT((Ctl::ep[1] & (CTR_TX | STAT_TX)) == TX_NAK);
  // 没有停止未结束：下一个请求立即装填 / No stop is pending: the next request is armed
  // at once
  TEST_ASSERT(f.klass.Send(Pattern(10, 2)) == ErrorCode::OK);
  TEST_ASSERT((Ctl::ep[1] & STAT_TX) == TX_VALID);
}

/// 停止的 NAK 写入撞上事务完成：硬件刚回的 NAK 被这次翻转写回 VALID。后端必须在下一次
/// 读寄存器之前把它改回去，否则主机的下一个令牌被接受，同一包再发一遍
/// The NAK write of a stop meets a completing transaction: the toggle turns the NAK the
/// hardware has just set back into VALID. The backend has to correct it before its next
/// register read, or the host's next token is accepted and the same packet goes out again
void TestStopWriteUndoesNak()
{
  Fixture f;
  TEST_ASSERT(f.klass.Send(Pattern(64, 5)) == ErrorCode::OK);
  bool raced = false;
  unsigned reads = 0U;
  bool retaken = false;
  Ctl::before_write = [&](uint8_t num, uint32_t value)
  {
    // 第一次把 VALID 翻成 NAK 的写入之前，事务完成 / The transaction completes just
    // before the first write that toggles VALID to NAK
    if (num == 1U && !raced && (Ctl::ep[1] & STAT_TX) == TX_VALID &&
        (value & STAT_TX) == 0x0010U)
    {
      raced = true;
      Ctl::AckIn(1);
    }
  };
  Ctl::before_read = [&](uint8_t num)
  {
    // 那次写入之后的第二次读时主机的下一个令牌到达 / The host's next token arrives at
    // the second read after that write
    if (num == 1U && raced && ++reads == 2U && Ctl::Answers(1) &&
        (Ctl::ep[1] & STAT_TX) == TX_VALID)
    {
      retaken = true;
      Ctl::AckIn(1);
    }
  };
  f.klass.in->Flush(false);
  Ctl::before_write = nullptr;
  Ctl::before_read = nullptr;
  TEST_ASSERT(raced && !retaken);
  TEST_ASSERT(f.klass.in_done.size() == 1U &&
              f.klass.in_done[0].result == ErrorCode::OK &&
              f.klass.in_done[0].actual == 64U);
  TEST_ASSERT((Ctl::ep[1] & (CTR_TX | STAT_TX)) == TX_NAK);
  TEST_ASSERT(Ctl::Answers(1));

  // 主机的下一个令牌在那次写入之后的第一次读时就到：只有地址不匹配才挡得住它
  // The host's next token arrives at the very first read after that write: only the
  // mismatching address keeps it out
  TEST_ASSERT(f.klass.Send(Pattern(64, 7)) == ErrorCode::OK);
  raced = false;
  reads = 0U;
  Ctl::before_write = [&](uint8_t num, uint32_t value)
  {
    if (num == 1U && !raced && (Ctl::ep[1] & STAT_TX) == TX_VALID &&
        (value & STAT_TX) == 0x0010U)
    {
      raced = true;
      Ctl::AckIn(1);
    }
  };
  Ctl::before_read = [&](uint8_t num)
  {
    if (num == 1U && raced && ++reads == 1U && Ctl::Answers(1) &&
        (Ctl::ep[1] & STAT_TX) == TX_VALID)
    {
      retaken = true;
      Ctl::AckIn(1);
    }
  };
  f.klass.in->Flush(false);
  Ctl::before_write = nullptr;
  Ctl::before_read = nullptr;
  TEST_ASSERT(raced && !retaken && Ctl::Answers(1));
  TEST_ASSERT((Ctl::ep[1] & (CTR_TX | STAT_TX)) == TX_NAK);
}

/// 主机对没有 STALL、硬件上有请求的端点发 CLEAR_FEATURE(ENDPOINT_HALT)：翻转位回到
/// DATA0，请求留在硬件上继续等主机，下一个包照样完成它
/// The host sends CLEAR_FEATURE(ENDPOINT_HALT) to an endpoint that is not halted and
/// has a request on the hardware: the toggle returns to DATA0, the request stays on the
/// hardware and the next packet completes it
void TestClearStallKeepsOut()
{
  // OUT：接收已装填 / OUT with a receive armed
  {
    Fixture f;
    BulkClass& k = f.klass;
    TEST_ASSERT(k.out->Receive(64U, false) == ErrorCode::OK);
    TEST_ASSERT((Ctl::ep[1] & STAT_RX) == RX_VALID);
    Ctl::ep[1] |= DTOG_RX | DTOG_TX;  // 之前收过一个 DATA0 / A DATA0 came before
    TEST_ASSERT(k.out->ClearStall(false) == ErrorCode::OK);
    TEST_ASSERT((Ctl::ep[1] & STAT_RX) == RX_VALID);
    TEST_ASSERT((Ctl::ep[1] & DTOG_RX) == 0U);
    TEST_ASSERT((Ctl::ep[1] & DTOG_TX) != 0U);  // 另一方向不动 / Other half untouched
    TEST_ASSERT(Ctl::Answers(1));
    SendOut(1, Pattern(64, 0x11));
    TEST_ASSERT(k.out_done.size() == 1U && k.out_done[0].result == ErrorCode::OK &&
                k.out_done[0].actual == 64U);
    TEST_ASSERT(k.received == Pattern(64, 0x11));
    TEST_ASSERT(k.out->Receive(64U, false) == ErrorCode::OK);
    SendOut(1, Pattern(10, 0x21));
    TEST_ASSERT(k.out_done.size() == 2U && k.out_done[1].actual == 10U);
  }
  // 没装填：照旧 NAK、DATA0 / Nothing armed: NAK and DATA0 as before
  {
    Fixture f;
    Ctl::ep[1] |= DTOG_RX;
    TEST_ASSERT(f.klass.out->ClearStall(false) == ErrorCode::OK);
    TEST_ASSERT((Ctl::ep[1] & (STAT_RX | DTOG_RX)) == RX_NAK && Ctl::Answers(1));
  }
}

void TestClearStallKeepsIn()
{
  // IN：包已装填 / IN with a packet loaded
  {
    Fixture f;
    BulkClass& k = f.klass;
    TEST_ASSERT(k.Send(Pattern(40, 0x31)) == ErrorCode::OK);
    TEST_ASSERT((Ctl::ep[1] & STAT_TX) == TX_VALID);
    Ctl::ep[1] |= DTOG_TX | DTOG_RX;
    TEST_ASSERT(k.in->ClearStall(false) == ErrorCode::OK);
    TEST_ASSERT((Ctl::ep[1] & STAT_TX) == TX_VALID);
    TEST_ASSERT((Ctl::ep[1] & DTOG_TX) == 0U);
    TEST_ASSERT((Ctl::ep[1] & DTOG_RX) != 0U);
    TEST_ASSERT(Ctl::Answers(1));
    TEST_ASSERT(TakeIn(1) == Pattern(40, 0x31));
    TEST_ASSERT(k.in_done.size() == 1U && k.in_done[0].result == ErrorCode::OK &&
                k.in_done[0].actual == 40U);
    TEST_ASSERT(k.Send(Pattern(5, 0x41)) == ErrorCode::OK);
    TEST_ASSERT(TakeIn(1) == Pattern(5, 0x41));
  }
}

/// 有 STALL 时照旧：解除后 NAK 或装填排队的请求，DATA0 / The halted case as before:
/// after the clear NAK, or the queued request armed, and DATA0
void TestClearStallHalted()
{
  Fixture f;
  BulkClass& k = f.klass;
  // 没有排队的请求 / Nothing queued
  TEST_ASSERT(k.out->Stall(false) == ErrorCode::OK);
  TEST_ASSERT((Ctl::ep[1] & STAT_RX) == 0x1000U);
  Ctl::ep[1] |= DTOG_RX;
  TEST_ASSERT(k.out->ClearStall(false) == ErrorCode::OK);
  TEST_ASSERT((Ctl::ep[1] & (STAT_RX | DTOG_RX)) == RX_NAK && Ctl::Answers(1));
  // STALL 期间提交的请求在解除后装填 / A request submitted while halted is armed after
  TEST_ASSERT(k.in->Stall(false) == ErrorCode::OK);
  TEST_ASSERT((Ctl::ep[1] & STAT_TX) == 0x0010U);
  TEST_ASSERT(k.Send(Pattern(20, 0x51)) == ErrorCode::OK);
  TEST_ASSERT((Ctl::ep[1] & STAT_TX) == 0x0010U);
  Ctl::ep[1] |= DTOG_TX;
  TEST_ASSERT(k.in->ClearStall(false) == ErrorCode::OK);
  TEST_ASSERT((Ctl::ep[1] & (STAT_TX | DTOG_TX)) == TX_VALID && Ctl::Answers(1));
  TEST_ASSERT(TakeIn(1) == Pattern(20, 0x51));
  TEST_ASSERT(k.in_done.size() == 1U && k.in_done[0].result == ErrorCode::OK);
}

/// 解除时线上那个包恰在改翻转位之前结束、硬件自己翻了翻转位：读回发现后再改一次，最后
/// 仍是 DATA0；那个包照常完成请求，只完成一次
/// The packet on the wire when the clear starts ends just before the toggle write and
/// the hardware toggles DTOG itself: the read-back sees it and writes once more, so the
/// result is still DATA0; that packet completes the request as usual, once
void TestClearStallRace()
{
  Fixture f;
  BulkClass& k = f.klass;
  TEST_ASSERT(k.out->Receive(64U, false) == ErrorCode::OK);
  Ctl::ep[1] |= DTOG_RX;
  const Bytes data = Pattern(64, 0x61);
  bool fired = false;
  bool taken_inside = false;
  unsigned reads = 0U;
  bool clearing = true;
  Ctl::before_write = [&](uint8_t num, uint32_t value)
  {
    if (num == 1U && !fired && (value & DTOG_RX) != 0U)
    {
      fired = true;
      TEST_ASSERT(Ctl::RxPacket(1, data.data(), data.size(), false, true));
      Ctl::ep[1] ^= DTOG_RX;  // 硬件在包结束时翻转 / Hardware toggles at the end
    }
  };
  Ctl::before_read = [&](uint8_t num)
  {
    // 第二次读起主机的新令牌到达：没有地址不匹配挡着就会被收下
    // From the second read on, a new host token arrives: without the mismatching
    // address it would be taken
    if (num == 1U && clearing && ++reads >= 2U && Ctl::Answers(1) &&
        (Ctl::ep[1] & STAT_RX) == RX_VALID)
    {
      taken_inside = true;
    }
  };
  TEST_ASSERT(k.out->ClearStall(false) == ErrorCode::OK);
  clearing = false;
  Ctl::before_write = nullptr;
  Ctl::before_read = nullptr;
  TEST_ASSERT(fired);
  TEST_ASSERT(!taken_inside);
  TEST_ASSERT((Ctl::ep[1] & DTOG_RX) == 0U && Ctl::Answers(1));
  Irq();
  TEST_ASSERT(k.out_done.size() == 1U && k.out_done[0].actual == 64U);
  TEST_ASSERT(k.received == data);
  TEST_ASSERT((Ctl::ep[1] & (CTR_RX | STAT_RX)) == RX_NAK);
  TEST_ASSERT(k.out->Receive(64U, false) == ErrorCode::OK);
  SendOut(1, Pattern(8, 0x71));
  TEST_ASSERT(k.out_done.size() == 2U && k.out_done[1].actual == 8U);
}

/// EP0 的地址字段从不改动：停止它的时候来的 SETUP 照样被收下 / The address field of
/// EP0 is never changed: a SETUP arriving while it is stopped is taken all the same
void TestEp0KeepsAddress()
{
  Fixture f;
  f.klass.response = Pattern(100, 0x30);
  SendSetup(Make(0xC1U, REQ_IN, 0U, 0U, 100U));
  Irq();
  TEST_ASSERT((Ctl::ep[0] & STAT_TX) == TX_VALID);
  bool wrote = false;
  bool sent = false;
  bool taken = false;
  Ctl::before_write = [&](uint8_t num, uint32_t)
  {
    if (num == 0U)
    {
      wrote = true;
    }
  };
  Ctl::before_read = [&](uint8_t num)
  {
    // 停止对 EP0R 的第一次写之后，主机的下一个 SETUP 到达 / The host's next SETUP
    // arrives right after the first write the stop makes to EP0R
    if (num == 0U && wrote && !sent)
    {
      sent = true;
      uint8_t raw[8];
      const SetupPacket next = Make(0x80U, 6U, 0x0100U, 0U, 18U);
      std::memcpy(raw, &next, sizeof(raw));
      taken = Ctl::RxPacket(0, raw, sizeof(raw), true);
    }
  };
  f.device.Deinit(false);
  Ctl::before_write = nullptr;
  Ctl::before_read = nullptr;
  TEST_ASSERT(sent && taken);
}

/// 总线挂起和总线复位结束未结束的停止；挂起期间的停止当场完成
/// Suspend and bus reset end a pending stop; a stop while suspended completes at once
void TestStopBusEvents()
{
  Fixture f;
  BulkClass& k = f.klass;
  // 挂起结束未结束的停止；挂起期间不启动请求，唤醒后排队的请求立即装填，不用等帧起始
  // Suspend ends the pending stop; no request starts while suspended, and after the
  // wake-up the queued request is armed at once, without waiting for a start of frame
  TEST_ASSERT(k.Send(Pattern(64, 5)) == ErrorCode::OK);
  k.in->Flush(false);
  TEST_ASSERT(k.in_done.size() == 1U);
  TEST_ASSERT(k.Send(Pattern(10, 2)) == ErrorCode::OK);
  TEST_ASSERT((Ctl::ep[1] & STAT_TX) == TX_NAK);
  Ctl::istr |= 0x0800U;  // SUSP
  Irq();
  Ctl::istr |= 0x1000U;  // WKUP
  Irq();
  TEST_ASSERT((Ctl::ep[1] & STAT_TX) == TX_VALID && Ctl::ArmedIn(1) == Pattern(10, 2));
  // 挂起期间的停止当场完成 / A stop while suspended completes at once
  Ctl::istr |= 0x0800U;
  Irq();
  k.in->Flush(false);
  TEST_ASSERT(k.in_done.size() == 2U && k.in_done[1].result == ErrorCode::ABORTED);
  TEST_ASSERT(k.Send(Pattern(12, 3)) == ErrorCode::OK);
  Ctl::istr |= 0x1000U;
  Irq();
  TEST_ASSERT((Ctl::ep[1] & STAT_TX) == TX_VALID && Ctl::ArmedIn(1) == Pattern(12, 3));
  // 唤醒之后又按帧号等 / After the wake-up the frame rule applies again
  k.in->Flush(false);
  TEST_ASSERT(k.in_done.size() == 3U);
  TEST_ASSERT(k.Send(Pattern(14, 4)) == ErrorCode::OK);
  Irq();
  TEST_ASSERT((Ctl::ep[1] & STAT_TX) == TX_NAK);
  // 总线复位：停止在报告复位之前结束，之后照常枚举和收发
  // Bus reset: the stop ends before the reset is reported; enumeration and transfers
  // work afterwards
  Ctl::BusReset();
  Irq();
  TEST_ASSERT(k.in == nullptr);
  ControlNone(Make(0x00U, 9U, 1U, 0U, 0U));
  TEST_ASSERT(k.in != nullptr);
  TEST_ASSERT(k.Send(Pattern(30, 4)) == ErrorCode::OK);
  TEST_ASSERT(TakeIn(1) == Pattern(30, 4));
}

/// 停止结束之后才出现的 CTR 不属于下一个请求 / A CTR showing up after the stop ended
/// does not belong to the next request
void TestCompletionAfterStop()
{
  Fixture f;
  TEST_ASSERT(f.klass.Send(Pattern(64, 1)) == ErrorCode::OK);
  f.klass.in->Flush(false);
  TEST_ASSERT(f.klass.in_done.size() == 1U &&
              f.klass.in_done[0].result == ErrorCode::ABORTED);
  Sof();
  Ctl::ep[1] = (Ctl::ep[1] & ~STAT_TX) | CTR_TX | TX_NAK;  // 中断还没跑 / no ISR yet
  TEST_ASSERT(f.klass.Send(Pattern(10, 2)) == ErrorCode::OK);
  TEST_ASSERT((Ctl::ep[1] & (CTR_TX | STAT_TX)) == TX_VALID);
  TEST_ASSERT(TakeIn(1) == Pattern(10, 2));
  TEST_ASSERT(f.klass.in_done.size() == 2U && f.klass.in_done[1].actual == 10U);

  // 没有请求时中断里看到的 CTR 只清除：不装填、不写槽 / A CTR the interrupt sees with
  // no request is only cleared: nothing is armed and no slot is written
  Ctl::AckIn(1);
  Irq();
  TEST_ASSERT((Ctl::ep[1] & (CTR_TX | STAT_TX)) == TX_NAK);
  TEST_ASSERT(f.klass.out->Receive(128U, false) == ErrorCode::OK);
  f.klass.out->Flush(false);
  TEST_ASSERT(f.klass.out_done.size() == 1U);
  Sof();
  std::memset(f.ep1_out.bytes, 0xEE, sizeof(f.ep1_out.bytes));
  const Bytes late = Pattern(64, 9);
  TEST_ASSERT(Ctl::RxPacket(1, late.data(), late.size(), false, true));
  Irq();
  TEST_ASSERT((Ctl::ep[1] & (CTR_RX | STAT_RX)) == RX_NAK &&
              f.klass.out_done.size() == 1U);
  TEST_ASSERT(f.ep1_out.bytes[0] == 0xEE && f.ep1_out.bytes[128] == 0xEE);
}

/// NAK 写不进：下一轮重写一次，仍写不进报 TIMEOUT，设备核心复位控制器；重写成功则照常
/// 按帧号结束
/// NAK cannot be written: it is written once more in the next pass; TIMEOUT if that
/// fails too and the device core resets the controller; if it succeeds the stop ends by
/// the frame rule as usual
void TestStopTimeout()
{
  {
    Fixture f;
    TEST_ASSERT(f.klass.Send(Pattern(64, 1)) == ErrorCode::OK);
    Ctl::stuck = 1;
    f.klass.in->Flush(false);
    // 请求当场回调一次 / The request is called back once, at the stop
    TEST_ASSERT(f.klass.in_done.size() == 1U &&
                f.klass.in_done[0].result == ErrorCode::ABORTED);
    const unsigned connects = Ctl::connects;
    Irq();  // 重写仍失败：TIMEOUT，控制器复位 / the rewrite fails too: TIMEOUT, reset
    TEST_ASSERT(Ctl::connects == connects + 1U && Ctl::pull_up);
    TEST_ASSERT((Ctl::cntr & 0x8000U) != 0U && Ctl::stuck == -1);
    TEST_ASSERT(f.klass.in_done.size() == 1U);
    // 主机重新枚举后照常工作 / Works again after the host enumerates again
    Ctl::BusReset();
    Irq();
    ControlNone(Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(f.klass.Send(Pattern(30, 4)) == ErrorCode::OK);
    TEST_ASSERT(TakeIn(1) == Pattern(30, 4));
    TEST_ASSERT(f.klass.in_done.size() == 2U && f.klass.in_done[1].actual == 30U);
  }
  {
    Fixture f;
    TEST_ASSERT(f.klass.Send(Pattern(64, 1)) == ErrorCode::OK);
    Ctl::stuck = 1;
    f.klass.in->Flush(false);
    TEST_ASSERT((Ctl::ep[1] & STAT_TX) == TX_VALID);
    Ctl::stuck = -1;
    const unsigned connects = Ctl::connects;
    TEST_ASSERT(f.klass.Send(Pattern(10, 2)) == ErrorCode::OK);
    // 帧号已变，但 NAK 这一轮才写进；写的时候又来了一个帧起始。都不算：从写进之后读到
    // 的帧号重新算
    // The frame has changed, but the NAK is written only in this pass, and another
    // start of frame comes during the write. Neither counts: the rule restarts from the
    // frame number read after the write
    Ctl::fnr = 3U;
    Ctl::before_write = [](uint8_t num, uint32_t)
    {
      if (num == 1U)
      {
        Ctl::fnr = 4U;
      }
    };
    Irq();
    Ctl::before_write = nullptr;
    TEST_ASSERT((Ctl::ep[1] & STAT_TX) == TX_NAK && Ctl::connects == connects);
    Irq();
    TEST_ASSERT((Ctl::ep[1] & STAT_TX) == TX_NAK);
    Sof();
    TEST_ASSERT((Ctl::ep[1] & STAT_TX) == TX_VALID && Ctl::ArmedIn(1) == Pattern(10, 2));
    TEST_ASSERT(f.klass.in_done.size() == 1U && Ctl::connects == connects);
  }
}

/// 总线复位的 SE0 期间寄存器读作 0、写不进（RESET 中断约 10 ms 后才来）：这时启动的
/// 请求不算硬件失败，不回调 FAILED，留到复位时以 ABORTED 结束一次
/// During the SE0 of a bus reset the registers read 0 and ignore writes (the RESET
/// interrupt comes about 10 ms later): a request started then is no hardware failure,
/// gets no FAILED callback and ends once with ABORTED at the reset
void TestStartDuringReset()
{
  Fixture f;
  BulkClass& k = f.klass;
  Ctl::in_reset = true;
  TEST_ASSERT(k.Send(Pattern(10, 1)) == ErrorCode::OK);
  TEST_ASSERT(k.out->Receive(128U, false) == ErrorCode::OK);
  Irq();
  TEST_ASSERT(k.in_done.empty() && k.out_done.empty());
  // 这期间的停止当场完成 / A stop during this time completes at once
  k.out->Flush(false);
  TEST_ASSERT(k.out_done.size() == 1U && k.out_done[0].result == ErrorCode::ABORTED);
  Ctl::BusReset();
  Irq();
  TEST_ASSERT(k.in_done.size() == 1U && k.in_done[0].result == ErrorCode::ABORTED);
  TEST_ASSERT(k.in == nullptr);
  ControlNone(Make(0x00U, 9U, 1U, 0U, 0U));
  TEST_ASSERT(k.Send(Pattern(30, 4)) == ErrorCode::OK);
  TEST_ASSERT(TakeIn(1) == Pattern(30, 4));
}

/// Stop 经过 owner、不等总线：在途请求当场 ABORTED、类解绑；未结束的停止由下一个
/// 帧起始结束，之后才断开、关中断源；再 Start、主机复位，照常枚举和收发
/// Stop goes through the owner and never waits for the bus: requests in flight are
/// ABORTED at once and the class unbound; the pending stops end at the next start of
/// frame and only then does it detach and switch the interrupt sources off; after
/// Start and the host's reset, enumeration and transfers work as usual
void TestDeviceStop()
{
  Fixture f;
  BulkClass& k = f.klass;
  TEST_ASSERT(k.Send(Pattern(64, 5)) == ErrorCode::OK);
  k.in->Flush(false);  // 停止未结束 / the stop is pending
  TEST_ASSERT(k.in_done.size() == 1U && (Ctl::ep[1] & STAT_TX) == TX_NAK);
  TEST_ASSERT(k.Send(Pattern(10, 2)) == ErrorCode::OK);       // 排队 / queued
  TEST_ASSERT(k.out->Receive(128U, false) == ErrorCode::OK);  // 在途 / in flight
  f.device.Stop(false);
  TEST_ASSERT(k.in == nullptr && k.out == nullptr);
  TEST_ASSERT(k.in_done.size() == 2U && k.in_done[1].result == ErrorCode::ABORTED);
  TEST_ASSERT(k.out_done.size() == 1U && k.out_done[0].result == ErrorCode::ABORTED);
  // 两个停止还没结束：总线还接着，中断源还开着 / Both stops are still pending: the bus
  // stays attached and the interrupt sources on
  TEST_ASSERT(Ctl::pull_up && (Ctl::cntr & 0x8200U) == 0x8200U);
  Irq();
  TEST_ASSERT(Ctl::pull_up);
  Sof();
  TEST_ASSERT(!Ctl::pull_up && (Ctl::cntr & 0x8200U) == 0U);
  // 没有停止留到之后：再来的中断不读帧号 / No stop is left behind: a later interrupt
  // does not read the frame number
  const unsigned reads = Ctl::fnr_reads;
  Irq();
  TEST_ASSERT(Ctl::fnr_reads == reads);
  // 紧接着 Start：上拉要在断开至少 5 ms 之后才接回 / Start right away: the pull-up
  // comes back no earlier than 5 ms after the detach
  const auto detached = std::chrono::steady_clock::now();
  f.device.Start(false);
  TEST_ASSERT(std::chrono::steady_clock::now() - detached >=
              std::chrono::microseconds(4000));
  TEST_ASSERT(Ctl::pull_up && (Ctl::cntr & 0x8200U) == 0x8200U);
  Ctl::BusReset();
  Irq();
  ControlNone(Make(0x00U, 9U, 1U, 0U, 0U));
  TEST_ASSERT(k.in != nullptr);
  TEST_ASSERT(k.Send(Pattern(30, 4)) == ErrorCode::OK);
  TEST_ASSERT(TakeIn(1) == Pattern(30, 4));
}

// ---------------------------------------------------------------------------
// SETUP / SETUP
// ---------------------------------------------------------------------------

/// 一次读到 IN 完成和 SETUP：完成先交付，旧传输的下一包不装填 / IN completion
/// and SETUP read together: the completion goes first and the old transfer's next packet
/// is not armed
void TestSetupWithInCompletion()
{
  Fixture f;
  f.klass.response = Pattern(100, 0x50);
  SendSetup(Make(0xC1U, REQ_IN, 0U, 0U, 100U));
  Irq();
  TEST_ASSERT(Ctl::ArmedIn(0) == Pattern(64, 0x50));
  f.device.log.clear();
  Ctl::AckIn(0);                                 // 第一包被 ACK / first packet ACKed
  SendSetup(Make(0x80U, 6U, 0x0100U, 0U, 18U));  // 紧接着新 SETUP / then a new SETUP
  Irq();
  TEST_ASSERT(Ctl::setup_violations == 0U);
  // 完成报告不启动下一个请求（启动在本轮末尾的端点阶段）：SETUP 先让旧传输作废，第二包
  // （36 字节）连 PMA 都不写
  // A completion report starts no request (starts happen in the endpoint step at the
  // end of the pass): the SETUP voids the old transfer first, and the second packet (36
  // bytes) is not even written to the PMA
  TEST_ASSERT(f.device.log.size() == 2U && f.device.log[0] == "setup" &&
              f.device.log[1] == "in18");
  TEST_ASSERT(f.klass.aborts == 1U && f.klass.completes == 0U);
  const Bytes device = TakeIn(0);
  TEST_ASSERT(device.size() == 18U && device[1] == 1U);
  SendOut(0, {});
}

/// SETUP 落在装填 EP0 IN 的读改写窗口里：写回 NAK / A SETUP lands
/// in the read-modify-write window of arming EP0 IN: NAK is written back
void TestSetupInArmWindow()
{
  Fixture f;
  f.klass.response = Pattern(20, 0x60);
  bool injected = false;
  Ctl::before_write = [&injected](uint8_t num, uint32_t value)
  {
    if (num == 0U && !injected && ((Ctl::ep[0] ^ value) & STAT_TX) == TX_VALID)
    {
      injected = true;
      SendSetup(Make(0x80U, 6U, 0x0100U, 0U, 18U));
    }
  };
  SendSetup(Make(0xC1U, REQ_IN, 0U, 0U, 20U));
  Irq();
  Ctl::before_write = nullptr;
  TEST_ASSERT(injected);
  TEST_ASSERT((Ctl::ep[0] & STAT_TX) == TX_NAK);  // 旧数据没留在线上 / stale data off
  Irq();
  const Bytes device = TakeIn(0);
  TEST_ASSERT(device.size() == 18U && device[1] == 1U);
  SendOut(0, {});
  TEST_ASSERT(f.klass.completes == 0U && f.klass.aborts == 1U);
}

/// 停 EP0 OUT 时带 SETUP 的 CTR_RX 不被吞掉 / Stopping EP0 OUT does not
/// swallow a CTR_RX that carries a SETUP
void TestStopKeepsSetup()
{
  for (const bool late : {true, false})
  {
    Ctl::late_status = late;
    Fixture f;
    f.klass.response = Pattern(20, 0x70);
    SendSetup(Make(0xC1U, REQ_IN, 0U, 0U, 20U));
    Irq();
    // 状态 OUT 已受理：晚放行时留在 NAK，否则已 VALID / The status OUT is accepted: NAK
    // with the late release, VALID without
    TEST_ASSERT((Ctl::ep[0] & STAT_RX) == (late ? RX_NAK : RX_VALID));
    SendSetup(Make(0x80U, 6U, 0x0100U, 0U, 18U));  // ISR 还没跑 / ISR not yet run
    f.device.Deinit(false);  // 就地 owner：关 EP0 / in-place owner closes EP0
    TEST_ASSERT((Ctl::ep[0] & (CTR_RX | SETUP)) == (CTR_RX | SETUP));
    f.device.Init(false);
    // 重新配置 EP0 不动 SETUP 设好的 DATA1 / Configuring EP0 again keeps the DATA1 the
    // SETUP set
    TEST_ASSERT((Ctl::ep[0] & DTOG_TX) != 0U);
    Irq();
    const Bytes device = TakeIn(0);
    TEST_ASSERT(device.size() == 18U && device[1] == 1U);
    SendOut(0, {});
  }
  Ctl::late_status = true;
}

/// 把“EP0 的 STAT_RX 被写成 VALID”记进设备日志 / Log every write that makes STAT_RX of
/// EP0 VALID
void LogRxValid(TestDevice& device)
{
  Ctl::before_write = [&device](uint8_t num, uint32_t value)
  {
    if (num == 0U && (Ctl::ep[0] & STAT_RX) != RX_VALID &&
        ((Ctl::ep[0] ^ value) & STAT_RX) == RX_VALID)
    {
      device.log.push_back("rx_valid");
    }
  };
}

bool Logged(const TestDevice& device, const char* what)
{
  return std::find(device.log.begin(), device.log.end(), what) != device.log.end();
}

/// 状态 OUT 的晚放行（RM0008 p.631）：数据包在途时 STAT_RX 留在 NAK，最后一包处理完、
/// owner 那一轮结束之后才 VALID / Late release of the status OUT (RM0008 p.631): STAT_RX
/// stays NAK while a data packet is in flight and becomes VALID after the pass that
/// handled the last one
void TestLateStatus()
{
  Fixture f;
  LogRxValid(f.device);
  // 100 字节：64 + 36 两包 / 100 bytes: two packets, 64 + 36
  f.klass.response = Pattern(100, 0x20);
  f.device.log.clear();
  SendSetup(Make(0xC1U, REQ_IN, 0U, 0U, 100U));
  Irq();
  TEST_ASSERT((Ctl::ep[0] & STAT_TX) == TX_VALID && (Ctl::ep[0] & STAT_RX) == RX_NAK);
  TEST_ASSERT(!Logged(f.device, "rx_valid"));
  // 主机提前进入状态阶段：得到 NAK / A host entering the status stage early gets NAK
  TEST_ASSERT(!Ctl::RxPacket(0, nullptr, 0U, false));
  // 第一包完成，第二包装上：仍然 NAK / First packet done, second armed: still NAK
  TEST_ASSERT(TakeIn(0) == Pattern(64, 0x20));
  TEST_ASSERT((Ctl::ep[0] & STAT_TX) == TX_VALID && (Ctl::ep[0] & STAT_RX) == RX_NAK);
  TEST_ASSERT(!Logged(f.device, "rx_valid"));
  // 最后一包被 ACK，状态包紧跟着到：owner 还没处理“数据发完”，得到 NAK
  // The last packet is ACKed and the status packet follows at once: the owner has not
  // handled the sent data yet, so it gets NAK
  Ctl::AckIn(0);
  TEST_ASSERT(!Ctl::RxPacket(0, nullptr, 0U, false));
  // 这一轮里主机每次重试都得到 NAK；VALID 在这一轮之后才写
  // Every retry during the pass gets NAK; VALID is written after the pass
  unsigned retries = 0;
  bool taken_in_pass = false;
  Ctl::before_read = [&](uint8_t num)
  {
    if (num == 0U && f.device.masked)
    {
      ++retries;
      taken_in_pass = Ctl::RxPacket(0, nullptr, 0U, false) || taken_in_pass;
    }
  };
  Irq();
  Ctl::before_read = nullptr;
  TEST_ASSERT(retries != 0U && !taken_in_pass);
  TEST_ASSERT(Logged(f.device, "rx_valid") && (Ctl::ep[0] & STAT_RX) == RX_VALID);
  TEST_ASSERT(f.klass.completes == 0U);
  // 状态包被收下，传输完成；下一个 SETUP 不被拒收
  // The status packet is taken and the transfer completes; the next SETUP is not refused
  SendOut(0, {});
  TEST_ASSERT(f.klass.completes == 1U);
  TEST_ASSERT(ControlIn(Make(0xC1U, REQ_IN, 0U, 0U, 100U)) == Pattern(100, 0x20));
  TEST_ASSERT(f.klass.completes == 2U && Ctl::setups_refused == 0U);

  // 应答正好一个满包、比 wLength 短：结尾零长度包完成之后才放行
  // A reply of exactly one full packet, shorter than wLength: released only after the
  // closing zero-length packet completed
  f.klass.response = Pattern(64, 0x30);
  f.device.log.clear();
  SendSetup(Make(0xC1U, REQ_IN, 0U, 0U, 100U));
  Irq();
  TEST_ASSERT(TakeIn(0) == Pattern(64, 0x30));
  TEST_ASSERT((Ctl::ep[0] & STAT_TX) == TX_VALID && (Ctl::ep[0] & STAT_RX) == RX_NAK);
  TEST_ASSERT(!Logged(f.device, "rx_valid"));
  TEST_ASSERT(TakeIn(0).empty());
  TEST_ASSERT((Ctl::ep[0] & STAT_RX) == RX_VALID);
  SendOut(0, {});
  TEST_ASSERT(f.klass.completes == 3U);
  Ctl::before_write = nullptr;
}

/// 状态 OUT 被留住时来了新的 SETUP：旧传输作废，不留下 VALID / A new SETUP while the
/// status OUT is held: the old transfer is void and no VALID is left behind
void TestLateStatusSetup()
{
  {
    Fixture f;
    f.klass.response = Pattern(100, 0x40);
    SendSetup(Make(0xC1U, REQ_IN, 0U, 0U, 100U));
    Irq();
    TEST_ASSERT((Ctl::ep[0] & STAT_RX) == RX_NAK);
    SendSetup(Make(0x80U, 6U, 0x0100U, 0U, 18U));
    Irq();
    TEST_ASSERT(f.klass.aborts == 1U && (Ctl::ep[0] & STAT_RX) == RX_NAK);
    const Bytes device = TakeIn(0);
    TEST_ASSERT(device.size() == 18U && device[1] == 1U);
    SendOut(0, {});
    TEST_ASSERT(f.klass.completes == 0U && Ctl::setup_violations == 0U);
  }
  {
    // 最后一个数据包的 ACK 丢了：主机收下了数据，设备看不到发送完成，状态包一直得到
    // NAK；主机放弃之后的下一个 SETUP 照常处理，旧传输作废
    // The ACK of the last data packet is lost: the host has the data, the device never
    // sees the transmit completion, and the status packet keeps getting NAK; the SETUP
    // after the host gives up is handled as usual and the old transfer is void
    Fixture f;
    f.klass.response = Pattern(10, 0x58);
    SendSetup(Make(0xC1U, REQ_IN, 0U, 0U, 10U));
    Irq();
    TEST_ASSERT(Ctl::ArmedIn(0) == Pattern(10, 0x58));
    TEST_ASSERT((Ctl::ep[0] & STAT_TX) == TX_VALID && (Ctl::ep[0] & STAT_RX) == RX_NAK);
    for (int i = 0; i < 3; ++i)
    {
      TEST_ASSERT(!Ctl::RxPacket(0, nullptr, 0U, false));
      Irq();
    }
    TEST_ASSERT(f.klass.completes == 0U && f.klass.aborts == 0U);
    SendSetup(Make(0x80U, 6U, 0x0100U, 0U, 18U));
    Irq();
    TEST_ASSERT(f.klass.aborts == 1U && (Ctl::ep[0] & STAT_RX) == RX_NAK);
    const Bytes device = TakeIn(0);
    TEST_ASSERT(device.size() == 18U && device[1] == 1U);
    SendOut(0, {});
    TEST_ASSERT(f.klass.completes == 0U && Ctl::setups_refused == 0U);
    TEST_ASSERT(Ctl::setup_violations == 0U);
    // 之后的类请求完整走完 / A later class request runs to the end
    f.klass.response = Pattern(10, 0x59);
    TEST_ASSERT(ControlIn(Make(0xC1U, REQ_IN, 0U, 0U, 10U)) == Pattern(10, 0x59));
    TEST_ASSERT(f.klass.completes == 1U && f.klass.aborts == 1U);
  }
  {
    // 新的 SETUP 被类拒绝：两方向 STALL，被停掉的状态请求不再放行
    // The new SETUP is refused by the class: both directions STALL, and the stopped
    // status request is not released any more
    Fixture f;
    f.klass.response = Pattern(100, 0x48);
    SendSetup(Make(0xC1U, REQ_IN, 0U, 0U, 100U));
    Irq();
    SendSetup(Make(0x41U, 0x77U, 0U, 0U, 0U));
    Irq();
    TEST_ASSERT((Ctl::ep[0] & STAT_TX) == 0x0010U && (Ctl::ep[0] & STAT_RX) == 0x1000U);
    TEST_ASSERT(f.klass.aborts == 1U);
  }
  {
    // SETUP 落在放行的读改写窗口里：写回 NAK / A SETUP lands in the read-modify-write
    // window of the release: NAK is written back
    Fixture f;
    f.klass.response = Pattern(10, 0x50);
    SendSetup(Make(0xC1U, REQ_IN, 0U, 0U, 10U));
    Irq();
    bool injected = false;
    Ctl::before_write = [&injected](uint8_t num, uint32_t value)
    {
      if (num == 0U && !injected && (Ctl::ep[0] & STAT_RX) != RX_VALID &&
          ((Ctl::ep[0] ^ value) & STAT_RX) == RX_VALID)
      {
        injected = true;
        SendSetup(Make(0x80U, 6U, 0x0100U, 0U, 18U));
      }
    };
    TEST_ASSERT(TakeIn(0) == Pattern(10, 0x50));
    Ctl::before_write = nullptr;
    TEST_ASSERT(injected && (Ctl::ep[0] & STAT_RX) == RX_NAK);
    Irq();
    const Bytes device = TakeIn(0);
    TEST_ASSERT(device.size() == 18U && device[1] == 1U);
    SendOut(0, {});
    TEST_ASSERT(f.klass.completes == 0U && f.klass.aborts == 1U);
  }
}

/// 放行时的硬件异常与总线复位 / Hardware trouble and a bus reset at the release
void TestLateStatusFaults()
{
  {
    // VALID 写不进：状态包继续得到 NAK；下一次中断重写，传输照常完成
    // VALID does not take: the status packet keeps getting NAK; the next interrupt
    // writes again and the transfer completes as usual
    Fixture f;
    f.klass.response = Pattern(10, 0x60);
    SendSetup(Make(0xC1U, REQ_IN, 0U, 0U, 10U));
    Irq();
    Ctl::AckIn(0);
    Ctl::stuck = 0;
    Irq();
    TEST_ASSERT((Ctl::ep[0] & STAT_RX) == RX_NAK);
    TEST_ASSERT(!Ctl::RxPacket(0, nullptr, 0U, false));
    Ctl::stuck = -1;
    Irq();
    TEST_ASSERT((Ctl::ep[0] & STAT_RX) == RX_VALID && f.klass.completes == 0U);
    SendOut(0, {});
    TEST_ASSERT(f.klass.completes == 1U);
    TEST_ASSERT(ControlIn(Make(0xC1U, REQ_IN, 0U, 0U, 10U)) == Pattern(10, 0x60));
    TEST_ASSERT(f.klass.completes == 2U);
  }
  {
    // 放行落在总线复位的 SE0 里：不算失败，复位之后重新枚举
    // The release falls into the SE0 of a bus reset: no failure, and enumeration works
    // after the reset
    Fixture f;
    f.klass.response = Pattern(10, 0x70);
    SendSetup(Make(0xC1U, REQ_IN, 0U, 0U, 10U));
    Irq();
    Ctl::AckIn(0);
    // owner 那一轮结束之后的第一次 EP0R 读（放行）起，寄存器被按在 0 上
    // From the first EP0R read after the owner's pass (the release) the registers are
    // held at 0
    bool in_pass = false;
    Ctl::before_read = [&](uint8_t num)
    {
      if (num != 0U)
      {
        return;
      }
      if (f.device.masked)
      {
        in_pass = true;
      }
      else if (in_pass)
      {
        Ctl::in_reset = true;
      }
    };
    Irq();
    Ctl::before_read = nullptr;
    TEST_ASSERT(Ctl::in_reset);
    Ctl::BusReset();
    Irq();
    ControlNone(Make(0x00U, 5U, 9U, 0U, 0U));
    ControlNone(Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(ControlIn(Make(0xC1U, REQ_IN, 0U, 0U, 10U)) == Pattern(10, 0x70));
  }
  {
    // 数据包在途时总线复位：留住的状态请求随端点关闭结束
    // A bus reset while the data packet is in flight: the held status request ends with
    // the endpoint's close
    Fixture f;
    f.klass.response = Pattern(10, 0x80);
    SendSetup(Make(0xC1U, REQ_IN, 0U, 0U, 10U));
    Irq();
    Ctl::BusReset();
    Irq();
    TEST_ASSERT((Ctl::ep[0] & STAT_RX) != RX_VALID);
    ControlNone(Make(0x00U, 5U, 9U, 0U, 0U));
    ControlNone(Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(ControlIn(Make(0xC1U, REQ_IN, 0U, 0U, 10U)) == Pattern(10, 0x80));
    TEST_ASSERT(f.klass.aborts == 1U);
  }
}

/// 一个控制读走到“状态 OUT 已放行、状态包还没到” / Run a control read up to "status OUT
/// released, status packet not yet taken"
void ReadUpToStatus(Fixture& f, uint8_t seed)
{
  f.klass.response = Pattern(10, seed);
  SendSetup(Make(0xC1U, REQ_IN, 0U, 0U, 10U));
  Irq();
  TEST_ASSERT(TakeIn(0) == Pattern(10, seed));
  TEST_ASSERT((Ctl::ep[0] & STAT_RX) == RX_VALID);
}

/// 等状态包期间：其他端点的完成只清 CTR、记下，owner 不进来；状态包一到 owner 才进来，
/// 先处理记下的完成，数据完整 / While the status packet is awaited, completions of
/// other endpoints only have their CTR cleared and are latched, and the owner stays out;
/// when the status packet arrives the owner comes in, handles the latched completions
/// first, and the data are intact
void TestStatusWindow()
{
  for (const bool fast : {true, false})
  {
    Ctl::fast_ack = fast;
    Fixture f(true);
    // 短包，结束接收请求 / A short packet, which ends the receive request
    const Bytes out_data = Pattern(40, 0x31);
    TEST_ASSERT(f.klass.Send(Pattern(40, 0x11)) == ErrorCode::OK);
    TEST_ASSERT((Ctl::ep[1] & STAT_TX) == TX_VALID && (Ctl::ep[1] & STAT_RX) == RX_VALID);
    ReadUpToStatus(f, 0x21);
    const unsigned masks = f.device.masks;
    // 批量 IN 被主机取走，批量 OUT 收到一包 / The bulk IN is taken and a bulk OUT packet
    // arrives
    Ctl::AckIn(1);
    TEST_ASSERT(Ctl::RxPacket(1, out_data.data(), out_data.size(), false));
    Irq();
    TEST_ASSERT(f.device.masks == masks);
    TEST_ASSERT((Ctl::ep[1] & (CTR_RX | CTR_TX)) == 0U && (Ctl::istr & 0x8000U) == 0U);
    TEST_ASSERT(f.klass.in_done.empty() && f.klass.received.empty());
    // 帧起始：只清标志 / A start of frame: only its flag is cleared
    Sof();
    TEST_ASSERT(f.device.masks == masks && (Ctl::istr & 0x0200U) == 0U);
    // 状态包到 / The status packet arrives
    SendOut(0, {});
    TEST_ASSERT(f.device.masks > masks && f.klass.completes == 1U);
    TEST_ASSERT(f.klass.in_done.size() == 1U &&
                f.klass.in_done[0].result == ErrorCode::OK);
    TEST_ASSERT(f.klass.received == out_data);
    TEST_ASSERT(Ctl::setups_refused == 0U);
    // 之后回到平常：完成当场处理 / Back to normal afterwards: a completion is handled
    // at once
    TEST_ASSERT(f.klass.Send(Pattern(40, 0x12)) == ErrorCode::OK);
    TEST_ASSERT(TakeIn(1) == Pattern(40, 0x12));
    TEST_ASSERT(f.klass.in_done.size() == 2U);
  }
  Ctl::fast_ack = true;

  // 状态包在记下别的端点的完成时才到：同一次中断里再确认一次
  // The status packet arrives while another endpoint's completion is being latched: it
  // is acknowledged within the same interrupt
  Fixture f;
  TEST_ASSERT(f.klass.Send(Pattern(40, 0x13)) == ErrorCode::OK);
  ReadUpToStatus(f, 0x22);
  Ctl::AckIn(1);
  bool sent = false;
  Ctl::before_read = [&sent](uint8_t num)
  {
    if (num == 1U && !sent)
    {
      sent = true;
      TEST_ASSERT(Ctl::RxPacket(0, nullptr, 0U, false));
    }
  };
  Irq();
  Ctl::before_read = nullptr;
  TEST_ASSERT(sent && (Ctl::ep[0] & CTR_RX) == 0U);
  TEST_ASSERT(f.klass.completes == 1U && f.klass.in_done.size() == 1U);
}

/// 等待怎样结束：新的 SETUP、数满帧起始、总线复位 / How the wait ends: a new SETUP, the
/// full count of starts of frame, a bus reset
void TestStatusWindowEnds()
{
  {
    // 新的 SETUP：记下的完成交付，旧传输作废，新传输照常
    // A new SETUP: the latched completion is delivered, the old transfer is void and the
    // new one works
    Fixture f;
    TEST_ASSERT(f.klass.Send(Pattern(40, 0x14)) == ErrorCode::OK);
    ReadUpToStatus(f, 0x23);
    Ctl::AckIn(1);
    Irq();
    TEST_ASSERT(f.klass.in_done.empty());
    SendSetup(Make(0x80U, 6U, 0x0100U, 0U, 18U));
    Irq();
    TEST_ASSERT(f.klass.in_done.size() == 1U &&
                f.klass.in_done[0].result == ErrorCode::OK);
    TEST_ASSERT(f.klass.aborts == 1U && f.klass.completes == 0U);
    const Bytes device = TakeIn(0);
    TEST_ASSERT(device.size() == 18U && device[1] == 1U);
    SendOut(0, {});
  }
  {
    // 主机一直不发状态包：数到第 4 个帧起始恢复处理，之后状态包照样收下
    // The host sends no status packet: handling resumes at the fourth start of frame,
    // and the status packet is still taken afterwards
    Fixture f;
    TEST_ASSERT(f.klass.Send(Pattern(40, 0x15)) == ErrorCode::OK);
    ReadUpToStatus(f, 0x24);
    const unsigned masks = f.device.masks;
    Ctl::AckIn(1);
    Irq();
    Sof();
    Sof();
    Sof();
    TEST_ASSERT(f.device.masks == masks && f.klass.in_done.empty());
    Sof();
    TEST_ASSERT(f.device.masks > masks && f.klass.in_done.size() == 1U);
    SendOut(0, {});
    TEST_ASSERT(f.klass.completes == 1U);
  }
  {
    // 每次等待从头数：上一次数过的帧起始不带到下一次
    // Every wait counts from zero: starts of frame counted in one wait do not carry
    // over to the next
    Fixture f;
    ReadUpToStatus(f, 0x26);
    Sof();
    Sof();
    Sof();
    SendOut(0, {});
    TEST_ASSERT(f.klass.completes == 1U);
    TEST_ASSERT(f.klass.Send(Pattern(40, 0x18)) == ErrorCode::OK);
    ReadUpToStatus(f, 0x27);
    const unsigned masks = f.device.masks;
    Ctl::AckIn(1);
    Irq();
    Sof();
    TEST_ASSERT(f.device.masks == masks && f.klass.in_done.empty());
    SendOut(0, {});
    TEST_ASSERT(f.klass.completes == 2U && f.klass.in_done.size() == 1U);
  }
  {
    // 总线复位：记下的完成作废，请求以 ABORTED 结束，重新枚举后照常
    // A bus reset: the latched completion is void, the request ends ABORTED, and
    // everything works after the enumeration
    Fixture f;
    TEST_ASSERT(f.klass.Send(Pattern(40, 0x16)) == ErrorCode::OK);
    ReadUpToStatus(f, 0x25);
    Ctl::AckIn(1);
    Irq();
    Ctl::BusReset();
    Irq();
    TEST_ASSERT(f.klass.in_done.size() == 1U &&
                f.klass.in_done[0].result == ErrorCode::ABORTED);
    ControlNone(Make(0x00U, 5U, 9U, 0U, 0U));
    ControlNone(Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(ControlIn(Make(0xC1U, REQ_IN, 0U, 0U, 10U)) == Pattern(10, 0x25));
    TEST_ASSERT(f.klass.Send(Pattern(40, 0x17)) == ErrorCode::OK);
    TEST_ASSERT(TakeIn(1) == Pattern(40, 0x17));
  }
}

/// 留住的只是控制读的状态包：控制写的数据阶段当场装填，不开等待，其他端点的完成当场
/// 处理 / Only the status packet of a control read is held: the data stage of a control
/// write is armed at once and opens no wait, so completions of other endpoints are
/// handled at once
void TestStatusWindowDataStage()
{
  Fixture f;
  const Bytes payload = Pattern(70, 0x41);
  TEST_ASSERT(f.klass.Send(Pattern(40, 0x19)) == ErrorCode::OK);
  SendSetup(Make(0x41U, REQ_OUT, 0U, 0U, 70U));
  Irq();
  TEST_ASSERT((Ctl::ep[0] & STAT_RX) == RX_VALID);
  TEST_ASSERT(TakeIn(1) == Pattern(40, 0x19));
  TEST_ASSERT(f.klass.in_done.size() == 1U);
  SendOut(0, Bytes(payload.begin(), payload.begin() + 64));
  SendOut(0, Bytes(payload.begin() + 64, payload.end()));
  TEST_ASSERT(f.klass.data_seen == payload);
  TEST_ASSERT(TakeIn(0).empty());
  TEST_ASSERT(f.klass.completes == 1U);
}

/// EP0 快速确认：owner 一轮里（类的回调还没返回）主机的状态包和下一个 SETUP 都已经
/// 到了。不开快速确认时那个 SETUP 不被收下；这里用假控制器的规则直接比较两种配置在
/// 中断入口那一刻的差别
/// Fast acknowledgement of EP0. The host's status packet and its next SETUP arrive
/// around the interrupt entry. Without the fast acknowledgement the SETUP is not taken
/// until the owner has cleared CTR_RX; the fake's rule lets the two configurations be
/// compared at the moment of the interrupt entry
/// 状态 OUT 在数据包之前就 VALID 才有这个时序，所以关掉晚放行 / The timing exists only
/// with the status OUT VALID before the data packet, so the late release is off
void TestEp0FastAck()
{
  Ctl::late_status = false;
  for (const bool fast : {true, false})
  {
    Ctl::fast_ack = fast;
    Fixture f;
    f.klass.response = Pattern(10, 0x10);
    SendSetup(Make(0xC1U, REQ_IN, 0U, 0U, 10U));
    Irq();
    Ctl::AckIn(0);  // 数据包被取走 / the data packet is taken
    TEST_ASSERT(Ctl::RxPacket(0, nullptr, 0U, false));  // 状态包 / the status packet
    // 主机的下一个 SETUP 在这次中断的第三次 EP0R 读时到达：开了快速确认时那是 owner
    // 开始处理的时刻（CTR_RX 已清）；不开时 owner 正要清 CTR_RX、还没清
    // The host's next SETUP arrives at the third EP0R read of this interrupt: with the
    // fast acknowledgement that is when the owner starts (CTR_RX already clear);
    // without it the owner is about to clear CTR_RX and has not yet
    bool sent = false;
    bool taken = false;
    unsigned reads = 0;
    Ctl::before_read = [&](uint8_t num)
    {
      if (num == 0U && ++reads == 3U && !sent)
      {
        sent = true;
        uint8_t raw[8];
        const SetupPacket next = Make(0xC1U, REQ_IN, 0U, 0U, 10U);
        std::memcpy(raw, &next, sizeof(raw));
        taken = Ctl::RxPacket(0, raw, sizeof(raw), true);
      }
    };
    Irq();
    Ctl::before_read = nullptr;
    TEST_ASSERT(sent);
    // 快速确认：第二次读之前 CTR_RX 已清，SETUP 被收下，两个传输都完成
    // With the fast acknowledgement CTR_RX is clear before the second read, the SETUP is
    // taken and both transfers complete
    TEST_ASSERT(taken == fast);
    TEST_ASSERT(f.klass.completes == 1U);
    if (fast)
    {
      Irq();
      TEST_ASSERT(TakeIn(0) == Pattern(10, 0x10));
      SendOut(0, {});
      TEST_ASSERT(f.klass.completes == 2U && Ctl::setups_refused == 0U);
    }
    else
    {
      TEST_ASSERT(Ctl::setups_refused == 1U);
    }
  }
  Ctl::fast_ack = true;
  Ctl::late_status = true;
}

/// 状态 OUT 带数据：OUT_OF_RANGE，传输不算完成 / A status OUT carrying data:
/// OUT_OF_RANGE, the transfer does not count as complete
void TestStatusOutWithData()
{
  Fixture f;
  f.klass.response = Pattern(10, 0x10);
  SendSetup(Make(0xC1U, REQ_IN, 0U, 0U, 10U));
  Irq();
  TEST_ASSERT(TakeIn(0) == Pattern(10, 0x10));
  SendOut(0, Pattern(8, 0));
  TEST_ASSERT(f.klass.completes == 0U);
  // 下一个传输照常 / The next transfer works
  TEST_ASSERT(ControlIn(Make(0xC1U, REQ_IN, 0U, 0U, 10U)) == Pattern(10, 0x10));
  TEST_ASSERT(f.klass.completes == 1U);
}

/// 控制端点不按 STAT_RX 回 NAK 的控制器：OUT 包在请求启动之前就被收下 / A controller
/// whose control endpoint does not answer NAK from STAT_RX: OUT packets are taken
/// before their request starts
void TestEp0EarlyOut()
{
  Ctl::early_out = true;
  {
    // 控制写：每个数据包都在 CTR_RX 清掉之后、装填之前到达
    // Control write: every data packet arrives after CTR_RX was cleared and before the
    // arm
    Fixture f;
    const Bytes payload = Pattern(150, 0x40);
    size_t sent = 0U;
    Ctl::before_read = [&](uint8_t num)
    {
      if (num != 0U || sent >= payload.size() || (Ctl::ep[0] & CTR_RX) != 0U)
      {
        return;
      }
      const size_t len = std::min<size_t>(64U, payload.size() - sent);
      TEST_ASSERT(Ctl::RxPacket(0, payload.data() + sent, len, false, true));
      sent += len;
    };
    SendSetup(Make(0x41U, REQ_OUT, 0U, 0U, 150U));
    Irq();
    for (unsigned i = 0U;
         i < 16U && (sent < payload.size() || (Ctl::istr & 0x8000U) != 0U); ++i)
    {
      Irq();
    }
    Ctl::before_read = nullptr;
    TEST_ASSERT(f.klass.data_seen == payload);
    TEST_ASSERT(TakeIn(0).empty());  // 状态 IN / status IN
    TEST_ASSERT(f.klass.completes == 1U && Ctl::rx_overflows == 0U);
  }
  {
    // 类拒绝的控制写：早到的包被丢弃，CTR_RX 不留着，下一个 SETUP 照常收下
    // A refused control write: the early packet is dropped, CTR_RX does not stay and
    // the next SETUP is taken as usual
    Fixture f;
    bool sent = false;
    Ctl::before_read = [&](uint8_t num)
    {
      if (num == 0U && !sent && (Ctl::ep[0] & CTR_RX) == 0U)
      {
        sent = true;
        const Bytes data = Pattern(8, 0x55);
        TEST_ASSERT(Ctl::RxPacket(0, data.data(), data.size(), false, true));
      }
    };
    SendSetup(Make(0x41U, 0x77U, 0U, 0U, 8U));
    Irq();
    Irq();
    Ctl::before_read = nullptr;
    TEST_ASSERT(sent && (Ctl::ep[0] & CTR_RX) == 0U);
    TEST_ASSERT((Ctl::ep[0] & STAT_TX) == 0x0010U && (Ctl::ep[0] & STAT_RX) == 0x1000U);
    f.klass.response = Pattern(100, 0x30);
    TEST_ASSERT(ControlIn(Make(0xC1U, REQ_IN, 0U, 0U, 100U)) == Pattern(100, 0x30));
    TEST_ASSERT(f.klass.completes == 1U && Ctl::setups_refused == 0U);
  }
  {
    // 控制读：状态包在被留住（STAT_RX 为 NAK）时就被收下，与最后一包数据的完成一起到
    // Control read: the status packet is taken while it is held (STAT_RX at NAK) and
    // arrives together with the completion of the last data packet
    Fixture f;
    f.klass.response = Pattern(10, 0x10);
    SendSetup(Make(0xC1U, REQ_IN, 0U, 0U, 10U));
    Irq();
    TEST_ASSERT(Ctl::ArmedIn(0) == Pattern(10, 0x10));
    TEST_ASSERT((Ctl::ep[0] & STAT_RX) == RX_NAK);
    Ctl::AckIn(0);
    TEST_ASSERT(Ctl::RxPacket(0, nullptr, 0U, false, true));
    Irq();
    TEST_ASSERT(f.klass.completes == 1U);
    // 没有多余的放行：之后 EP0 OUT 不是 VALID / No stray release: EP0 OUT is not VALID
    TEST_ASSERT((Ctl::ep[0] & STAT_RX) != RX_VALID && (Ctl::ep[0] & CTR_RX) == 0U);
    TEST_ASSERT(ControlIn(Make(0xC1U, REQ_IN, 0U, 0U, 10U)) == Pattern(10, 0x10));
    TEST_ASSERT(f.klass.completes == 2U && Ctl::setups_refused == 0U);
  }
  {
    // 状态包还被留住的控制读被主机放弃，接着是数据早到的控制写：早到的包不被旧的
    // “留住”标记放行时改写
    // A control read whose status is still held is abandoned by the host and followed by
    // a control write with early data: the stale hold must not rewrite the early packet
    // when it is released
    Fixture f;
    f.klass.response = Pattern(100, 0x30);
    SendSetup(Make(0xC1U, REQ_IN, 0U, 0U, 100U));
    Irq();
    TEST_ASSERT((Ctl::ep[0] & STAT_TX) == TX_VALID);
    const Bytes payload = Pattern(40, 0x70);
    bool sent = false;
    Ctl::before_read = [&](uint8_t num)
    {
      if (num == 0U && !sent && (Ctl::ep[0] & CTR_RX) == 0U)
      {
        sent = true;
        TEST_ASSERT(Ctl::RxPacket(0, payload.data(), payload.size(), false, true));
      }
    };
    SendSetup(Make(0x41U, REQ_OUT, 0U, 0U, 40U));
    Irq();
    Irq();
    Ctl::before_read = nullptr;
    TEST_ASSERT(sent && f.klass.data_seen == payload);
    TEST_ASSERT(TakeIn(0).empty());
    TEST_ASSERT(f.klass.completes == 1U && Ctl::rx_overflows == 0U);
  }
  {
    // 早到的包恰好落在“启动时没看到它”和装填之间：装填不能改写接收区大小，否则它的
    // 长度被抹掉
    // An early packet lands exactly between "not seen at the start" and the arm: the arm
    // must not rewrite the receive size, or the packet's length is wiped
    Fixture f;
    const Bytes payload = Pattern(40, 0x21);
    bool sent = false;
    Ctl::before_rx_size = [&](uint8_t num)
    {
      if (num == 0U && !sent)
      {
        sent = true;
        TEST_ASSERT(Ctl::RxPacket(0, payload.data(), payload.size(), false, true));
      }
    };
    Ctl::before_write = [&](uint8_t num, uint32_t value)
    {
      // 装填 EP0 OUT 的那次写之前：没有重写过接收区大小时由这里送入
      // Before the write that arms EP0 OUT: sent from here if the receive size was not
      // rewritten
      if (num == 0U && !sent && ((Ctl::ep[0] ^ value) & STAT_RX) == RX_VALID)
      {
        sent = true;
        TEST_ASSERT(Ctl::RxPacket(0, payload.data(), payload.size(), false, true));
      }
    };
    SendSetup(Make(0x41U, REQ_OUT, 0U, 0U, 40U));
    Irq();
    Irq();
    Ctl::before_rx_size = nullptr;
    Ctl::before_write = nullptr;
    TEST_ASSERT(sent && f.klass.data_seen == payload);
    TEST_ASSERT(TakeIn(0).empty());
    TEST_ASSERT(f.klass.completes == 1U);
  }
  {
    // 只有 EP0 这样：批量 OUT 上没有请求时到的包当场丢弃；请求启动时已锁存的包不算它的
    // Only EP0 is treated so: a packet arriving on a bulk OUT with no request is dropped
    // at once, and one latched when a request starts is not counted as its packet
    Fixture f;
    const Bytes stale = Pattern(64, 0x11);
    TEST_ASSERT(Ctl::RxPacket(1, stale.data(), stale.size(), false, true));
    Irq();
    TEST_ASSERT((Ctl::ep[1] & CTR_RX) == 0U && f.klass.out_done.empty());
    bool sent = false;
    Ctl::before_read = [&](uint8_t num)
    {
      if (num == 1U && !sent)
      {
        sent = true;
        TEST_ASSERT(Ctl::RxPacket(1, stale.data(), stale.size(), false, true));
      }
    };
    TEST_ASSERT(f.klass.out->Receive(64U, false) == ErrorCode::OK);
    Ctl::before_read = nullptr;
    Irq();
    TEST_ASSERT(sent && f.klass.out_done.empty());
    TEST_ASSERT((Ctl::ep[1] & STAT_RX) == RX_VALID);
  }
  Ctl::early_out = false;
}

/// CDC 串口跑在 FSDEV 上：数据 IN、OUT 共用 EP1，通知 EP2 / CDC UART on FSDEV: data IN
/// and OUT share EP1, notification on EP2
void TestCdcEcho()
{
  using LibXR::ReadOperation;
  using LibXR::WriteOperation;
  using LibXR::USB::CDCUart;
  Ctl::Reset();
  Block ep0_out, ep0_in, ep1_out, ep1_in, ep2_in;
  CDCUart cdc(EPNumber::EP1, EPNumber::EP1, EPNumber::EP2, 256, 1024, 8);
  TestDevice device({{ep0_out.Raw(64), 64}, {ep1_out.Raw(128), 64}},
                    {{ep0_in.Raw(64), 64}, {ep1_in.Raw(128), 64}, {ep2_in.Raw(128), 16}},
                    {{&cdc}});
  device.Init(false);
  device.Start(false);
  Ctl::BusReset();
  Irq();
  ControlNone(Make(0x00U, 9U, 1U, 0U, 0U));
  TEST_ASSERT((Ctl::ep[1] & STAT_RX) == RX_VALID);

  // 主机写 → CDC 读 / The host writes, the CDC reads
  SendOut(1, Pattern(64, 0x55));
  Bytes got(64);
  std::atomic<ReadOperation::OperationPollingStatus> status{
      ReadOperation::OperationPollingStatus::READY};
  ReadOperation read(status);
  TEST_ASSERT(cdc.Read({got.data(), got.size()}, read) == ErrorCode::OK);
  TEST_ASSERT(got == Pattern(64, 0x55));

  // CDC 写 → 多个 IN 包 / A CDC write goes out as several IN packets
  WriteOperation write;
  const Bytes payload = Pattern(150, 0x66);
  TEST_ASSERT(cdc.Write({payload.data(), payload.size()}, write) == ErrorCode::OK);
  Bytes wire;
  while ((Ctl::ep[1] & STAT_TX) == TX_VALID)
  {
    const Bytes packet = TakeIn(1);
    wire.insert(wire.end(), packet.begin(), packet.end());
  }
  TEST_ASSERT(wire == payload);
}

// ---------------------------------------------------------------------------
// 总线事件 / Bus events
// ---------------------------------------------------------------------------

void TestBusEvents()
{
  Fixture f(true);
  TEST_ASSERT(f.klass.Send(Pattern(64, 1)) == ErrorCode::OK);
  TEST_ASSERT((Ctl::ep[1] & STAT_TX) == TX_VALID);
  // 复位：在途请求以 ABORTED 结束，地址归零，EP0 重新配置 / Reset: requests in flight
  // end with ABORTED, the address is zero, EP0 is configured again
  Ctl::BusReset();
  Irq();
  TEST_ASSERT(Ctl::daddr == 0x0080U && (Ctl::ep[0] & 0x0600U) == 0x0200U);
  TEST_ASSERT(f.klass.in_done.size() == 1U &&
              f.klass.in_done[0].result == ErrorCode::ABORTED);
  TEST_ASSERT(f.klass.out_done.size() == 1U &&
              f.klass.out_done[0].result == ErrorCode::ABORTED);
  TEST_ASSERT(f.klass.in == nullptr);  // 已解绑 / unbound
  // 挂起关 SOF、置 FSUSP，唤醒恢复 / Suspend: SOF off, FSUSP; wake-up restores
  TEST_ASSERT((Ctl::cntr & 0x0200U) != 0U);
  Ctl::istr |= 0x0800U;
  Irq();
  TEST_ASSERT((Ctl::cntr & 0x0200U) == 0U && (Ctl::cntr & 0x0008U) != 0U);
  Ctl::istr |= 0x1000U;
  Irq();
  TEST_ASSERT((Ctl::cntr & 0x0200U) != 0U && (Ctl::cntr & 0x0008U) == 0U);
  TEST_ASSERT(!f.device.masked);
  // 线路为空闲电平 "10" 或不允许的 "11" 的唤醒是噪声：回到挂起，SOF 仍关；恢复信号
  // "01" 才恢复（RM0365 表 176）
  // A wake-up with the line at the idle level "10" or the illegal "11" is noise: back
  // to suspend with SOF still off; resume signalling "01" resumes (RM0365 table 176)
  Ctl::istr |= 0x0800U;
  Irq();
  for (const uint16_t level : {uint16_t{0x8000U}, uint16_t{0xC000U}})
  {
    Ctl::fnr = level;
    Ctl::istr |= 0x1000U;
    Irq();
    TEST_ASSERT((Ctl::istr & 0x1000U) == 0U);
    TEST_ASSERT((Ctl::cntr & 0x0200U) == 0U && (Ctl::cntr & 0x0008U) != 0U);
  }
  Ctl::fnr = 0x4000U;
  Ctl::istr |= 0x1000U;
  Irq();
  TEST_ASSERT((Ctl::cntr & 0x0200U) != 0U && (Ctl::cntr & 0x0008U) == 0U);
  // 中断来晚了、读到的一直是空闲电平：第三次唤醒按真的恢复处理
  // The interrupt is late and only ever reads the idle level: the third wake-up is
  // handled as a real resume
  Ctl::istr |= 0x0800U;
  Irq();
  Ctl::fnr = 0x8000U;
  for (int i = 0; i < 2; ++i)
  {
    Ctl::istr |= 0x1000U;
    Irq();
    TEST_ASSERT((Ctl::cntr & 0x0008U) != 0U);
  }
  Ctl::istr |= 0x1000U;
  Irq();
  TEST_ASSERT((Ctl::cntr & 0x0200U) != 0U && (Ctl::cntr & 0x0008U) == 0U);
  // 没挂起时的唤醒标志不进挂起 / A wake-up flag while not suspended does not suspend
  Ctl::istr |= 0x0800U;
  Irq();
  Ctl::fnr = 0x4000U;
  Ctl::istr |= 0x1000U;
  Irq();
  Ctl::fnr = 0x8000U;
  Ctl::istr |= 0x1000U;
  Irq();
  TEST_ASSERT((Ctl::cntr & 0x0008U) == 0U);
  Ctl::fnr = 0U;
  f.device.Stop(false);
  TEST_ASSERT(!Ctl::pull_up && (Ctl::cntr & 0x8000U) == 0U);
}

}  // namespace

void test_usb_fsdev_control()
{
  TestEnumeration();
  TestControlWrite();
  TestStatusOutWithData();
  TestEp0FastAck();
  TestEp0EarlyOut();
}

void test_usb_fsdev_bulk()
{
  TestBulk();
  TestConfigure();
  TestBusEvents();
  TestCdcEcho();
}

void test_usb_fsdev_stop()
{
  TestStopPending();
  TestStopLatchedCompletion();
  TestStopCompletesInWrite();
  TestStopWriteUndoesNak();
  TestEp0KeepsAddress();
  TestStopBusEvents();
  TestCompletionAfterStop();
  TestStopTimeout();
  TestDeviceStop();
  TestStartDuringReset();
}

void test_usb_fsdev_clear_out() { TestClearStallKeepsOut(); }
void test_usb_fsdev_clear_in() { TestClearStallKeepsIn(); }
void test_usb_fsdev_clear_halted() { TestClearStallHalted(); }
void test_usb_fsdev_clear_race() { TestClearStallRace(); }

void test_usb_fsdev_setup()
{
  TestSetupWithInCompletion();
  TestSetupInArmWindow();
  TestStopKeepsSetup();
  TestLateStatus();
  TestLateStatusSetup();
  TestLateStatusFaults();
  TestStatusWindow();
  TestStatusWindowEnds();
  TestStatusWindowDataStage();
}
