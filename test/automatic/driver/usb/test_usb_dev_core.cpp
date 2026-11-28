/**
 * @file test_usb_dev_core.cpp
 * @brief 设备核心、组合层和类接口的假后端测试 / Fake-backend tests of the device core,
 *        the composition and the class interface
 *
 * 假后端设备同时派生 DeviceService 和 DeviceCore，实现四个入口函数和全部后端钩子；测试
 * 把“硬件事件”排进它的队列，再调用控制器中断入口，ServiceHardware 按队列顺序把事件交给
 * 核心。假端点记录每次后端调用，由测试决定请求何时、以什么结果完成，停止时返回什么。
 * 一个只供测试的类有一个批量 IN 和一个批量 OUT 端点。主机一侧由几个辅助函数扮演：发
 * SETUP、收 IN 包、发 OUT 包、发状态包。每条断言的依据是三个头文件的文档。
 * The fake backend device derives from both DeviceService and DeviceCore and implements
 * the four entry functions and every backend hook; a test queues "hardware events" and
 * then calls the controller interrupt entry, and ServiceHardware hands the events to the
 * core in queue order. Fake endpoints record every backend call and let the test decide
 * when and how a request completes and what a stop returns. A test-only class has one
 * bulk IN and one bulk OUT endpoint. A few helpers play the host: send a SETUP, take IN
 * packets, send OUT packets, send status packets. Every assertion rests on the
 * documentation of the three headers.
 */

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <new>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "fake_usb.hpp"
#include "test_assert.hpp"
#include "webusb.hpp"

// 分配计数：只在 g_counting 为真的线程上计数；假后端和测试类的记账用 NoCount 关掉它。
// Allocation count: only on a thread whose g_counting is set; the fake backend's and
// the test class's bookkeeping switch it off with NoCount.
namespace
{
thread_local bool g_counting = false;
std::atomic<size_t> g_allocations{0U};

void* CountedAlloc(std::size_t size)
{
  if (g_counting)
  {
    g_allocations.fetch_add(1U, std::memory_order_relaxed);
  }
  if (void* p = std::malloc(size != 0U ? size : 1U))
  {
    return p;
  }
  throw std::bad_alloc();
}
}  // namespace

void* operator new(std::size_t size) { return CountedAlloc(size); }
void* operator new[](std::size_t size) { return CountedAlloc(size); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

namespace
{
using namespace FakeUsbTest;
using LibXR::Callback;
using LibXR::ConstRawData;
using NoCount = ScopedNoAllocCount;

/// 全部假端点和端点池 / All fake endpoints and the pool
struct Hw : FakeHw
{
  Hw()
  {
    Add(ep1_in);
    Add(ep2_out);
    Add(ep3_in);
    Add(ep4_out);
  }

  AlignedMemory<1024U> mem[4];
  FakeEp ep1_in{EPNumber::EP1, Direction::IN, {mem[0].bytes, 1024U}};
  FakeEp ep2_out{EPNumber::EP2, Direction::OUT, {mem[1].bytes, 1024U}};
  FakeEp ep3_in{EPNumber::EP3, Direction::IN, {mem[2].bytes, 1024U}};
  FakeEp ep4_out{EPNumber::EP4, Direction::OUT, {mem[3].bytes, 1024U}};
};

/// 共享的事件记录，检查先后 / Shared event log for ordering checks
std::vector<std::string> g_log;

void Log(const std::string& entry)
{
  NoCount quiet;
  g_log.push_back(entry);
}

/// 测试类的请求码 / Request codes of the test class
constexpr uint8_t REQ_IN = 0x01U;    ///< IN 数据，内容为 response / IN data: response
constexpr uint8_t REQ_OUT = 0x02U;   ///< OUT 数据 / OUT data
constexpr uint8_t REQ_NONE = 0x03U;  ///< 无数据 / No data

/// 一个批量 IN、一个批量 OUT 端点的测试类 / Test class with one bulk IN and one bulk OUT
class TestClass final : public DeviceClass
{
 public:
  explicit TestClass(EPNumber in = EPNumber::EP1, EPNumber out = EPNumber::EP2,
                     Type in_type = Type::BULK, const char* name = nullptr)
      : in_num_(in), out_num_(out), in_type_(in_type), name_(name)
  {
    in_cb_ =
        Callback<const Completion&>::Create([](bool, TestClass* self, const Completion& c)
                                            { self->Record(self->in_done, c); }, this);
    out_cb_ = Callback<const Completion&>::Create(
        [](bool, TestClass* self, const Completion& c)
        {
          self->Record(self->out_done, c);
          if (c.slot.Valid())
          {
            (void)self->out->ReleaseSlot(c.slot);
          }
        },
        this);
  }

  using DeviceClass::RequestService;

  uint8_t InAddress() const
  {
    return static_cast<uint8_t>(0x80U | static_cast<uint8_t>(in_num_));
  }
  uint8_t OutAddress() const { return static_cast<uint8_t>(out_num_); }

  /// 从任何上下文发一个 IN 请求 / Submit an IN request from any context
  ErrorCode SendIn(size_t length)
  {
    Endpoint::Slot slot;
    ErrorCode result = in->AcquireSlot(slot);
    if (result == ErrorCode::OK)
    {
      result = in->Submit(slot, length, false, false);
    }
    return result;
  }

  Endpoint* in = nullptr;
  Endpoint* out = nullptr;
  ErrorCode bind_result = ErrorCode::OK;  ///< 非 OK 时取了 IN 再失败 / Fails after IN
  ErrorCode data_result = ErrorCode::OK;  ///< OnControlData 的返回 / OnControlData result
  bool arm_out = true;                    ///< 绑定时挂 OUT 接收 / Arm OUT at bind
  const char* guid = nullptr;             ///< WinUSB GUID
  Bytes response;                         ///< REQ_IN 的应答 / Answer of REQ_IN
  /// 可选 BOS 能力 / Optional BOS capability
  LibXR::USB::BosCapability* bos_cap = nullptr;
  Bytes data_seen;  ///< OnControlData 收到的 / Seen by OnControlData
  std::vector<SetupPacket> requests;
  std::vector<SetupPacket> completes;
  std::vector<SetupPacket> aborts;
  std::vector<Completion> in_done;
  std::vector<Completion> out_done;
  std::vector<uint8_t> halts_cleared;
  std::vector<uint16_t> frames;
  size_t writes = 0;
  size_t binds = 0;
  size_t unbinds = 0;
  size_t services = 0;
  size_t bos_cap_queries = 0;
  size_t custom_itf_count = 1U;
  size_t custom_max_config_size = 0U;
  ConstRawData bad_config_block{nullptr, 0};
  bool use_bad_config_block = false;
  uint8_t alt = 0;
  uint8_t class_desc[4] = {4U, 0x21U, 0xaaU, 0xbbU};
  std::atomic<int>* owner_check = nullptr;  ///< 回调不并发 / Callbacks never overlap

 protected:
  size_t GetInterfaceCount() const override { return custom_itf_count; }
  bool HasIAD() const override { return false; }
  size_t GetMaxConfigSize() const override
  {
    return custom_max_config_size != 0U ? custom_max_config_size : sizeof(desc_);
  }

  ErrorCode WriteDescriptors(const EndpointPool& pool, Speed speed,
                             uint8_t first_interface, ConstRawData& block) override
  {
    {
      NoCount quiet;
      ++writes;
    }
    const uint16_t in_size =
        pool.PacketSizeFor(in_num_, {Direction::IN, in_type_, 512U}, speed);
    const uint16_t out_size =
        pool.PacketSizeFor(out_num_, {Direction::OUT, Type::BULK, 512U}, speed);
    if (in_size == 0U || out_size == 0U)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    const uint8_t attr_in = static_cast<uint8_t>(in_type_);
    const uint8_t init[sizeof(desc_)] = {9U,
                                         4U,
                                         first_interface,
                                         0U,
                                         2U,
                                         0xffU,
                                         0U,
                                         0U,
                                         GetInterfaceStringIndex(0U),
                                         7U,
                                         5U,
                                         InAddress(),
                                         attr_in,
                                         static_cast<uint8_t>(in_size),
                                         static_cast<uint8_t>(in_size >> 8U),
                                         1U,
                                         7U,
                                         5U,
                                         OutAddress(),
                                         2U,
                                         static_cast<uint8_t>(out_size),
                                         static_cast<uint8_t>(out_size >> 8U),
                                         0U};
    std::memcpy(desc_, init, sizeof(desc_));
    if (use_bad_config_block)
    {
      block = bad_config_block;
      return ErrorCode::OK;
    }
    block = {desc_, sizeof(desc_)};
    return ErrorCode::OK;
  }

  ErrorCode BindEndpoints(EndpointPool& pool, uint8_t, bool in_isr) override
  {
    {
      NoCount quiet;
      ++binds;
    }
    if (pool.Get(in, Direction::IN, in_num_) != ErrorCode::OK)
    {
      in = nullptr;
      return ErrorCode::NOT_FOUND;
    }
    if (bind_result != ErrorCode::OK)
    {
      return bind_result;
    }
    if (pool.Get(out, Direction::OUT, out_num_) != ErrorCode::OK)
    {
      out = nullptr;
      return ErrorCode::NOT_FOUND;
    }
    in->SetOnComplete(in_cb_);
    out->SetOnComplete(out_cb_);
    ErrorCode result = in->Configure({Direction::IN, in_type_, 512U}, in_isr);
    if (result != ErrorCode::OK)
    {
      return result;
    }
    result = out->Configure({Direction::OUT, Type::BULK, 512U}, in_isr);
    if (result != ErrorCode::OK)
    {
      return result;
    }
    // 读回的包长必须等于刚写的描述符里的值。
    // The size read back must equal the one in the descriptor just written.
    if (in->MaxPacketSize() != (desc_[13] | desc_[14] << 8U) ||
        out->MaxPacketSize() != (desc_[20] | desc_[21] << 8U))
    {
      return ErrorCode::CHECK_ERR;
    }
    if (arm_out)
    {
      return out->Receive(out->MaxPacketSize(), in_isr);
    }
    return ErrorCode::OK;
  }

  void UnbindEndpoints(EndpointPool& pool, bool in_isr) override
  {
    {
      NoCount quiet;
      ++unbinds;
    }
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

  ErrorCode SetAltSetting(bool, uint8_t, uint8_t value) override
  {
    if (value > 1U)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    alt = value;
    return ErrorCode::OK;
  }

  ErrorCode GetAltSetting(uint8_t, uint8_t& value) override
  {
    value = alt;
    return ErrorCode::OK;
  }

  bool OwnsEndpoint(uint8_t ep_addr) const override
  {
    return ep_addr == InAddress() || ep_addr == OutAddress();
  }

  const char* GetInterfaceString(size_t) const override { return name_; }
  const char* GetWinUsbInterfaceGuid() const override { return guid; }
  size_t GetControlReceiveCapacity() const override { return 128U; }
  size_t GetBosCapabilityCount() override { return bos_cap != nullptr ? 1U : 0U; }
  LibXR::USB::BosCapability* GetBosCapability(size_t index) override
  {
    {
      NoCount quiet;
      ++bos_cap_queries;
    }
    return index == 0U ? bos_cap : nullptr;
  }

  ErrorCode OnGetDescriptor(bool, const SetupPacket& setup, ConstRawData& data) override
  {
    if ((setup.wValue >> 8U) != 0x21U)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    data = {class_desc, sizeof(class_desc)};
    return ErrorCode::OK;
  }

  ErrorCode OnControlRequest(bool, const SetupPacket& setup,
                             ConstRawData& in_data) override
  {
    NoCount quiet;
    requests.push_back(setup);
    switch (setup.bRequest)
    {
      case REQ_IN:
        in_data = {response.data(), response.size()};
        return ErrorCode::OK;
      case REQ_OUT:
      case REQ_NONE:
        return ErrorCode::OK;
      default:
        return ErrorCode::NOT_SUPPORT;
    }
  }

  ErrorCode OnControlData(bool, const SetupPacket&, ConstRawData data) override
  {
    NoCount quiet;
    const auto* p = static_cast<const uint8_t*>(data.addr_);
    data_seen.assign(p, p + data.size_);
    return data_result;
  }

  void OnControlComplete(bool, const SetupPacket& setup) override
  {
    NoCount quiet;
    completes.push_back(setup);
    Log("complete");
  }

  void OnControlAbort(bool, const SetupPacket& setup) override
  {
    NoCount quiet;
    aborts.push_back(setup);
    Log("abort");
  }

  void OnService(bool) override
  {
    Enter();
    ++services;
    Leave();
  }

  void OnHaltCleared(bool, uint8_t ep_addr) override
  {
    NoCount quiet;
    halts_cleared.push_back(ep_addr);
  }

  void OnStartOfFrame(bool, uint16_t frame, uint8_t) override
  {
    Enter();
    {
      NoCount quiet;
      frames.push_back(frame);
    }
    Leave();
  }

 private:
  void Record(std::vector<Completion>& list, const Completion& c)
  {
    NoCount quiet;
    list.push_back(c);
  }

  void Enter()
  {
    if (owner_check != nullptr)
    {
      TEST_ASSERT(owner_check->fetch_add(1) == 0);
    }
  }

  void Leave()
  {
    if (owner_check != nullptr)
    {
      owner_check->fetch_sub(1);
    }
  }

  EPNumber in_num_;
  EPNumber out_num_;
  Type in_type_;
  const char* name_;
  uint8_t desc_[23] = {};
  Callback<const Completion&> in_cb_;
  Callback<const Completion&> out_cb_;
};

using Configs = std::initializer_list<const std::initializer_list<DeviceClass*>>;

/// 带操作日志的假后端设备 / Fake backend device with shared event logging
class DevCoreUsb final : public FakeUsbTest::FakeUsb
{
 public:
  DevCoreUsb(Hw& hw, const Configs& configs, Speed speed = Speed::FULL,
             DeviceDescriptor::PacketSize0 ep0 = DeviceDescriptor::PacketSize0::SIZE_64,
             USBSpec spec = USBSpec::USB_2_0)
      : FakeUsbTest::FakeUsb(hw, configs, speed, ep0, spec)
  {
    on_log = [](const char* entry) { Log(entry); };
  }
};

using FakeUsb = DevCoreUsb;

// ---- 主机一侧 / The host side ----

/// 复位、编址、设为配置 1 / Reset, address and set configuration 1
void Enumerate(FakeUsb& dev, Hw& hw, Speed speed = Speed::FULL)
{
  dev.Reset(speed);
  TEST_ASSERT(dev.Inited());
  (void)GetDescriptor(dev, hw, 1U, 0U, 64U);
  ControlNone(dev, hw, Make(0x00U, 5U, 7U, 0U, 0U));
  ControlNone(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
}

uint8_t Configuration(FakeUsb& dev, Hw& hw) { return GetConfiguration(dev, hw); }

size_t Count(const std::string& entry)
{
  size_t n = 0;
  for (const std::string& e : g_log)
  {
    n += (e == entry) ? 1U : 0U;
  }
  return n;
}

// ---- 用例 / Cases ----

/// 从复位到 SET_CONFIGURATION / From reset to SET_CONFIGURATION
void TestEnumeration()
{
  Hw hw;
  TestClass cls(EPNumber::EP1, EPNumber::EP2, Type::BULK, "Test");
  FakeUsb dev(hw, {{&cls}});

  // 构造后什么都没打开；复位在 Init 之前不初始化。
  // Nothing is open after construction; a reset before Init does not initialize.
  TEST_ASSERT(!dev.Inited() && hw.in0.configures == 0U);
  dev.Reset();
  TEST_ASSERT(!dev.Inited() && dev.ready == 0U);

  dev.Init(false);
  TEST_ASSERT(dev.Inited() && dev.ready == 1U);
  TEST_ASSERT(hw.in0.MaxPacketSize() == 64U && hw.out0.MaxPacketSize() == 64U);
  dev.Reset();
  TEST_ASSERT(dev.Inited() && dev.ready == 2U && hw.in0.closes == 1U);

  // 设备描述符：18 字节，包长 64，一个配置；按 wLength 截短。
  // Device descriptor: 18 bytes, packet size 64, one configuration; cut to wLength.
  Bytes device = GetDescriptor(dev, hw, 1U, 0U, 64U);
  TEST_ASSERT(device.size() == 18U && device[0] == 18U && device[1] == 1U);
  TEST_ASSERT(device[7] == 64U && device[8] == 0x09U && device[9] == 0x12U);
  TEST_ASSERT(device[17] == 1U);
  TEST_ASSERT(GetDescriptor(dev, hw, 1U, 0U, 8U).size() == 8U);

  // SET_ADDRESS：地址在状态 IN 之前和之后各写一次，取自 SETUP。
  // SET_ADDRESS: the address is written before and after the status IN, from the SETUP.
  ControlNone(dev, hw, Make(0x00U, 5U, 7U, 0U, 0U));
  TEST_ASSERT(dev.addresses.size() == 2U);
  TEST_ASSERT(dev.addresses[0].first == 7U &&
              dev.addresses[0].second == DeviceCore::AddressStage::BEFORE_STATUS);
  TEST_ASSERT(dev.addresses[1].first == 7U &&
              dev.addresses[1].second == DeviceCore::AddressStage::STATUS_COMPLETE);

  // 配置描述符：先头部 9 字节，再全部；全速批量包长 64。
  // Configuration descriptor: the 9-byte header first, then all; full-speed bulk 64.
  Bytes head = GetDescriptor(dev, hw, 2U, 0U, 9U);
  TEST_ASSERT(head.size() == 9U && head[1] == 2U && head[2] == 32U && head[3] == 0U);
  Bytes config = GetDescriptor(dev, hw, 2U, 0U, 255U);
  TEST_ASSERT(config.size() == 32U && config[4] == 1U && config[5] == 1U);
  TEST_ASSERT(config[7] == LibXR::USB::CFG_BUS_POWERED && config[8] == 50U);
  TEST_ASSERT(config[9] == 9U && config[10] == 4U && config[11] == 0U);
  TEST_ASSERT(config[20] == 0x81U && config[22] == 64U && config[23] == 0U);
  TEST_ASSERT(config[27] == 0x02U && config[29] == 64U);
  // 写描述符不绑定、不碰端点 / Writing descriptors binds nothing and touches no endpoint
  TEST_ASSERT(cls.binds == 0U && hw.ep1_in.configures == 0U);
  TEST_ASSERT(cls.writes == 2U);

  // 字符串：语言表、三个内建字符串、接口字符串（4 号）。
  // Strings: language table, three built-in strings, the interface string (index 4).
  Bytes langs = GetDescriptor(dev, hw, 3U, 0U, 255U);
  TEST_ASSERT(langs.size() == 4U && langs[2] == 0x09U && langs[3] == 0x04U);
  Bytes product = GetDescriptor(dev, hw, 3U, 2U, 255U, 0x0409U);
  TEST_ASSERT(product.size() == 10U && product[2] == 'C' && product[1] == 3U);
  TEST_ASSERT(config[17] == 4U);
  Bytes itf = GetDescriptor(dev, hw, 3U, 4U, 255U, 0x0409U);
  TEST_ASSERT(itf.size() == 10U && itf[0] == 10U && itf[2] == 'T' && itf[8] == 't');
  Refused(dev, hw, Make(0x80U, 6U, 0x0304U, 0x0804U, 255U));
  Refused(dev, hw, Make(0x80U, 6U, 0x0305U, 0x0409U, 255U));

  // SET_CONFIGURATION：写描述符后绑定，数据端点打开，挂好的接收开始。
  // SET_CONFIGURATION: descriptors written, then bound; the data endpoints open and
  // the armed receive starts.
  TEST_ASSERT(Configuration(dev, hw) == 0U);
  ControlNone(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
  TEST_ASSERT(cls.binds == 1U && cls.writes == 3U);
  TEST_ASSERT(hw.ep2_out.inflight && hw.ep2_out.LastLength() == 64U);
  TEST_ASSERT(Configuration(dev, hw) == 1U);
  TEST_ASSERT(cls.SendIn(10U) == ErrorCode::OK && hw.ep1_in.inflight);

  // 同一个值再设一次：解绑再绑定 / The same value again: unbind and bind again
  ControlNone(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
  TEST_ASSERT(cls.unbinds == 1U && cls.binds == 2U);
  TEST_ASSERT(cls.in_done.size() == 1U && cls.in_done[0].result == ErrorCode::ABORTED);

  // 配置 0：解绑，数据端点关闭 / Configuration 0: unbind, data endpoints closed
  ControlNone(dev, hw, Make(0x00U, 9U, 0U, 0U, 0U));
  TEST_ASSERT(cls.unbinds == 2U && Configuration(dev, hw) == 0U);
  TEST_ASSERT(hw.ep1_in.GetState() == Endpoint::State::DISABLED);

  // 复合配置中既有有名称类也有无名称类，验证接口字符串索引只分配给前者。
  // Composite config with both named and unnamed classes assigns string index only to the
  // named one.
  {
    Hw comp_hw;
    TestClass named(EPNumber::EP1, EPNumber::EP2, Type::BULK, "Named");
    TestClass unnamed(EPNumber::EP3, EPNumber::EP4, Type::BULK, nullptr);
    FakeUsb comp_dev(comp_hw, {{&named, &unnamed}});
    comp_dev.Init(false);
    comp_dev.Reset();
    const Bytes comp_cfg = GetDescriptor(comp_dev, comp_hw, 2U, 0U, 255U);
    TEST_ASSERT(comp_cfg.size() == 55U);
    TEST_ASSERT(comp_cfg[17] == 4U && comp_cfg[40] == 0U);
    const Bytes comp_itf = GetDescriptor(comp_dev, comp_hw, 3U, 4U, 255U, 0x0409U);
    TEST_ASSERT(comp_itf.size() == 12U && comp_itf[2] == 'N');
  }
}

/// 每个标准请求 / Every standard request
void TestStandardRequests()
{
  Hw hw;
  TestClass cls;
  FakeUsb dev(hw, {{&cls}});
  dev.Init(false);

  // 未配置时只有设备和 EP0 可查 / Unconfigured: only the device and EP0 answer
  dev.Reset();
  TEST_ASSERT((ControlIn(dev, hw, Make(0x80U, 0U, 0U, 0U, 2U)) == Bytes{0U, 0U}));
  TEST_ASSERT((ControlIn(dev, hw, Make(0x82U, 0U, 0U, 0x80U, 2U)) == Bytes{0U, 0U}));
  Refused(dev, hw, Make(0x81U, 0U, 0U, 0U, 2U));
  Refused(dev, hw, Make(0x82U, 0U, 0U, 0x81U, 2U));
  Refused(dev, hw, Make(0x81U, 10U, 0U, 0U, 1U));

  Enumerate(dev, hw);

  // GET_STATUS
  TEST_ASSERT((ControlIn(dev, hw, Make(0x80U, 0U, 0U, 0U, 2U)) == Bytes{0U, 0U}));
  TEST_ASSERT((ControlIn(dev, hw, Make(0x81U, 0U, 0U, 0U, 2U)) == Bytes{0U, 0U}));
  TEST_ASSERT((ControlIn(dev, hw, Make(0x82U, 0U, 0U, 0x81U, 2U)) == Bytes{0U, 0U}));
  Refused(dev, hw, Make(0x81U, 0U, 0U, 1U, 2U));
  Refused(dev, hw, Make(0x82U, 0U, 0U, 0x83U, 2U));
  Refused(dev, hw, Make(0x80U, 0U, 0U, 0U, 1U));
  Refused(dev, hw, Make(0x83U, 0U, 0U, 0U, 2U));

  // SET_FEATURE / CLEAR_FEATURE(ENDPOINT_HALT)：在途请求结束，GET_STATUS 跟随，清除
  // 时通知类。
  // SET_FEATURE / CLEAR_FEATURE(ENDPOINT_HALT): the in-flight request ends, GET_STATUS
  // follows, and the class is told on clear.
  TEST_ASSERT(hw.ep2_out.inflight);
  ControlNone(dev, hw, Make(0x02U, 3U, 0U, 0x02U, 0U));
  TEST_ASSERT(hw.ep2_out.IsStalled() && hw.ep2_out.stalls == 1U && !hw.ep2_out.inflight);
  TEST_ASSERT(cls.out_done.back().result == ErrorCode::ABORTED);
  TEST_ASSERT((ControlIn(dev, hw, Make(0x82U, 0U, 0U, 0x02U, 2U)) == Bytes{1U, 0U}));
  ControlNone(dev, hw, Make(0x02U, 1U, 0U, 0x02U, 0U));
  TEST_ASSERT(!hw.ep2_out.IsStalled() && hw.ep2_out.clears == 1U);
  TEST_ASSERT((cls.halts_cleared == std::vector<uint8_t>{0x02U}));
  TEST_ASSERT((ControlIn(dev, hw, Make(0x82U, 0U, 0U, 0x02U, 2U)) == Bytes{0U, 0U}));
  // 没 STALL 的端点也可以清除（复位数据翻转）/ A non-halted endpoint may be cleared
  // too (it resets the data toggle)
  ControlNone(dev, hw, Make(0x02U, 1U, 0U, 0x81U, 0U));
  TEST_ASSERT(hw.ep1_in.clears == 1U && cls.halts_cleared.size() == 2U);
  // EP0、未绑定端点、远程唤醒、测试模式、接口特性：拒绝。
  // EP0, an unbound endpoint, remote wakeup, test mode, interface features: refused.
  Refused(dev, hw, Make(0x02U, 3U, 0U, 0x80U, 0U));
  Refused(dev, hw, Make(0x02U, 3U, 0U, 0x83U, 0U));
  Refused(dev, hw, Make(0x00U, 3U, 1U, 0U, 0U));
  Refused(dev, hw, Make(0x00U, 1U, 1U, 0U, 0U));
  Refused(dev, hw, Make(0x00U, 3U, 2U, 0x0100U, 0U));
  Refused(dev, hw, Make(0x01U, 3U, 0U, 0U, 0U));

  // GET_INTERFACE / SET_INTERFACE
  TEST_ASSERT((ControlIn(dev, hw, Make(0x81U, 10U, 0U, 0U, 1U)) == Bytes{0U}));
  ControlNone(dev, hw, Make(0x01U, 11U, 1U, 0U, 0U));
  TEST_ASSERT((ControlIn(dev, hw, Make(0x81U, 10U, 0U, 0U, 1U)) == Bytes{1U}));
  Refused(dev, hw, Make(0x01U, 11U, 2U, 0U, 0U));
  Refused(dev, hw, Make(0x01U, 11U, 0U, 1U, 0U));
  Refused(dev, hw, Make(0x81U, 10U, 0U, 1U, 1U));

  // 配置状态下 SET_ADDRESS、越界的 SET_CONFIGURATION、SET_DESCRIPTOR、SYNCH_FRAME、
  // 未知请求：拒绝。
  // SET_ADDRESS while configured, SET_CONFIGURATION out of range, SET_DESCRIPTOR,
  // SYNCH_FRAME and an unknown request: refused.
  Refused(dev, hw, Make(0x00U, 5U, 3U, 0U, 0U));
  Refused(dev, hw, Make(0x00U, 9U, 2U, 0U, 0U));
  TEST_ASSERT(Configuration(dev, hw) == 1U);
  Refused(dev, hw, Make(0x00U, 7U, 0x0100U, 0U, 18U));
  Refused(dev, hw, Make(0x82U, 12U, 0U, 0x81U, 2U));
  Refused(dev, hw, Make(0x00U, 2U, 0U, 0U, 0U));
  Refused(dev, hw, Make(0x00U, 6U, 0x0100U, 0U, 18U));

  // GET_DESCRIPTOR：类特定描述符、拒绝的种类 / GET_DESCRIPTOR: class-specific
  // descriptor and the refused kinds
  TEST_ASSERT((ControlIn(dev, hw, Make(0x81U, 6U, 0x2100U, 0U, 64U)) ==
               Bytes{4U, 0x21U, 0xaaU, 0xbbU}));
  Refused(dev, hw, Make(0x81U, 6U, 0x2200U, 0U, 64U));
  Refused(dev, hw, Make(0x80U, 6U, 0x2100U, 0U, 64U));
  Refused(dev, hw, Make(0x80U, 6U, 0x0101U, 0U, 18U));
  Refused(dev, hw, Make(0x80U, 6U, 0x0201U, 0U, 9U));
  Refused(dev, hw, Make(0x80U, 6U, 0x0600U, 0U, 10U));
  Refused(dev, hw, Make(0x80U, 6U, 0x0700U, 0U, 9U));
  Refused(dev, hw, Make(0x80U, 6U, 0x0F00U, 0U, 5U));
  Refused(dev, hw, Make(0x80U, 6U, 0x0100U, 0U, 0U));

  // 类、厂商请求的接收者规则 / Recipient rules of class and vendor requests
  Refused(dev, hw, Make(0xA0U, REQ_IN, 0U, 0U, 4U));
  Refused(dev, hw, Make(0xC0U, 0x33U, 0U, 0U, 4U));
  Refused(dev, hw, Make(0xE1U, REQ_IN, 0U, 0U, 4U));
  TEST_ASSERT(cls.requests.empty());

  // 下一个正常请求清掉 STALL / The next good request clears the halt
  const size_t clears = hw.in0.clears;
  TEST_ASSERT(Configuration(dev, hw) == 1U);
  TEST_ASSERT(hw.in0.clears == clears + 1U && !hw.in0.IsStalled());
}

/// 高速、BOS 和 WinUSB / High speed, BOS and WinUSB
void TestHighSpeedAndBos()
{
  Hw hw;
  TestClass cls;
  cls.guid = "{01234567-89AB-CDEF-0123-456789ABCDEF}";
  FakeUsb dev(hw, {{&cls}}, Speed::HIGH, DeviceDescriptor::PacketSize0::SIZE_64,
              USBSpec::USB_2_1);
  dev.Init(false);
  dev.Reset(Speed::HIGH);

  // 高速下批量包长 512；其他速度的配置是全速的，类型字节改为 7。
  // At high speed bulk packets are 512; the other-speed configuration is the
  // full-speed one with type byte 7.
  Bytes config = GetDescriptor(dev, hw, 2U, 0U, 255U);
  TEST_ASSERT(config.size() == 32U && config[1] == 2U);
  TEST_ASSERT(config[22] == 0x00U && config[23] == 0x02U);
  Bytes other = GetDescriptor(dev, hw, 7U, 0U, 255U);
  TEST_ASSERT(other.size() == 32U && other[1] == 7U && other[22] == 64U);
  Bytes again = GetDescriptor(dev, hw, 2U, 0U, 255U);
  TEST_ASSERT(again[1] == 2U);
  Bytes qualifier = GetDescriptor(dev, hw, 6U, 0U, 10U);
  TEST_ASSERT(qualifier.size() == 10U && qualifier[0] == 10U && qualifier[1] == 6U);
  TEST_ASSERT(qualifier[2] == 0x10U && qualifier[3] == 0x02U && qualifier[7] == 64U);
  TEST_ASSERT(qualifier[8] == 1U);

  // 全速复位后，其他速度的就是高速的 / After a full-speed reset the other speed is high
  dev.Reset(Speed::FULL);
  Bytes other_high = GetDescriptor(dev, hw, 7U, 0U, 255U);
  TEST_ASSERT(other_high[22] == 0x00U && other_high[23] == 0x02U);

  // BOS：头、WinUSB 平台能力、自动补的 USB 2.0 扩展 / BOS: header, the WinUSB platform
  // capability and the USB 2.0 extension added automatically
  Bytes bos = GetDescriptor(dev, hw, 15U, 0U, 255U);
  TEST_ASSERT(bos.size() == 5U + 28U + 7U && bos[1] == 15U && bos[4] == 2U);
  TEST_ASSERT(bos[5] == 28U && bos[6] == 0x10U && bos[7] == 0x05U);
  const uint8_t vendor = bos[5 + 26];
  const uint16_t set_length = static_cast<uint16_t>(bos[5 + 24] | bos[5 + 25] << 8U);

  // MS OS 2.0 描述符集经设备为接收者的厂商请求取得；单功能非复合设备用设备级形式。
  // The MS OS 2.0 descriptor set comes through a vendor request to the device; a
  // single function on a non-composite device gets the device-level form.
  Bytes set = ControlIn(dev, hw, Make(0xC0U, vendor, 0U, 7U, 0xffU));
  TEST_ASSERT(set.size() == set_length && set_length == 0x00A2U);
  TEST_ASSERT(set[0] == 10U && set[2] == 0U && set[4] == 0U && set[8] == set_length);
  TEST_ASSERT(set[12] == 0x03U && set[14] == 'W');

  // 枚举在高速下照常 / Enumeration at high speed works as usual
  Enumerate(dev, hw, Speed::HIGH);
  TEST_ASSERT(hw.ep1_in.MaxPacketSize() == 512U && hw.ep2_out.LastLength() == 512U);

  // 非复合多配置且仅含 1 个 WinUSB 功能时必须用配置子集头，不可退化成设备级格式 /
  // Multi-config non-composite with 1 WinUSB function must emit Configuration Subset
  {
    Hw mhw;
    TestClass c1;
    c1.guid = "{01234567-89AB-CDEF-0123-456789ABCDEF}";
    TestClass c2;
    FakeUsb mdev(mhw, {{&c1}, {&c2}}, Speed::HIGH, DeviceDescriptor::PacketSize0::SIZE_64,
                 USBSpec::USB_2_1);
    mdev.Init(false);
    mdev.Reset(Speed::HIGH);
    Bytes mbos = GetDescriptor(mdev, mhw, 15U, 0U, 255U);
    TEST_ASSERT(mbos.size() >= 5U + 28U);
    const uint8_t mvendor = mbos[5 + 26];
    const uint16_t mlen = static_cast<uint16_t>(mbos[5 + 24] | (mbos[5 + 25] << 8U));
    Bytes mset = ControlIn(mdev, mhw, Make(0xC0U, mvendor, 0U, 7U, 0xFFU));
    TEST_ASSERT(mset.size() == mlen && mlen == 0x00B2U);
    // Offset 10..17: Configuration Subset Header (length=8, type=1, bConfigValue=0)
    // Offset 18..25: Function Subset Header (length=8, type=2, bFirstInterface=0)
    TEST_ASSERT(mset[10] == 8U && mset[11] == 0U && mset[12] == 1U && mset[13] == 0U &&
                mset[14] == 0U);
    TEST_ASSERT(mset[18] == 8U && mset[19] == 0U && mset[20] == 2U && mset[21] == 0U &&
                mset[22] == 0U);
  }
}

/// 类请求的数据阶段，两个方向 / Data stages of class requests, both directions
void TestControlTransfers()
{
  Hw hw;
  TestClass cls;
  FakeUsb dev(hw, {{&cls}});
  dev.Init(false);
  Enumerate(dev, hw);

  // 多包 IN，最后一包短，不需要零长度包 / Multi-packet IN ending short, no ZLP
  cls.response.resize(150U);
  for (size_t i = 0; i < cls.response.size(); ++i)
  {
    cls.response[i] = static_cast<uint8_t>(i);
  }
  std::vector<size_t> lengths;
  TEST_ASSERT(ControlIn(dev, hw, Make(0xA1U, REQ_IN, 0U, 0U, 200U), &lengths) ==
              cls.response);
  TEST_ASSERT((lengths == std::vector<size_t>{64U, 64U, 22U}));
  TEST_ASSERT(cls.completes.size() == 1U && cls.aborts.empty());

  // 比 wLength 短且正好满包：结尾零长度包在同一个请求里 / Shorter than wLength and
  // ending on a full packet: the closing ZLP is in the same request
  cls.response.assign(128U, 0x5aU);
  lengths.clear();
  TEST_ASSERT(ControlIn(dev, hw, Make(0xA1U, REQ_IN, 0U, 0U, 255U), &lengths) ==
              cls.response);
  TEST_ASSERT((lengths == std::vector<size_t>{64U, 64U, 0U}));
  // 正好 wLength：没有零长度包 / Exactly wLength: no ZLP
  lengths.clear();
  (void)ControlIn(dev, hw, Make(0xA1U, REQ_IN, 0U, 0U, 128U), &lengths);
  TEST_ASSERT((lengths == std::vector<size_t>{64U, 64U}));
  // 比 wLength 长：截短 / Longer than wLength: truncated
  lengths.clear();
  TEST_ASSERT(ControlIn(dev, hw, Make(0xA1U, REQ_IN, 0U, 0U, 100U), &lengths).size() ==
              100U);
  TEST_ASSERT((lengths == std::vector<size_t>{64U, 36U}));
  // 空应答：一个零长度数据包 / Empty answer: one zero-length data packet
  cls.response.clear();
  lengths.clear();
  TEST_ASSERT(ControlIn(dev, hw, Make(0xA1U, REQ_IN, 0U, 0U, 16U), &lengths).empty());
  TEST_ASSERT((lengths == std::vector<size_t>{0U}));
  TEST_ASSERT(cls.completes.size() == 5U && cls.aborts.empty());

  // 多包 OUT，收齐后交给类 / Multi-packet OUT handed to the class when complete
  Bytes data(100U);
  for (size_t i = 0; i < data.size(); ++i)
  {
    data[i] = static_cast<uint8_t>(0xff - i);
  }
  ControlOut(dev, hw, Make(0x21U, REQ_OUT, 0U, 0U, 100U), data);
  TEST_ASSERT(cls.data_seen == data && cls.completes.size() == 6U);
  // 主机的短包提前结束数据阶段 / A short packet from the host ends the data stage early
  dev.Setup(Make(0x21U, REQ_OUT, 0U, 0U, 100U));
  dev.Irq([&hw](bool isr) { hw.out0.HostSends(isr, Bytes(64U, 1U)); });
  dev.Irq([&hw](bool isr) { hw.out0.HostSends(isr, Bytes(10U, 2U)); });
  TEST_ASSERT(cls.data_seen.size() == 74U && hw.in0.LastLength() == 0U);
  InDone(dev, hw);
  TEST_ASSERT(cls.completes.size() == 7U);

  // 无数据的类请求，端点为接收者的类请求 / A no-data class request, and one whose
  // recipient is an endpoint
  ControlNone(dev, hw, Make(0x21U, REQ_NONE, 0U, 0U, 0U));
  ControlNone(dev, hw, Make(0x22U, REQ_NONE, 0U, 0x81U, 0U));
  TEST_ASSERT(cls.completes.size() == 9U && cls.requests.back().wIndex == 0x81U);

  // 类拒绝：STALL，不回调 Abort / The class refuses: stall, no Abort
  Refused(dev, hw, Make(0x21U, 0x09U, 0U, 0U, 0U));
  TEST_ASSERT(cls.aborts.empty());
  // 接受后出错：OnControlData 出错、wLength 超过控制缓冲。各得一次 Abort。
  // Errors after acceptance: OnControlData fails, wLength exceeds the control buffer.
  // One Abort each.
  cls.data_result = ErrorCode::FAILED;
  dev.Setup(Make(0x21U, REQ_OUT, 0U, 0U, 8U));
  dev.Irq([&hw](bool isr) { hw.out0.HostSends(isr, Bytes(8U, 3U)); });
  TEST_ASSERT(Stalled(hw) && cls.aborts.size() == 1U && cls.completes.size() == 9U);
  cls.data_result = ErrorCode::OK;
  Refused(dev, hw, Make(0x21U, REQ_OUT, 0U, 0U, 129U));
  TEST_ASSERT(cls.aborts.size() == 2U && cls.completes.size() == 9U);
  // 之后照常 / Normal afterwards
  ControlNone(dev, hw, Make(0x21U, REQ_NONE, 0U, 0U, 0U));
  TEST_ASSERT(cls.completes.size() == 10U && cls.aborts.size() == 2U);
}

/// 新 SETUP 在每个阶段取代未完成的传输 / A new SETUP replaces an unfinished transfer
/// in every stage
void TestSetupReplaces()
{
  Hw hw;
  TestClass cls;
  FakeUsb dev(hw, {{&cls}});
  dev.Init(false);
  Enumerate(dev, hw);
  cls.response.assign(150U, 0x11U);

  // 每次：旧传输被类 Abort 一次，不 Complete；旧的 OnControlReady 不出现；新请求照常。
  // Each time: the old transfer gets one Abort and no Complete; no OnControlReady for
  // it; the new request is served normally.
  auto replaced = [&](const char* stage)
  {
    const size_t aborts = cls.aborts.size();
    const size_t completes = cls.completes.size();
    const size_t ready = dev.ready;
    TEST_ASSERT(Configuration(dev, hw) == 1U);
    TEST_ASSERT(cls.aborts.size() == aborts + 1U);
    TEST_ASSERT(cls.completes.size() == completes);
    TEST_ASSERT(dev.ready == ready + 1U);
    UNUSED(stage);
  };

  // IN 数据阶段，第一包在途 / IN data stage, first packet in flight
  dev.Setup(Make(0xA1U, REQ_IN, 0U, 0U, 200U));
  TEST_ASSERT(hw.in0.inflight && hw.in0.LastLength() == 64U);
  size_t stops = hw.in0.stops;
  replaced("data-in");
  TEST_ASSERT(hw.in0.stops == stops + 1U);

  // IN 数据阶段，两包之间（第二包在途）/ IN data stage, second packet in flight
  dev.Setup(Make(0xA1U, REQ_IN, 0U, 0U, 200U));
  InDone(dev, hw);
  TEST_ASSERT(hw.in0.inflight);
  replaced("data-in-2");

  // 数据发完，等状态 OUT / Data sent, waiting for the status OUT
  dev.Setup(Make(0xA1U, REQ_IN, 0U, 0U, 200U));
  (void)TakeIn(dev, hw);
  TEST_ASSERT(hw.out0.inflight && hw.out0.LastLength() == 0U);
  stops = hw.out0.stops;
  replaced("status-out");
  TEST_ASSERT(hw.out0.stops == stops + 1U);

  // OUT 数据阶段 / OUT data stage
  cls.data_seen.clear();
  dev.Setup(Make(0x21U, REQ_OUT, 0U, 0U, 100U));
  dev.Irq([&hw](bool isr) { hw.out0.HostSends(isr, Bytes(64U, 1U)); });
  replaced("data-out");
  TEST_ASSERT(cls.data_seen.empty());

  // 状态 IN 在途（数据已交给类）/ Status IN in flight (data already handed over)
  dev.Setup(Make(0x21U, REQ_OUT, 0U, 0U, 8U));
  dev.Irq([&hw](bool isr) { hw.out0.HostSends(isr, Bytes(8U, 1U)); });
  TEST_ASSERT(cls.data_seen.size() == 8U && hw.in0.inflight);
  replaced("status-in");

  // 上一个传输以 STALL 结束：用 ClearStall，不用 Flush / The previous transfer ended
  // in a stall: ClearStall, not Flush
  Refused(dev, hw, Make(0x21U, 0x09U, 0U, 0U, 0U));
  const size_t in_clears = hw.in0.clears;
  const size_t out_clears = hw.out0.clears;
  TEST_ASSERT(Configuration(dev, hw) == 1U);
  TEST_ASSERT(hw.in0.clears == in_clears + 1U && hw.out0.clears == out_clears + 1U);

  // 标准请求被取代：没有类，也没有 OnControlReady / A standard request replaced: no
  // class, and no OnControlReady
  dev.Setup(Make(0x80U, 6U, 0x0200U, 0U, 255U));
  const size_t ready = dev.ready;
  const size_t aborts = cls.aborts.size();
  TEST_ASSERT(Configuration(dev, hw) == 1U);
  TEST_ASSERT(dev.ready == ready + 1U && cls.aborts.size() == aborts);
}

/// 复位和断开：每种状态，含在途请求 / Reset and disconnect in every state, requests in
/// flight included
void TestResetAndDisconnect()
{
  enum class State
  {
    BEFORE_INIT,
    IDLE,
    CONTROL,
    CONFIGURED,
    SUSPENDED
  };
  for (const bool disconnect : {false, true})
  {
    for (const State state : {State::BEFORE_INIT, State::IDLE, State::CONTROL,
                              State::CONFIGURED, State::SUSPENDED})
    {
      Hw hw;
      TestClass cls;
      FakeUsb dev(hw, {{&cls}});
      cls.response.assign(150U, 1U);
      if (state != State::BEFORE_INIT)
      {
        dev.Init(false);
        Enumerate(dev, hw);
      }
      if (state == State::CONTROL)
      {
        dev.Setup(Make(0xA1U, REQ_IN, 0U, 0U, 200U));
        TEST_ASSERT(hw.in0.inflight);
      }
      if (state == State::CONFIGURED || state == State::SUSPENDED)
      {
        TEST_ASSERT(cls.SendIn(64U) == ErrorCode::OK && hw.ep1_in.inflight);
        TEST_ASSERT(cls.SendIn(32U) == ErrorCode::OK);
        TEST_ASSERT(hw.ep2_out.inflight);
      }
      if (state == State::SUSPENDED)
      {
        dev.Suspend();
      }
      const size_t unbinds = cls.unbinds;
      const size_t ready = dev.ready;
      const bool configured = state != State::BEFORE_INIT;

      if (disconnect)
      {
        dev.Disconnect();
        TEST_ASSERT(!dev.Inited());
      }
      else
      {
        dev.Reset();
        TEST_ASSERT(dev.Inited() == (state != State::BEFORE_INIT));
        TEST_ASSERT(dev.ready == ready + (configured ? 1U : 0U));
      }
      // 当前配置的类解绑；数据端点和 EP0 的请求都已回调，没有在途的。
      // The configured class is unbound; requests of data endpoints and EP0 have all
      // been called back, none is in flight.
      TEST_ASSERT(cls.unbinds == unbinds + (configured ? 1U : 0U));
      for (FakeEp* ep : hw.All())
      {
        TEST_ASSERT(!ep->inflight);
      }
      TEST_ASSERT(hw.ep1_in.GetState() == Endpoint::State::DISABLED);
      if (state == State::CONFIGURED || state == State::SUSPENDED)
      {
        TEST_ASSERT(cls.in_done.size() == 2U);
        TEST_ASSERT(cls.in_done[0].result == ErrorCode::ABORTED);
        TEST_ASSERT(cls.in_done[1].result == ErrorCode::ABORTED);
        TEST_ASSERT(cls.out_done.back().result == ErrorCode::ABORTED);
      }
      TEST_ASSERT(cls.aborts.size() == (state == State::CONTROL ? 1U : 0U));
      if (disconnect)
      {
        // 之后的复位按记下的启用重新初始化，挂起已清除。
        // A later reset initializes again as enabled, with suspend cleared.
        dev.Reset();
        TEST_ASSERT(dev.Inited() == (state != State::BEFORE_INIT));
      }
      if (state != State::BEFORE_INIT)
      {
        // 挂起已清：EP0 照常工作，可以重新配置 / Suspend is cleared: EP0 works and the
        // device can be configured again
        TEST_ASSERT(Configuration(dev, hw) == 0U);
        ControlNone(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
        TEST_ASSERT(hw.ep2_out.inflight);
      }
    }
  }

  // Deinit：解绑、关 EP0；之后的复位不再初始化，Init 再打开。
  // Deinit: unbind and close EP0; later resets do not initialize, Init opens again.
  Hw hw;
  TestClass cls;
  FakeUsb dev(hw, {{&cls}});
  dev.Init(false);
  Enumerate(dev, hw);
  dev.Deinit(false);
  TEST_ASSERT(!dev.Inited() && cls.unbinds == 1U);
  TEST_ASSERT(hw.in0.GetState() == Endpoint::State::DISABLED);
  dev.Reset();
  TEST_ASSERT(!dev.Inited());
  dev.Setup(Make(0x80U, 8U, 0U, 0U, 1U));
  TEST_ASSERT(!hw.in0.inflight);
  dev.Init(false);
  TEST_ASSERT(dev.Inited());
  dev.Init(false);
  TEST_ASSERT(hw.in0.configures == 3U);
}

/// 挂起、恢复、帧起始和类服务 / Suspend, resume, start of frame and class service
void TestSuspendSofService()
{
  Hw hw;
  TestClass cls;
  FakeUsb dev(hw, {{&cls}});
  dev.Init(false);

  // 服务请求不论是否配置都送达，多次请求可合并 / A service request arrives whether
  // configured or not; several may merge
  cls.RequestService(false);
  TEST_ASSERT(cls.services == 1U);

  // 未配置时 SOF 不交给类 / Unconfigured: SOF does not reach the class
  dev.Reset();
  dev.Sof(5U);
  TEST_ASSERT(cls.frames.empty());
  Enumerate(dev, hw);
  dev.Sof(6U);
  dev.Sof(0xffffU);
  TEST_ASSERT((cls.frames == std::vector<uint16_t>{6U, 0xffffU}));

  // 挂起：在途的不停，新的不启动；恢复后启动。
  // Suspend: in flight is not stopped, new ones do not start; they start on resume.
  TEST_ASSERT(cls.SendIn(10U) == ErrorCode::OK && hw.ep1_in.inflight);
  dev.Suspend();
  TEST_ASSERT(hw.ep1_in.inflight && hw.ep1_in.stops == 0U && hw.ep2_out.stops == 0U);
  dev.Irq([&hw](bool isr) { hw.ep1_in.Done(isr, 10U); });
  TEST_ASSERT(cls.in_done.size() == 1U && cls.in_done[0].result == ErrorCode::OK);
  const size_t starts = hw.ep1_in.starts.size();
  TEST_ASSERT(cls.SendIn(20U) == ErrorCode::OK);
  TEST_ASSERT(hw.ep1_in.starts.size() == starts && !hw.ep1_in.inflight);
  cls.RequestService(false);
  TEST_ASSERT(cls.services == 2U);
  dev.Resume();
  TEST_ASSERT(hw.ep1_in.starts.size() == starts + 1U && hw.ep1_in.LastLength() == 20U);
  TEST_ASSERT(Configuration(dev, hw) == 1U);

  // 挂起时 EP0 也不启动 / EP0 does not start while suspended either
  dev.Suspend();
  dev.Setup(Make(0x80U, 8U, 0U, 0U, 1U));
  TEST_ASSERT(!hw.in0.inflight);
  dev.Resume();
  TEST_ASSERT(hw.in0.inflight);
  (void)TakeIn(dev, hw);
  StatusOut(dev, hw);

  // 只调用请求过的类；请求在调用前取走，不会重复 / Only the class that asked is
  // called; the request is taken before the call and does not repeat
  Hw hw2;
  TestClass a;
  TestClass b(EPNumber::EP3, EPNumber::EP4);
  FakeUsb dev2(hw2, {{&a, &b}});
  a.RequestService(false);
  TEST_ASSERT(a.services == 1U && b.services == 0U);
  b.RequestService(false);
  TEST_ASSERT(a.services == 1U && b.services == 1U);
}

/// 停止失败导致控制器复位 / A failed stop leads to a controller reset
void TestStopFailure()
{
  // 数据端点的 STALL 挂起，之后报告超时 / The stall of a data endpoint stays pending,
  // then is reported as timed out
  {
    Hw hw;
    TestClass cls;
    FakeUsb dev(hw, {{&cls}});
    dev.Init(false);
    Enumerate(dev, hw);
    TEST_ASSERT(hw.ep2_out.inflight);
    hw.ep2_out.stuck = true;
    g_log.clear();
    // 不等停止：请求当场以 ABORTED 回调，状态 IN 照发，STALL 推迟。
    // No waiting for the stop: the request is called back ABORTED at once, the status
    // IN goes out and the stall is deferred.
    dev.Setup(Make(0x02U, 3U, 0U, 0x02U, 0U));
    TEST_ASSERT(dev.resets == 0U && dev.Inited() && !Stalled(hw));
    TEST_ASSERT(hw.ep2_out.IsStalled() && hw.ep2_out.stalls == 0U);
    TEST_ASSERT(cls.out_done.size() == 1U &&
                cls.out_done[0].result == ErrorCode::ABORTED);
    TEST_ASSERT(hw.in0.inflight && hw.in0.LastLength() == 0U);
    // 复位控制器后关闭所有端点，类解绑；推迟的 STALL 不再执行，不再回调。
    // After the controller reset every endpoint is closed and the class is unbound;
    // the deferred stall never runs and no callback follows.
    dev.Irq([&hw](bool isr) { hw.ep2_out.TimedOut(isr); });
    TEST_ASSERT(dev.resets == 1U && !dev.Inited());
    TEST_ASSERT(cls.unbinds == 1U && !hw.ep2_out.inflight);
    TEST_ASSERT(cls.out_done.size() == 1U && hw.ep2_out.stalls == 0U);
    TEST_ASSERT(hw.ep2_out.stops == 2U && hw.ep2_out.closes >= 1U);
    TEST_ASSERT(hw.in0.GetState() == Endpoint::State::DISABLED);
    TEST_ASSERT(Count("controller-reset") == 1U && Count("ready") == 0U);
    // 主机重新枚举 / The host enumerates again
    hw.ep2_out.stuck = false;
    Enumerate(dev, hw);
    TEST_ASSERT(cls.binds == 2U && hw.ep2_out.inflight);
  }
  // EP0 的停止在新 SETUP 时仍挂起：硬件直接读写槽，新应答借不到槽，传输被拒绝；之后
  // 报告超时，复位控制器。
  // EP0's stop is still pending at a new SETUP: the hardware accesses the slot
  // directly, the new reply gets no slot and the transfer is refused; the later
  // timeout resets the controller.
  {
    Hw hw;
    TestClass cls;
    FakeUsb dev(hw, {{&cls}});
    dev.Init(false);
    Enumerate(dev, hw);
    cls.response.assign(150U, 1U);
    dev.Setup(Make(0xA1U, REQ_IN, 0U, 0U, 200U));
    TEST_ASSERT(hw.in0.inflight);
    hw.in0.stuck = true;
    dev.Setup(Make(0x80U, 8U, 0U, 0U, 1U));
    TEST_ASSERT(dev.resets == 0U && Stalled(hw) && cls.aborts.size() == 1U);
    dev.Irq([&hw](bool isr) { hw.in0.TimedOut(isr); });
    TEST_ASSERT(dev.resets == 1U && !dev.Inited() && !hw.in0.inflight);
    TEST_ASSERT(cls.aborts.size() == 1U && cls.unbinds == 1U);
    hw.in0.stuck = false;
    Enumerate(dev, hw);
    TEST_ASSERT(Configuration(dev, hw) == 1U);
  }
  // 复位时 Close 的停止挂起：端点已交回池；下一次绑定的 Configure 照常成功，接收排队，
  // 停止结束时依次 CloseHardware、ConfigureHardware，然后启动。
  // The stop of a Close during a reset stays pending and the endpoint is back in the
  // pool; the Configure of the next bind succeeds, the receive is queued, and when the
  // stop ends CloseHardware and ConfigureHardware run in this order, then it starts.
  {
    Hw hw;
    TestClass cls;
    FakeUsb dev(hw, {{&cls}});
    dev.Init(false);
    Enumerate(dev, hw);
    hw.ep2_out.stuck = true;
    const size_t closes = hw.ep2_out.closes;
    const size_t configures = hw.ep2_out.configures;
    const size_t starts = hw.ep2_out.starts.size();
    dev.Reset();
    TEST_ASSERT(dev.resets == 0U && dev.Inited() && hw.ep2_out.inflight);
    TEST_ASSERT(cls.out_done.back().result == ErrorCode::ABORTED);
    ControlNone(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(hw.ep2_out.GetState() == Endpoint::State::IDLE);
    TEST_ASSERT(hw.ep2_out.closes == closes && hw.ep2_out.configures == configures);
    TEST_ASSERT(hw.ep2_out.starts.size() == starts);
    hw.ep2_out.stuck = false;
    dev.Irq([&hw](bool isr) { hw.ep2_out.Done(isr, 0U, ErrorCode::ABORTED); });
    TEST_ASSERT(hw.ep2_out.closes == closes + 1U);
    TEST_ASSERT(hw.ep2_out.configures == configures + 1U);
    TEST_ASSERT(hw.ep2_out.starts.size() == starts + 1U && hw.ep2_out.inflight);
    TEST_ASSERT(Configuration(dev, hw) == 1U && dev.resets == 0U);
  }

  // Stop 因未决停止进入 detach_pending_ 期间收到 OnBusReset：不得重新 Initialize EP0，
  // 待停止完成后 FinishDetach 再完整收尾 / OnBusReset during detach_pending_ must not
  // re-open EP0 before FinishDetach runs
  {
    Hw hw;
    TestClass cls;
    FakeUsb dev(hw, {{&cls}});
    dev.Init(false);
    Enumerate(dev, hw);
    hw.ep2_out.stuck = true;
    dev.Stop(false);
    TEST_ASSERT(!dev.Inited() && hw.ep2_out.inflight);
    const size_t ep0_configs = hw.in0.configures;
    dev.Reset();
    TEST_ASSERT(!dev.Inited() && hw.in0.configures == ep0_configs);
    hw.ep2_out.stuck = false;
    dev.Irq([&hw](bool isr) { hw.ep2_out.Done(isr, 0U, ErrorCode::ABORTED); });
    TEST_ASSERT(!dev.Inited() && hw.in0.GetState() == Endpoint::State::DISABLED);
    dev.Start(false);
    dev.Reset();
    TEST_ASSERT(dev.Inited() && hw.in0.GetState() == Endpoint::State::IDLE);
  }
}

/// 绑定失败经 SET_CONFIGURATION 报告 / A bind failure is reported through
/// SET_CONFIGURATION
void TestBindFailure()
{
  Hw hw;
  TestClass first;
  TestClass second(EPNumber::EP3, EPNumber::EP4);
  second.bind_result = ErrorCode::FAILED;
  FakeUsb dev(hw, {{&first, &second}});
  dev.Init(false);
  dev.Reset();

  // 复合设备：接口号累加 / Composite device: interface numbers accumulate
  Bytes config = GetDescriptor(dev, hw, 2U, 0U, 255U);
  TEST_ASSERT(config.size() == 9U + 23U + 23U && config[4] == 2U && config[34] == 1U);

  // 第二个类取了 IN 后失败：两个类都解绑（失败的那个撤销一半），回到未配置，STALL。
  // The second class fails after taking its IN: both are unbound (the failing one
  // undoes its half), back to unconfigured, stall.
  Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
  TEST_ASSERT(first.binds == 1U && second.binds == 1U);
  TEST_ASSERT(first.unbinds == 1U && second.unbinds == 1U);
  TEST_ASSERT(first.in == nullptr && second.in == nullptr);
  TEST_ASSERT(hw.ep1_in.GetState() == Endpoint::State::DISABLED);
  TEST_ASSERT(Configuration(dev, hw) == 0U);
  // 端点已交回池，按地址查不到 / The endpoints are back in the pool, not found by
  // address
  Refused(dev, hw, Make(0x82U, 0U, 0U, 0x81U, 2U));

  // 改好后成功，端点可再取 / Fixed, it succeeds and the endpoints can be taken again
  second.bind_result = ErrorCode::OK;
  ControlNone(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
  TEST_ASSERT(Configuration(dev, hw) == 1U);
  TEST_ASSERT(hw.ep3_in.GetState() == Endpoint::State::IDLE && hw.ep4_out.inflight);

  // 写描述符失败：不绑定 / A descriptor write fails: no bind
  Hw hw2;
  TestClass missing(EPNumber::EP5, EPNumber::EP2);
  FakeUsb dev2(hw2, {{&missing}});
  dev2.Init(false);
  dev2.Reset();
  Refused(dev2, hw2, Make(0x80U, 6U, 0x0200U, 0U, 255U));
  Refused(dev2, hw2, Make(0x00U, 9U, 1U, 0U, 0U));
  TEST_ASSERT(missing.binds == 0U && missing.unbinds == 0U);
  TEST_ASSERT(Configuration(dev2, hw2) == 0U);
}

// ---- 四个缺陷的验收测试 / Acceptance tests of the four defects ----

/// IN 数据阶段里主机提前发状态 OUT / An early status OUT during the IN
/// data stage
void TestEarlyStatusOut()
{
  Hw hw;
  TestClass cls;
  FakeUsb dev(hw, {{&cls}}, Speed::FULL, DeviceDescriptor::PacketSize0::SIZE_8);
  dev.Init(false);
  dev.Reset();

  // 基线：8+8+2 / Baseline: 8+8+2
  std::vector<size_t> lengths;
  TEST_ASSERT(GetDescriptor(dev, hw, 1U, 0U, 18U).size() == 18U);
  (void)ControlIn(dev, hw, Make(0x80U, 6U, 0x0100U, 0U, 18U), &lengths);
  TEST_ASSERT((lengths == std::vector<size_t>{8U, 8U, 2U}));

  // 第一包在途时主机发状态 OUT：不再发数据，传输结束一次，下一个 SETUP 正常。
  // The host sends the status OUT while the first packet is in flight: no more data,
  // the transfer ends once, the next SETUP is normal.
  const size_t ready = dev.ready;
  const size_t starts = hw.in0.starts.size();
  dev.Setup(Make(0x80U, 6U, 0x0100U, 0U, 18U));
  TEST_ASSERT(hw.in0.starts.size() == starts + 1U && hw.in0.inflight);
  hw.in0.stop_outcomes.push_back({ErrorCode::OK, 8U});
  StatusOut(dev, hw);
  TEST_ASSERT(hw.in0.starts.size() == starts + 1U && !hw.in0.inflight);
  TEST_ASSERT(dev.ready == ready + 1U);
  TEST_ASSERT(Configuration(dev, hw) == 0U && dev.ready == ready + 2U);

  // 同样发生在类的 IN 传输上：Complete 恰好一次 / The same on a class IN transfer:
  // exactly one Complete
  Enumerate(dev, hw);
  cls.response.assign(20U, 7U);
  const size_t ready2 = dev.ready;
  dev.Setup(Make(0xA1U, REQ_IN, 0U, 0U, 20U));
  InDone(dev, hw);
  TEST_ASSERT(hw.in0.inflight);
  StatusOut(dev, hw);
  TEST_ASSERT(cls.completes.size() == 1U && cls.aborts.empty());
  TEST_ASSERT(dev.ready == ready2 + 1U && !hw.in0.inflight);
  TEST_ASSERT(Configuration(dev, hw) == 1U && cls.completes.size() == 1U);
}

/// 迟到的完成不得重新打开 EP0 / A late completion must not reopen EP0
void TestLateCompletion()
{
  Hw hw;
  TestClass cls;
  FakeUsb dev(hw, {{&cls}});
  dev.Init(false);
  dev.Reset();

  // 顺序一：状态 IN 的完成在 SETUP 之前报告（同一次中断）。地址写入，OnControlReady
  // 在 SETUP 之前出现一次，新请求照常。
  // Order one: the status IN completion is reported before the SETUP (same
  // interrupt). The address is written, OnControlReady appears once before the SETUP,
  // the new request is served.
  dev.Setup(Make(0x00U, 5U, 9U, 0U, 0U));
  TEST_ASSERT(hw.in0.inflight && hw.in0.LastLength() == 0U);
  g_log.clear();
  dev.Irq(
      [&](bool isr)
      {
        hw.in0.Done(isr, 0U);
        dev.HwSetup(isr, Make(0x80U, 8U, 0U, 0U, 1U));
      });
  TEST_ASSERT((g_log == std::vector<std::string>{"address-after", "ready", "setup"}));
  TEST_ASSERT(hw.in0.inflight && hw.in0.starts.back().bytes == Bytes{0U});
  (void)TakeIn(dev, hw);
  StatusOut(dev, hw);
  TEST_ASSERT(Count("ready") == 2U);

  // 顺序二：SETUP 先到，旧的完成在 Flush 里以停止结果 OK 交回：不调用 OnControlReady。
  // Order two: the SETUP comes first and the old completion is handed back by the
  // flush with stop outcome OK: no OnControlReady.
  dev.Setup(Make(0x00U, 5U, 10U, 0U, 0U));
  hw.in0.stop_outcomes.push_back({ErrorCode::OK, 0U});
  g_log.clear();
  dev.Setup(Make(0x80U, 8U, 0U, 0U, 1U));
  TEST_ASSERT((g_log == std::vector<std::string>{"setup"}));
  TEST_ASSERT(hw.in0.inflight && hw.in0.starts.back().bytes == Bytes{0U});
  (void)TakeIn(dev, hw);
  StatusOut(dev, hw);
  TEST_ASSERT(Count("ready") == 1U);

  // 类的 IN 传输，数据和状态都在下一个 SETUP 之前完成（同一次中断）：Complete 不是
  // Abort，OnControlReady 在 SETUP 之前一次。
  // A class IN transfer whose data and status both complete before the next SETUP
  // (same interrupt): Complete, not Abort, and OnControlReady once before the SETUP.
  ControlNone(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
  cls.response.assign(6U, 3U);
  dev.Setup(Make(0xA1U, REQ_IN, 0U, 0U, 6U));
  g_log.clear();
  dev.Irq(
      [&](bool isr)
      {
        hw.in0.Done(isr, 6U);
        hw.out0.HostSends(isr, {});
        dev.HwSetup(isr, Make(0x80U, 8U, 0U, 0U, 1U));
      });
  TEST_ASSERT((g_log == std::vector<std::string>{"complete", "ready", "setup"}));
  TEST_ASSERT(cls.completes.size() == 1U && cls.aborts.empty());
  TEST_ASSERT(hw.in0.starts.back().bytes == Bytes{1U});
}

/// 没有帧号时的 SYNCH_FRAME / SYNCH_FRAME without a frame number
void TestSynchFrame()
{
  Hw hw;
  TestClass cls(EPNumber::EP1, EPNumber::EP2, Type::ISOCHRONOUS);
  FakeUsb dev(hw, {{&cls}});
  dev.Init(false);
  Enumerate(dev, hw);
  TEST_ASSERT(hw.ep1_in.GetType() == Type::ISOCHRONOUS);
  // 端点存在（GET_STATUS 回答），SYNCH_FRAME 仍被拒绝：SOF 之前、之后、帧号未知时。
  // The endpoint exists (GET_STATUS answers), yet SYNCH_FRAME is refused: before any
  // SOF, after one, and with an unknown frame number.
  TEST_ASSERT((ControlIn(dev, hw, Make(0x82U, 0U, 0U, 0x81U, 2U)) == Bytes{0U, 0U}));
  Refused(dev, hw, Make(0x82U, 12U, 0U, 0x81U, 2U));
  dev.Sof(100U);
  Refused(dev, hw, Make(0x82U, 12U, 0U, 0x81U, 2U));
  dev.Sof(0xffffU);
  Refused(dev, hw, Make(0x82U, 12U, 0U, 0x81U, 2U));
  // 等时端点不接受 HALT / An isochronous endpoint takes no halt
  Refused(dev, hw, Make(0x02U, 3U, 0U, 0x81U, 0U));
}

/// 构造之后不再分配 / Nothing is allocated after construction
void TestNoAllocation()
{
  Hw hw;
  TestClass good;
  TestClass bad(EPNumber::EP5, EPNumber::EP4);  // EP5 没有登记 / EP5 is not registered
  FakeUsb dev(hw, {{&good}, {&bad}});
  const size_t before = g_allocations.load();
  FakeUsbTest::g_alloc_counting_flag = &g_counting;
  g_counting = true;
  dev.Init(false);
  dev.Reset();
  dev.Setup(Make(0x80U, 6U, 0x0201U, 0U, 255U));
  TEST_ASSERT(Stalled(hw));
  dev.Setup(Make(0x00U, 9U, 2U, 0U, 0U));
  TEST_ASSERT(Stalled(hw));
  dev.Setup(Make(0x80U, 6U, 0x0200U, 0U, 255U));
  while (hw.in0.inflight)
  {
    InDone(dev, hw);
  }
  StatusOut(dev, hw);
  dev.Setup(Make(0x00U, 9U, 1U, 0U, 0U));
  InDone(dev, hw);
  dev.Reset();
  dev.Reset();
  dev.Deinit(false);
  dev.Init(false);
  good.RequestService(false);
  g_counting = false;
  FakeUsbTest::g_alloc_counting_flag = nullptr;
  TEST_ASSERT(g_allocations.load() == before);
  TEST_ASSERT(dev.Inited() && good.binds == 1U && bad.binds == 0U && good.unbinds == 1U);
}

/// 多线程：类服务请求、SOF 中断和 Init/Deinit 并发 / Threads: class service requests,
/// SOF interrupts and Init/Deinit at once
/**
 * @brief SETUP 在本轮取走事件之后、端点阶段之前锁存 / A SETUP latched after the pass
 *        took its events and before its endpoint step
 * @param wire 撤回要等线上一包 / The retract waits for a packet on the wire
 * @note 旧传输的应答在端点阶段装入后被撤回，以 FAILED 或之后的 ABORTED 结束，主机只收到
 *       新传输的应答。
 *       The old transfer's reply is retracted after its load in the endpoint step and
 *       ends with FAILED or a later ABORTED; the host receives the new transfer's reply
 *       only.
 */
void TestSetupLatchedBeforeStart(bool wire)
{
  Hw hw;
  TestClass cls;
  FakeUsb dev(hw, {{&cls}});
  dev.Init(false);
  Enumerate(dev, hw);
  hw.in0.retract_on_wire = wire;
  hw.out0.retract_on_wire = wire;
  const size_t starts = hw.in0.starts.size();
  g_log.clear();
  dev.Irq(
      [&](bool isr)
      {
        dev.HwSetup(isr, Make(0x80U, 6U, 0x0100U, 0U, 18U));
        hw.in0.setup_latched = true;
        hw.out0.setup_latched = true;
        dev.Latch(
            [&](bool later)
            {
              for (FakeEp* ep : {&hw.out0, &hw.in0})
              {
                // 撤回结束先于 SETUP 报告 / The retract ends before the SETUP report
                if (ep->retracting)
                {
                  ep->retracting = false;
                  ep->Done(later, 0U, ErrorCode::ABORTED);
                }
                ep->setup_latched = false;
              }
              dev.HwSetup(later, Make(0x80U, 8U, 0U, 0U, 1U));
            });
      });
  // 端点阶段装入旧应答后复读到 SETUP，撤回；主机没有收到它 / The endpoint step loaded
  // the old reply, re-read the SETUP and retracted it; the host never got it
  TEST_ASSERT(hw.in0.retracts == 1U && hw.out0.retracts == 1U);
  TEST_ASSERT(hw.in0.starts.size() == starts && Count("setup") == 1U);
  TEST_ASSERT(hw.in0.inflight == wire && hw.out0.inflight == wire);
  dev.Fire();
  TEST_ASSERT(Count("setup") == 2U && !Stalled(hw));
  TEST_ASSERT(hw.in0.starts.size() == starts + 1U);
  TEST_ASSERT(hw.in0.starts.back().bytes == Bytes{1U});
  TEST_ASSERT(hw.out0.inflight && hw.out0.LastLength() == 0U);
  TEST_ASSERT(TakeIn(dev, hw) == Bytes{1U});
  StatusOut(dev, hw);
  TEST_ASSERT(Configuration(dev, hw) == 1U);
}

/// 硬件不直接读写 EP0 的槽：停止仍挂起时新应答借到槽、排队 / The hardware never
/// accesses EP0's slot itself: with the stop still pending the new reply gets the slot
/// and waits in the queue
void TestEp0IndirectNewReply()
{
  Hw hw;
  hw.in0.limits.direct_memory = false;
  TestClass cls;
  FakeUsb dev(hw, {{&cls}});
  dev.Init(false);
  Enumerate(dev, hw);
  cls.response.assign(150U, 1U);
  dev.Setup(Make(0xA1U, REQ_IN, 0U, 0U, 200U));
  TEST_ASSERT(hw.in0.inflight);
  hw.in0.stuck = true;
  const size_t starts = hw.in0.starts.size();
  dev.Setup(Make(0x80U, 8U, 0U, 0U, 1U));
  TEST_ASSERT(!Stalled(hw) && dev.resets == 0U && cls.aborts.size() == 1U);
  TEST_ASSERT(hw.in0.starts.size() == starts);
  hw.in0.stuck = false;
  dev.Irq([&hw](bool isr) { hw.in0.Done(isr, 0U, ErrorCode::ABORTED); });
  TEST_ASSERT(hw.in0.starts.size() == starts + 1U);
  TEST_ASSERT(TakeIn(dev, hw) == Bytes{1U});
  StatusOut(dev, hw);
}

/// Deinit 时 EP0 的停止挂起，Init 的 OnControlReady 先于推迟的 CloseHardware、
/// ConfigureHardware / EP0's stop is pending at Deinit; Init's OnControlReady comes
/// before the deferred CloseHardware and ConfigureHardware
void TestDeinitInitWithEp0Pending()
{
  Hw hw;
  TestClass cls;
  FakeUsb dev(hw, {{&cls}});
  dev.Init(false);
  Enumerate(dev, hw);
  cls.response.assign(150U, 1U);
  dev.Setup(Make(0xA1U, REQ_IN, 0U, 0U, 200U));
  hw.in0.stuck = true;
  const size_t closes = hw.in0.closes;
  const size_t configures = hw.in0.configures;
  const size_t ready = dev.ready;
  dev.Deinit(false);
  TEST_ASSERT(!dev.Inited() && cls.unbinds == 1U && hw.in0.closes == closes);
  dev.Init(false);
  TEST_ASSERT(dev.Inited() && dev.ready == ready + 1U);
  TEST_ASSERT(hw.in0.closes == closes && hw.in0.configures == configures);
  hw.in0.stuck = false;
  dev.Irq([&hw](bool isr) { hw.in0.Done(isr, 0U, ErrorCode::ABORTED); });
  TEST_ASSERT(hw.in0.closes == closes + 1U && hw.in0.configures == configures + 1U);
  TEST_ASSERT(dev.ready == ready + 1U);
  TEST_ASSERT(Configuration(dev, hw) == 0U);
}

void TestThreads()
{
  Hw hw;
  TestClass cls;
  std::atomic<int> inside{0};
  cls.owner_check = &inside;
  FakeUsb dev(hw, {{&cls}});
  dev.Init(false);
  Enumerate(dev, hw);

  constexpr int ROUNDS = 2000;
  std::atomic<bool> go{false};
  std::thread service(
      [&]
      {
        while (!go.load())
        {
        }
        for (int i = 0; i < ROUNDS; ++i)
        {
          cls.RequestService(false);
        }
      });
  std::thread interrupt(
      [&]
      {
        while (!go.load())
        {
        }
        for (int i = 0; i < ROUNDS; ++i)
        {
          dev.Sof(static_cast<uint16_t>(i & 0x7ff));
        }
      });
  go.store(true);
  for (int i = 0; i < 50; ++i)
  {
    dev.Deinit(false);
    dev.Init(false);
  }
  service.join();
  interrupt.join();

  // 每个请求都被某一轮处理（合并后至少一次）；最后一个请求之后必有一次 OnService。
  // Every request was handled by some pass (merged, at least once); after the last
  // request there is always one OnService.
  TEST_ASSERT(cls.services >= 1U && inside.load() == 0);
  const size_t services = cls.services;
  cls.RequestService(false);
  TEST_ASSERT(cls.services == services + 1U);
  TEST_ASSERT(!dev.masked.load() && dev.Inited());
}

// BOS 空能力、动态启用、能力校验与运行期错误保护 / BOS disabled/dynamic caps, block
// validation, multi-config deduplication, and runtime error guards
void TestBosAndCompositionEdgeCases()
{
  class RawBosCap final : public LibXR::USB::BosCapability
  {
   public:
    RawBosCap(const void* data, size_t size, size_t max_size = 0U)
        : raw_{data, size}, max_size_(max_size)
    {
    }
    ConstRawData GetCapabilityDescriptor() const override { return raw_; }
    size_t GetMaxCapabilityDescriptorSize() const override
    {
      return max_size_ != 0U ? max_size_ : raw_.size_;
    }
    ConstRawData raw_;
    size_t max_size_;
  };

  // 1) 禁用 WebUSB 返回 {nullptr, 0} 且后续调用 SetLandingPageUrl() 启用，仍可输出。
  // 1) Disabled WebUSB returns {nullptr, 0} then enables via SetLandingPageUrl().
  {
    Hw hw;
    LibXR::USB::WebUsb::WebUsbBosCapability webusb(nullptr);
    TestClass cls;
    cls.bos_cap = &webusb;
    cls.guid = "{01234567-89AB-CDEF-0123-456789ABCDEF}";
    FakeUsb dev(hw, {{&cls}}, Speed::FULL, DeviceDescriptor::PacketSize0::SIZE_64,
                USBSpec::USB_2_1);
    dev.Init(false);
    dev.Reset();
    const Bytes bos_disabled = GetDescriptor(dev, hw, 15U, 0U, 255U);
    TEST_ASSERT(bos_disabled.size() == 5U + 28U + 7U && bos_disabled[4] == 2U);

    TEST_ASSERT(webusb.SetLandingPageUrl("https://example.com"));
    const Bytes bos_enabled = GetDescriptor(dev, hw, 15U, 0U, 255U);
    TEST_ASSERT(bos_enabled.size() == 5U + 28U + 24U + 7U && bos_enabled[4] == 3U);
  }

  // 2) 独立禁用 WebUSB（无 WinUSB）仅补齐 7 字节 USB 2.0 扩展能力。
  // 2) Disabled WebUSB alone skips null block and auto-appends only USB 2.0 Extension.
  {
    Hw hw;
    LibXR::USB::WebUsb::WebUsbBosCapability disabled_webusb(nullptr);
    TestClass cls;
    cls.bos_cap = &disabled_webusb;
    FakeUsb dev(hw, {{&cls}}, Speed::FULL, DeviceDescriptor::PacketSize0::SIZE_64,
                USBSpec::USB_2_1);
    dev.Init(false);
    dev.Reset();
    const Bytes bos = GetDescriptor(dev, hw, 15U, 0U, 255U);
    TEST_ASSERT(bos.size() == 12U && bos[1] == 15U && bos[4] == 1U);
    TEST_ASSERT(bos[5] == 7U && bos[6] == 0x10U && bos[7] == 0x02U);
  }

  // 3) 同一类出现在多个配置中：BOS 去重取样且只注册并输出一份能力。
  // 3) Same class in multiple configurations: BOS deduplicates queries and emits once.
  {
    Hw hw;
    LibXR::USB::WebUsb::WebUsbBosCapability webusb("https://example.com");
    TestClass shared_cls;
    shared_cls.bos_cap = &webusb;
    FakeUsb dev(hw, {{&shared_cls}, {&shared_cls}}, Speed::FULL,
                DeviceDescriptor::PacketSize0::SIZE_64, USBSpec::USB_2_1);
    TEST_ASSERT(shared_cls.bos_cap_queries == 2U);
    dev.Init(false);
    dev.Reset();
    const Bytes bos = GetDescriptor(dev, hw, 15U, 0U, 255U);
    TEST_ASSERT(bos.size() == 5U + 24U + 7U && bos[4] == 2U);
    TEST_ASSERT(bos[5] == 24U && bos[6] == 0x10U && bos[7] == 0x05U);
  }

  // 4) 显式提供合法 7 字节 USB 2.0 Extension 时不再重复补齐。
  // 4) Explicit valid 7-byte USB 2.0 Extension suppresses automatic duplicate.
  {
    const uint8_t ext[7] = {7U, 0x10U, 0x02U, 0x02U, 0U, 0U, 0U};
    RawBosCap cap(ext, sizeof(ext));
    Hw hw;
    TestClass cls;
    cls.bos_cap = &cap;
    FakeUsb dev(hw, {{&cls}}, Speed::FULL, DeviceDescriptor::PacketSize0::SIZE_64,
                USBSpec::USB_2_1);
    dev.Init(false);
    dev.Reset();
    const Bytes bos = GetDescriptor(dev, hw, 15U, 0U, 255U);
    TEST_ASSERT(bos.size() == 12U && bos[4] == 1U && bos[8] == 0x02U);
  }

  // 5) 畸形能力块（短 USB 2.0 扩展、bLength 不符、类型错、空/零长）通过 EP0 STALL。
  // 5) Malformed capability blocks fail cleanly and STALL EP0 GET_DESCRIPTOR(BOS).
  {
    const uint8_t short_ext[3] = {3U, 0x10U, 0x02U};
    const uint8_t bad_len[4] = {6U, 0x10U, 0x05U, 0U};
    const uint8_t bad_type[7] = {7U, 0x99U, 0x02U, 0U, 0U, 0U, 0U};
    const RawBosCap invalid_caps[] = {
        {short_ext, sizeof(short_ext)},
        {bad_len, sizeof(bad_len)},
        {bad_type, sizeof(bad_type)},
        {bad_type, 0U},
        {nullptr, 7U},
    };
    for (RawBosCap bad : invalid_caps)
    {
      LibXR::USB::BosManager mgr(64U, 1U);
      TEST_ASSERT(mgr.AddCapability(&bad));
      const ConstRawData direct = mgr.GetBosDescriptor();
      TEST_ASSERT(direct.addr_ == nullptr && direct.size_ == 0U);

      Hw hw;
      TestClass cls;
      cls.bos_cap = &bad;
      FakeUsb dev(hw, {{&cls}}, Speed::FULL, DeviceDescriptor::PacketSize0::SIZE_64,
                  USBSpec::USB_2_1);
      dev.Init(false);
      dev.Reset();
      Refused(dev, hw, Make(0x80U, 6U, 0x0F00U, 0U, 255U));
    }

    // 成功 -> 中途能力报错 STALL -> 恢复能力后再次成功。
    // Success -> malformed capability STALLs EP0 -> recovers once restored.
    const uint8_t ok_ext[7] = {7U, 0x10U, 0x02U, 0x02U, 0U, 0U, 0U};
    RawBosCap cap(ok_ext, sizeof(ok_ext), sizeof(ok_ext));
    Hw hw;
    TestClass cls;
    cls.bos_cap = &cap;
    FakeUsb dev(hw, {{&cls}}, Speed::FULL, DeviceDescriptor::PacketSize0::SIZE_64,
                USBSpec::USB_2_1);
    dev.Init(false);
    dev.Reset();
    const Bytes first = GetDescriptor(dev, hw, 15U, 0U, 255U);
    TEST_ASSERT(first.size() == 12U && first[4] == 1U);
    cap.raw_ = {short_ext, sizeof(short_ext)};
    Refused(dev, hw, Make(0x80U, 6U, 0x0F00U, 0U, 255U));
    cap.raw_ = {ok_ext, sizeof(ok_ext)};
    const Bytes recovered = GetDescriptor(dev, hw, 15U, 0U, 255U);
    TEST_ASSERT(recovered.size() == 12U && recovered[4] == 1U);
  }

  // 6) BosManager 同输入容量边界：同一 7 B 能力在 12 B 成功、11 B 失败；空/满返回 false。
  // 6) BosManager bounds: same 7 B cap passes at 12 B, fails at 11 B; null/full false.
  {
    const uint8_t ext[7] = {7U, 0x10U, 0x02U, 0U, 0U, 0U, 0U};
    RawBosCap ok_cap(ext, sizeof(ext));
    LibXR::USB::BosManager exact(12U, 1U);
    TEST_ASSERT(!exact.AddCapability(nullptr));
    TEST_ASSERT(exact.AddCapability(&ok_cap));
    TEST_ASSERT(!exact.AddCapability(&ok_cap));
    const ConstRawData got = exact.GetBosDescriptor();
    TEST_ASSERT(got.addr_ != nullptr && got.size_ == 12U);

    LibXR::USB::BosManager tight(11U, 1U);
    TEST_ASSERT(tight.AddCapability(&ok_cap));
    const ConstRawData fail = tight.GetBosDescriptor();
    TEST_ASSERT(fail.addr_ == nullptr && fail.size_ == 0U);

    LibXR::USB::BosManager tight_auto(11U, 0U);
    TEST_ASSERT(tight_auto.GetBosDescriptor().addr_ == nullptr);
  }

  // 7) 单个空配置：构造与 WriteDeviceDescriptor 不得越界读 configs_[0].classes[0]。
  // 7) Single empty configuration: construction and WriteDeviceDescriptor must not read
  // configs_[0].classes[0] out of bounds.
  {
    Hw hw;
    FakeUsb dev(hw, {{}}, Speed::FULL, DeviceDescriptor::PacketSize0::SIZE_64,
                USBSpec::USB_2_0);
    dev.Init(false);
    dev.Reset();
    const Bytes cfg = GetDescriptor(dev, hw, 2U, 0U, 255U);
    TEST_ASSERT(cfg.size() == 9U && cfg[4] == 0U);
    const Bytes ddesc = GetDescriptor(dev, hw, 1U, 0U, 18U);
    TEST_ASSERT(ddesc.size() == 18U);
  }

  // 8) 运行期两条配置路径统一拒绝对外越界/空块，并验证 SET_CONFIGURATION 首类回滚。
  // 8) Runtime config paths reject null/oversized blocks uniformly and roll back bind.
  {
    Hw hw;
    TestClass cls;
    FakeUsb dev(hw, {{&cls}}, Speed::FULL, DeviceDescriptor::PacketSize0::SIZE_64,
                USBSpec::USB_2_0);
    dev.Init(false);
    dev.Reset();

    cls.use_bad_config_block = true;
    cls.bad_config_block = {nullptr, 9U};
    Refused(dev, hw, Make(0x80U, 6U, 0x0200U, 0U, 255U));
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));

    uint8_t oversized[24] = {};
    cls.bad_config_block = {oversized, sizeof(oversized)};
    Refused(dev, hw, Make(0x80U, 6U, 0x0200U, 0U, 255U));
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));

    cls.use_bad_config_block = false;
    cls.custom_itf_count = 300U;
    Refused(dev, hw, Make(0x80U, 6U, 0x0200U, 0U, 255U));
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    cls.custom_itf_count = 1U;
  }

  // 9) 第二类接口溢出时 SET_CONFIGURATION 失败 STALL、回滚第一类、设备回到未配置。
  // 9) SET_CONFIGURATION STALLs on 2nd-class interface overflow and unbinds 1st class.
  {
    Hw hw;
    TestClass c1(EPNumber::EP1, EPNumber::EP2);
    TestClass c2(EPNumber::EP3, EPNumber::EP4);
    FakeUsb dev(hw, {{&c1, &c2}});
    dev.Init(false);
    dev.Reset();
    c2.custom_itf_count = 255U;
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(c1.binds == 1U && c1.unbinds == 1U && c2.binds == 0U);
    TEST_ASSERT(Configuration(dev, hw) == 0U);
  }

  // 10) 多类各自上界之和 > 65535 但单次实际描述 <= 65535 时，允许构造与枚举。
  // 10) Allows construction when sum of per-class bounds > 65535 while actual <= 65535.
  {
    Hw hw;
    TestClass c1(EPNumber::EP1, EPNumber::EP2);
    TestClass c2(EPNumber::EP3, EPNumber::EP4);
    c1.custom_max_config_size = 40000U;
    c2.custom_max_config_size = 40000U;
    FakeUsb dev(hw, {{&c1, &c2}});
    dev.Init(false);
    dev.Reset();
    const Bytes cfg = GetDescriptor(dev, hw, 2U, 0U, 255U);
    TEST_ASSERT(cfg.size() == 9U + 23U + 23U);
  }

  // 11) 两类各自符合上界但单速实际总长 > 65535 时，GET_DESCRIPTOR 与 SET_CONFIGURATION 均
  //     拒绝并回滚已绑定类。
  // 11) When per-class sizes fit individually but actual total > 65535, both
  //     GET_DESCRIPTOR and SET_CONFIGURATION STALL and roll back bound classes.
  {
    Hw hw;
    TestClass c1(EPNumber::EP1, EPNumber::EP2);
    TestClass c2(EPNumber::EP3, EPNumber::EP4);
    Bytes big(33000U, 0U);
    c1.custom_max_config_size = 40000U;
    c2.custom_max_config_size = 40000U;
    c1.use_bad_config_block = true;
    c2.use_bad_config_block = true;
    c1.bad_config_block = {big.data(), big.size()};
    c2.bad_config_block = {big.data(), big.size()};
    FakeUsb dev(hw, {{&c1, &c2}});
    dev.Init(false);
    dev.Reset();
    Refused(dev, hw, Make(0x80U, 6U, 0x0200U, 0U, 255U));
    const size_t c1_writes = c1.writes;
    const size_t c2_writes = c2.writes;
    TEST_ASSERT(c1_writes == 1U && c2_writes == 1U);
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(c1.writes == c1_writes + 1U && c2.writes == c2_writes + 1U);
    TEST_ASSERT(c1.binds == 1U && c1.unbinds == 1U && c2.binds == 0U);
    TEST_ASSERT(Configuration(dev, hw) == 0U);
  }

  // 12) 单类声明保守上界 65536 但实际输出 23 字节时，允许构造、枚举并完成配置。
  // 12) A single class with conservative GetMaxConfigSize()==65536 and 23-byte actual
  //     output succeeds construction, enumeration, and SET_CONFIGURATION.
  {
    Hw hw;
    TestClass cls(EPNumber::EP1, EPNumber::EP2);
    cls.custom_max_config_size = 65536U;
    FakeUsb dev(hw, {{&cls}});
    dev.Init(false);
    dev.Reset();
    const Bytes cfg = GetDescriptor(dev, hw, 2U, 0U, 255U);
    TEST_ASSERT(cfg.size() == 9U + 23U);
    ControlNone(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(Configuration(dev, hw) == 1U);
  }
}

// UTF-8 截断/过长/代理项/254 字节上限及接口字符串防静默截短 / UTF-8 truncation, overlong,
// surrogate, 254-byte cap, and runtime interface-string truncation rejection
void TestUtf8EdgeCases()
{
  static constexpr auto PACK = DescriptorStrings::MakeLanguagePack(
      DescriptorStrings::Language::EN_US, "A\xC2", "B\xE4\xB8\xAD\xE4\xB8",
      "S\xF0\x9F\x98\x80\x80\xE4\xB8");
  const uint8_t uid[4] = {0x12U, 0xABU, 0x00U, 0xEFU};
  DescriptorStrings strings({&PACK}, uid, sizeof(uid));
  using StrIdx = DescriptorStrings::Index;

  // Manufacturer: "A\xC2" -> 仅保留 'A'（4 字节），截断首字节安全跳过。
  // Manufacturer: "A\xC2" -> only 'A' (4 B), truncated UTF-8 lead ignored safely.
  TEST_ASSERT(strings.GenerateString(StrIdx::MANUFACTURER_STRING, 0x0409U) ==
              ErrorCode::OK);
  const LibXR::RawData manu = strings.GetData();
  TEST_ASSERT(manu.size_ == 4U);
  const auto* m = static_cast<const uint8_t*>(manu.addr_);
  TEST_ASSERT(m[0] == 4U && m[1] == 3U && m[2] == 'A' && m[3] == 0U);

  // Product: "B\xE4\xB8\xAD\xE4\xB8" -> 'B' + '中'，尾部半截 3 字节序列跳过。
  // Product: "B\xE4\xB8\xAD\xE4\xB8" -> 'B' + U+4E2D, trailing incomplete prefix ignored.
  TEST_ASSERT(strings.GenerateString(StrIdx::PRODUCT_STRING, 0x0409U) == ErrorCode::OK);
  const LibXR::RawData prod = strings.GetData();
  TEST_ASSERT(prod.size_ == 6U);
  const auto* p = static_cast<const uint8_t*>(prod.addr_);
  TEST_ASSERT(p[0] == 6U && p[1] == 3U && p[2] == 'B' && p[3] == 0U && p[4] == 0x2DU &&
              p[5] == 0x4EU);

  // Serial: "S" + 跳过非 BMP/孤立续字节/截断前缀 + UID 十六进制后缀。
  // Serial: "S" + skipped non-BMP/orphan/truncated prefix + UID hex suffix.
  TEST_ASSERT(strings.GenerateString(StrIdx::SERIAL_NUMBER_STRING, 0x0409U) ==
              ErrorCode::OK);
  const LibXR::RawData ser = strings.GetData();
  TEST_ASSERT(ser.size_ == 2U + 2U + 16U);
  const auto* s = static_cast<const uint8_t*>(ser.addr_);
  TEST_ASSERT(s[0] == 20U && s[2] == 'S' && s[4] == '1' && s[6] == '2' && s[8] == 'A' &&
              s[10] == 'B' && s[12] == '0' && s[14] == '0' && s[16] == 'E' &&
              s[18] == 'F');

  // 拒绝 overlong（C0 80、E0 80 80）与 UTF-16 代理项（ED A0 80），并保留后续字符。
  // Reject overlong (C0 80, E0 80 80) and surrogate (ED A0 80), preserving later ASCII.
  static constexpr auto PACK_ILLEGAL =
      DescriptorStrings::MakeLanguagePack(DescriptorStrings::Language::EN_US, "\300\200A",
                                          "\340\200\200B", "\355\240\200C\355\277\277D");
  DescriptorStrings illegal_strings({&PACK_ILLEGAL});
  TEST_ASSERT(illegal_strings.GenerateString(StrIdx::MANUFACTURER_STRING, 0x0409U) ==
              ErrorCode::OK);
  TEST_ASSERT(illegal_strings.GetData().size_ == 4U &&
              static_cast<const uint8_t*>(illegal_strings.GetData().addr_)[2] == 'A');
  TEST_ASSERT(illegal_strings.GenerateString(StrIdx::PRODUCT_STRING, 0x0409U) ==
              ErrorCode::OK);
  TEST_ASSERT(illegal_strings.GetData().size_ == 4U &&
              static_cast<const uint8_t*>(illegal_strings.GetData().addr_)[2] == 'B');
  TEST_ASSERT(illegal_strings.GenerateString(StrIdx::SERIAL_NUMBER_STRING, 0x0409U) ==
              ErrorCode::OK);
  const LibXR::RawData cd = illegal_strings.GetData();
  TEST_ASSERT(cd.size_ == 6U && static_cast<const uint8_t*>(cd.addr_)[2] == 'C' &&
              static_cast<const uint8_t*>(cd.addr_)[4] == 'D');

  // 126 个 ASCII 字符恰好达到 254 字节字符串描述符上限。
  // 126 ASCII characters hit the exact 254-byte USB string descriptor limit.
  static constexpr auto PACK_254 = DescriptorStrings::MakeLanguagePack(
      DescriptorStrings::Language::EN_US,
      "0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF"
      "0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCD",
      "P", "S");
  DescriptorStrings max_strings({&PACK_254});
  TEST_ASSERT(max_strings.GenerateString(StrIdx::MANUFACTURER_STRING, 0x0409U) ==
              ErrorCode::OK);
  TEST_ASSERT(max_strings.GetData().size_ == 254U &&
              static_cast<const uint8_t*>(max_strings.GetData().addr_)[0] == 254U);

  // DeviceComposition 接口字符串共用同一套编码规则，且运行期缓冲不足时拒绝而不静默截短。
  // DeviceComposition interface strings share the encoder and refuse overflow instead of
  // silently truncating.
  Hw hw;
  char dyn_name[4] = "A";
  TestClass c1(EPNumber::EP1, EPNumber::EP2, Type::BULK, "X\xC2");
  TestClass c2(EPNumber::EP3, EPNumber::EP4, Type::BULK, dyn_name);
  FakeUsb dev(hw, {{&c1, &c2}});
  dev.Init(false);
  dev.Reset();
  const Bytes s1 = GetDescriptor(dev, hw, 3U, 4U, 255U, 0x0409U);
  TEST_ASSERT((s1 == Bytes{4U, 3U, 'X', 0U}));
  const Bytes s2_ok = GetDescriptor(dev, hw, 3U, 5U, 255U, 0x0409U);
  TEST_ASSERT((s2_ok == Bytes{4U, 3U, 'A', 0U}));
  dyn_name[1] = 'B';
  dyn_name[2] = '\0';
  Refused(dev, hw, Make(0x80U, 6U, 0x0305U, 0x0409U, 255U));

  // 手写错配 LanguagePack：生成超长串失败后 GetData() 长度必须清零，不得拼旧头与新载荷。
  // Mismatched LanguagePack failure clears GetData() length instead of pairing stale len.
  const DescriptorStrings::LanguagePack mismatched_pack{
      DescriptorStrings::Language::EN_US, {"A", "BO", "S"}, {2U, 2U, 2U}, 2U};
  DescriptorStrings mismatched({&mismatched_pack});
  TEST_ASSERT(mismatched.GetData().size_ == 0U);
  TEST_ASSERT(mismatched.GenerateString(StrIdx::MANUFACTURER_STRING, 0x0409U) ==
              ErrorCode::OK);
  TEST_ASSERT(mismatched.GetData().size_ == 4U);
  TEST_ASSERT(mismatched.GenerateString(StrIdx::PRODUCT_STRING, 0x0409U) ==
              ErrorCode::FAILED);
  TEST_ASSERT(mismatched.GetData().size_ == 0U);
}

}  // namespace

void test_usb_dev_core()
{
  TestEnumeration();
  TestStandardRequests();
  TestHighSpeedAndBos();
  TestBosAndCompositionEdgeCases();
  TestUtf8EdgeCases();
  TestControlTransfers();
  TestSetupReplaces();
  TestResetAndDisconnect();
  TestSuspendSofService();
  TestStopFailure();
  TestBindFailure();
  TestEarlyStatusOut();
  TestLateCompletion();
  TestSynchFrame();
  TestNoAllocation();
  TestThreads();
  TestSetupLatchedBeforeStart(false);
  TestSetupLatchedBeforeStart(true);
  TestEp0IndirectNewReply();
  TestDeinitInitWithEp0Pending();
}
