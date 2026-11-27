/**
 * @file test_dap.cpp
 * @brief DAP v1、v2 类的假后端测试 / Fake-backend tests of the DAP v1 and v2 classes
 *
 * 用真实的设备核心、组合层、端点和 DAP 类；假后端设备同时派生 DeviceService 和
 * DeviceCore，测试把"硬件事件"排进它的队列再调用控制器中断入口，假端点记录每次启动，由
 * 测试决定何时完成。Step() 在测试线程或单独的线程里运行，和应用一样。覆盖命令往返、
 * 流水与顺序、补零、描述符、绑定失败、带在途请求解绑、结尾零长度包和多线程。
 * Uses the real device core, composition, endpoints and DAP classes; the fake backend
 * device derives from both DeviceService and DeviceCore, a test queues "hardware events"
 * and calls the controller interrupt entry, and fake endpoints record every start and
 * complete when the test says so. Step() runs in the test thread or a thread of its own,
 * as in an application. Covered: command round trips, pipeline and order, padding,
 * descriptors, bind failure, unbind with requests in flight, the closing zero-length
 * packet and threads.
 */

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "../debug/sim/jtag_target.hpp"
#include "daplink_v1.hpp"
#include "daplink_v2.hpp"
#include "debug/jtag_general_gpio.hpp"
#include "debug/swd.hpp"
#include "fake_usb.hpp"
#include "test_assert.hpp"

namespace
{
using namespace FakeUsbTest;
using LibXR::USB::DapLinkV1Class;
using LibXR::USB::DapLinkV2Class;
using CommandId = LibXR::USB::DapLinkV2Def::CommandId;
using InfoId = LibXR::USB::DapLinkV2Def::InfoId;
using Port = LibXR::USB::DapLinkV2Def::Port;

constexpr uint8_t INFO = static_cast<uint8_t>(CommandId::INFO);
constexpr uint8_t CONNECT = static_cast<uint8_t>(CommandId::CONNECT);
constexpr uint8_t DELAY = static_cast<uint8_t>(CommandId::DELAY);
constexpr uint8_t TRANSFER = static_cast<uint8_t>(CommandId::TRANSFER);
constexpr uint8_t TRANSFER_BLOCK = static_cast<uint8_t>(CommandId::TRANSFER_BLOCK);
constexpr uint8_t TRANSFER_ABORT = static_cast<uint8_t>(CommandId::TRANSFER_ABORT);
constexpr uint8_t QUEUE = static_cast<uint8_t>(CommandId::QUEUE_COMMANDS);
constexpr uint8_t EXECUTE = static_cast<uint8_t>(CommandId::EXECUTE_COMMANDS);
constexpr uint8_t SWJ_SEQUENCE = static_cast<uint8_t>(CommandId::SWJ_SEQUENCE);
constexpr uint8_t PACKET_SIZE_ID = static_cast<uint8_t>(InfoId::PACKET_SIZE);
constexpr uint8_t PACKET_COUNT_ID = static_cast<uint8_t>(InfoId::PACKET_COUNT);
constexpr uint8_t VENDOR_ID = static_cast<uint8_t>(InfoId::VENDOR);
constexpr uint8_t PRODUCT_ID = static_cast<uint8_t>(InfoId::PRODUCT);
constexpr uint8_t SWD = static_cast<uint8_t>(Port::SWD);

/// 最小的 SWD 探针：每次传输都成功，读到固定的字；记录每次调用所在的线程 / Minimal SWD
/// probe: every transfer succeeds and reads a fixed word; records the thread of each call
class FakeSwd : public LibXR::Debug::Swd
{
 public:
  unsigned enters = 0;
  unsigned closes = 0;
  unsigned sequences = 0;
  uint32_t clock_hz = 0;
  /// Step 开始一个会话时最后调用 SetClockHz：等到它就知道 Step 已拿着命令 / Step ends the
  /// start of a session with SetClockHz: once it was called, Step holds the command
  std::atomic<unsigned> clock_calls{0};
  std::vector<std::thread::id> callers;
  /// SeqWriteBits 执行期间调用一次 / Called once inside SeqWriteBits
  std::function<void()> on_sequence;

  ErrorCode SetClockHz(uint32_t hz) override
  {
    Note();
    clock_hz = hz;
    clock_calls.fetch_add(1);
    return ErrorCode::OK;
  }
  void Close() override
  {
    Note();
    ++closes;
  }
  ErrorCode LineReset() override
  {
    Note();
    return ErrorCode::OK;
  }
  ErrorCode EnterSwd() override
  {
    Note();
    ++enters;
    return ErrorCode::OK;
  }
  ErrorCode Transfer(const LibXR::Debug::SwdProtocol::Request& req,
                     LibXR::Debug::SwdProtocol::Response& resp) override
  {
    Note();
    if (req.rnw)
    {
      resp.rdata = 0x12345678U;
    }
    resp.ack = LibXR::Debug::SwdProtocol::Ack::OK;
    resp.parity_ok = true;
    return ErrorCode::OK;
  }
  void IdleClocks(uint32_t) override { Note(); }
  ErrorCode SeqWriteBits(uint32_t, const uint8_t*) override
  {
    Note();
    ++sequences;
    if (on_sequence)
    {
      const std::function<void()> hook = std::move(on_sequence);
      on_sequence = nullptr;
      hook();
    }
    return ErrorCode::OK;
  }
  ErrorCode SeqReadBits(uint32_t cycles, uint8_t* out) override
  {
    Note();
    if (out != nullptr)
    {
      std::memset(out, 0, (cycles + 7U) / 8U);
    }
    return ErrorCode::OK;
  }

 private:
  void Note() { callers.push_back(std::this_thread::get_id()); }
};

/**
 * @brief 假端点和端点池：EP1 IN、EP2 OUT 给第一个 DAP，EP3 IN、EP4 OUT 给第二个 / Fake
 *        endpoints and the pool: EP1 IN and EP2 OUT for the first DAP, EP3 IN and EP4 OUT
 *        for a second one
 */
struct Hw : FakeHw
{
  /// in_size：EP1 IN 的内存字节数；data_slots：数据端点的槽数，0 为核心默认 / in_size:
  /// bytes of EP1 IN memory; data_slots: slot count of the data endpoints, 0 for default
  explicit Hw(size_t in_size = 2048U, uint8_t data_slots = 0U)
      : ep1_in{EPNumber::EP1, Direction::IN, {mem[0].bytes, in_size}, data_slots},
        ep2_out{EPNumber::EP2, Direction::OUT, {mem[1].bytes, 2048U}, data_slots},
        ep3_in{EPNumber::EP3, Direction::IN, {mem[2].bytes, 2048U}, data_slots},
        ep4_out{EPNumber::EP4, Direction::OUT, {mem[3].bytes, 2048U}, data_slots}
  {
    Add(ep1_in);
    Add(ep2_out);
    Add(ep3_in);
    Add(ep4_out);
  }

  AlignedMemory<2048U> mem[4];
  FakeEp ep1_in;
  FakeEp ep2_out;
  FakeEp ep3_in;
  FakeEp ep4_out;
};

/// 等到条件成立，最多 seconds 秒 / Wait until the condition holds, seconds at most
bool WaitUntil(const std::function<bool()>& condition, int seconds = 5)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
  while (!condition())
  {
    if (std::chrono::steady_clock::now() > deadline)
    {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  }
  return true;
}

/// 按应用 DAP 线程的方式执行所有已到达的工作 / Run all work that has arrived, the way
/// the application's DAP thread would
template <typename Dap>
void Run(Dap& dap)
{
  for (size_t guard = 0; dap.Step(0) == ErrorCode::OK; ++guard)
  {
    TEST_ASSERT(guard < 10000U);
  }
}

/// 不含零长度包的 IN 请求 / The IN requests without zero-length packets
std::vector<Bytes> Responses(const FakeEp& ep)
{
  std::vector<Bytes> out;
  for (const Bytes& bytes : ep.transfers)
  {
    if (!bytes.empty())
    {
      out.push_back(bytes);
    }
  }
  return out;
}

/// 两个 IN 槽都能借到并交回 / Both IN slots can be borrowed and handed back
void BothInSlotsFree(FakeEp& ep)
{
  Endpoint::Slot a;
  Endpoint::Slot b;
  TEST_ASSERT(ep.AcquireSlot(a) == ErrorCode::OK);
  TEST_ASSERT(ep.AcquireSlot(b) == ErrorCode::OK);
  TEST_ASSERT(ep.ReleaseSlot(a) == ErrorCode::OK);
  TEST_ASSERT(ep.ReleaseSlot(b) == ErrorCode::OK);
}

// ---- DAP v2 ----

/// 可以挡住 OnService 的 DAP v2；services 只在 owner 中 / DAP v2 whose OnService can be
/// blocked; services is owner only
class Dap2 : public DapLinkV2Class<FakeSwd>
{
 public:
  using DapLinkV2Class<FakeSwd>::DapLinkV2Class;
  bool block_service = false;
  size_t services = 0;

 protected:
  void OnService(bool in_isr) override
  {
    ++services;
    if (!block_service)
    {
      DapLinkV2Class<FakeSwd>::OnService(in_isr);
    }
  }
};

/// 一个 DAP v2 的设备 / A device with one DAP v2
struct V2Bench
{
  explicit V2Bench(Speed speed = Speed::FULL, USBSpec spec = USBSpec::USB_2_0,
                   size_t in_size = 2048U, uint8_t slots = 0U)
      : hw(in_size, slots),
        dap(EPNumber::EP1, EPNumber::EP2, swd),
        dev(hw, {{&dap}}, speed, spec)
  {
  }

  /// 主机发一个包，Step 执行到没有工作 / The host sends a packet; Step runs until idle
  void Send(const Bytes& bytes)
  {
    HostOut(dev, hw.ep2_out, bytes);
    Run(dap);
  }

  /// 主机取走 IN 上全部请求，每次之后 Step 执行 / The host takes every IN request,
  /// with Step running after each
  void DrainIn()
  {
    for (size_t guard = 0; hw.ep1_in.inflight; ++guard)
    {
      TEST_ASSERT(guard < 10000U);
      InDone(dev, hw.ep1_in);
      Run(dap);
    }
  }

  Hw hw;
  FakeSwd swd;
  Dap2 dap;
  FakeUsb dev;
};

/// IN 上每个请求的长度 / Length of every IN request
std::vector<size_t> SentLengths(const FakeEp& ep)
{
  std::vector<size_t> lengths;
  for (const Bytes& sent : ep.transfers)
  {
    lengths.push_back(sent.size());
  }
  return lengths;
}

/// 读 count 个字的 DAP_TransferBlock，响应 4 + 4 * count 字节 / DAP_TransferBlock
/// reading count words; the response is 4 + 4 * count bytes
Bytes Block(uint16_t count)
{
  return Bytes{TRANSFER_BLOCK, 0x00U, static_cast<uint8_t>(count),
               static_cast<uint8_t>(count >> 8U), 0x0FU};
}

/// 描述符、字符串、WinUSB、绑定 / Descriptors, strings, WinUSB, binding
void TestV2Enumeration()
{
  {
    V2Bench b(Speed::HIGH, USBSpec::USB_2_1);
    b.dev.Init(false);
    b.dev.Reset(Speed::FULL);
    TEST_ASSERT(!b.dap.IsInited());

    // 全速：配置头、厂商接口、OUT、IN，共 32 字节 / Full speed: header, the vendor
    // interface, OUT and IN, 32 bytes
    const Bytes fs = GetDescriptor(b.dev, b.hw, 2U, 0U, 255U);
    TEST_ASSERT(fs.size() == 32U && fs[2] == 32U && fs[4] == 1U);
    TEST_ASSERT(fs[9] == 9U && fs[10] == 4U && fs[11] == 0U && fs[13] == 2U);
    TEST_ASSERT(fs[14] == 0xFFU && fs[15] == 0U && fs[16] == 0U && fs[17] == 4U);
    TEST_ASSERT(fs[18] == 7U && fs[19] == 5U && fs[20] == 0x02U && fs[21] == 2U);
    TEST_ASSERT(fs[22] == 64U && fs[23] == 0U && fs[24] == 0U);
    TEST_ASSERT(fs[25] == 7U && fs[27] == 0x81U && fs[28] == 2U && fs[29] == 64U);
    // 高速 512 / High speed: 512
    const Bytes hs = GetDescriptor(b.dev, b.hw, 7U, 0U, 255U);
    TEST_ASSERT(hs.size() == 32U && hs[22] == 0U && hs[23] == 2U && hs[30] == 2U);

    // 接口字符串 / Interface string
    const Bytes text = GetDescriptor(b.dev, b.hw, 3U, 4U, 255U, 0x0409U);
    TEST_ASSERT(text.size() == 2U + 2U * 12U && text[2] == 'C' &&
                text[2 + 2 * 11] == '2');

    // WinUSB：设备级描述符集里有本类的 GUID / WinUSB: the device-level set carries the
    // class's GUID
    const Bytes bos = GetDescriptor(b.dev, b.hw, 15U, 0U, 255U);
    const uint8_t vendor = bos[5 + 26];
    const Bytes set = ControlIn(b.dev, b.hw, Make(0xC0U, vendor, 0U, 7U, 0xffU));
    const std::string guid = "{CDB3B5AD-293B-4663-AA36-1AAE46463776}";
    Bytes utf16;
    for (char c : guid)
    {
      utf16.push_back(static_cast<uint8_t>(c));
      utf16.push_back(0U);
    }
    TEST_ASSERT(std::search(set.begin(), set.end(), utf16.begin(), utf16.end()) !=
                set.end());

    // 写描述符不碰端点；绑定挂好两个接收，第一个开始 / Writing descriptors touches no
    // endpoint; the bind arms two receives and the first starts
    TEST_ASSERT(b.hw.ep1_in.configures == 0U && b.hw.ep2_out.configures == 0U);
    b.dev.Reset(Speed::HIGH);
    ControlNone(b.dev, b.hw, Make(0x00U, 5U, 7U, 0U, 0U));
    SetConfiguration(b.dev, b.hw, 1U);
    TEST_ASSERT(b.hw.ep2_out.inflight && b.hw.ep2_out.LastLength() == 512U);
    TEST_ASSERT(b.hw.ep2_out.starts.size() == 1U && b.hw.ep1_in.starts.empty());
    // 高速下 DAP 包长 1024 / The DAP packet size is 1024 at high speed
    b.Send({INFO, PACKET_SIZE_ID});
    TEST_ASSERT(b.hw.ep1_in.InFlight() == (Bytes{INFO, 2U, 0x00U, 0x04U}));
  }

  // 复合设备：两个 DAP，各一个功能子集，接口 0 和 1 / Composite: two DAPs, one function
  // subset each, interfaces 0 and 1
  {
    Hw hw;
    FakeSwd swd_a;
    FakeSwd swd_b;
    Dap2 a(EPNumber::EP1, EPNumber::EP2, swd_a);
    Dap2 b(EPNumber::EP3, EPNumber::EP4, swd_b);
    FakeUsb dev(hw, {{&a, &b}}, Speed::FULL, USBSpec::USB_2_1);
    dev.Init(false);
    dev.Reset();
    const Bytes config = GetDescriptor(dev, hw, 2U, 0U, 255U);
    TEST_ASSERT(config.size() == 9U + 2U * 23U && config[4] == 2U && config[34] == 1U);
    TEST_ASSERT(config[43] == 0x04U && config[50] == 0x83U);
    const Bytes bos = GetDescriptor(dev, hw, 15U, 0U, 255U);
    const Bytes set = ControlIn(dev, hw, Make(0xC0U, bos[5 + 26], 0U, 7U, 0xffU));
    // 集合头 10、配置子集 8，之后两个功能子集 / Set header 10, configuration subset 8,
    // then two function subsets
    TEST_ASSERT(set[10 + 2] == 0x01U && set[18 + 2] == 0x02U && set[18 + 4] == 0U);
    const size_t function = set[18 + 6] | set[18 + 7] << 8U;
    TEST_ASSERT(set[18 + function + 2] == 0x02U && set[18 + function + 4] == 1U);
    Enumerate(dev, hw);
    TEST_ASSERT(hw.ep2_out.inflight && hw.ep4_out.inflight);
  }
}

/// 往返、流水、顺序、延时、批处理、补零 / Round trips, pipeline, order, delays,
/// batches, padding
void TestV2Commands()
{
  // 往返；PACKET_COUNT 回答实际的在途深度 4 / Round trip; PACKET_COUNT answers the
  // effective depth of 4
  {
    V2Bench b;
    Enumerate(b.dev, b.hw);
    TEST_ASSERT(b.hw.ep2_out.inflight && b.hw.ep2_out.LastLength() == 64U);
    b.Send({INFO, PACKET_COUNT_ID});
    TEST_ASSERT(b.hw.ep1_in.InFlight() == (Bytes{INFO, 1U, 0x04U}));
    InDone(b.dev, b.hw.ep1_in);
    b.Send({CONNECT, SWD});
    TEST_ASSERT(b.hw.ep1_in.InFlight() == (Bytes{CONNECT, SWD}));
    TEST_ASSERT(b.swd.enters == 1U && b.swd.clock_hz == 1000000U);
    InDone(b.dev, b.hw.ep1_in);
    TEST_ASSERT(b.hw.ep2_out.inflight);
  }

  // 四条命令在途：两个直接进 IN 槽，两个暂存，主机取走时按序提交 / Four commands in
  // flight: two straight into IN slots, two staged and submitted in order as the host
  // takes responses
  {
    V2Bench b;
    Enumerate(b.dev, b.hw);
    for (int i = 0; i < 4; ++i)
    {
      b.Send({INFO, PACKET_SIZE_ID});
    }
    TEST_ASSERT(b.hw.ep1_in.starts.size() == 1U);
    TEST_ASSERT(b.dap.Step(0) == ErrorCode::TIMEOUT);
    for (int i = 0; i < 4; ++i)
    {
      InDone(b.dev, b.hw.ep1_in);
    }
    TEST_ASSERT(!b.hw.ep1_in.inflight && b.hw.ep1_in.transfers.size() == 4U);
    TEST_ASSERT(b.hw.ep2_out.inflight);
  }

  // 有空闲 IN 槽时响应不经 OnService：Step 在本线程提交 / With an IN slot free the
  // response does not need OnService: Step submits it in its own thread
  {
    V2Bench b;
    Enumerate(b.dev, b.hw);
    b.dap.block_service = true;
    b.Send({INFO, PACKET_SIZE_ID});
    TEST_ASSERT(b.hw.ep1_in.InFlight() == (Bytes{INFO, 2U, 0x00U, 0x04U}));
    TEST_ASSERT(b.dap.services >= 1U);
    b.dap.block_service = false;
    InDone(b.dev, b.hw.ep1_in);
  }

  // 直接与暂存的响应交错，仍按命令顺序（3 字节和 4 字节可区分）/ Direct and staged
  // responses interleave and still keep the command order (3 and 4 bytes tell apart)
  {
    V2Bench b;
    Enumerate(b.dev, b.hw);
    std::vector<size_t> expected;
    auto send = [&](uint8_t id)
    {
      b.Send({INFO, id});
      expected.push_back(id == PACKET_COUNT_ID ? 3U : 4U);
    };
    send(PACKET_COUNT_ID);
    send(PACKET_SIZE_ID);
    send(PACKET_COUNT_ID);
    send(PACKET_SIZE_ID);
    TEST_ASSERT(b.hw.ep1_in.starts.size() == 1U);
    InDone(b.dev, b.hw.ep1_in);
    send(PACKET_COUNT_ID);
    b.DrainIn();
    send(PACKET_SIZE_ID);
    TEST_ASSERT(b.hw.ep1_in.inflight);
    b.DrainIn();
    const auto responses = Responses(b.hw.ep1_in);
    TEST_ASSERT(responses.size() == expected.size());
    for (size_t i = 0; i < expected.size(); ++i)
    {
      TEST_ASSERT(responses[i].size() == expected[i]);
    }
  }

  // 暂存响应还没被 owner 提交时有 IN 槽空出：下一条命令也得暂存，不能越过它 / An IN
  // slot comes free while a staged response is not yet submitted by the owner: the next
  // command is staged too and must not overtake it
  {
    V2Bench b;
    Enumerate(b.dev, b.hw);
    b.Send({INFO, PACKET_COUNT_ID});
    b.Send({INFO, PACKET_COUNT_ID});
    b.dap.block_service = true;
    // 第一个响应在 SWJ_Sequence 执行中途被取走：此时还没有暂存，空出的槽留着。
    // The first response is taken while SWJ_Sequence executes: nothing is staged yet,
    // so the freed slot stays free.
    b.swd.on_sequence = [&b]() { InDone(b.dev, b.hw.ep1_in); };
    b.Send({SWJ_SEQUENCE, 8U, 0xFFU});
    b.Send({INFO, PACKET_SIZE_ID});
    b.dap.block_service = false;
    b.DrainIn();
    TEST_ASSERT(SentLengths(b.hw.ep1_in) == (std::vector<size_t>{3U, 3U, 2U, 4U}));
  }

  // 延时命令：Step 之前只被收下；Step 在本线程等完再应答 / A delay is only received
  // before Step; Step waits it out in its own thread, then answers
  {
    V2Bench b;
    Enumerate(b.dev, b.hw);
    HostOut(b.dev, b.hw.ep2_out, {DELAY, 0x10U, 0x27U});
    TEST_ASSERT(b.hw.ep1_in.starts.empty() && b.hw.ep2_out.inflight);
    TEST_ASSERT(b.dap.Step(0) == ErrorCode::OK);
    TEST_ASSERT(b.hw.ep1_in.InFlight() == (Bytes{DELAY, 0x00U}));
    InDone(b.dev, b.hw.ep1_in);
    b.Send({DELAY, 0x00U, 0x00U});
    TEST_ASSERT(b.hw.ep1_in.InFlight() == (Bytes{DELAY, 0x00U}));
  }

  // 带延时的批处理整体应答一次 / A batch with a delay answers once, whole
  {
    V2Bench b;
    Enumerate(b.dev, b.hw);
    b.Send({QUEUE, 2U, INFO, VENDOR_ID, DELAY, 0xD0U, 0x07U});
    const Bytes batch = b.hw.ep1_in.InFlight();
    TEST_ASSERT(batch.size() == 13U && batch[0] == EXECUTE && batch[1] == 2U);
    TEST_ASSERT(batch[2] == INFO && batch[3] == 7U && batch[10] == 0U);
    TEST_ASSERT(batch[11] == DELAY && batch[12] == 0U);
    InDone(b.dev, b.hw.ep1_in);
    TEST_ASSERT(!b.hw.ep1_in.inflight && b.hw.ep2_out.inflight);
  }

  // 整包结尾的命令（128 字节、主机不发零长度包）按自己的长度结束 / A command ending on
  // a whole packet (128 bytes, no zero-length packet from the host) ends by its length
  {
    V2Bench b;
    Enumerate(b.dev, b.hw);
    Bytes queue{QUEUE, 63U};
    for (int i = 0; i < 63; ++i)
    {
      queue.push_back(INFO);
      queue.push_back(VENDOR_ID);
    }
    b.Send(Bytes(queue.begin(), queue.begin() + 64));
    TEST_ASSERT(b.hw.ep2_out.inflight && b.hw.ep2_out.LastLength() == 64U);
    TEST_ASSERT(b.hw.ep1_in.starts.empty());
    b.Send(Bytes(queue.begin() + 64, queue.end()));
    TEST_ASSERT(b.hw.ep1_in.InFlight()[0] == EXECUTE);
  }

  // 补零到 DAP 包长的传输：补零不是命令，按最少的请求收，命令只执行一次 / A transfer
  // padded to the DAP packet size: padding is no command, taken in as few requests as
  // possible, and the command runs once
  {
    V2Bench b;
    Enumerate(b.dev, b.hw);
    Bytes first(64, 0U);
    first[0] = INFO;
    first[1] = PACKET_SIZE_ID;
    b.Send(first);
    TEST_ASSERT(b.hw.ep1_in.starts.size() == 1U);
    InDone(b.dev, b.hw.ep1_in);
    b.Send(Bytes(64, 0U));
    TEST_ASSERT(b.hw.ep2_out.LastLength() == 64U);
    b.Send(Bytes(64, 0U));
    TEST_ASSERT(b.hw.ep2_out.LastLength() == 1024U - 192U);
    b.Send(Bytes(1024U - 192U, 0U));
    TEST_ASSERT(b.hw.ep2_out.LastLength() == 64U && b.hw.ep1_in.starts.size() == 1U);
    b.Send({INFO, PACKET_SIZE_ID});
    TEST_ASSERT(b.hw.ep1_in.starts.size() == 2U);
  }

  // DAP_TransferAbort 没有响应、不占命令槽 / DAP_TransferAbort has no response and
  // takes no command slot
  {
    V2Bench b;
    Enumerate(b.dev, b.hw);
    b.Send({TRANSFER_ABORT});
    TEST_ASSERT(b.hw.ep1_in.starts.empty() && b.hw.ep2_out.inflight);
    b.Send({CONNECT, SWD});
    InDone(b.dev, b.hw.ep1_in);
    b.Send({TRANSFER, 0x00U, 0x01U, 0x02U});
    const Bytes response = b.hw.ep1_in.InFlight();
    TEST_ASSERT(response[0] == TRANSFER && response[1] == 1U);
  }

  // QUEUE / EXECUTE 每包一个响应；畸形包回答错误 / QUEUE and EXECUTE answer per
  // packet; a malformed packet answers an error
  {
    V2Bench b;
    Enumerate(b.dev, b.hw);
    b.Send({QUEUE, 1U, INFO, PACKET_COUNT_ID});
    TEST_ASSERT(b.hw.ep1_in.InFlight() == (Bytes{EXECUTE, 1U, INFO, 1U, 0x04U}));
    InDone(b.dev, b.hw.ep1_in);
    b.Send({EXECUTE, 2U, INFO, PACKET_COUNT_ID, INFO, PACKET_SIZE_ID});
    const Bytes batch = b.hw.ep1_in.InFlight();
    TEST_ASSERT(batch.size() == 9U && batch[1] == 2U && batch[5] == INFO &&
                batch[6] == 2U);
    InDone(b.dev, b.hw.ep1_in);
    b.Send({EXECUTE});
    TEST_ASSERT(b.hw.ep1_in.InFlight() == (Bytes{EXECUTE, 0xFFU}));
  }

  // QUEUE / EXECUTE 的请求在最后一条命令之后补零：尾部字节忽略，与不补零时应答相同
  // （CMSIS-DAP 参考实现的做法） / QUEUE and EXECUTE padded after the last command: the
  // trailing bytes are ignored and the answer equals the unpadded one (as the CMSIS-DAP
  // reference does)
  {
    V2Bench b;
    Enumerate(b.dev, b.hw);
    b.Send({QUEUE, 1U, INFO, PACKET_COUNT_ID, 0U, 0U, 0U, 0U});
    TEST_ASSERT(b.hw.ep1_in.InFlight() == (Bytes{EXECUTE, 1U, INFO, 1U, 0x04U}));
    InDone(b.dev, b.hw.ep1_in);
    b.Send({EXECUTE, 2U, INFO, PACKET_COUNT_ID, INFO, PACKET_SIZE_ID, 0U, 0U, 0U});
    const Bytes batch = b.hw.ep1_in.InFlight();
    TEST_ASSERT(batch.size() == 9U && batch[0] == EXECUTE && batch[1] == 2U &&
                batch[5] == INFO && batch[6] == 2U);
    InDone(b.dev, b.hw.ep1_in);
    // 命令数比请求里的命令多：仍然回答错误 / More commands counted than the request
    // holds: still an error
    b.Send({EXECUTE, 3U, INFO, PACKET_COUNT_ID, INFO, PACKET_SIZE_ID});
    TEST_ASSERT(b.hw.ep1_in.InFlight() == (Bytes{EXECUTE, 0xFFU}));
  }

  // 响应放不下的批处理回答错误 / A batch whose responses do not fit answers an error
  {
    V2Bench b;
    Enumerate(b.dev, b.hw);
    Bytes req{EXECUTE, 121U, DELAY, 0xE8U, 0x03U};
    for (int i = 0; i < 120; ++i)
    {
      req.push_back(INFO);
      req.push_back(VENDOR_ID);
    }
    for (size_t off = 0; off < req.size(); off += 64U)
    {
      const size_t end = std::min(req.size(), off + 64U);
      b.Send(Bytes(req.begin() + static_cast<std::ptrdiff_t>(off),
                   req.begin() + static_cast<std::ptrdiff_t>(end)));
    }
    TEST_ASSERT(b.hw.ep1_in.InFlight() == (Bytes{EXECUTE, 0xFFU}));
  }

  // EXECUTE_COMMANDS 只剩 1 字节响应余量时的 INFO：绝不可越界写 resp[1] /
  // EXECUTE_COMMANDS with exactly 1 byte of resp_cap remaining before INFO must fail
  // cleanly without 1-byte out-of-bounds write
  {
    V2Bench b;
    Enumerate(b.dev, b.hw);
    // 2 (hdr) + 113 * 9 (INFO VENDOR_ID -> "XRobot\0") + 4 (PACKET_SIZE) = 1023 -> 1 B
    // left
    Bytes req{EXECUTE, 115U};
    for (int i = 0; i < 113; ++i)
    {
      req.push_back(INFO);
      req.push_back(VENDOR_ID);
    }
    req.push_back(INFO);
    req.push_back(PACKET_SIZE_ID);
    req.push_back(INFO);
    req.push_back(0xEEU);
    for (size_t off = 0; off < req.size(); off += 64U)
    {
      const size_t end = std::min(req.size(), off + 64U);
      b.Send(Bytes(req.begin() + static_cast<std::ptrdiff_t>(off),
                   req.begin() + static_cast<std::ptrdiff_t>(end)));
    }
    TEST_ASSERT(b.hw.ep1_in.InFlight() == (Bytes{EXECUTE, 0xFFU}));
  }

  // pad_skip_ 期间主机提前发短包 / ZLP 结束填充：下一条命令头字节不能被吞掉 /
  // Early short packet / ZLP during pad_skip_ must clear remaining skip
  {
    V2Bench b;
    Enumerate(b.dev, b.hw);
    Bytes first(64, 0U);
    first[0] = INFO;
    first[1] = PACKET_SIZE_ID;
    b.Send(first);
    InDone(b.dev, b.hw.ep1_in);
    b.Send(Bytes(64, 0U));  // opens pad_skip_ = 1024 - 128 = 896
    b.Send(Bytes(16, 0U));  // short packet (< 64): ends USB transfer early
    b.Send({INFO, VENDOR_ID});
    TEST_ASSERT(b.hw.ep1_in.inflight && b.hw.ep1_in.InFlight()[0] == INFO &&
                b.hw.ep1_in.InFlight()[1] == 7U);
    InDone(b.dev, b.hw.ep1_in);
  }

  // 补零的请求占满命令槽：补零照收，第五条命令的包留在类里，最后全部应答 / Padded
  // requests fill the command slots: padding is still taken, the fifth command's packet
  // is held by the class, and everything is answered in the end
  {
    V2Bench b;
    Enumerate(b.dev, b.hw);
    auto padded = [&b]()
    {
      Bytes first(64, 0U);
      first[0] = INFO;
      first[1] = PACKET_SIZE_ID;
      b.Send(first);
      b.Send(Bytes(64, 0U));
      b.Send(Bytes(64, 0U));
      b.Send(Bytes(1024U - 192U, 0U));
    };
    for (int i = 0; i < 4; ++i)
    {
      TEST_ASSERT(b.hw.ep2_out.inflight);
      padded();
    }
    TEST_ASSERT(b.hw.ep1_in.starts.size() == 1U && b.hw.ep2_out.inflight);
    Bytes fifth(64, 0x11U);
    fifth[0] = INFO;
    fifth[1] = PACKET_SIZE_ID;
    b.Send(fifth);
    TEST_ASSERT(b.hw.ep1_in.starts.size() == 1U);
    b.DrainIn();
    TEST_ASSERT(Responses(b.hw.ep1_in).size() == 5U && b.hw.ep2_out.inflight);
  }

  // 主机 HALT 数据 OUT：在拼的命令丢弃，清除后照常 / The host halts data OUT: the command
  // being assembled is dropped, and after the clear all works
  {
    V2Bench b;
    Enumerate(b.dev, b.hw);
    Bytes queue{QUEUE, 63U};
    for (int i = 0; i < 63; ++i)
    {
      queue.push_back(INFO);
      queue.push_back(VENDOR_ID);
    }
    b.Send(Bytes(queue.begin(), queue.begin() + 64));
    SetHalt(b.dev, b.hw, 0x02U);
    TEST_ASSERT(!b.hw.ep2_out.inflight && b.hw.ep2_out.IsStalled());
    ClearHalt(b.dev, b.hw, 0x02U);
    TEST_ASSERT(b.hw.ep2_out.inflight && b.hw.ep2_out.LastLength() == 64U);
    b.Send({INFO, PACKET_COUNT_ID});
    TEST_ASSERT(b.hw.ep1_in.InFlight() == (Bytes{INFO, 1U, 0x04U}));
    TEST_ASSERT(b.hw.ep2_out.inflight);
  }
}

/**
 * 结尾零长度包：短于 DAP 包长且是包长整数倍的响应带一个，等于 DAP 包长的不带；Step 直接
 * 提交和 owner 提交暂存响应两条路都一样；高速 512 字节包同理。
 * The closing zero-length packet: a response shorter than the DAP packet and a multiple
 * of the packet size gets one, one as long as the DAP packet does not; the same on the
 * direct path from Step and on the staged path from the owner; likewise with 512-byte
 * packets at high speed.
 */
void TestV2ClosingZlp()
{
  {
    V2Bench b;
    Enumerate(b.dev, b.hw);
    // [INFO, 62, 61 个字符 + NUL] = 64 字节 / [INFO, 62, 61 chars + NUL] = 64 bytes
    const std::string vendor(61, 'V');
    Dap2::InfoStrings info{};
    info.vendor = vendor.c_str();
    b.dap.SetInfoStrings(info);
    b.Send({INFO, VENDOR_ID});
    b.DrainIn();
    TEST_ASSERT(SentLengths(b.hw.ep1_in) == (std::vector<size_t>{64U, 0U}));

    // 1024 = DAP 包长：没有零长度包；128 有；3 没有 / 1024 = the DAP packet: none; 128
    // has one; 3 has none
    b.Send({CONNECT, SWD});
    b.Send(Block(255U));
    b.Send(Block(31U));
    b.Send({INFO, PACKET_COUNT_ID});
    b.DrainIn();
    TEST_ASSERT(SentLengths(b.hw.ep1_in) ==
                (std::vector<size_t>{64U, 0U, 2U, 1024U, 128U, 0U, 3U}));

    // 暂存路径：两个 IN 槽占着，64 字节的响应暂存，由 owner 提交时也带零长度包。
    // Staged path: both IN slots busy, the 64-byte response is staged and the owner's
    // submission carries the zero-length packet too.
    b.hw.ep1_in.transfers.clear();
    b.Send({INFO, PACKET_COUNT_ID});
    b.Send({INFO, PACKET_COUNT_ID});
    b.Send(Block(15U));
    TEST_ASSERT(b.hw.ep1_in.inflight && b.hw.ep1_in.transfers.empty());
    b.DrainIn();
    TEST_ASSERT(SentLengths(b.hw.ep1_in) == (std::vector<size_t>{3U, 3U, 64U, 0U}));
  }
  // IN 槽 256 字节：DAP 包长随之为 256，256 字节的响应读满即止，不带零长度包 / 256-byte
  // IN slots: the DAP packet becomes 256 and a 256-byte response fills the read, no
  // zero-length packet
  {
    V2Bench b(Speed::FULL, USBSpec::USB_2_0, 512U);
    Enumerate(b.dev, b.hw);
    b.Send({INFO, PACKET_SIZE_ID});
    b.Send({CONNECT, SWD});
    b.Send(Block(63U));
    b.DrainIn();
    TEST_ASSERT(Responses(b.hw.ep1_in)[0] == (Bytes{INFO, 2U, 0x00U, 0x01U}));
    TEST_ASSERT(SentLengths(b.hw.ep1_in) == (std::vector<size_t>{4U, 2U, 256U}));
  }
  {
    V2Bench b(Speed::HIGH);
    Enumerate(b.dev, b.hw, Speed::HIGH);
    b.Send({CONNECT, SWD});
    b.Send(Block(127U));
    b.Send(Block(255U));
    b.Send(Block(15U));
    b.DrainIn();
    TEST_ASSERT(SentLengths(b.hw.ep1_in) ==
                (std::vector<size_t>{2U, 512U, 0U, 1024U, 64U}));
  }
}

/**
 * @brief 读回 CHECK_ERR、Configure 失败、两个 DAP 同一组端点、端点没登记 / Read-back
 *        CHECK_ERR, a failed Configure, two DAPs on the same endpoints, an unregistered
 *        endpoint
 */
void TestV2BindFailure()
{
  auto works = [](V2Bench& b)
  {
    TEST_ASSERT(b.hw.ep2_out.inflight);
    b.Send({INFO, PACKET_COUNT_ID});
    TEST_ASSERT(b.hw.ep1_in.InFlight() == (Bytes{INFO, 1U, 0x04U}));
    InDone(b.dev, b.hw.ep1_in);
  };

  // IN 的 Configure 失败：OUT 已配置，被关闭交回，不挂接收；修好后同一配置可用。
  // IN's Configure fails: OUT was configured, it is closed and returned and nothing is
  // armed; fixed, the same configuration works.
  {
    V2Bench b;
    b.hw.ep1_in.configure_result = ErrorCode::FAILED;
    b.dev.Init(false);
    b.dev.Reset();
    Refused(b.dev, b.hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(b.hw.ep2_out.configures == 1U && b.hw.ep2_out.closes == 1U);
    TEST_ASSERT(b.hw.ep1_in.configures == 0U && b.hw.ep1_in.closes == 0U);
    TEST_ASSERT(b.hw.ep2_out.starts.empty() &&
                b.hw.ep2_out.GetState() == Endpoint::State::DISABLED);
    TEST_ASSERT(ControlIn(b.dev, b.hw, Make(0x80U, 8U, 0U, 0U, 1U)) == Bytes{0U});
    b.hw.ep1_in.configure_result = ErrorCode::OK;
    SetConfiguration(b.dev, b.hw, 1U);
    works(b);
  }

  // 读回的包长与描述符不同：CHECK_ERR / The size read back differs: CHECK_ERR
  {
    V2Bench b;
    b.dev.Init(false);
    b.dev.Reset();
    b.hw.ep1_in.shrink_after = b.hw.ep1_in.limit_calls + 1;
    Refused(b.dev, b.hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(b.hw.ep1_in.MaxPacketSize() == 32U && b.hw.ep1_in.closes == 1U);
    TEST_ASSERT(b.hw.ep2_out.closes == 1U && b.hw.ep2_out.starts.empty());
    b.hw.ep1_in.shrink_after = -1;
    SetConfiguration(b.dev, b.hw, 1U);
    works(b);
  }

  // 两个 DAP 用同一组端点：第二个 Get 失败，两个都解绑 / Two DAPs on the same endpoints:
  // the second Get fails and both are unbound
  {
    Hw hw;
    FakeSwd swd;
    Dap2 first(EPNumber::EP1, EPNumber::EP2, swd);
    Dap2 second(EPNumber::EP1, EPNumber::EP2, swd);
    FakeUsb dev(hw, {{&first, &second}});
    dev.Init(false);
    dev.Reset();
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(hw.ep1_in.closes == 1U && hw.ep2_out.closes == 1U);
    // 第一个的接收已排队、还没启动（启动在本轮硬件事件之后）/ The first one's receive
    // was queued and not started yet (starting comes after this pass's hardware events)
    TEST_ASSERT(hw.ep2_out.starts.empty() && hw.ep2_out.stops == 0U);
    TEST_ASSERT(swd.callers.empty());
  }

  // IN 端点没登记：写描述符失败，GET_DESCRIPTOR 和 SET_CONFIGURATION 都被拒 / IN not
  // registered: writing the descriptors fails; both requests are refused
  {
    Hw hw;
    FakeSwd swd;
    Dap2 dap(EPNumber::EP9, EPNumber::EP2, swd);
    FakeUsb dev(hw, {{&dap}});
    dev.Init(false);
    dev.Reset();
    Refused(dev, hw, Make(0x80U, 6U, 0x0200U, 0U, 255U));
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(hw.ep2_out.configures == 0U);
  }
}

/**
 * 带在途请求解绑：IN 一个在途一个排队，四个暂存响应，命令池满，两个 OUT 槽留在类里，
 * Step 拿着一条等响应缓冲的命令。四种解绑：配置 0、复位、同一配置再设、Deinit。之后旧会话
 * 的东西一样不发，SWD 由 Step 关闭，两个 OUT 槽、两个 IN 槽、四个暂存槽都能用。
 * Unbind with requests in flight: one IN in flight and one queued, four staged
 * responses, the command pool full, both OUT slots held by the class, and Step holding a
 * command that waits for a response buffer. Four ways to unbind: configuration 0, reset,
 * the same configuration again, Deinit. Afterwards nothing of the old session is sent,
 * Step closes SWD, and both OUT slots, both IN slots and the four staging slots work.
 */
void TestV2UnbindInFlight()
{
  for (int how = 0; how < 4; ++how)
  {
    V2Bench b;
    Enumerate(b.dev, b.hw);
    TEST_ASSERT(b.dap.IsInited());
    for (int i = 0; i < 6; ++i)
    {
      b.Send({INFO, PACKET_SIZE_ID});
    }
    TEST_ASSERT(b.hw.ep1_in.starts.size() == 1U && b.hw.ep1_in.inflight);
    for (int i = 0; i < 6; ++i)
    {
      HostOut(b.dev, b.hw.ep2_out, {SWJ_SEQUENCE, 8U, 0xFFU});
    }
    // 4 条进命令池、2 个包留住：OUT 没有接收了 / Four in the command pool and two
    // packets held: OUT has no receive left
    TEST_ASSERT(!b.hw.ep2_out.inflight);
    TEST_ASSERT(b.dap.Step(0) == ErrorCode::TIMEOUT);
    TEST_ASSERT(b.swd.sequences == 0U);
    const size_t out_starts = b.hw.ep2_out.starts.size();
    const unsigned closes = b.swd.closes;

    switch (how)
    {
      case 0:
        SetConfiguration(b.dev, b.hw, 0U);
        break;
      case 1:
        b.dev.Reset();
        break;
      case 2:
        SetConfiguration(b.dev, b.hw, 1U);
        break;
      default:
        b.dev.Deinit(false);
        break;
    }
    TEST_ASSERT(b.hw.ep1_in.stops == 1U && b.hw.ep1_in.closes == 1U);
    TEST_ASSERT(b.hw.ep2_out.stops == 0U && b.hw.ep2_out.closes == 1U);
    TEST_ASSERT(b.hw.ep1_in.transfers.empty() && b.swd.closes == closes);
    // 只有重新配置（how 2）之后端点还绑着 / The endpoints are bound again only after
    // the reconfiguration (how 2)
    TEST_ASSERT(b.dap.IsInited() == (how == 2));
    // Step 丢弃拿着的命令，落实会话结束 / Step drops the command it holds and applies
    // the session end
    Run(b.dap);
    TEST_ASSERT(b.swd.sequences == 0U && b.swd.closes == closes + 1U);
    if (how != 2)
    {
      TEST_ASSERT(b.hw.ep2_out.starts.size() == out_starts);
      if (how == 3)
      {
        b.dev.Init(false);
      }
      if (how != 0)
      {
        b.dev.Reset();
        ControlNone(b.dev, b.hw, Make(0x00U, 5U, 7U, 0U, 0U));
      }
      SetConfiguration(b.dev, b.hw, 1U);
    }
    TEST_ASSERT(b.hw.ep1_in.starts.size() == 1U);

    // 两个 OUT 槽都在收：发一个包之后仍有接收在途 / Both OUT slots receive: after one
    // packet a receive is still in flight
    TEST_ASSERT(b.hw.ep2_out.inflight);
    HostOut(b.dev, b.hw.ep2_out, {INFO, PACKET_COUNT_ID});
    TEST_ASSERT(b.hw.ep2_out.inflight);
    // 六条新命令：两个直接、四个暂存，按序全部发出 / Six new commands: two direct and
    // four staged, all sent in order
    for (int i = 0; i < 5; ++i)
    {
      b.Send({INFO, PACKET_SIZE_ID});
    }
    b.DrainIn();
    const auto responses = Responses(b.hw.ep1_in);
    TEST_ASSERT(responses.size() == 6U && responses[0].size() == 3U);
    for (size_t i = 1; i < responses.size(); ++i)
    {
      TEST_ASSERT(responses[i] == (Bytes{INFO, 2U, 0x00U, 0x04U}));
    }
    BothInSlotsFree(b.hw.ep1_in);
    TEST_ASSERT(b.swd.sequences == 0U);
  }

  // 旧会话的暂存响应：命令执行中途重新绑定，响应带旧 epoch 暂存，owner 丢弃它 / A staged
  // response of the old session: rebound while the command runs, the response is staged
  // with the old epoch and the owner drops it
  {
    V2Bench b;
    Enumerate(b.dev, b.hw);
    b.Send({INFO, PACKET_SIZE_ID});
    b.Send({INFO, PACKET_SIZE_ID});
    b.swd.on_sequence = [&b]() { SetConfiguration(b.dev, b.hw, 1U); };
    b.Send({SWJ_SEQUENCE, 8U, 0xFFU});
    TEST_ASSERT(b.swd.sequences == 1U && !b.swd.on_sequence);
    TEST_ASSERT(b.hw.ep1_in.starts.size() == 1U && !b.hw.ep1_in.inflight);
    for (int i = 0; i < 4; ++i)
    {
      b.Send({INFO, i % 2 == 0 ? PACKET_COUNT_ID : PACKET_SIZE_ID});
    }
    b.DrainIn();
    const auto responses = Responses(b.hw.ep1_in);
    TEST_ASSERT(responses.size() == 4U);
    for (size_t i = 0; i < 4U; ++i)
    {
      TEST_ASSERT(responses[i].size() == (i % 2U == 0U ? 3U : 4U));
    }
  }

  // Step 在延时里拿着 IN 槽：重新绑定（单条或批处理）则响应作废、槽交回；或者延时期间
  // 收到下一条命令，之后按序应答 / Step holds an IN slot inside a delay: a rebind (single
  // command or batch) voids the response and gives the slot back; or the next command
  // arrives during the delay and is answered after it, in order
  for (int variant = 0; variant < 3; ++variant)
  {
    V2Bench b;
    Enumerate(b.dev, b.hw);
    const Bytes delay =
        (variant == 2) ? Bytes{EXECUTE, 2U, DELAY, 0xFFU, 0xFFU, INFO, PACKET_SIZE_ID}
                       : Bytes{DELAY, 0xFFU, 0xFFU};
    HostOut(b.dev, b.hw.ep2_out, delay);
    std::atomic<bool> returned{false};
    std::thread step(
        [&]
        {
          (void)b.dap.Step(1000);
          returned.store(true);
        });
    TEST_ASSERT(WaitUntil([&] { return b.swd.clock_calls.load() != 0U; }));
    if (variant == 1)
    {
      HostOut(b.dev, b.hw.ep2_out, {INFO, PACKET_SIZE_ID});
    }
    else
    {
      SetConfiguration(b.dev, b.hw, 1U);
    }
    TEST_ASSERT(!returned.load());
    step.join();
    Run(b.dap);
    if (variant == 1)
    {
      b.DrainIn();
      const auto responses = Responses(b.hw.ep1_in);
      TEST_ASSERT(responses.size() == 2U && responses[0] == (Bytes{DELAY, 0x00U}));
      TEST_ASSERT(responses[1][0] == INFO);
      continue;
    }
    TEST_ASSERT(b.hw.ep1_in.starts.empty());
    BothInSlotsFree(b.hw.ep1_in);
    b.Send({DELAY, 0x10U, 0x00U});
    TEST_ASSERT(b.hw.ep1_in.InFlight() == (Bytes{DELAY, 0x00U}));
    InDone(b.dev, b.hw.ep1_in);
    TEST_ASSERT(b.hw.ep2_out.inflight);
  }

  // 多个响应暂存且 owner 正好在解绑期间排空 resp_pool_ 时，staged_outstanding_ 只能减
  // 去实际清空的槽数，不可因 store(0) 导致 Step  THREAD 稍后归还时无符号下溢 /
  // ResetChannels must subtract only the drained staged count instead of storing 0
  {
    V2Bench b;
    Enumerate(b.dev, b.hw);
    b.Send({INFO, PACKET_SIZE_ID});
    b.Send({INFO, PACKET_SIZE_ID});
    HostOut(b.dev, b.hw.ep2_out, {DELAY, 0xFFU, 0xFFU});
    std::atomic<bool> returned{false};
    std::thread step(
        [&]
        {
          (void)b.dap.Step(1000);
          returned.store(true);
        });
    TEST_ASSERT(WaitUntil([&] { return b.swd.clock_calls.load() != 0U; }));
    SetConfiguration(b.dev, b.hw, 1U);
    step.join();
    Run(b.dap);
    BothInSlotsFree(b.hw.ep1_in);
    b.Send({INFO, PACKET_COUNT_ID});
    TEST_ASSERT(b.hw.ep1_in.InFlight() == (Bytes{INFO, 1U, 0x04U}));
    InDone(b.dev, b.hw.ep1_in);
  }

  // SWD 只归 Step 线程：绑定和解绑不调用 SWD / SWD belongs to the Step thread: bind and
  // unbind make no SWD call
  {
    V2Bench b;
    Enumerate(b.dev, b.hw);
    SetConfiguration(b.dev, b.hw, 0U);
    SetConfiguration(b.dev, b.hw, 1U);
    TEST_ASSERT(b.swd.callers.empty());
    HostOut(b.dev, b.hw.ep2_out, {CONNECT, SWD});
    std::thread first([&] { Run(b.dap); });
    const std::thread::id first_id = first.get_id();
    first.join();
    TEST_ASSERT(b.hw.ep1_in.InFlight() == (Bytes{CONNECT, SWD}));
    // 响应由 Step 线程提交，它的 Submit 就地运行 owner 启动了 IN / Step submitted the
    // response, and its Submit ran the owner in place and started IN
    TEST_ASSERT(b.hw.ep1_in.start_threads.back() == first_id);
    TEST_ASSERT(b.swd.enters == 1U && b.swd.closes == 1U);
    SetConfiguration(b.dev, b.hw, 0U);
    TEST_ASSERT(b.swd.closes == 1U);
    std::thread second([&] { Run(b.dap); });
    second.join();
    TEST_ASSERT(b.swd.closes == 2U);
    for (const auto& id : b.swd.callers)
    {
      TEST_ASSERT(id != std::this_thread::get_id());
    }
  }
}

/// 一条命令和它的预期响应 / One command and its expected response
struct Exchange
{
  Bytes command;
  uint8_t first;  ///< 响应的第一个字节 / First byte of the response
  size_t length;  ///< 响应长度 / Response length
};

/**
 * @brief 线程用例的命令表：长度覆盖短包、64 与 128（带零长度包）和 1024（DAP 包长）/ The
 *        command list of the thread cases: lengths cover short responses, 64 and 128
 *        (with a zero-length packet) and 1024 (the DAP packet)
 */
std::vector<Exchange> Script(size_t count)
{
  std::vector<Exchange> script;
  script.push_back({{CONNECT, SWD}, CONNECT, 2U});
  const Exchange cycle[] = {
      {{INFO, PACKET_COUNT_ID}, INFO, 3U},  {{INFO, VENDOR_ID}, INFO, 9U},
      {Block(15U), TRANSFER_BLOCK, 64U},    {Block(31U), TRANSFER_BLOCK, 128U},
      {Block(255U), TRANSFER_BLOCK, 1024U}, {Block(10U), TRANSFER_BLOCK, 44U},
      {{DELAY, 0x01U, 0x00U}, DELAY, 2U}};
  for (size_t i = 0; i < count; ++i)
  {
    script.push_back(cycle[(i * 5U) % (sizeof(cycle) / sizeof(cycle[0]))]);
  }
  return script;
}

/**
 * @brief 主机收到的 IN 请求与命令表逐条相符，零长度包恰好跟在短于 1024 的 64 整数倍之后 /
 *        The IN requests the host got match the script one by one, and a zero-length
 *        packet follows exactly the multiples of 64 below 1024
 */
void CheckStream(const std::vector<Bytes>& sent, const std::vector<Exchange>& script)
{
  size_t at = 0;
  for (const Exchange& exchange : script)
  {
    TEST_ASSERT(at < sent.size());
    TEST_ASSERT(sent[at].size() == exchange.length && sent[at][0] == exchange.first);
    ++at;
    if (exchange.length % 64U == 0U && exchange.length < 1024U)
    {
      TEST_ASSERT(at < sent.size() && sent[at].empty());
      ++at;
    }
  }
  TEST_ASSERT(at == sent.size());
}

/// 命令表在主机一侧会产生的 IN 请求数 / Number of IN requests the script produces
size_t StreamLength(const std::vector<Exchange>& script)
{
  size_t count = 0;
  for (const Exchange& exchange : script)
  {
    count += (exchange.length % 64U == 0U && exchange.length < 1024U) ? 2U : 1U;
  }
  return count;
}

/**
 * Step 在自己的线程里执行并直接提交响应，另一个线程扮控制器中断：完成 IN、送下一条命令。
 * 主机收到的响应与命令一一对应、按序，零长度包只在该有的地方。第二遍 IN 在 StartHardware
 * 内完成，完成回调和暂存响应的提交在 Step 线程里就地运行。
 * Step runs in its own thread, executing and submitting responses directly, and another
 * thread plays the controller interrupt: it completes IN requests and sends the next
 * command. The host gets one response per command, in order, with zero-length packets
 * exactly where they belong. In the second pass IN completes inside StartHardware, so
 * the completion callback and the staged submissions run in place in the Step thread.
 */
void TestV2Threads()
{
  for (int variant = 0; variant < 2; ++variant)
  {
    V2Bench b;
    Enumerate(b.dev, b.hw);
    b.hw.ep1_in.complete_in_start = (variant == 1);
    const std::vector<Exchange> script = Script(300U);
    const size_t total = StreamLength(script);
    size_t next = 0;  // 只在 owner 中 / Owner only
    std::atomic<bool> stop{false};

    auto irq = [&](bool isr)
    {
      if (b.hw.ep1_in.inflight)
      {
        b.hw.ep1_in.Done(isr, b.hw.ep1_in.LastLength());
      }
      if (b.hw.ep2_out.inflight && next < script.size())
      {
        b.hw.ep2_out.HostSends(isr, script[next++].command);
      }
    };
    std::thread step(
        [&]
        {
          while (!stop.load(std::memory_order_acquire))
          {
            (void)b.dap.Step(1);
          }
        });
    const std::thread::id step_id = step.get_id();
    std::thread interrupt(
        [&]
        {
          while (!stop.load(std::memory_order_acquire))
          {
            b.dev.Irq(irq);
            std::this_thread::sleep_for(std::chrono::microseconds(20));
          }
        });
    TEST_ASSERT(
        WaitUntil([&] { return b.hw.ep1_in.transfer_count.load() >= total; }, 120));
    stop.store(true, std::memory_order_release);
    step.join();
    interrupt.join();

    CheckStream(b.hw.ep1_in.transfers, script);
    TEST_ASSERT(b.hw.ep2_out.inflight && !b.hw.ep1_in.inflight);
    for (const auto& id : b.swd.callers)
    {
      TEST_ASSERT(id == step_id);
    }
  }
}

/**
 * 三个线程同时：Step 执行命令，中断线程收发并每 37 轮在 owner 里 SET_CONFIGURATION 一次
 * （带在途请求解绑再绑定），共 150 次。之后单线程检查：只有合法的响应，两个 OUT 槽、两个
 * IN 槽都在，新命令照常应答。
 * Three threads at once: Step executes commands, and the interrupt thread moves data and
 * every 37 rounds issues SET_CONFIGURATION inside the owner (unbind and bind with
 * requests in flight), 150 times. Then, single-threaded: only well-formed responses were
 * sent, both OUT and both IN slots are there, and a new command is answered as usual.
 */
void TestV2RebindRace()
{
  V2Bench b;
  Enumerate(b.dev, b.hw);
  const Bytes commands[] = {{INFO, PACKET_COUNT_ID}, {INFO, VENDOR_ID}, {DELAY, 1U, 0U}};
  size_t steps = 0;  // 只在 owner 中 / Owner only
  size_t rebinds = 0;
  std::atomic<bool> rebinds_done{false};
  std::atomic<bool> stop{false};

  auto irq = [&](bool isr)
  {
    ++steps;
    if (b.hw.in0.inflight)
    {
      b.hw.in0.Done(isr, 0U);
    }
    if (b.hw.ep1_in.inflight)
    {
      b.hw.ep1_in.Done(isr, b.hw.ep1_in.LastLength());
    }
    if (b.hw.ep2_out.inflight)
    {
      b.hw.ep2_out.HostSends(isr, commands[steps % 3U]);
    }
    if (steps % 37U == 0U && rebinds < 150U)
    {
      b.dev.HwSetup(isr, Make(0x00U, 9U, 1U, 0U, 0U));
      if (++rebinds == 150U)
      {
        rebinds_done.store(true, std::memory_order_release);
      }
    }
  };
  std::thread step(
      [&]
      {
        while (!stop.load(std::memory_order_acquire))
        {
          (void)b.dap.Step(1);
        }
      });
  std::thread interrupt(
      [&]
      {
        while (!stop.load(std::memory_order_acquire))
        {
          b.dev.Irq(irq);
          std::this_thread::sleep_for(std::chrono::microseconds(20));
        }
      });
  TEST_ASSERT(WaitUntil([&] { return rebinds_done.load(); }, 120));
  stop.store(true, std::memory_order_release);
  step.join();
  interrupt.join();

  // 收尾：完成 EP0 状态阶段和在途 IN，让 Step 处理剩下的 / Wind down: finish the EP0
  // status stage and the IN in flight and let Step handle the rest
  if (b.hw.in0.inflight)
  {
    InDone(b.dev, b.hw.in0);
  }
  Run(b.dap);
  b.DrainIn();
  for (const Bytes& response : Responses(b.hw.ep1_in))
  {
    TEST_ASSERT(
        (response[0] == INFO && (response.size() == 3U || response.size() == 9U)) ||
        (response[0] == DELAY && response.size() == 2U));
  }
  TEST_ASSERT(Responses(b.hw.ep1_in).size() > 100U);
  BothInSlotsFree(b.hw.ep1_in);
  // 两个 OUT 槽都在收 / Both OUT slots receive
  TEST_ASSERT(b.hw.ep2_out.inflight);
  HostOut(b.dev, b.hw.ep2_out, {INFO, PACKET_COUNT_ID});
  TEST_ASSERT(b.hw.ep2_out.inflight);
  Run(b.dap);
  TEST_ASSERT(b.hw.ep1_in.InFlight() == (Bytes{INFO, 1U, 0x04U}));
}

// ---- DAP v1 ----

using Dap1 = DapLinkV1Class<FakeSwd>;

/// 以给定字节开头、其余为零的 64 字节报告 / A 64-byte report starting with the given
/// bytes, the rest zero
Bytes Report(std::initializer_list<uint8_t> prefix)
{
  Bytes report(64U, 0U);
  std::copy(prefix.begin(), prefix.end(), report.begin());
  return report;
}

/// 一个 DAP v1 的设备 / A device with one DAP v1
struct V1Bench
{
  explicit V1Bench(Speed speed = Speed::FULL, uint8_t slots = 0U)
      : hw(2048U, slots), dap(EPNumber::EP1, EPNumber::EP2, swd), dev(hw, {{&dap}}, speed)
  {
  }

  /// 主机发一个报告，Step 执行到没有工作 / The host sends a report; Step runs until idle
  void Send(const Bytes& report)
  {
    HostOut(dev, hw.ep2_out, report);
    Run(dap);
  }

  /// 主机取走 IN 上全部报告，每次之后 Step 执行 / The host takes every IN report, with
  /// Step running after each
  void DrainIn()
  {
    for (size_t guard = 0; hw.ep1_in.inflight; ++guard)
    {
      TEST_ASSERT(guard < 10000U);
      InDone(dev, hw.ep1_in);
      Run(dap);
    }
  }

  Hw hw;
  FakeSwd swd;
  Dap1 dap;
  FakeUsb dev;
};

/// 响应是 [INFO, n, 字符串 + NUL] 且其余为零 / The response is [INFO, n, string + NUL]
/// and zero after it
bool IsInfoString(const Bytes& report, const char* text)
{
  const size_t n = std::strlen(text) + 1U;
  return report.size() == 64U && report[0] == INFO && report[1] == n &&
         std::memcmp(&report[2], text, n) == 0 &&
         std::all_of(report.begin() + 2 + static_cast<std::ptrdiff_t>(n), report.end(),
                     [](uint8_t byte) { return byte == 0U; });
}

/// HID 描述符、报告描述符、HID 类请求、设备类字段、与 v2 复合 / HID descriptors, the
/// report descriptor, HID class requests, the device class fields, composite with v2
void TestV1Enumeration()
{
  {
    V1Bench b(Speed::HIGH);
    b.dev.Init(false);
    b.dev.Reset(Speed::FULL);
    TEST_ASSERT(!b.dap.IsInited());
    // 接口级识别：设备类为 PER_INTERFACE，HID 类在接口描述符上 / Interface-level class:
    // device class is PER_INTERFACE, HID class sits on the interface descriptor
    const Bytes device = GetDescriptor(b.dev, b.hw, 1U, 0U, 18U);
    TEST_ASSERT(device.size() == 18U && device[4] == 0x00U);

    // 配置头、接口、HID、IN、OUT，共 41 字节 / Header, interface, HID, IN, OUT: 41
    const Bytes fs = GetDescriptor(b.dev, b.hw, 2U, 0U, 255U);
    TEST_ASSERT(fs.size() == 41U && fs[2] == 41U && fs[4] == 1U);
    TEST_ASSERT(fs[9] == 9U && fs[13] == 2U && fs[14] == 0x03U && fs[17] == 4U);
    TEST_ASSERT(fs[18] == 9U && fs[19] == 0x21U && fs[20] == 0x11U && fs[21] == 0x01U);
    TEST_ASSERT(fs[23] == 1U && fs[24] == 0x22U && fs[25] == 33U && fs[26] == 0U);
    TEST_ASSERT(fs[29] == 0x81U && fs[30] == 3U && fs[31] == 64U && fs[33] == 1U);
    TEST_ASSERT(fs[36] == 0x02U && fs[37] == 3U && fs[38] == 64U && fs[40] == 1U);
    // 高速也是 64 字节的报告 / High speed keeps 64-byte reports
    const Bytes hs = GetDescriptor(b.dev, b.hw, 7U, 0U, 255U);
    TEST_ASSERT(hs.size() == 41U && hs[31] == 64U && hs[38] == 64U);
    const Bytes text = GetDescriptor(b.dev, b.hw, 3U, 4U, 255U, 0x0409U);
    TEST_ASSERT(text.size() == 2U + 2U * 9U && text[2] == 'C');

    // 未配置时接口请求找不到类 / Unconfigured, an interface request finds no class
    Refused(b.dev, b.hw, Make(0x81U, 6U, 0x2200U, 0U, 255U));
    ControlNone(b.dev, b.hw, Make(0x00U, 5U, 7U, 0U, 0U));
    SetConfiguration(b.dev, b.hw, 1U);
    TEST_ASSERT(b.hw.ep2_out.inflight && b.hw.ep2_out.LastLength() == 64U);
    TEST_ASSERT(b.hw.ep2_out.starts.size() == 1U);

    const Bytes report = ControlIn(b.dev, b.hw, Make(0x81U, 6U, 0x2200U, 0U, 255U));
    TEST_ASSERT(report.size() == 33U && report[0] == 0x06U && report[32] == 0xC0U);
    const Bytes hid = ControlIn(b.dev, b.hw, Make(0x81U, 6U, 0x2100U, 0U, 255U));
    TEST_ASSERT(hid == Bytes(fs.begin() + 18, fs.begin() + 27));
    Refused(b.dev, b.hw, Make(0x81U, 6U, 0x2300U, 0U, 255U));

    // SET_IDLE / GET_IDLE；报告和协议请求被拒 / SET_IDLE and GET_IDLE; report and
    // protocol requests are refused
    TEST_ASSERT(ControlIn(b.dev, b.hw, Make(0xA1U, 0x02U, 0U, 0U, 1U)) == Bytes{0U});
    ControlNone(b.dev, b.hw, Make(0x21U, 0x0AU, 0x0400U, 0U, 0U));
    TEST_ASSERT(ControlIn(b.dev, b.hw, Make(0xA1U, 0x02U, 0U, 0U, 1U)) == Bytes{4U});
    Refused(b.dev, b.hw, Make(0x21U, 0x0AU, 0x0401U, 0U, 0U));
    Refused(b.dev, b.hw, Make(0xA1U, 0x02U, 0U, 0U, 2U));
    Refused(b.dev, b.hw, Make(0xA1U, 0x01U, 0x0100U, 0U, 64U));
    Refused(b.dev, b.hw, Make(0x21U, 0x09U, 0x0200U, 0U, 64U));
    Refused(b.dev, b.hw, Make(0x21U, 0x0BU, 0U, 0U, 0U));
    Refused(b.dev, b.hw, Make(0xA1U, 0x03U, 0U, 0U, 1U));
    // 重新绑定后空闲率归零 / A new bind resets the idle rate
    SetConfiguration(b.dev, b.hw, 1U);
    TEST_ASSERT(ControlIn(b.dev, b.hw, Make(0xA1U, 0x02U, 0U, 0U, 1U)) == Bytes{0U});
    TEST_ASSERT(b.swd.callers.empty());
  }

  // 与 v2 组成复合设备：设备类不改，接口 0 是 HID、接口 1 是厂商 / Composite with v2:
  // the device class is left alone; interface 0 is HID, interface 1 vendor
  {
    Hw hw;
    FakeSwd swd_a;
    FakeSwd swd_b;
    Dap1 a(EPNumber::EP1, EPNumber::EP2, swd_a);
    Dap2 b(EPNumber::EP3, EPNumber::EP4, swd_b);
    FakeUsb dev(hw, {{&a, &b}});
    dev.Init(false);
    dev.Reset();
    TEST_ASSERT(GetDescriptor(dev, hw, 1U, 0U, 18U)[4] == 0xEFU);
    const Bytes config = GetDescriptor(dev, hw, 2U, 0U, 255U);
    TEST_ASSERT(config.size() == 9U + 32U + 23U && config[4] == 2U);
    TEST_ASSERT(config[14] == 0x03U && config[43] == 1U && config[46] == 0xFFU);
    Enumerate(dev, hw);
    TEST_ASSERT(hw.ep2_out.inflight && hw.ep4_out.inflight);
    const Bytes report = ControlIn(dev, hw, Make(0x81U, 6U, 0x2200U, 0U, 255U));
    TEST_ASSERT(report.size() == 33U);
    Refused(dev, hw, Make(0x81U, 6U, 0x2200U, 1U, 255U));
  }
}

/**
 * @brief 往返、背压、IN 槽用完、两报告按序 / Round trip, backpressure, IN slots used
 *        up and two reports in order
 */
void TestV1Commands()
{
  // 往返：响应是 64 字节报告，提交后 OUT 槽交回并补挂 / Round trip: the response is a
  // 64-byte report; once it is submitted the OUT slot goes back and is armed again
  {
    V1Bench b;
    Enumerate(b.dev, b.hw);
    Run(b.dap);
    TEST_ASSERT(b.hw.ep1_in.starts.empty());
    HostOut(b.dev, b.hw.ep2_out, Report({INFO, VENDOR_ID}));
    // 另一个槽开始接收；第一个槽借给 Step / The other slot starts receiving; the first
    // one is lent to Step
    TEST_ASSERT(b.hw.ep2_out.inflight && b.hw.ep2_out.starts.size() == 2U);
    Run(b.dap);
    TEST_ASSERT(IsInfoString(b.hw.ep1_in.InFlight(), "XRobot"));
    // 交回的槽已排在在途接收之后 / The returned slot is queued behind the receive in
    // flight
    HostOut(b.dev, b.hw.ep2_out, Report({INFO, PRODUCT_ID}));
    TEST_ASSERT(b.hw.ep2_out.inflight && b.hw.ep2_out.starts.size() == 3U);
    Run(b.dap);
    b.DrainIn();
    TEST_ASSERT(b.hw.ep1_in.transfers.size() == 2U &&
                IsInfoString(b.hw.ep1_in.transfers[1], "DAPLinkV1"));
  }

  // 背压：Step 没跑时两个报告占住两个 OUT 槽，主机被 NAK；Step 跑完后恢复接收 /
  // Backpressure: with Step not running two reports hold both OUT slots and the host is
  // NAKed; receiving resumes once Step ran
  {
    V1Bench b;
    Enumerate(b.dev, b.hw);
    HostOut(b.dev, b.hw.ep2_out, Report({INFO, VENDOR_ID}));
    HostOut(b.dev, b.hw.ep2_out, Report({INFO, PRODUCT_ID}));
    TEST_ASSERT(!b.hw.ep2_out.inflight && b.hw.ep1_in.starts.empty());
    Run(b.dap);
    TEST_ASSERT(b.hw.ep2_out.inflight && b.hw.ep1_in.starts.size() == 1U);
    b.DrainIn();
    TEST_ASSERT(IsInfoString(b.hw.ep1_in.transfers[0], "XRobot") &&
                IsInfoString(b.hw.ep1_in.transfers[1], "DAPLinkV1"));
  }

  // 连接、延时、未知命令 / Connect, delay, unknown command
  {
    V1Bench b;
    Enumerate(b.dev, b.hw);
    b.Send(Report({CONNECT, SWD}));
    TEST_ASSERT(b.hw.ep1_in.InFlight()[0] == CONNECT && b.hw.ep1_in.InFlight()[1] == SWD);
    TEST_ASSERT(b.swd.enters == 1U && b.swd.clock_hz == 1000000U);
    InDone(b.dev, b.hw.ep1_in);
    b.Send(Report({DELAY, 0x64U, 0x00U}));
    TEST_ASSERT(b.hw.ep1_in.InFlight()[0] == DELAY && b.hw.ep1_in.InFlight()[1] == 0U);
    InDone(b.dev, b.hw.ep1_in);
    b.Send(Report({0x42U}));
    TEST_ASSERT(b.hw.ep1_in.InFlight()[0] == 0xFFU);
    InDone(b.dev, b.hw.ep1_in);
    TEST_ASSERT(b.hw.ep2_out.inflight);
  }

  // IN 槽用完：第三个响应在 Step 里等，主机取走一个后下一次 Step 提交 / IN slots used
  // up: the third response waits in Step, and the next Step submits it once the host
  // took one
  {
    V1Bench b;
    Enumerate(b.dev, b.hw);
    const Bytes info = Report({INFO, PRODUCT_ID});
    b.Send(info);
    b.Send(info);
    TEST_ASSERT(b.hw.ep1_in.starts.size() == 1U);
    HostOut(b.dev, b.hw.ep2_out, info);
    TEST_ASSERT(b.dap.Step(0) == ErrorCode::TIMEOUT);
    TEST_ASSERT(b.dap.Step(0) == ErrorCode::TIMEOUT);
    TEST_ASSERT(b.hw.ep1_in.starts.size() == 1U);
    // Step 拿着一个 OUT 槽，另一个在收 / Step holds one OUT slot, the other receives
    TEST_ASSERT(b.hw.ep2_out.inflight);
    InDone(b.dev, b.hw.ep1_in);
    TEST_ASSERT(b.hw.ep1_in.starts.size() == 2U);
    Run(b.dap);
    b.DrainIn();
    TEST_ASSERT(b.hw.ep1_in.transfers.size() == 3U && b.hw.ep2_out.inflight);
  }

  // HALT 之后连发两个报告：两条都按到达顺序应答 / Two reports back to back after a
  // HALT: both answered in arrival order
  {
    V1Bench b;
    Enumerate(b.dev, b.hw);
    SetHalt(b.dev, b.hw, 0x02U);
    TEST_ASSERT(!b.hw.ep2_out.inflight);
    ClearHalt(b.dev, b.hw, 0x02U);
    HostOut(b.dev, b.hw.ep2_out, Report({INFO, VENDOR_ID}));
    HostOut(b.dev, b.hw.ep2_out, Report({INFO, PRODUCT_ID}));
    Run(b.dap);
    b.DrainIn();
    TEST_ASSERT(b.hw.ep1_in.transfers.size() == 2U);
    TEST_ASSERT(IsInfoString(b.hw.ep1_in.transfers[0], "XRobot") &&
                IsInfoString(b.hw.ep1_in.transfers[1], "DAPLinkV1"));
    TEST_ASSERT(b.hw.ep2_out.inflight);
    HostOut(b.dev, b.hw.ep2_out, Report({INFO, VENDOR_ID}));
    TEST_ASSERT(b.hw.ep2_out.inflight);
  }

  // 未绑定时 Step 没有工作，也不碰端点 / Unbound, Step has no work and touches no
  // endpoint
  {
    V1Bench b;
    TEST_ASSERT(b.dap.Step(0) == ErrorCode::TIMEOUT);
    TEST_ASSERT(b.hw.ep1_in.starts.empty() && b.swd.callers.empty());
  }
}

/// IN Configure 失败、读回 CHECK_ERR、包长不是 64、两个 v1 同一组端点 / IN Configure
/// fails, read-back CHECK_ERR, a packet that is not 64, two v1 on the same endpoints
void TestV1BindFailure()
{
  {
    V1Bench b;
    b.hw.ep1_in.configure_result = ErrorCode::FAILED;
    b.dev.Init(false);
    b.dev.Reset();
    Refused(b.dev, b.hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(b.hw.ep1_in.configures == 0U && b.hw.ep2_out.configures == 0U);
    TEST_ASSERT(b.hw.ep1_in.closes == 0U && b.hw.ep2_out.starts.empty());
    b.hw.ep1_in.configure_result = ErrorCode::OK;
    SetConfiguration(b.dev, b.hw, 1U);
    b.Send(Report({INFO, VENDOR_ID}));
    TEST_ASSERT(IsInfoString(b.hw.ep1_in.InFlight(), "XRobot"));
  }
  {
    V1Bench b;
    b.dev.Init(false);
    b.dev.Reset();
    b.hw.ep2_out.shrink_after = b.hw.ep2_out.limit_calls + 1;
    Refused(b.dev, b.hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(b.hw.ep2_out.MaxPacketSize() == 32U && b.hw.ep2_out.closes == 1U);
    TEST_ASSERT(b.hw.ep1_in.closes == 1U && b.hw.ep2_out.starts.empty());
    b.hw.ep2_out.shrink_after = -1;
    SetConfiguration(b.dev, b.hw, 1U);
    TEST_ASSERT(b.hw.ep2_out.inflight);
  }
  // 后端只给 32 字节包：报告不是一个包，描述符写不出 / The backend gives 32-byte
  // packets only: a report would not be one packet, so no descriptor is written
  {
    V1Bench b;
    b.hw.ep1_in.shrink_after = 0;
    b.dev.Init(false);
    b.dev.Reset();
    Refused(b.dev, b.hw, Make(0x80U, 6U, 0x0200U, 0U, 255U));
    Refused(b.dev, b.hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(b.hw.ep1_in.configures == 0U);
  }
  {
    Hw hw;
    FakeSwd swd;
    Dap1 first(EPNumber::EP1, EPNumber::EP2, swd);
    Dap1 second(EPNumber::EP1, EPNumber::EP2, swd);
    FakeUsb dev(hw, {{&first, &second}});
    dev.Init(false);
    dev.Reset();
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(hw.ep1_in.closes == 1U && hw.ep2_out.closes == 1U);
    TEST_ASSERT(hw.ep2_out.starts.empty() && swd.callers.empty());
  }
}

/**
 * 带在途请求解绑：IN 一个在途一个排队，两个报告在环里（OUT 没有接收），Step 拿着一个已
 * 执行、等 IN 槽的响应。四种解绑。Step 交回作废的槽；同一配置再设时绑定只能先挂上零个
 * 接收，Step 交回之后由 OnService 补上。旧会话的响应一个不发。
 * Unbind with requests in flight: one IN in flight and one queued, two reports in the
 * ring (OUT has no receive), and Step holding an executed response that waits for an IN
 * slot. Four ways to unbind. Step hands the stale slots back; when the same
 * configuration is set again the bind can arm no receive at first, and OnService arms
 * them after Step handed the slots back. No response of the old session is sent.
 */
void TestV1UnbindInFlight()
{
  for (int how = 0; how < 4; ++how)
  {
    V1Bench b;
    Enumerate(b.dev, b.hw);
    TEST_ASSERT(b.dap.IsInited());
    b.Send(Report({INFO, VENDOR_ID}));
    b.Send(Report({INFO, VENDOR_ID}));
    HostOut(b.dev, b.hw.ep2_out, Report({CONNECT, SWD}));
    HostOut(b.dev, b.hw.ep2_out, Report({CONNECT, SWD}));
    TEST_ASSERT(!b.hw.ep2_out.inflight);
    TEST_ASSERT(b.dap.Step(0) == ErrorCode::TIMEOUT);
    TEST_ASSERT(b.swd.enters == 1U);
    const unsigned closes = b.swd.closes;
    switch (how)
    {
      case 0:
        SetConfiguration(b.dev, b.hw, 0U);
        break;
      case 1:
        b.dev.Reset();
        break;
      case 2:
        SetConfiguration(b.dev, b.hw, 1U);
        // 两个 OUT 槽都作废、仍在 Step 手里：一个接收也挂不上 / Both OUT slots are stale
        // and still with Step: not a single receive can be armed
        TEST_ASSERT(!b.hw.ep2_out.inflight);
        break;
      default:
        b.dev.Deinit(false);
        break;
    }
    TEST_ASSERT(b.hw.ep1_in.stops == 1U && b.hw.ep1_in.closes == 1U);
    TEST_ASSERT(b.hw.ep2_out.stops == 0U && b.hw.ep2_out.closes == 1U);
    TEST_ASSERT(b.dap.IsInited() == (how == 2));
    Run(b.dap);
    TEST_ASSERT(b.swd.enters == 1U && b.swd.closes == closes + 1U);
    if (how != 2)
    {
      if (how == 3)
      {
        b.dev.Init(false);
      }
      if (how != 0)
      {
        b.dev.Reset();
        ControlNone(b.dev, b.hw, Make(0x00U, 5U, 7U, 0U, 0U));
      }
      SetConfiguration(b.dev, b.hw, 1U);
    }
    TEST_ASSERT(b.hw.ep1_in.starts.size() == 1U && b.hw.ep1_in.transfers.empty());
    TEST_ASSERT(b.hw.ep2_out.inflight);
    HostOut(b.dev, b.hw.ep2_out, Report({INFO, PRODUCT_ID}));
    TEST_ASSERT(b.hw.ep2_out.inflight);
    b.Send(Report({INFO, VENDOR_ID}));
    b.DrainIn();
    TEST_ASSERT(b.hw.ep1_in.transfers.size() == 2U);
    TEST_ASSERT(IsInfoString(b.hw.ep1_in.transfers[0], "DAPLinkV1") &&
                IsInfoString(b.hw.ep1_in.transfers[1], "XRobot"));
    BothInSlotsFree(b.hw.ep1_in);
  }

  // 延时执行中重新绑定：响应不进新绑定 / Rebound while a delay executes: the response
  // does not reach the new binding
  {
    V1Bench b;
    Enumerate(b.dev, b.hw);
    HostOut(b.dev, b.hw.ep2_out, Report({DELAY, 0xFFU, 0xFFU}));
    std::atomic<bool> returned{false};
    std::thread step(
        [&]
        {
          (void)b.dap.Step(1000);
          returned.store(true);
        });
    TEST_ASSERT(WaitUntil([&] { return b.swd.clock_calls.load() != 0U; }));
    SetConfiguration(b.dev, b.hw, 1U);
    TEST_ASSERT(!returned.load());
    step.join();
    Run(b.dap);
    TEST_ASSERT(b.hw.ep1_in.starts.empty());
    b.Send(Report({INFO, VENDOR_ID}));
    TEST_ASSERT(IsInfoString(b.hw.ep1_in.InFlight(), "XRobot"));
    InDone(b.dev, b.hw.ep1_in);
    BothInSlotsFree(b.hw.ep1_in);
  }

  // SWD 只归 Step 线程 / SWD belongs to the Step thread
  {
    V1Bench b;
    Enumerate(b.dev, b.hw);
    SetConfiguration(b.dev, b.hw, 0U);
    SetConfiguration(b.dev, b.hw, 1U);
    TEST_ASSERT(b.swd.callers.empty());
    HostOut(b.dev, b.hw.ep2_out, Report({CONNECT, SWD}));
    std::thread first([&] { Run(b.dap); });
    const std::thread::id first_id = first.get_id();
    first.join();
    TEST_ASSERT(b.hw.ep1_in.InFlight()[0] == CONNECT);
    TEST_ASSERT(b.hw.ep1_in.start_threads.back() == first_id);
    TEST_ASSERT(b.swd.enters == 1U && b.swd.closes == 1U);
    SetConfiguration(b.dev, b.hw, 0U);
    std::thread second([&] { Run(b.dap); });
    second.join();
    TEST_ASSERT(b.swd.closes == 2U);
    for (const auto& id : b.swd.callers)
    {
      TEST_ASSERT(id != std::this_thread::get_id());
    }
  }
}

/**
 * Step 在自己的线程里提交，中断线程完成 IN、送报告；第二遍每 23 轮在 owner 里重新绑定。
 * 第一遍响应与报告一一对应；第二遍只检查响应都合法，之后两个 OUT 槽、两个 IN 槽都在。
 * Step submits from its own thread while the interrupt thread completes IN and sends
 * reports; in the second pass it rebinds inside the owner every 23 rounds. In the first
 * pass the responses match the reports one by one; in the second only that every
 * response is well formed, and afterwards both OUT and both IN slots are there.
 */
void TestV1Threads()
{
  for (int variant = 0; variant < 2; ++variant)
  {
    V1Bench b;
    Enumerate(b.dev, b.hw);
    const Bytes reports[] = {Report({INFO, VENDOR_ID}), Report({INFO, PRODUCT_ID}),
                             Report({DELAY, 1U, 0U})};
    constexpr size_t COUNT = 600U;
    size_t sent = 0;  // 只在 owner 中 / Owner only
    size_t steps = 0;
    size_t rebinds = 0;
    std::atomic<bool> done{false};
    std::atomic<bool> stop{false};

    auto irq = [&](bool isr)
    {
      ++steps;
      if (b.hw.in0.inflight)
      {
        b.hw.in0.Done(isr, 0U);
      }
      if (b.hw.ep1_in.inflight)
      {
        b.hw.ep1_in.Done(isr, b.hw.ep1_in.LastLength());
      }
      if (b.hw.ep2_out.inflight && sent < COUNT)
      {
        b.hw.ep2_out.HostSends(isr, reports[sent % 3U]);
        ++sent;
      }
      if (variant == 1 && steps % 23U == 0U && rebinds < 100U)
      {
        b.dev.HwSetup(isr, Make(0x00U, 9U, 1U, 0U, 0U));
        if (++rebinds == 100U)
        {
          done.store(true, std::memory_order_release);
        }
      }
    };
    std::thread step(
        [&]
        {
          while (!stop.load(std::memory_order_acquire))
          {
            (void)b.dap.Step(1);
          }
        });
    std::thread interrupt(
        [&]
        {
          while (!stop.load(std::memory_order_acquire))
          {
            b.dev.Irq(irq);
            std::this_thread::sleep_for(std::chrono::microseconds(20));
          }
        });
    if (variant == 0)
    {
      TEST_ASSERT(
          WaitUntil([&] { return b.hw.ep1_in.transfer_count.load() >= COUNT; }, 120));
    }
    else
    {
      TEST_ASSERT(WaitUntil([&] { return done.load(); }, 120));
    }
    stop.store(true, std::memory_order_release);
    step.join();
    interrupt.join();
    if (b.hw.in0.inflight)
    {
      InDone(b.dev, b.hw.in0);
    }
    Run(b.dap);
    b.DrainIn();

    if (variant == 0)
    {
      TEST_ASSERT(b.hw.ep1_in.transfers.size() == COUNT);
      for (size_t i = 0; i < COUNT; ++i)
      {
        const Bytes& response = b.hw.ep1_in.transfers[i];
        TEST_ASSERT(i % 3U == 0U   ? IsInfoString(response, "XRobot")
                    : i % 3U == 1U ? IsInfoString(response, "DAPLinkV1")
                                   : response[0] == DELAY && response[1] == 0U);
      }
    }
    else
    {
      for (const Bytes& response : b.hw.ep1_in.transfers)
      {
        TEST_ASSERT(IsInfoString(response, "XRobot") ||
                    IsInfoString(response, "DAPLinkV1") ||
                    (response[0] == DELAY && response[1] == 0U));
      }
      TEST_ASSERT(b.hw.ep1_in.transfers.size() > 100U);
    }
    BothInSlotsFree(b.hw.ep1_in);
    TEST_ASSERT(b.hw.ep2_out.inflight);
    HostOut(b.dev, b.hw.ep2_out, Report({INFO, VENDOR_ID}));
    TEST_ASSERT(b.hw.ep2_out.inflight);
    Run(b.dap);
    TEST_ASSERT(IsInfoString(b.hw.ep1_in.InFlight(), "XRobot"));
  }
}

/**
 * @brief DAP v2 在默认、1、3、8 个槽下的命令池满反压 / DAP v2 backpressure with the
 *        command pool full, with the default, 1, 3 and 8 slots
 * @note Step 不跑：四条命令进命令池，之后每个 OUT 槽留住一个包，全部留住时主机被 NAK。
 *       Step 跑完后全部按序应答，所有槽重新接收；HALT 再清除后所有槽仍在收；带着留住的
 *       槽重新绑定，槽全部交回。
 *       With Step not running four commands enter the command pool, then every OUT slot
 *       holds one packet and the host is NAKed once all are held. After Step ran every
 *       command is answered in order and every slot receives again; after a halt and its
 *       clear all slots still receive; a rebind with slots held gives them all back.
 */
void TestV2SlotCounts()
{
  for (uint8_t slots : SLOT_COUNTS)
  {
    V2Bench b(Speed::FULL, USBSpec::USB_2_0, 2048U, slots);
    Enumerate(b.dev, b.hw);
    for (int round = 0; round < 2; ++round)
    {
      std::vector<size_t> expected;
      for (size_t guard = 0; b.hw.ep2_out.inflight; ++guard)
      {
        TEST_ASSERT(guard < 32U);
        const uint8_t id = (guard % 2U == 0U) ? PACKET_COUNT_ID : PACKET_SIZE_ID;
        HostOut(b.dev, b.hw.ep2_out, {INFO, id});
        expected.push_back(id == PACKET_COUNT_ID ? 3U : 4U);
      }
      TEST_ASSERT(expected.size() == 4U + SlotsOf(slots));
      if (round == 1)
      {
        // 带着留住的槽重新绑定：旧会话的命令丢弃 / Rebound with slots held: the old
        // session's commands are dropped
        SetConfiguration(b.dev, b.hw, 1U);
        Run(b.dap);
        TEST_ASSERT(AllReceiving(b.hw.ep2_out));
        b.Send({INFO, PACKET_COUNT_ID});
        TEST_ASSERT(b.hw.ep1_in.InFlight() == (Bytes{INFO, 1U, 0x04U}));
        break;
      }
      Run(b.dap);
      b.DrainIn();
      const auto responses = Responses(b.hw.ep1_in);
      TEST_ASSERT(responses.size() == expected.size());
      for (size_t i = 0; i < expected.size(); ++i)
      {
        TEST_ASSERT(responses[i].size() == expected[i]);
      }
      TEST_ASSERT(AllReceiving(b.hw.ep2_out));
      b.hw.ep1_in.transfers.clear();

      // 主机 HALT 数据 OUT：每个接收以 ABORTED 结束并在回调里重挂，清除后全部在收。
      // The host halts data OUT: every receive ends ABORTED and is armed again in the
      // callback; after the clear all of them receive.
      SetHalt(b.dev, b.hw, 0x02U);
      ClearHalt(b.dev, b.hw, 0x02U);
      TEST_ASSERT(AllReceiving(b.hw.ep2_out));
    }
  }
}

/**
 * @brief DAP v1 在默认、1、3、8 个槽下的反压 / DAP v1 backpressure with the default, 1,
 *        3 and 8 slots
 * @note Step 不跑时每个 OUT 槽借出一个报告，全部借出时主机被 NAK；Step 跑完后按到达
 *       顺序应答，所有槽重新接收。
 *       With Step not running every OUT slot lends one report and the host is NAKed once
 *       all are lent; after Step ran they are answered in arrival order and every slot
 *       receives again.
 */
void TestV1SlotCounts()
{
  for (uint8_t slots : SLOT_COUNTS)
  {
    V1Bench b(Speed::FULL, slots);
    Enumerate(b.dev, b.hw);
    TEST_ASSERT(AllReceiving(b.hw.ep2_out));
    size_t reports = 0;
    for (; b.hw.ep2_out.inflight; ++reports)
    {
      TEST_ASSERT(reports < 32U);
      HostOut(b.dev, b.hw.ep2_out,
              Report({INFO, reports % 2U == 0U ? VENDOR_ID : PRODUCT_ID}));
    }
    TEST_ASSERT(reports == SlotsOf(slots));
    Run(b.dap);
    b.DrainIn();
    TEST_ASSERT(b.hw.ep1_in.transfers.size() == reports);
    for (size_t i = 0; i < reports; ++i)
    {
      TEST_ASSERT(
          IsInfoString(b.hw.ep1_in.transfers[i], i % 2U == 0U ? "XRobot" : "DAPLinkV1"));
    }
    TEST_ASSERT(AllReceiving(b.hw.ep2_out));
    // HALT 再清除：所有槽仍在收 / A halt and its clear: every slot still receives
    SetHalt(b.dev, b.hw, 0x02U);
    ClearHalt(b.dev, b.hw, 0x02U);
    TEST_ASSERT(AllReceiving(b.hw.ep2_out));
  }
}

/// 发送一次 DAP 命令并取回唯一 IN 响应包 / Send one DAP command and collect the single IN
/// response packet
template <typename Bench>
Bytes ExchangeDapCmd(Bench& b, Bytes req, bool pad_to_64)
{
  if (pad_to_64 && req.size() < 64U)
  {
    req.resize(64U, 0U);
  }
  b.Send(req);
  TEST_ASSERT(b.hw.ep1_in.inflight);
  const Bytes resp = b.hw.ep1_in.InFlight();
  b.DrainIn();
  TEST_ASSERT(b.hw.ep2_out.inflight);
  return resp;
}

void AppendLe32(Bytes& out, uint32_t value)
{
  out.push_back(static_cast<uint8_t>(value & 0xFFU));
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xFFU));
  out.push_back(static_cast<uint8_t>((value >> 16) & 0xFFU));
  out.push_back(static_cast<uint8_t>((value >> 24) & 0xFFU));
}

uint32_t ReadLe32(const Bytes& buf, size_t off)
{
  TEST_ASSERT(off + 4U <= buf.size());
  return static_cast<uint32_t>(buf[off]) | (static_cast<uint32_t>(buf[off + 1U]) << 8) |
         (static_cast<uint32_t>(buf[off + 2U]) << 16) |
         (static_cast<uint32_t>(buf[off + 3U]) << 24);
}

/// 经 JtagGeneralGPIO + JtagTarget 测 DAP v1/v2 的完整 JTAG 路径 / Full JTAG protocol
/// path for DAP v1/v2 over JtagGeneralGPIO and pin-level JtagTarget
template <typename Bench>
void RunDapJtagSuite(bool pad_to_64)
{
  namespace JSim = LibXR::Debug::Sim;
  namespace DDef = LibXR::USB::DapLinkV2Def;

  constexpr uint8_t CAPABILITIES = static_cast<uint8_t>(InfoId::CAPABILITIES);
  constexpr uint8_t DISCONNECT = static_cast<uint8_t>(CommandId::DISCONNECT);
  constexpr uint8_t TRANSFER_CONFIGURE =
      static_cast<uint8_t>(CommandId::TRANSFER_CONFIGURE);
  constexpr uint8_t WRITE_ABORT = static_cast<uint8_t>(CommandId::WRITE_ABORT);
  constexpr uint8_t SWJ_CLOCK = static_cast<uint8_t>(CommandId::SWJ_CLOCK);
  constexpr uint8_t JTAG_SEQUENCE = static_cast<uint8_t>(CommandId::JTAG_SEQUENCE);
  constexpr uint8_t JTAG_CONFIGURE = static_cast<uint8_t>(CommandId::JTAG_CONFIGURE);
  constexpr uint8_t JTAG_IDCODE = static_cast<uint8_t>(CommandId::JTAG_IDCODE);
  constexpr uint8_t JTAG_PORT = static_cast<uint8_t>(Port::JTAG);

  // 1. 未绑定 JTAG 前 CAPABILITIES 不含 JTAG 位；绑定后置位 / CAPABILITIES reports
  // DAP_CAP_JTAG only when SetJtag is attached
  JSim::JtagTarget target({
      JSim::JtagTapSpec{5U, 0x1BA00477U, 0x01U, false},
      JSim::JtagTapSpec{4U, JSim::JtagDebugPort::JTAG_DP_IDCODE,
                        LibXR::Debug::JtagProtocol::JTAG_IR_IDCODE, true},
      JSim::JtagTapSpec{4U, 0x2BA00477U, LibXR::Debug::JtagProtocol::JTAG_IR_IDCODE,
                        false},
  });
  JSim::SimTckPin tck(target.wire);
  JSim::SimTmsPin tms(target.wire);
  JSim::SimTdiPin tdi(target.wire);
  JSim::SimTdoPin tdo(target.wire);
  LibXR::Debug::JtagGeneralGPIO<JSim::SimTckPin, JSim::SimTmsPin, JSim::SimTdiPin,
                                JSim::SimTdoPin>
      jtag(tck, tms, tdi, tdo, 0U);

  Bench b;
  Enumerate(b.dev, b.hw);

  Bytes caps = ExchangeDapCmd(b, {INFO, CAPABILITIES}, pad_to_64);
  TEST_ASSERT(caps[0] == INFO && caps[1] == 1U && (caps[2] & DDef::DAP_CAP_JTAG) == 0U &&
              (caps[2] & DDef::DAP_CAP_SWD) != 0U);

  b.dap.SetJtag(&jtag);
  caps = ExchangeDapCmd(b, {INFO, CAPABILITIES}, pad_to_64);
  TEST_ASSERT(caps[0] == INFO && caps[1] == 1U && (caps[2] & DDef::DAP_CAP_JTAG) != 0U);

  // 2. CONNECT(JTAG) + SWJ_CLOCK + 3-TAP CONFIGURE + IDCODE（含 5-bit 0x01 回退与越界）/
  // CONNECT(JTAG), SWJ_CLOCK, 3-TAP CONFIGURE and IDCODE (incl. 5-bit 0x01 &
  // out-of-range)
  const Bytes conn = ExchangeDapCmd(b, {CONNECT, JTAG_PORT}, pad_to_64);
  TEST_ASSERT(conn[0] == CONNECT && conn[1] == JTAG_PORT);

  Bytes clk_req{SWJ_CLOCK};
  AppendLe32(clk_req, 1'000'000U);
  const Bytes clk_resp = ExchangeDapCmd(b, clk_req, pad_to_64);
  TEST_ASSERT(clk_resp[0] == SWJ_CLOCK && clk_resp[1] == 0U);

  const Bytes cfg_resp = ExchangeDapCmd(b, {JTAG_CONFIGURE, 3U, 5U, 4U, 4U}, pad_to_64);
  TEST_ASSERT(cfg_resp[0] == JTAG_CONFIGURE && cfg_resp[1] == 0U);

  const Bytes id0 = ExchangeDapCmd(b, {JTAG_IDCODE, 0U}, pad_to_64);
  TEST_ASSERT(id0[0] == JTAG_IDCODE && id0[1] == 0U && ReadLe32(id0, 2U) == 0x1BA00477U);

  const Bytes id1 = ExchangeDapCmd(b, {JTAG_IDCODE, 1U}, pad_to_64);
  TEST_ASSERT(id1[0] == JTAG_IDCODE && id1[1] == 0U &&
              ReadLe32(id1, 2U) == JSim::JtagDebugPort::JTAG_DP_IDCODE);

  const Bytes id2 = ExchangeDapCmd(b, {JTAG_IDCODE, 2U}, pad_to_64);
  TEST_ASSERT(id2[0] == JTAG_IDCODE && id2[1] == 0U && ReadLe32(id2, 2U) == 0x2BA00477U);

  const Bytes id3 = ExchangeDapCmd(b, {JTAG_IDCODE, 3U}, pad_to_64);
  TEST_ASSERT(id3[0] == JTAG_IDCODE && id3[1] == 0xFFU);

  // 3. JTAG_SEQUENCE（含 TMS=1、count=0->64、TDO 采集，并故意停在非 IDLE 状态）紧跟
  //    DAP_TRANSFER / JTAG_SEQUENCE (TMS=1, count=0->64, TDO capture, ending off-idle)
  //    followed directly by DAP_TRANSFER
  {
    Bytes jseq_long{
        JTAG_SEQUENCE,
        2U,
        static_cast<uint8_t>(DDef::JTAG_SEQUENCE_TMS | 5U),
        0x00U,
        static_cast<uint8_t>(DDef::JTAG_SEQUENCE_TDO | 0U),
        0xAAU,
        0x55U,
        0x12U,
        0x34U,
        0x56U,
        0x78U,
        0x9AU,
        0xBCU,
    };
    const unsigned edges_before = target.wire.tck_rising_edges;
    const Bytes jseq_long_resp = ExchangeDapCmd(b, jseq_long, pad_to_64);
    TEST_ASSERT(jseq_long_resp[0] == JTAG_SEQUENCE && jseq_long_resp[1] == 0U);
    if (!pad_to_64)
    {
      TEST_ASSERT(jseq_long_resp.size() == 2U + 8U);
    }
    TEST_ASSERT(target.wire.tck_rising_edges == edges_before + 69U);
  }

  // 把目标停到 SHIFT_DR 并在该状态下通过 JTAG_SEQUENCE 采出 TAP0 的 IDCODE / Leave TAP
  // at SHIFT_DR while capturing TAP0 IDCODE via JTAG_SEQUENCE
  {
    Bytes cap_seq{
        JTAG_SEQUENCE,
        4U,
        static_cast<uint8_t>(DDef::JTAG_SEQUENCE_TMS | 5U),
        0x00U,  // -> TEST_LOGIC_RESET (IR=IDCODE)
        0x01U,
        0x00U,  // 1 cycle TMS=0 -> RUN_TEST_IDLE
        static_cast<uint8_t>(DDef::JTAG_SEQUENCE_TMS | 1U),
        0x00U,  // 1 cycle TMS=1 -> SELECT_DR_SCAN
        0x02U,
        0x00U,  // 2 cycles TMS=0 -> CAPTURE_DR -> SHIFT_DR
    };
    const Bytes cap_enter = ExchangeDapCmd(b, cap_seq, pad_to_64);
    TEST_ASSERT(cap_enter[0] == JTAG_SEQUENCE && cap_enter[1] == 0U);
    TEST_ASSERT(target.wire.State() == LibXR::Debug::TapState::SHIFT_DR);

    Bytes cap_tdo{
        JTAG_SEQUENCE, 1U,    static_cast<uint8_t>(DDef::JTAG_SEQUENCE_TDO | 32U),
        0x00U,         0x00U, 0x00U,
        0x00U,
    };
    const Bytes cap_resp = ExchangeDapCmd(b, cap_tdo, pad_to_64);
    TEST_ASSERT(cap_resp[0] == JTAG_SEQUENCE && cap_resp[1] == 0U);
    TEST_ASSERT(ReadLe32(cap_resp, 2U) == 0x1BA00477U);
    TEST_ASSERT(target.wire.State() == LibXR::Debug::TapState::SHIFT_DR);
  }

  // 此时硬件 TAP 还在 SHIFT_DR！紧接着做 DAP_TRANSFER（DP 上电写 + 读 CTRL/STAT）必须能
  // 先安全回到 IDLE 再读写 / Target is still at SHIFT_DR; immediate DAP_TRANSFER must
  // resync and access TAP1 DP CTRL/STAT cleanly
  {
    Bytes xfer{TRANSFER, 1U, 2U};
    xfer.push_back(DDef::DAP_TRANSFER_A2);  // DP write CTRL/STAT
    AppendLe32(xfer, 0x50000000U);
    xfer.push_back(DDef::DAP_TRANSFER_RNW | DDef::DAP_TRANSFER_A2);  // DP read CTRL/STAT
    const Bytes xresp = ExchangeDapCmd(b, xfer, pad_to_64);
    TEST_ASSERT(xresp[0] == TRANSFER && xresp[1] == 2U &&
                xresp[2] == DDef::DAP_TRANSFER_OK);
    TEST_ASSERT((ReadLe32(xresp, 3U) & 0xF0000000U) == 0xF0000000U);
    TEST_ASSERT(target.wire.protocol_errors == 0U);
  }

  // 4. JTAG 模式下发送 SWJ_SEQUENCE：必须走 jtag_ 而非 swd_，且之后紧跟的 TRANSFER 正常 /
  // SWJ_SEQUENCE in JTAG mode must drive jtag_ (not swd_) and keep TAP state coherent
  {
    const unsigned swd_seqs_before = b.swd.sequences;
    const unsigned tck_before = target.wire.tck_rising_edges;
    // 7 位 LSB-first: 1,1,1,1,1,0,1 (0x5FU) -> TLR -> RTI -> SELECT_DR_SCAN
    const Bytes swj_resp = ExchangeDapCmd(b, {SWJ_SEQUENCE, 7U, 0x5FU}, pad_to_64);
    TEST_ASSERT(swj_resp[0] == SWJ_SEQUENCE && swj_resp[1] == 0U);
    TEST_ASSERT(b.swd.sequences == swd_seqs_before);
    TEST_ASSERT(target.wire.tck_rising_edges == tck_before + 7U);
    TEST_ASSERT(target.wire.State() == LibXR::Debug::TapState::SELECT_DR_SCAN);
  }

  // 5. 经 DAP_TRANSFER 读写 SRAM 与读取 CPUID / DAP_TRANSFER SRAM write-read and CPUID
  {
    Bytes xfer{TRANSFER, 1U, 6U};
    xfer.push_back(DDef::DAP_TRANSFER_A3);  // DP write SELECT = 0
    AppendLe32(xfer, 0x00000000U);
    xfer.push_back(DDef::DAP_TRANSFER_APNDP);  // AP write CSW = 0x23000012 (word + inc)
    AppendLe32(xfer, 0x23000012U);
    xfer.push_back(DDef::DAP_TRANSFER_APNDP | DDef::DAP_TRANSFER_A2);  // AP write TAR
    AppendLe32(xfer, 0x20000100U);
    xfer.push_back(DDef::DAP_TRANSFER_APNDP | DDef::DAP_TRANSFER_A3 |
                   DDef::DAP_TRANSFER_A2);  // AP write DRW
    AppendLe32(xfer, 0xCAFEBABEU);
    xfer.push_back(DDef::DAP_TRANSFER_APNDP | DDef::DAP_TRANSFER_A2);  // AP write TAR
    AppendLe32(xfer, 0x20000100U);
    xfer.push_back(DDef::DAP_TRANSFER_APNDP | DDef::DAP_TRANSFER_RNW |
                   DDef::DAP_TRANSFER_A3 | DDef::DAP_TRANSFER_A2);  // AP read DRW
    const Bytes xresp = ExchangeDapCmd(b, xfer, pad_to_64);
    TEST_ASSERT(xresp[0] == TRANSFER && xresp[1] == 6U &&
                xresp[2] == DDef::DAP_TRANSFER_OK);
    TEST_ASSERT(ReadLe32(xresp, 3U) == 0xCAFEBABEU);
  }

  {
    Bytes xfer{TRANSFER, 1U, 2U};
    xfer.push_back(DDef::DAP_TRANSFER_APNDP | DDef::DAP_TRANSFER_A2);  // AP write TAR
    AppendLe32(xfer, JSim::TargetMemory::CPUID);
    xfer.push_back(DDef::DAP_TRANSFER_APNDP | DDef::DAP_TRANSFER_RNW |
                   DDef::DAP_TRANSFER_A3 | DDef::DAP_TRANSFER_A2);  // AP read DRW
    const Bytes xresp = ExchangeDapCmd(b, xfer, pad_to_64);
    TEST_ASSERT(xresp[0] == TRANSFER && xresp[1] == 2U &&
                xresp[2] == DDef::DAP_TRANSFER_OK);
    TEST_ASSERT(ReadLe32(xresp, 3U) == 0x410FC241U);
  }

  // 6. MATCH_MASK 与 MATCH_VALUE（不匹配返回 MISMATCH，匹配返回 OK）/ MATCH_MASK &
  // MATCH_VALUE
  {
    const Bytes cfg =
        ExchangeDapCmd(b, {TRANSFER_CONFIGURE, 2U, 8U, 0U, 4U, 0U}, pad_to_64);
    TEST_ASSERT(cfg[0] == TRANSFER_CONFIGURE && cfg[1] == 0U);

    Bytes mismatch_req{TRANSFER, 1U, 2U};
    mismatch_req.push_back(DDef::DAP_TRANSFER_MATCH_MASK);
    AppendLe32(mismatch_req, 0x0000FFFFU);
    mismatch_req.push_back(DDef::DAP_TRANSFER_APNDP | DDef::DAP_TRANSFER_RNW |
                           DDef::DAP_TRANSFER_A3 | DDef::DAP_TRANSFER_A2 |
                           DDef::DAP_TRANSFER_MATCH_VALUE);
    AppendLe32(mismatch_req, 0x0000DEADU);
    const Bytes mismatch_resp = ExchangeDapCmd(b, mismatch_req, pad_to_64);
    TEST_ASSERT(mismatch_resp[0] == TRANSFER &&
                mismatch_resp[2] ==
                    (DDef::DAP_TRANSFER_OK | DDef::DAP_TRANSFER_MISMATCH));

    // TAR 因 CSW auto-inc 前进了 4 字节，先设回 CPUID 再测匹配成功 / Reset TAR to CPUID
    Bytes match_req{TRANSFER, 1U, 3U};
    match_req.push_back(DDef::DAP_TRANSFER_APNDP | DDef::DAP_TRANSFER_A2);
    AppendLe32(match_req, JSim::TargetMemory::CPUID);
    match_req.push_back(DDef::DAP_TRANSFER_MATCH_MASK);
    AppendLe32(match_req, 0x0000FFFFU);
    match_req.push_back(DDef::DAP_TRANSFER_APNDP | DDef::DAP_TRANSFER_RNW |
                        DDef::DAP_TRANSFER_A3 | DDef::DAP_TRANSFER_A2 |
                        DDef::DAP_TRANSFER_MATCH_VALUE);
    AppendLe32(match_req, 0x0000C241U);
    const Bytes match_resp = ExchangeDapCmd(b, match_req, pad_to_64);
    TEST_ASSERT(match_resp[0] == TRANSFER && match_resp[1] == 3U &&
                match_resp[2] == DDef::DAP_TRANSFER_OK);
  }

  // 7. WAIT 重试成功 + 重试耗尽 -> WRITE_ABORT 恢复 + 总线错误置 STICKYERR 与 W1C 清除 /
  // WAIT retry recovery, WRITE_ABORT after WAIT exhaustion, bus error STICKYERR + W1C
  {
    TEST_ASSERT(
        ExchangeDapCmd(b, {TRANSFER_CONFIGURE, 0U, 4U, 0U, 0U, 0U}, pad_to_64)[1] == 0U);
    target.dp.inject_wait = 3U;
    Bytes read_ram{TRANSFER, 1U, 2U};
    read_ram.push_back(DDef::DAP_TRANSFER_APNDP | DDef::DAP_TRANSFER_A2);
    AppendLe32(read_ram, 0x20000100U);
    read_ram.push_back(DDef::DAP_TRANSFER_APNDP | DDef::DAP_TRANSFER_RNW |
                       DDef::DAP_TRANSFER_A3 | DDef::DAP_TRANSFER_A2);
    const Bytes r_ok = ExchangeDapCmd(b, read_ram, pad_to_64);
    TEST_ASSERT(r_ok[0] == TRANSFER && r_ok[1] == 2U && r_ok[2] == DDef::DAP_TRANSFER_OK);
    TEST_ASSERT(ReadLe32(r_ok, 3U) == 0xCAFEBABEU);
    TEST_ASSERT(target.dp.inject_wait == 0U);

    // 等待重试耗尽后发 WRITE_ABORT 清挂起请求 / Exhaust wait retries then WRITE_ABORT
    TEST_ASSERT(
        ExchangeDapCmd(b, {TRANSFER_CONFIGURE, 0U, 1U, 0U, 0U, 0U}, pad_to_64)[1] == 0U);
    target.dp.inject_wait = 10U;
    const Bytes r_wait = ExchangeDapCmd(b, read_ram, pad_to_64);
    TEST_ASSERT(r_wait[0] == TRANSFER && r_wait[2] == DDef::DAP_TRANSFER_WAIT);

    Bytes abort_req{WRITE_ABORT, 1U};
    AppendLe32(abort_req, 0x00000001U);  // DAPABORT
    TEST_ASSERT(ExchangeDapCmd(b, abort_req, pad_to_64)[1] == 0U);
    TEST_ASSERT(target.dp.inject_wait == 0U);

    const Bytes r_after_abort = ExchangeDapCmd(b, read_ram, pad_to_64);
    TEST_ASSERT(r_after_abort[0] == TRANSFER &&
                r_after_abort[2] == DDef::DAP_TRANSFER_OK &&
                ReadLe32(r_after_abort, 3U) == 0xCAFEBABEU);

    // 写只读 Flash 地址引发总线错误 -> CTRL/STAT.STICKYERR 置位 -> W1C 清除后恢复 /
    // Writing read-only flash sets STICKYERR; W1C CTRL/STAT clears it
    Bytes fault_req{TRANSFER, 1U, 3U};
    fault_req.push_back(DDef::DAP_TRANSFER_APNDP | DDef::DAP_TRANSFER_A2);
    AppendLe32(fault_req, 0x08000000U);
    fault_req.push_back(DDef::DAP_TRANSFER_APNDP | DDef::DAP_TRANSFER_A3 |
                        DDef::DAP_TRANSFER_A2);
    AppendLe32(fault_req, 0x11223344U);
    fault_req.push_back(DDef::DAP_TRANSFER_RNW |
                        DDef::DAP_TRANSFER_A2);  // read CTRL/STAT
    const Bytes f_resp = ExchangeDapCmd(b, fault_req, pad_to_64);
    TEST_ASSERT(f_resp[0] == TRANSFER && f_resp[1] == 3U &&
                f_resp[2] == DDef::DAP_TRANSFER_OK);
    TEST_ASSERT((ReadLe32(f_resp, 3U) & JSim::JtagDebugPort::STICKYERR) != 0U);

    Bytes clear_err{TRANSFER, 1U, 2U};
    clear_err.push_back(DDef::DAP_TRANSFER_A2);
    AppendLe32(clear_err, 0x50000000U | JSim::JtagDebugPort::STICKYERR);
    clear_err.push_back(DDef::DAP_TRANSFER_RNW | DDef::DAP_TRANSFER_A2);
    const Bytes c_resp = ExchangeDapCmd(b, clear_err, pad_to_64);
    TEST_ASSERT(c_resp[0] == TRANSFER && c_resp[1] == 2U &&
                c_resp[2] == DDef::DAP_TRANSFER_OK);
    TEST_ASSERT((ReadLe32(c_resp, 3U) & JSim::JtagDebugPort::STICKYERR) == 0U);
  }

  // 8. TRANSFER_BLOCK 多字连续写入与回读（8 字 = 36 字节，适配 v1 64 字节报告和 v2）/
  // TRANSFER_BLOCK multi-word SRAM write & read (8 words fits v1 64-byte report and v2)
  {
    Bytes set_tar{TRANSFER, 1U, 1U};
    set_tar.push_back(DDef::DAP_TRANSFER_APNDP | DDef::DAP_TRANSFER_A2);
    AppendLe32(set_tar, 0x20000200U);
    TEST_ASSERT(ExchangeDapCmd(b, set_tar, pad_to_64)[2] == DDef::DAP_TRANSFER_OK);

    Bytes blk_w{TRANSFER_BLOCK, 1U, 8U, 0U,
                DDef::DAP_TRANSFER_APNDP | DDef::DAP_TRANSFER_A3 | DDef::DAP_TRANSFER_A2};
    for (uint32_t i = 0; i < 8U; ++i)
    {
      AppendLe32(blk_w, 0xA1000000U + i);
    }
    const Bytes bw_resp = ExchangeDapCmd(b, blk_w, pad_to_64);
    TEST_ASSERT(bw_resp[0] == TRANSFER_BLOCK && (bw_resp[1] | (bw_resp[2] << 8)) == 8U &&
                bw_resp[3] == DDef::DAP_TRANSFER_OK);

    TEST_ASSERT(ExchangeDapCmd(b, set_tar, pad_to_64)[2] == DDef::DAP_TRANSFER_OK);
    const Bytes blk_r =
        ExchangeDapCmd(b,
                       {TRANSFER_BLOCK, 1U, 8U, 0U,
                        DDef::DAP_TRANSFER_APNDP | DDef::DAP_TRANSFER_RNW |
                            DDef::DAP_TRANSFER_A3 | DDef::DAP_TRANSFER_A2},
                       pad_to_64);
    TEST_ASSERT(blk_r[0] == TRANSFER_BLOCK && (blk_r[1] | (blk_r[2] << 8)) == 8U &&
                blk_r[3] == DDef::DAP_TRANSFER_OK);
    for (uint32_t i = 0; i < 8U; ++i)
    {
      TEST_ASSERT(ReadLe32(blk_r, 4U + 4U * i) == 0xA1000000U + i);
    }
  }

  // 9. DISCONNECT + 第二次 CONNECT(JTAG) 会话状态复位校验 + USB 重绑 /
  // DISCONNECT, second CONNECT(JTAG) state reset check, and USB rebind
  {
    const Bytes disc = ExchangeDapCmd(b, {DISCONNECT}, pad_to_64);
    TEST_ASSERT(disc[0] == DISCONNECT && disc[1] == 0U);
    TEST_ASSERT(ExchangeDapCmd(b, {JTAG_IDCODE, 1U}, pad_to_64)[1] == 0xFFU);

    // 重连 JTAG 后链状态应被清空，重新 CONFIGURE 前 IDCODE 应报错 / Re-connecting JTAG
    // resets the configured chain until JTAG_CONFIGURE runs again
    TEST_ASSERT(ExchangeDapCmd(b, {CONNECT, JTAG_PORT}, pad_to_64)[1] == JTAG_PORT);
    TEST_ASSERT(ExchangeDapCmd(b, {JTAG_IDCODE, 1U}, pad_to_64)[1] == 0xFFU);
    TEST_ASSERT(ExchangeDapCmd(b, {JTAG_CONFIGURE, 3U, 5U, 4U, 4U}, pad_to_64)[1] == 0U);
    TEST_ASSERT(ReadLe32(ExchangeDapCmd(b, {JTAG_IDCODE, 1U}, pad_to_64), 2U) ==
                JSim::JtagDebugPort::JTAG_DP_IDCODE);

    // USB 解绑重绑：重绑后再次 JTAG 连接与传输 / USB unbind + rebind
    SetConfiguration(b.dev, b.hw, 0U);
    SetConfiguration(b.dev, b.hw, 1U);
    TEST_ASSERT(ExchangeDapCmd(b, {CONNECT, JTAG_PORT}, pad_to_64)[1] == JTAG_PORT);
    TEST_ASSERT(ExchangeDapCmd(b, {JTAG_CONFIGURE, 3U, 5U, 4U, 4U}, pad_to_64)[1] == 0U);
    TEST_ASSERT(ReadLe32(ExchangeDapCmd(b, {JTAG_IDCODE, 1U}, pad_to_64), 2U) ==
                JSim::JtagDebugPort::JTAG_DP_IDCODE);
  }

  TEST_ASSERT(target.wire.protocol_errors == 0U);
}

void TestV1Jtag() { RunDapJtagSuite<V1Bench>(true); }
void TestV2Jtag() { RunDapJtagSuite<V2Bench>(false); }

}  // namespace

void test_usb_dap()
{
  TestV1Enumeration();
  TestV1Commands();
  TestV1BindFailure();
  TestV1UnbindInFlight();
  TestV1Threads();
  TestV2Enumeration();
  TestV2Commands();
  TestV2ClosingZlp();
  TestV2BindFailure();
  TestV2UnbindInFlight();
  TestV2Threads();
  TestV2RebindRace();
  TestV1SlotCounts();
  TestV2SlotCounts();
  TestV1Jtag();
  TestV2Jtag();
}
