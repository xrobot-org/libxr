/**
 * @file test_gsusb.cpp
 * @brief gs_usb（CAN 适配器）设备类（槽模型）主机测试 / Host tests for the gs_usb (CAN
 *        adapter) device class.
 *
 * 覆盖槽模型迁移改动的路径：每通道一个 SPSC 接收队列加一个 echo 队列、
 * 服务里取槽打包提交、echo 总是优先、echo 队列满时保留 OUT 租约作背压、
 * RX 队列满时丢帧计数并在该通道发往主机的下一帧（含 echo）置
 * CAN_FLAG_OVERFLOW，以及 SET_USER_ID 只改 RAM 与待写标志、由 100 ms Timer
 * 任务落库。
 * Covers the paths the slot-model migration changed: one SPSC RX queue per channel
 * plus one echo queue, slot packing and submission in the class service, absolute
 * echo priority, the OUT lease held as backpressure while the echo queue is full,
 * dropped frames counted per channel with CAN_FLAG_OVERFLOW on that channel's next
 * frame to the host (echoes included), and SET_USER_ID only touching RAM plus a
 * pending flag with the 100 ms Timer task writing the Database.
 */

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <vector>

#include "can.hpp"
#include "device_composition.hpp"
#include "driver/usb/device/usb_device_test_common.hpp"
#include "driver/usb/usb_test_common.hpp"
#include "flash.hpp"
#include "gs_usb.hpp"
#include "gs_usb_protocol.hpp"
#include "lockfree_list.hpp"
#include "raw_sequential.hpp"
#include "test_assert.hpp"
#include "timer.hpp"

using namespace LibXR;
using namespace LibXR::USB;
using namespace LibXR::USB::Test;

namespace
{
static constexpr auto TEST_LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "LibXR", "USB tests", "0");

constexpr std::size_t CHANNELS = 2;
using Gs = GsUsbClass<CHANNELS>;

/// 公开该类受保护的绑定与请求入口
/// Exposes the protected bind/request entry points of the class.
class TestGs : public Gs
{
 public:
  using Gs::BindEndpoints;
  using Gs::Gs;
  using Gs::OnClassData;
  using Gs::OnVendorRequest;
  using Gs::UnbindEndpoints;
  using ControlTransferResult = DeviceClass::ControlTransferResult;
};

/// 让本测试使用自己的定时器链表，这样先前销毁的类不会给下一次 Timer::Refresh() 留下
/// 悬空的任务节点。
/// Give this test its own timer list so a class destroyed earlier cannot leave a
/// dangling task node behind for the next Timer::Refresh().
void FreshTimerList() { LibXR::Timer::list_ = new LibXR::LockFreeList(); }

/// 线上帧布局：echo_id、can_id、dlc、channel、flags、reserved，以及 8 字节数据
/// Wire frame layout: echo_id, can_id, dlc, channel, flags, reserved, 8 data bytes.
constexpr std::size_t WIRE_HEADER = 12;
constexpr std::size_t WIRE_CLASSIC = WIRE_HEADER + 8;

/// 最小的 CAN 控制器：记录类发送的内容，并注入 RX 帧
/// Minimal CAN controller: records what the class transmits and injects RX frames.
class StubCan : public LibXR::CAN
{
 public:
  struct Tx
  {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
  };

  ErrorCode SetConfig(const LibXR::CAN::Configuration&) override
  {
    ++configures;
    return ErrorCode::OK;
  }
  uint32_t GetClockFreq() const override { return 42000000u; }
  ErrorCode AddMessage(const ClassicPack& pack) override
  {
    Tx entry{};
    entry.id = pack.id;
    entry.dlc = pack.dlc;
    std::memcpy(entry.data, pack.data, 8);
    tx.push_back(entry);
    return ErrorCode::OK;
  }

  /// 通过已注册的订阅者向类注入一个收到的帧
  /// Play a received frame into the class through its registered subscribers.
  void Inject(uint32_t id, const std::vector<uint8_t>& data,
              LibXR::CAN::Type type = LibXR::CAN::Type::STANDARD)
  {
    ClassicPack pack{};
    pack.id = id;
    pack.type = type;
    pack.dlc = static_cast<uint8_t>(data.size());
    std::copy(data.begin(), data.end(), pack.data);
    OnMessage(pack, false);
  }

  unsigned configures = 0;
  std::vector<Tx> tx;
};

std::vector<uint8_t> ClassicFrame(uint32_t echo_id, uint32_t can_id, uint8_t dlc,
                                  uint8_t channel, uint8_t flags,
                                  const std::vector<uint8_t>& data)
{
  std::vector<uint8_t> frame(WIRE_CLASSIC, 0);
  std::memcpy(&frame[0], &echo_id, 4);
  std::memcpy(&frame[4], &can_id, 4);
  frame[8] = dlc;
  frame[9] = channel;
  frame[10] = flags;
  std::memcpy(&frame[WIRE_HEADER], data.data(), std::min<size_t>(data.size(), 8));
  return frame;
}

uint32_t ReadU32(const std::vector<uint8_t>& frame, std::size_t offset)
{
  uint32_t value = 0;
  std::memcpy(&value, &frame[offset], 4);
  return value;
}

/// 两个 CAN 通道、该类，以及为其提供服务池的组合
/// Two CAN channels, the class and the composition that gives it a service pool.
struct GsFixture
{
  Block<256> in_storage, out_storage;
  EndpointPool pool{4};
  MockEndpoint in{Number::EP1, Direction::IN, in_storage.Raw()};
  MockEndpoint out{Number::EP2, Direction::OUT, out_storage.Raw()};
  StubCan can0, can1;
  TestGs gs;
  DeviceComposition composition;
  Callback<uint32_t> handler;

  explicit GsFixture(size_t rx_queue_size = 32, size_t echo_queue_size = 32,
                     LibXR::Database* database = nullptr)
      : gs(Number::EP1, Number::EP2, {&can0, &can1}, rx_queue_size, echo_queue_size,
           nullptr, {}, database),
        composition(pool, {&TEST_LANGUAGE}, {{&gs}})
  {
    TEST_ASSERT(pool.Put(&in) == ErrorCode::OK);
    TEST_ASSERT(pool.Put(&out) == ErrorCode::OK);
    // 代替 DeviceCore 的类服务分发。
    // Stand-in for DeviceCore's class-service dispatch.
    handler = Callback<uint32_t>::Create(
        [](bool in_isr, DeviceClass* self, uint32_t events)
        {
          if ((events & EndpointPool::CLASS_EVENT) != 0U) self->OnService(in_isr);
        },
        static_cast<DeviceClass*>(&gs));
    pool.SetControlHandler(handler);
  }
};

/// 主机格式协商（小端标记）与每通道模式。每个带数据阶段的请求都要先发其厂商请求：
/// 由它选定通道。
/// Host format negotiation (little-endian marker) and per-channel mode. Every request
/// with a data stage needs its vendor request first: that is what selects the channel.
void Configure(GsFixture& f, bool enable_ch0 = true, bool enable_ch1 = true)
{
  const uint32_t byte_order = 0x0000beefu;
  ConstRawData fmt{&byte_order, sizeof(byte_order)};
  TEST_ASSERT(f.gs.OnClassData(false, static_cast<uint8_t>(GsUsb::BReq::HOST_FORMAT),
                               fmt) == ErrorCode::OK);

  const bool enabled[CHANNELS] = {enable_ch0, enable_ch1};
  for (uint16_t ch = 0; ch < CHANNELS; ++ch)
  {
    TestGs::ControlTransferResult result;
    TEST_ASSERT(f.gs.OnVendorRequest(false, static_cast<uint8_t>(GsUsb::BReq::MODE), ch,
                                     2u * sizeof(uint32_t), 0, result) == ErrorCode::OK);
    const uint32_t mode[2] = {enabled[ch] ? 1u : 0u, 0u};
    ConstRawData data{mode, sizeof(mode)};
    TEST_ASSERT(f.gs.OnClassData(false, static_cast<uint8_t>(GsUsb::BReq::MODE), data) ==
                ErrorCode::OK);
  }
}

void TestBindAndConfiguration()
{
  FreshTimerList();
  GsFixture f;
  f.gs.BindEndpoints(f.pool, 0, false);

  // 每个 OUT 租约对应一整个槽（receive_size 为 0）：一个帧在一次请求里到达。
  // One whole slot per OUT lease (receive_size 0): a frame arrives in one request.
  TEST_ASSERT(f.out.active && f.out.Last().length == 128);
  TEST_ASSERT(f.in.starts.empty());

  // HOST_FORMAT / MODE 的数据阶段经 OnClassData 到达类。
  // The data stage of HOST_FORMAT / MODE reaches the class through OnClassData.
  const uint32_t byte_order = 0x0000beefu;
  ConstRawData fmt{&byte_order, sizeof(byte_order)};
  TEST_ASSERT(f.gs.OnClassData(false, static_cast<uint8_t>(GsUsb::BReq::HOST_FORMAT),
                               fmt) == ErrorCode::OK);
  const uint32_t mode[2] = {1u, 0u};  // 通道 0 START / START on channel 0
  ConstRawData mode_data{mode, sizeof(mode)};
  TEST_ASSERT(f.gs.OnClassData(false, static_cast<uint8_t>(GsUsb::BReq::MODE),
                               mode_data) == ErrorCode::OK);
  TEST_ASSERT(f.can0.configures == 1);

  // 越界通道的 MODE 在任何数据阶段之前就被拒绝。
  // MODE for an out-of-range channel is refused before any data stage.
  TestGs::ControlTransferResult result;
  TEST_ASSERT(f.gs.OnVendorRequest(false, static_cast<uint8_t>(GsUsb::BReq::MODE), 9, 8,
                                   0, result) == ErrorCode::ARG_ERR);
  // mode 数据阶段必须恰好是一个 DeviceMode。
  // The mode data stage must be exactly one DeviceMode.
  const uint32_t short_mode[1] = {1u};
  ConstRawData short_data{short_mode, sizeof(short_mode)};
  TEST_ASSERT(f.gs.OnClassData(false, static_cast<uint8_t>(GsUsb::BReq::MODE),
                               short_data) == ErrorCode::ARG_ERR);
  // GET_DEVICE_CONFIG 无需数据阶段即可应答。
  // GET_DEVICE_CONFIG answers without a data stage.
  TEST_ASSERT(f.gs.OnVendorRequest(false,
                                   static_cast<uint8_t>(GsUsb::BReq::DEVICE_CONFIG), 0,
                                   12, 0, result) == ErrorCode::OK);
  TEST_ASSERT(result.write_data.addr_ != nullptr && result.write_data.size_ == 12U);
}

void TestHostFrameToCanAndEcho()
{
  FreshTimerList();
  GsFixture f;
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f);

  const std::vector<uint8_t> payload{0x11, 0x22, 0x33};
  f.out.HostSends(ClassicFrame(0x1234u, 0x123u, 3, 0, 0, payload));

  // 该帧到达 CAN 控制器一次，echo 从 IN 端点回到主机。
  // The frame reached the CAN controller once, and the echo went back on the IN endpoint.
  TEST_ASSERT(f.can0.tx.size() == 1);
  TEST_ASSERT(f.can0.tx[0].id == 0x123u && f.can0.tx[0].dlc == 3);
  TEST_ASSERT(std::equal(payload.begin(), payload.end(), f.can0.tx[0].data));
  TEST_ASSERT(f.in.active && f.in.Last().length == WIRE_CLASSIC);
  const auto& echo = f.in.Last().data;
  // 主机的 echo_id 原样返回 / The host's echo_id comes back.
  TEST_ASSERT(ReadU32(echo, 0) == 0x1234u);
  TEST_ASSERT(ReadU32(echo, 4) == 0x123u);
  TEST_ASSERT(echo[8] == 3 && echo[9] == 0 && echo[10] == 0);
  TEST_ASSERT(std::equal(payload.begin(), payload.end(), echo.begin() + WIRE_HEADER));
  // 租约已归还，所以 OUT 接收重新挂包。
  // The lease went back, so the OUT receive re-armed.
  TEST_ASSERT(f.out.active);
  f.in.Complete();

  // 发往未启动通道的帧被丢弃，不访问控制器。
  // A frame for a channel that is not started is dropped without touching the controller.
  TestGs::ControlTransferResult mode_result;
  TEST_ASSERT(f.gs.OnVendorRequest(false, static_cast<uint8_t>(GsUsb::BReq::MODE), 0, 8,
                                   0, mode_result) == ErrorCode::OK);
  const uint32_t stop[2] = {0u, 0u};
  ConstRawData stop_data{stop, sizeof(stop)};
  TEST_ASSERT(f.gs.OnClassData(false, static_cast<uint8_t>(GsUsb::BReq::MODE),
                               stop_data) == ErrorCode::OK);
  f.out.HostSends(ClassicFrame(0x1235u, 0x124u, 1, 0, 0, {0x01}));
  TEST_ASSERT(f.can0.tx.size() == 1);
  TEST_ASSERT(f.in.starts.size() == 1);
}

void TestCanRxToHost()
{
  FreshTimerList();
  GsFixture f;
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f);

  const std::vector<uint8_t> payload{0xAA, 0xBB};
  f.can0.Inject(0x456u, payload);
  TEST_ASSERT(f.in.active && f.in.Last().length == WIRE_CLASSIC);
  const auto& frame = f.in.Last().data;
  // RX 帧携带 ECHO_ID_RX / RX frames carry ECHO_ID_RX.
  TEST_ASSERT(ReadU32(frame, 0) == 0xFFFFFFFFu);
  TEST_ASSERT(ReadU32(frame, 4) == 0x456u);
  TEST_ASSERT(frame[8] == 2 && frame[9] == 0);
  TEST_ASSERT(frame[10] == 0);
  TEST_ASSERT(std::equal(payload.begin(), payload.end(), frame.begin() + WIRE_HEADER));
  f.in.Complete();
}

void TestRxQueueOverflowFlag()
{
  FreshTimerList();
  GsFixture f(4, 32);  // 每通道四帧 / four frames per channel
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f, true, false);

  // 两个 IN 槽在途，四深的 RX 队列被填满，所以第七帧无处可去而被丢弃。
  // Two IN slots are in flight and the four-deep RX queue fills, so the seventh frame
  // has nowhere to go and is dropped.
  for (unsigned i = 0; i < 7; ++i)
  {
    f.can0.Inject(0x100u + i, {static_cast<uint8_t>(i)});
  }
  // 排空：两帧已在总线上，其余的在槽释放后跟上。
  // Drain: two frames are on the wire, the rest follow once slots free up.
  for (unsigned i = 0; i < 12 && f.in.active; ++i)
  {
    f.in.Complete();
  }

  TEST_ASSERT(f.in.sent.size() == 6);  // 注入七帧，丢弃一帧 / seven injected, one dropped
  // Linux 主机照常交付带标志的帧，然后再补上它自己的 RX 溢出错误帧，
  // 所以标志附在通道 0 的一个真实帧上，且该帧保持完整。
  // The Linux host delivers the frame that carries the flag as usual and then adds
  // its own RX-overflow error frame, so the flag rides on a real frame of channel 0
  // and that frame stays intact.
  size_t overflow = 0;
  for (std::size_t i = 0; i < f.in.sent.size(); ++i)
  {
    const auto& frame = f.in.sent[i];
    TEST_ASSERT(ReadU32(frame, 0) == 0xFFFFFFFFu && frame[9] == 0);
    TEST_ASSERT(ReadU32(frame, 4) == 0x100u + i && frame[WIRE_HEADER] == i);
    if ((frame[10] & GsUsb::CAN_FLAG_OVERFLOW) != 0U) ++overflow;
  }
  // 丢帧恰好报告一次，附在该通道的下一帧上。
  // The drop is reported exactly once, on that channel's next frame.
  TEST_ASSERT(overflow == 1);
}

/// Linux 主机在处理任何帧（包括 echo）之后都会检查 GS_CAN_FLAG_OVERFLOW，并把它计到
/// 该帧所属的通道上。因此丢帧会报告在该通道发往主机的下一帧上，即使那一帧是 echo，
/// 也绝不会报告在其他通道的帧上。
/// The Linux host checks GS_CAN_FLAG_OVERFLOW after handling any frame, an echo
/// included, and counts it for that frame's channel. A drop is therefore reported on
/// the channel's next frame to the host even when that frame is an echo, and never on
/// another channel's frame.
void TestOverflowFlagRidesOnTheChannelsNextFrame()
{
  FreshTimerList();
  GsFixture f(4, 32);  // 每通道四帧 / four frames per channel
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f);

  // 两个 IN 槽在途，四帧排队，第七帧被丢弃。
  // Two IN slots in flight, four frames queued, the seventh dropped.
  for (unsigned i = 0; i < 7; ++i)
  {
    f.can0.Inject(0x100u + i, {static_cast<uint8_t>(i)});
  }
  // 通道 1 和通道 0 的 echo 随后排队；echo 优先发送。
  // Echoes for channel 1 and channel 0 queue next; echoes go first.
  f.out.HostSends(ClassicFrame(0x10u, 0x301u, 1, 1, 0, {0x11}));
  f.out.HostSends(ClassicFrame(0x20u, 0x300u, 1, 0, 0, {0x22}));
  for (unsigned i = 0; i < 16 && f.in.active; ++i)
  {
    f.in.Complete();
  }

  TEST_ASSERT(f.in.sent.size() == 8);
  size_t overflow = 0;
  for (const auto& frame : f.in.sent)
  {
    if ((frame[10] & GsUsb::CAN_FLAG_OVERFLOW) == 0U) continue;
    ++overflow;
    // 通道 0 的 echo 是丢帧后的第一帧；它保留自己的 echo_id。
    // Channel 0's echo is its first frame after the drop; it keeps its echo_id.
    TEST_ASSERT(ReadU32(frame, 0) == 0x20u && frame[9] == 0);
  }
  TEST_ASSERT(overflow == 1);
}

/// echo 帧先于任何排队的 RX 帧发往主机：主机的 TX 路径（Linux 驱动中的 echo ->
/// netif_wake_queue）不会被接收流量拖延。
/// Echo frames go to the host before any queued RX frame: the host's TX path (echo ->
/// netif_wake_queue in the Linux driver) is not delayed behind received traffic.
void TestEchoHasPriority()
{
  FreshTimerList();
  GsFixture f;
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f, true, false);

  // 两个 RX 帧占用两个 IN 槽，另外四个在通道队列里等待。
  // Two RX frames take the two IN slots, four more wait in the channel queue.
  for (unsigned i = 0; i < 6; ++i)
  {
    f.can0.Inject(0x100u + i, {static_cast<uint8_t>(i)});
  }
  // 三个主机帧发到总线上；它们的 echo 在占满的槽后面排队。
  // Three host frames are sent on the bus; their echoes queue behind the busy slots.
  for (unsigned i = 0; i < 3; ++i)
  {
    f.out.HostSends(ClassicFrame(0x3000u + i, 0x300u + i, 1, 0, 0, {0x5A}));
  }
  TEST_ASSERT(f.can0.tx.size() == 3);
  for (unsigned i = 0; i < 16 && f.in.active; ++i)
  {
    f.in.Complete();
  }

  TEST_ASSERT(f.in.sent.size() == 9);
  // 先是已在总线上的两帧，然后是所有 echo，最后是 RX 队列的其余部分。
  // The two frames already on the wire, then every echo, then the rest of the RX queue.
  for (unsigned i = 0; i < 2; ++i)
  {
    TEST_ASSERT(ReadU32(f.in.sent[i], 0) == 0xFFFFFFFFu);
    TEST_ASSERT(f.in.sent[i][WIRE_HEADER] == i);
  }
  for (unsigned i = 0; i < 3; ++i)
  {
    TEST_ASSERT(ReadU32(f.in.sent[2 + i], 0) == 0x3000u + i);
  }
  for (unsigned i = 0; i < 4; ++i)
  {
    TEST_ASSERT(ReadU32(f.in.sent[5 + i], 0) == 0xFFFFFFFFu);
    TEST_ASSERT(f.in.sent[5 + i][WIRE_HEADER] == 2 + i);
  }
}

void TestEchoBackpressure()
{
  FreshTimerList();
  GsFixture f(32, 4);  // 四个 echo 槽 / four echo slots
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f, true, false);

  // 两个 echo 占满两个 IN 槽，然后 echo 队列被填满；echo 排不进队列的那一帧保留其
  // OUT 租约，而不是丢弃 echo。
  // Two echoes fit the two IN slots, then the echo queue fills; the frame whose echo
  // cannot be queued holds its OUT lease instead of dropping the echo.
  unsigned sent = 0;
  while (f.out.active && sent < 16)
  {
    f.out.HostSends(ClassicFrame(0x2000u + sent, 0x200u + sent, 1, 0, 0, {0x5A}));
    ++sent;
  }
  // 背压：不再挂新的 OUT 接收 / Backpressure: no new OUT receive is armed.
  TEST_ASSERT(!f.out.active);
  TEST_ASSERT(sent >= 7);
  // 每帧恰好触发一次 CAN TX / The CAN TX happened exactly once per frame.
  TEST_ASSERT(f.can0.tx.size() == sent);

  // 释放 IN 槽后，服务推出被保留的 echo 并归还租约。
  // Freeing the IN slots lets the service push the held echo and return the lease.
  for (unsigned i = 0; i < 16 && !f.out.active; ++i)
  {
    f.in.Complete();
  }
  TEST_ASSERT(f.out.active);
}

void TestUserIdRoundTripAndDatabaseWrite()
{
  FreshTimerList();
  std::array<uint8_t, 8192> db_mem{};
  RamFlash db_flash(256, 8, {db_mem.data(), db_mem.size()});
  LibXR::DatabaseRawSequential db(db_flash, 256);
  db.Init();

  GsFixture f(32, 32, &db);
  f.gs.BindEndpoints(f.pool, 0, false);

  const uint32_t user_id = 0x11223344u;
  ConstRawData data{&user_id, sizeof(user_id)};
  TEST_ASSERT(f.gs.OnClassData(false, static_cast<uint8_t>(GsUsb::BReq::SET_USER_ID),
                               data) == ErrorCode::OK);

  // GET_USER_ID 立即读取 RAM 中的值（回调从不等待存储）。
  // GET_USER_ID reads the RAM value at once (the callback never waits for the storage).
  TestGs::ControlTransferResult result;
  TEST_ASSERT(f.gs.OnVendorRequest(false, static_cast<uint8_t>(GsUsb::BReq::GET_USER_ID),
                                   0, 4, 0, result) == ErrorCode::OK);
  TEST_ASSERT(result.write_data.size_ == 4U);
  uint32_t read_back = 0;
  std::memcpy(&read_back, result.write_data.addr_, sizeof(read_back));
  TEST_ASSERT(read_back == user_id);

  // 写入 Database 的是 100 ms 的 Timer 任务。
  // The 100 ms Timer task is what writes the Database.
  for (unsigned i = 0; i < 100; ++i)
  {
    Timer::Refresh();
  }
  LibXR::Database::Key<uint32_t> probe(db, "user_id", 0u);
  TEST_ASSERT(static_cast<uint32_t>(probe) == user_id);
}

void TestMultiChannelRoundRobin()
{
  FreshTimerList();
  GsFixture f;
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f);

  // 每通道四帧排在两个 IN 槽之后；总线上两个通道必须交替发送，而不是由某一个通道先发完。
  // Four frames per channel are queued behind the two IN slots; the channels must
  // alternate on the wire instead of one channel draining first.
  for (unsigned i = 0; i < 6; ++i)
  {
    f.can0.Inject(0x100u + i, {static_cast<uint8_t>(i)});
    f.can1.Inject(0x200u + i, {static_cast<uint8_t>(i)});
  }
  for (unsigned i = 0; i < 24 && f.in.active; ++i)
  {
    f.in.Complete();
  }
  TEST_ASSERT(f.in.sent.size() == 12);
  // 第一帧发出时另一个通道还没有排队的内容；此后两个通道交错发送，各通道保持自己的顺序。
  // The first frame went out before the other channel had anything queued; after that
  // the channels interleave and each channel keeps its own order.
  unsigned next[2] = {0, 0};
  unsigned max_run = 0, run = 0;
  uint8_t last_ch = 0xFF;
  for (const auto& frame : f.in.sent)
  {
    const uint8_t ch = frame[9];
    TEST_ASSERT(ch < 2);
    TEST_ASSERT(frame[WIRE_HEADER] == next[ch]);
    ++next[ch];
    run = (ch == last_ch) ? run + 1 : 1;
    max_run = std::max(max_run, run);
    last_ch = ch;
  }
  TEST_ASSERT(next[0] == 6 && next[1] == 6);
  TEST_ASSERT(max_run <= 2);
}

void TestErrorFrameHasOwnQueue()
{
  FreshTimerList();
  GsFixture f(2, 32);  // 每通道两个数据帧 / two data frames per channel
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f, true, false);

  // 通道 0 报告总线错误 / Channel 0 reports bus errors.
  TestGs::ControlTransferResult result;
  TEST_ASSERT(f.gs.OnVendorRequest(false, static_cast<uint8_t>(GsUsb::BReq::MODE), 0, 8,
                                   0, result) == ErrorCode::OK);
  const uint32_t mode[2] = {1u, GsUsb::GSCAN_MODE_BERR_REPORTING};
  ConstRawData mode_data{mode, sizeof(mode)};
  TEST_ASSERT(f.gs.OnClassData(false, static_cast<uint8_t>(GsUsb::BReq::MODE),
                               mode_data) == ErrorCode::OK);

  // 填满两个 IN 槽和数据队列，然后触发一个错误。错误帧仍必须通过，因为它有自己的队列。
  // Fill the two IN slots and the data queue, then raise an error. The error frame
  // must still get through because it has its own queue.
  for (unsigned i = 0; i < 6; ++i)
  {
    f.can0.Inject(0x100u + i, {static_cast<uint8_t>(i)});
  }
  f.can0.Inject(LibXR::CAN::FromErrorID(LibXR::CAN::ErrorID::CAN_ERROR_ID_BUS_OFF), {},
                LibXR::CAN::Type::ERROR);
  for (unsigned i = 0; i < 16 && f.in.active; ++i)
  {
    f.in.Complete();
  }

  unsigned errors = 0;
  for (const auto& frame : f.in.sent)
  {
    if ((ReadU32(frame, 4) & GsUsb::CAN_ERR_FLAG) != 0U)
    {
      ++errors;
    }
  }
  TEST_ASSERT(errors == 1);
}
}  // namespace

void test_usb_gsusb()
{
  TestBindAndConfiguration();
  TestHostFrameToCanAndEcho();
  TestCanRxToHost();
  TestRxQueueOverflowFlag();
  TestOverflowFlagRidesOnTheChannelsNextFrame();
  TestEchoHasPriority();
  TestEchoBackpressure();
  TestUserIdRoundTripAndDatabaseWrite();
  TestMultiChannelRoundRobin();
  TestErrorFrameHasOwnQueue();
}
