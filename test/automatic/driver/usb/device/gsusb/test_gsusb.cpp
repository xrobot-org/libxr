/**
 * @file test_gsusb.cpp
 * @brief gs_usb（CAN 适配器）设备类（槽模型）主机测试 /
 *        Host tests for the gs_usb (CAN adapter) device class.
 *
 * 覆盖槽模型迁移改动的路径：每通道一个 SPSC 接收队列加一个 echo 队列、
 * 服务里取槽打包提交、echo 与接收帧轮流发送、echo 队列满时保留 OUT 租约作背压、
 * RX 队列满时丢帧计数并在该通道发往主机的下一帧（含 echo）置
 * CAN_FLAG_OVERFLOW，以及 SET_USER_ID 只改 RAM 与待写标志、由 100 ms Timer
 * 任务落库。
 * Covers the paths the slot-model migration changed: one SPSC RX queue per channel
 * plus one echo queue, slot packing and submission in the class service, echoes and
 * received frames taking turns, the OUT lease held as backpressure while the echo queue
 * is full, dropped frames counted per channel with CAN_FLAG_OVERFLOW on that channel's
 * next frame to the host (echoes included), and SET_USER_ID only touching RAM plus a
 * pending flag with the 100 ms Timer task writing the Database.
 */

#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <string>
#include <type_traits>
#include <vector>

#include "../../mock_endpoint.hpp"
#include "can.hpp"
#include "device_composition.hpp"
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

/// When a test expects a fatal error in a forked child, the file that reported it goes
/// into this pipe and the child exits; otherwise the process aborts.
static int g_fatal_report_fd = -1;
extern "C" void libxr_fatal_error(const char* file, uint32_t line, bool)
{
  if (g_fatal_report_fd >= 0)
  {
    (void)!write(g_fatal_report_fd, file, std::strlen(file));
    _exit(42);
  }
  std::fprintf(stderr, "%s:%u: gs_usb test assertion failed\n", file, line);
  std::abort();
}

namespace
{
static constexpr auto TEST_LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "LibXR", "USB tests", "0");

constexpr std::size_t CHANNELS = 2;
using Gs = GsUsbClass<CHANNELS>;

/// Exposes the protected bind/request entry points of the class.
class TestGs : public Gs
{
 public:
  using Gs::BindEndpoints;
  using Gs::Gs;
  using Gs::OnClassData;
  using Gs::OnDeviceEvent;
  using Gs::OnVendorRequest;
  using Gs::UnbindEndpoints;
  using ControlTransferResult = DeviceClass::ControlTransferResult;
};

/// Give this test its own timer list so a class destroyed earlier cannot leave a
/// dangling task node behind for the next Timer::Refresh().
void FreshTimerList() { LibXR::Timer::list_ = new LibXR::LockFreeList(); }

/// Wire frame layout: echo_id, can_id, dlc, channel, flags, reserved, 8 data bytes.
constexpr std::size_t WIRE_HEADER = 12;
constexpr std::size_t WIRE_CLASSIC = WIRE_HEADER + 8;

/// A transmit queue that can be full: it takes `capacity` frames, answers FULL after
/// that, and `Drain` models frames leaving the queue on the bus.
struct TxQueueModel
{
  size_t capacity = SIZE_MAX;
  size_t queued = 0;
  unsigned refused = 0;  ///< how many AddMessage calls were answered FULL

  bool Take()
  {
    if (queued >= capacity)
    {
      ++refused;
      return false;
    }
    ++queued;
    return true;
  }
  void Drain(size_t n) { queued -= std::min(n, queued); }
};

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
    if (!queue.Take()) return ErrorCode::FULL;
    Tx entry{};
    entry.id = pack.id;
    entry.dlc = pack.dlc;
    std::memcpy(entry.data, pack.data, 8);
    tx.push_back(entry);
    return ErrorCode::OK;
  }

  /// Play a received frame into the class through its registered subscribers.
  void Inject(uint32_t id, const std::vector<uint8_t>& data,
              LibXR::CAN::Type type = LibXR::CAN::Type::STANDARD)
  {
    ClassicPack pack{};
    pack.id = id;
    pack.type = type;
    pack.dlc = static_cast<uint8_t>(data.size());
    std::memcpy(pack.data, data.data(), data.size());
    OnMessage(pack, false);
  }

  unsigned configures = 0;
  std::vector<Tx> tx;  ///< the frames the controller accepted, in order
  TxQueueModel queue;
};

/// The same for a CAN FD controller: records the accepted frames as (id, length, first
/// data byte).
class StubFdCan : public LibXR::FDCAN
{
 public:
  struct Tx
  {
    uint32_t id;
    uint8_t len;
    uint8_t first;
  };

  ErrorCode SetConfig(const LibXR::CAN::Configuration&) override { return ErrorCode::OK; }
  ErrorCode SetConfig(const LibXR::FDCAN::Configuration&) override
  {
    return ErrorCode::OK;
  }
  uint32_t GetClockFreq() const override { return 42000000u; }
  ErrorCode AddMessage(const ClassicPack& pack) override
  {
    if (!queue.Take()) return ErrorCode::FULL;
    tx.push_back({pack.id, pack.dlc, pack.data[0]});
    return ErrorCode::OK;
  }
  ErrorCode AddMessage(const FDPack& pack) override
  {
    if (!queue.Take()) return ErrorCode::FULL;
    tx.push_back({pack.id, pack.len, pack.data[0]});
    return ErrorCode::OK;
  }

  std::vector<Tx> tx;
  TxQueueModel queue;
};

/// RAM-backed Flash for the Database (erase fills with 0xFF, write copies).
class RamFlash : public LibXR::Flash
{
 public:
  RamFlash(size_t min_erase, size_t min_write, RawData area)
      : Flash(min_erase, min_write, area), area_(area)
  {
  }
  ErrorCode Erase(size_t offset, size_t size) override
  {
    std::memset(static_cast<uint8_t*>(area_.addr_) + offset, 0xFF, size);
    return ErrorCode::OK;
  }
  ErrorCode Write(size_t offset, ConstRawData data) override
  {
    std::memcpy(static_cast<uint8_t*>(area_.addr_) + offset, data.addr_, data.size_);
    return ErrorCode::OK;
  }

 private:
  RawData area_;
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

/// Two CAN channels, the class and the composition that gives it a service pool.
template <typename Can>
struct GsFixtureT
{
  /// The class constructor that is picked: FDCAN controllers offer CAN FD, plain CAN
  /// controllers do not.
  using Interface =
      std::conditional_t<std::is_base_of_v<LibXR::FDCAN, Can>, LibXR::FDCAN, LibXR::CAN>;
  Block<256> in_storage, out_storage;
  EndpointPool pool{4};
  MockEndpoint in{Number::EP1, Direction::IN, in_storage.Raw()};
  MockEndpoint out{Number::EP2, Direction::OUT, out_storage.Raw()};
  Can can0, can1;
  TestGs gs;
  DeviceComposition composition;
  Callback<uint32_t> handler;

  explicit GsFixtureT(size_t rx_queue_size = 32, size_t echo_queue_size = 32,
                      LibXR::Database* database = nullptr,
                      size_t tx_stage_size = Gs::DEFAULT_TX_STAGE_SIZE)
      : gs(Number::EP1, Number::EP2,
           {static_cast<Interface*>(&can0), static_cast<Interface*>(&can1)},
           rx_queue_size, echo_queue_size, nullptr, {}, database,
           Gs::DEFAULT_INTERFACE_STRING, tx_stage_size),
        composition(pool, {&TEST_LANGUAGE}, {{&gs}})
  {
    TEST_ASSERT(pool.Put(&in) == ErrorCode::OK);
    TEST_ASSERT(pool.Put(&out) == ErrorCode::OK);
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
using GsFixture = GsFixtureT<StubCan>;
using GsFdFixture = GsFixtureT<StubFdCan>;

/// Host format negotiation (little-endian marker) and per-channel mode. Every request
/// with a data stage needs its vendor request first: that is what selects the channel.
template <typename Fixture>
void Configure(Fixture& f, bool enable_ch0 = true, bool enable_ch1 = true)
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

/// One MODE request (vendor request that selects the channel, then its data stage).
template <typename Fixture>
void SetMode(Fixture& f, uint16_t ch, uint32_t mode_value, uint32_t flags = 0U)
{
  TestGs::ControlTransferResult result;
  TEST_ASSERT(f.gs.OnVendorRequest(false, static_cast<uint8_t>(GsUsb::BReq::MODE), ch,
                                   2u * sizeof(uint32_t), 0, result) == ErrorCode::OK);
  const uint32_t mode[2] = {mode_value, flags};
  ConstRawData data{mode, sizeof(mode)};
  TEST_ASSERT(f.gs.OnClassData(false, static_cast<uint8_t>(GsUsb::BReq::MODE), data) ==
              ErrorCode::OK);
}

void TestBindAndConfiguration()
{
  FreshTimerList();
  GsFixture f;
  f.gs.BindEndpoints(f.pool, 0, false);

  // One whole slot per OUT lease (receive_size 0): a frame arrives in one request.
  TEST_ASSERT(f.out.active && f.out.Last().length == 128);
  TEST_ASSERT(f.in.starts.empty());

  // The data stage of HOST_FORMAT / MODE reaches the class through OnClassData.
  const uint32_t byte_order = 0x0000beefu;
  ConstRawData fmt{&byte_order, sizeof(byte_order)};
  TEST_ASSERT(f.gs.OnClassData(false, static_cast<uint8_t>(GsUsb::BReq::HOST_FORMAT),
                               fmt) == ErrorCode::OK);
  const uint32_t mode[2] = {1u, 0u};  // START on channel 0
  ConstRawData mode_data{mode, sizeof(mode)};
  TEST_ASSERT(f.gs.OnClassData(false, static_cast<uint8_t>(GsUsb::BReq::MODE),
                               mode_data) == ErrorCode::OK);
  TEST_ASSERT(f.can0.configures == 1);

  // MODE for an out-of-range channel is refused before any data stage.
  TestGs::ControlTransferResult result;
  TEST_ASSERT(f.gs.OnVendorRequest(false, static_cast<uint8_t>(GsUsb::BReq::MODE), 9, 8,
                                   0, result) == ErrorCode::ARG_ERR);
  // The mode data stage must be exactly one DeviceMode.
  const uint32_t short_mode[1] = {1u};
  ConstRawData short_data{short_mode, sizeof(short_mode)};
  TEST_ASSERT(f.gs.OnClassData(false, static_cast<uint8_t>(GsUsb::BReq::MODE),
                               short_data) == ErrorCode::ARG_ERR);
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

  // The frame reached the CAN controller once, and the echo went back on the IN endpoint.
  TEST_ASSERT(f.can0.tx.size() == 1);
  TEST_ASSERT(f.can0.tx[0].id == 0x123u && f.can0.tx[0].dlc == 3);
  TEST_ASSERT(std::equal(payload.begin(), payload.end(), f.can0.tx[0].data));
  TEST_ASSERT(f.in.active && f.in.Last().length == WIRE_CLASSIC);
  const auto& echo = f.in.Last().data;
  TEST_ASSERT(ReadU32(echo, 0) == 0x1234u);  // the host's echo_id comes back
  TEST_ASSERT(ReadU32(echo, 4) == 0x123u);
  TEST_ASSERT(echo[8] == 3 && echo[9] == 0 && echo[10] == 0);
  TEST_ASSERT(std::equal(payload.begin(), payload.end(), echo.begin() + WIRE_HEADER));
  // The lease went back, so the OUT receive re-armed.
  TEST_ASSERT(f.out.active);
  f.in.Complete();

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
  TEST_ASSERT(ReadU32(frame, 0) == 0xFFFFFFFFu);  // RX frames carry ECHO_ID_RX
  TEST_ASSERT(ReadU32(frame, 4) == 0x456u);
  TEST_ASSERT(frame[8] == 2 && frame[9] == 0);
  TEST_ASSERT(frame[10] == 0);
  TEST_ASSERT(std::equal(payload.begin(), payload.end(), frame.begin() + WIRE_HEADER));
  f.in.Complete();
}

/// 通道启动时丢弃上一次会话留下的接收帧、错误帧和这个通道的回显；其他通道的帧和回显
/// 保留且顺序不变。已经交给 IN 端点的两帧照常发出。
/// A channel start drops the receive frames, error frames and this channel's echoes that
/// the previous session left; the frames and echoes of the other channel stay, in order.
/// The two frames already handed to the IN endpoint still go out.
void TestChannelStartDropsStaleFrames()
{
  FreshTimerList();
  GsFixture f;
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f);
  SetMode(f, 0, 1u, GsUsb::GSCAN_MODE_BERR_REPORTING);

  // Channel 0: six frames (two reach the IN slots, four wait) and a bus error.
  for (uint32_t i = 0; i < 6; ++i) f.can0.Inject(0x100u + i, {static_cast<uint8_t>(i)});
  f.can0.Inject(LibXR::CAN::FromErrorID(LibXR::CAN::ErrorID::CAN_ERROR_ID_BUS_OFF), {},
                LibXR::CAN::Type::ERROR);
  // Channel 1: two received frames.
  f.can1.Inject(0x200u, {0x20});
  f.can1.Inject(0x201u, {0x21});
  // Echoes of both channels wait behind the busy slots: ids 0x3000.. and 0x4000...
  for (uint32_t i = 0; i < 2; ++i)
  {
    f.out.HostSends(ClassicFrame(0x3000u + i, 0x300u + i, 1, 0, 0, {0x5A}));
    f.out.HostSends(ClassicFrame(0x4000u + i, 0x400u + i, 1, 1, 0, {0x5B}));
  }

  // The host restarts channel 0 (RESET, then START), as `ip link set down/up` does.
  SetMode(f, 0, 0u);
  SetMode(f, 0, 1u);

  // A frame received after the start goes out.
  f.can0.Inject(0x500u, {0x50});

  std::vector<uint32_t> rx0, rx1, echo0, echo1;
  unsigned errors = 0;
  for (unsigned i = 0; i < 32 && f.in.active; ++i) f.in.Complete();
  for (const auto& frame : f.in.sent)
  {
    const uint32_t echo_id = ReadU32(frame, 0);
    const uint32_t can_id = ReadU32(frame, 4);
    if ((can_id & GsUsb::CAN_ERR_FLAG) != 0U) ++errors;
    if (echo_id == 0xFFFFFFFFu)
      (frame[9] == 0 ? rx0 : rx1).push_back(can_id);
    else
      (frame[9] == 0 ? echo0 : echo1).push_back(echo_id);
  }
  // Channel 0: the two frames that were already in the slots, then the new one. The
  // error frame, the four queued frames and both channel 0 echoes are gone.
  TEST_ASSERT(rx0 == std::vector<uint32_t>({0x100u, 0x101u, 0x500u}));
  TEST_ASSERT(echo0.empty());
  TEST_ASSERT(errors == 0);
  // Channel 1 is untouched.
  TEST_ASSERT(rx1 == std::vector<uint32_t>({0x200u, 0x201u}));
  TEST_ASSERT(echo1 == std::vector<uint32_t>({0x4000u, 0x4001u}));
}

/// 总线复位或拔插之后，上一次绑定留下的队列内容在主机启动通道时丢弃，不会在新会话里
/// 先到达主机。
/// After a bus reset or replug the queue contents of the previous binding are dropped
/// when the host starts the channel, so they do not reach the host first in the new
/// session.
void TestRebindThenStartDropsStaleFrames()
{
  FreshTimerList();
  GsFixture f;
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f);

  for (uint32_t i = 0; i < 6; ++i) f.can0.Inject(0x100u + i, {static_cast<uint8_t>(i)});
  for (uint32_t i = 0; i < 2; ++i)
  {
    f.out.HostSends(ClassicFrame(0x3000u + i, 0x300u + i, 1, 0, 0, {0x5A}));
  }

  // Bus reset: the binding ends and a new one begins; the queues live in the class.
  f.gs.UnbindEndpoints(f.pool, false);
  f.gs.BindEndpoints(f.pool, 0, false);
  f.in.sent.clear();

  Configure(f);
  f.can0.Inject(0x500u, {0x50});
  for (unsigned i = 0; i < 32 && f.in.active; ++i) f.in.Complete();

  TEST_ASSERT(f.in.sent.size() == 1);
  TEST_ASSERT(ReadU32(f.in.sent[0], 0) == 0xFFFFFFFFu);
  TEST_ASSERT(ReadU32(f.in.sent[0], 4) == 0x500u);
}

void TestRxQueueOverflowFlag()
{
  FreshTimerList();
  GsFixture f(4, 32);  // four frames per channel
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f, true, false);

  // Two IN slots are in flight and the four-deep RX queue fills, so the seventh frame
  // has nowhere to go and is dropped.
  for (unsigned i = 0; i < 7; ++i)
  {
    f.can0.Inject(0x100u + i, {static_cast<uint8_t>(i)});
  }
  // Drain: two frames are on the wire, the rest follow once slots free up.
  for (unsigned i = 0; i < 12 && f.in.active; ++i)
  {
    f.in.Complete();
  }

  TEST_ASSERT(f.in.sent.size() == 6);  // seven injected, one dropped
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
  // The drop is reported exactly once, on that channel's next frame.
  TEST_ASSERT(overflow == 1);
}

/// The Linux host checks GS_CAN_FLAG_OVERFLOW after handling any frame, an echo
/// included, and counts it for that frame's channel. A drop is therefore reported on
/// the channel's next frame to the host even when that frame is an echo, and never on
/// another channel's frame.
void TestOverflowFlagRidesOnTheChannelsNextFrame()
{
  FreshTimerList();
  GsFixture f(4, 32);  // four frames per channel
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f);

  // Two IN slots in flight, four frames queued, the seventh dropped.
  for (unsigned i = 0; i < 7; ++i)
  {
    f.can0.Inject(0x100u + i, {static_cast<uint8_t>(i)});
  }
  // Echoes for channel 0 and channel 1 queue next; it is the echoes' turn first.
  f.out.HostSends(ClassicFrame(0x20u, 0x300u, 1, 0, 0, {0x22}));
  f.out.HostSends(ClassicFrame(0x10u, 0x301u, 1, 1, 0, {0x11}));
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
    // Channel 0's echo is its first frame after the drop; it keeps its echo_id.
    TEST_ASSERT(ReadU32(frame, 0) == 0x20u && frame[9] == 0);
  }
  TEST_ASSERT(overflow == 1);
}

/// 同时有 echo 和接收帧待发时轮流发送，只有一类时直接发送；每一类内部保持顺序。
/// Echoes and received frames alternate while both are pending and go out directly when
/// only one kind is; each kind keeps its own order.
void TestEchoAndReceivedFramesAlternate()
{
  FreshTimerList();
  GsFixture f;
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f, true, false);

  // Two RX frames take the two IN slots, four more wait in the channel queue.
  for (unsigned i = 0; i < 6; ++i)
  {
    f.can0.Inject(0x100u + i, {static_cast<uint8_t>(i)});
  }
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
  // The two frames already on the wire, then echo and RX frame take turns (the echo
  // first, as the last frame sent was a received one), then the RX frames left over.
  for (unsigned i = 0; i < 2; ++i)
  {
    TEST_ASSERT(ReadU32(f.in.sent[i], 0) == 0xFFFFFFFFu);
    TEST_ASSERT(f.in.sent[i][WIRE_HEADER] == i);
  }
  for (unsigned i = 0; i < 3; ++i)
  {
    TEST_ASSERT(ReadU32(f.in.sent[2 + 2 * i], 0) == 0x3000u + i);
    TEST_ASSERT(ReadU32(f.in.sent[3 + 2 * i], 0) == 0xFFFFFFFFu);
    TEST_ASSERT(f.in.sent[3 + 2 * i][WIRE_HEADER] == 2 + i);
  }
  TEST_ASSERT(ReadU32(f.in.sent[8], 0) == 0xFFFFFFFFu);
  TEST_ASSERT(f.in.sent[8][WIRE_HEADER] == 5);
}

/// 主机突发：一个 CAN 总线每毫秒只送出几帧，USB 每毫秒只有固定的批量事务数。每发一帧要
/// 占 OUT、echo 和（总线送回来之后）接收帧三个事务；echo 一律优先时接收帧在突发期间得不
/// 到 IN 事务，32 项的通道 1
/// 接收队列溢出丢帧；轮流发送时事务数刚好够，一帧不丢、顺序不变。 A host burst: a CAN bus
/// that delivers only a few frames per millisecond and a fixed number of bulk
/// transactions per millisecond. Each frame sent costs an OUT, an echo and (once the bus
/// returns it) a received frame transaction; with echoes always first the received frames
/// get no IN transaction during the burst and the 32-entry channel 1 RX queue overflows;
/// with alternation the budget just suffices and no frame is lost or reordered.
void TestHostBurstOnARateLimitedBus()
{
  FreshTimerList();
  GsFixture f;
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f);

  constexpr unsigned FRAMES = 600;
  constexpr unsigned BUS_PER_MS = 8;    // frames the CAN bus returns per millisecond
  constexpr unsigned BULK_PER_MS = 24;  // bulk transactions a full-speed frame holds
  unsigned sent = 0, delivered = 0;
  std::vector<uint32_t> got;  // can_id of every non-echo frame on channel 1
  bool overflow = false;
  unsigned idle = 0;
  for (unsigned ms = 0; ms < 4000 && idle < 20; ++ms)
  {
    for (unsigned i = 0; i < BUS_PER_MS && delivered < f.can0.tx.size(); ++i, ++delivered)
    {
      const auto& tx = f.can0.tx[delivered];
      f.can1.Inject(tx.id, std::vector<uint8_t>(tx.data, tx.data + tx.dlc));
    }
    unsigned budget = BULK_PER_MS;
    bool moved = false;
    while (budget > 0)
    {
      bool progress = false;
      if (f.in.active)
      {
        f.in.Complete();
        const auto& frame = f.in.sent.back();
        if (frame[9] == 1 && ReadU32(frame, 0) == 0xFFFFFFFFu)
          got.push_back(ReadU32(frame, 4));
        if ((frame[10] & GsUsb::CAN_FLAG_OVERFLOW) != 0U) overflow = true;
        --budget;
        progress = true;
      }
      if (budget > 0 && f.out.active && sent < FRAMES)
      {
        f.out.HostSends(
            ClassicFrame(sent, 0x100u + sent, 1, 0, 0, {static_cast<uint8_t>(sent)}));
        ++sent;
        --budget;
        progress = true;
      }
      if (!progress) break;
      moved = true;
    }
    idle = moved ? 0 : idle + 1;
  }
  TEST_ASSERT(sent == FRAMES);
  TEST_ASSERT(!overflow);
  TEST_ASSERT(got.size() == FRAMES);
  for (unsigned i = 0; i < FRAMES; ++i) TEST_ASSERT(got[i] == 0x100u + i);
}

void TestEchoBackpressure()
{
  FreshTimerList();
  GsFixture f(32, 4);  // four echo slots
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f, true, false);

  // Two echoes fit the two IN slots, then the echo queue fills; the frame whose echo
  // cannot be queued holds its OUT lease instead of dropping the echo.
  unsigned sent = 0;
  while (f.out.active && sent < 16)
  {
    f.out.HostSends(ClassicFrame(0x2000u + sent, 0x200u + sent, 1, 0, 0, {0x5A}));
    ++sent;
  }
  TEST_ASSERT(!f.out.active);  // backpressure: no new OUT receive is armed
  TEST_ASSERT(sent >= 7);
  TEST_ASSERT(f.can0.tx.size() == sent);  // the CAN TX happened exactly once per frame

  // Freeing the IN slots lets the service push the held echo and return the lease.
  for (unsigned i = 0; i < 16 && !f.out.active; ++i)
  {
    f.in.Complete();
  }
  TEST_ASSERT(f.out.active);
}

/// 归还租约重挂的接收算作在途：之后的 ENDPOINT_RESUMED 不再挂第二条，否则两个完成都被
/// 压住时后到的覆盖前一个。
/// A receive rearmed by returning the lease counts as in flight: a later
/// ENDPOINT_RESUMED arms no second one; otherwise, with both completions held, the later
/// one overwrites the earlier.
void TestResumeAfterReturnedLeaseArmsNothing()
{
  FreshTimerList();
  GsFixture f;
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f, true, false);
  f.out.HostSends(ClassicFrame(0x3000u, 0x300u, 1, 0, 0, {0x01}));
  TEST_ASSERT(f.out.active);  // the returned lease rearmed the receive
  f.gs.OnDeviceEvent(false, DeviceEvent::ENDPOINT_RESUMED,
                     static_cast<uint8_t>(f.out.GetAddress()));
  // One receive in flight: the other slot is still free.
  TEST_ASSERT(f.out.Receive(f.out.GetGranted().slot_capacity, false) == ErrorCode::OK);
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

  // GET_USER_ID reads the RAM value at once (the callback never waits for the storage).
  TestGs::ControlTransferResult result;
  TEST_ASSERT(f.gs.OnVendorRequest(false, static_cast<uint8_t>(GsUsb::BReq::GET_USER_ID),
                                   0, 4, 0, result) == ErrorCode::OK);
  TEST_ASSERT(result.write_data.size_ == 4U);
  uint32_t read_back = 0;
  std::memcpy(&read_back, result.write_data.addr_, sizeof(read_back));
  TEST_ASSERT(read_back == user_id);

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
  GsFixture f(2, 32);  // two data frames per channel
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f, true, false);

  // Channel 0 reports bus errors.
  TestGs::ControlTransferResult result;
  TEST_ASSERT(f.gs.OnVendorRequest(false, static_cast<uint8_t>(GsUsb::BReq::MODE), 0, 8,
                                   0, result) == ErrorCode::OK);
  const uint32_t mode[2] = {1u, GsUsb::GSCAN_MODE_BERR_REPORTING};
  ConstRawData mode_data{mode, sizeof(mode)};
  TEST_ASSERT(f.gs.OnClassData(false, static_cast<uint8_t>(GsUsb::BReq::MODE),
                               mode_data) == ErrorCode::OK);

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
/// The echo ids of the frames the class has sent to the host so far, in order (frames
/// from the CAN bus carry ECHO_ID_RX and are left out).
template <typename Fixture>
std::vector<uint32_t> EchoIds(Fixture& f)
{
  std::vector<uint32_t> ids;
  for (const auto& frame : f.in.sent)
  {
    if (ReadU32(frame, 0) != 0xFFFFFFFFu) ids.push_back(ReadU32(frame, 0));
  }
  return ids;
}

/// The echo ids in [low, high) (one channel uses one id range in these tests).
template <typename Fixture>
std::vector<uint32_t> EchoIdsIn(Fixture& f, uint32_t low, uint32_t high)
{
  std::vector<uint32_t> ids;
  for (uint32_t id : EchoIds(f))
  {
    if (id >= low && id < high) ids.push_back(id);
  }
  return ids;
}

/// The ids 'first', 'first' + 1, ... ('count' of them).
std::vector<uint32_t> Run(uint32_t first, unsigned count)
{
  std::vector<uint32_t> ids;
  for (unsigned i = 0; i < count; ++i) ids.push_back(first + i);
  return ids;
}

/// Deliver the IN transfer that is waiting, if any (the host reads the echo).
template <typename Fixture>
void HostReadsIn(Fixture& f)
{
  if (f.in.active) f.in.Complete();
}

/// One SOF tick: the frame-tick callback the device core delivers only while the class
/// wants it.
template <typename Fixture>
void SofTick(Fixture& f)
{
  if (f.gs.NeedsFrameTick()) f.gs.OnFrameTick(false, FrameTick{});
}

/// A host frame with one data byte on channel `ch`: the CAN id is `echo_id & 0x7FF`.
std::vector<uint8_t> HostFrame(uint32_t echo_id, uint8_t ch)
{
  return ClassicFrame(echo_id, echo_id & 0x7FFu, 1, ch, 0,
                      {static_cast<uint8_t>(echo_id)});
}

/// 主机帧多于 CAN 发送队列容量：主机按 Linux 驱动的方式每通道最多 10 帧在途（收到 echo
/// 才发下一帧）。暂存的帧一帧不丢、不重复、顺序不变，每帧恰好一个 echo，echo 不会跑到
/// 控制器接受那一帧之前，而且 OUT 端点从不被占住。
/// A host burst larger than the CAN transmit queue, sent the way the Linux driver does:
/// at most 10 frames in flight per channel (the next frame goes out after an echo). The
/// staged frames are not lost or duplicated, the order is kept, every frame gets exactly
/// one echo, an echo never runs ahead of the controller accepting its frame, and the OUT
/// endpoint is never held.
void TestFullCanQueueStagesTheFrames()
{
  FreshTimerList();
  GsFixture f;
  f.can0.queue.capacity = 3;
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f);
  TEST_ASSERT(!f.gs.NeedsFrameTick());

  constexpr unsigned FRAMES = 100, WINDOW = 10;
  unsigned sent = 0, staged_ticks = 0;
  size_t max_unechoed = 0;
  bool done = false;
  for (unsigned guard = 0; guard < 20000 && !done; ++guard)
  {
    HostReadsIn(f);
    const size_t echoed = EchoIds(f).size();
    TEST_ASSERT(echoed <= f.can0.tx.size());  // an echo means the driver accepted it
    if (sent < FRAMES && sent - echoed < WINDOW)
    {
      TEST_ASSERT(f.out.active);  // the OUT endpoint is never held
      f.out.HostSends(HostFrame(0x1000u + sent, 0));
      ++sent;
      max_unechoed = std::max<size_t>(max_unechoed, sent - EchoIds(f).size());
      continue;
    }
    if (sent == FRAMES && echoed == FRAMES && !f.in.active)
    {
      done = true;
      break;
    }
    if (f.gs.NeedsFrameTick()) ++staged_ticks;
    f.can0.queue.Drain(1);  // one frame leaves on the bus
    SofTick(f);
  }
  TEST_ASSERT(done);
  TEST_ASSERT(staged_ticks > 0);
  TEST_ASSERT(f.can0.queue.refused > 0);
  TEST_ASSERT(max_unechoed <= WINDOW);
  TEST_ASSERT(f.can0.tx.size() == FRAMES);  // each frame accepted once, none lost
  TEST_ASSERT(EchoIds(f) == Run(0x1000u, FRAMES));
  for (unsigned i = 0; i < FRAMES; ++i)
  {
    TEST_ASSERT(f.can0.tx[i].id == ((0x1000u + i) & 0x7FFu) &&
                f.can0.tx[i].data[0] == static_cast<uint8_t>(0x1000u + i));
  }
  TEST_ASSERT(f.out.active);
  TEST_ASSERT(!f.gs.NeedsFrameTick());
  TEST_ASSERT(f.can1.tx.empty());
}

/// 新帧不会越过暂存的帧：暂存队列非空时，即使驱动已有空位，新帧也排在队尾。
/// A new frame never overtakes a staged one: while the staging queue is not empty, the
/// new frame goes to its end even if the driver has room.
void TestNewFrameQueuesBehindStagedFrames()
{
  FreshTimerList();
  GsFixture f;
  f.can0.queue.capacity = 1;
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f);

  f.out.HostSends(HostFrame(0x10u, 0));  // A: accepted
  f.out.HostSends(HostFrame(0x11u, 0));  // B: the driver is full, staged
  TEST_ASSERT(f.out.active);
  TEST_ASSERT(f.can0.tx.size() == 1 && f.gs.NeedsFrameTick());

  f.can0.queue.Drain(1);                 // room for exactly one frame, no service yet
  f.out.HostSends(HostFrame(0x12u, 0));  // C arrives behind B
  TEST_ASSERT(f.out.active);
  // B (not C) took the room, C waits behind it.
  TEST_ASSERT(f.can0.tx.size() == 2 && f.can0.tx[1].id == 0x11u);
  TEST_ASSERT(f.gs.NeedsFrameTick());

  f.can0.queue.Drain(1);
  SofTick(f);
  while (f.in.active) f.in.Complete();
  TEST_ASSERT(f.can0.tx.size() == 3 && f.can0.tx[2].id == 0x12u);
  TEST_ASSERT((EchoIds(f) == std::vector<uint32_t>{0x10u, 0x11u, 0x12u}));
  TEST_ASSERT(!f.gs.NeedsFrameTick());
}

/// 重试不只靠 SOF 节拍：收到总线帧、IN 完成而请求的类服务也会重试。
/// The retry does not depend on the SOF tick alone: the class service that a received
/// bus frame or an IN completion requests retries as well.
void TestStagedFrameIsRetriedByTheService()
{
  FreshTimerList();
  GsFixture f;
  f.can0.queue.capacity = 1;
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f);

  f.out.HostSends(HostFrame(0x10u, 0));
  f.out.HostSends(HostFrame(0x11u, 0));
  TEST_ASSERT(f.out.active && f.gs.NeedsFrameTick());
  TEST_ASSERT(f.can0.tx.size() == 1);

  f.can0.queue.Drain(1);
  f.can1.Inject(0x7FFu, {0x00});  // a frame from the bus requests the service
  TEST_ASSERT(f.can0.tx.size() == 2 && f.can0.tx[1].id == 0x11u);
  TEST_ASSERT(!f.gs.NeedsFrameTick());
  while (f.in.active) f.in.Complete();
  TEST_ASSERT((EchoIds(f) == std::vector<uint32_t>{0x10u, 0x11u}));
}

/// 通道 0 的发送队列永远满，通道 1 照常发送：通道 1 不受影响，OUT 端点一直可收；通道 0
/// 恰好有“暂存容量”个没有 echo 的帧（主机在这里停下），故障消除后按顺序发出并回 echo。
/// Channel 0's transmit queue stays full forever while channel 1 keeps sending: channel 1
/// is not affected and the OUT endpoint stays armed; channel 0 has exactly the staging
/// capacity of unechoed frames (where the host stops), and after the fault is gone they
/// go out in order and are echoed.
void TestStuckChannelDoesNotBlockTheOtherChannel()
{
  FreshTimerList();
  GsFixture f;
  f.can0.queue.capacity = 0;  // channel 0 never accepts a frame
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f);

  constexpr unsigned WINDOW = 10;  // the host's in-flight limit == the staging capacity
  for (unsigned i = 0; i < WINDOW; ++i)
  {
    TEST_ASSERT(f.out.active);
    f.out.HostSends(HostFrame(0x1000u + i, 0));
  }
  TEST_ASSERT(f.out.active);
  TEST_ASSERT(f.can0.tx.empty() && f.gs.NeedsFrameTick());

  for (unsigned i = 0; i < 200; ++i)
  {
    HostReadsIn(f);
    TEST_ASSERT(f.out.active);
    f.out.HostSends(HostFrame(0x2000u + i, 1));
    if ((i % 8) == 0) SofTick(f);  // the retries of channel 0 are refused
  }
  while (f.in.active) f.in.Complete();

  TEST_ASSERT(f.can1.tx.size() == 200);
  TEST_ASSERT(EchoIdsIn(f, 0x2000u, 0x3000u) == Run(0x2000u, 200));
  TEST_ASSERT(f.can0.tx.empty());
  TEST_ASSERT(EchoIdsIn(f, 0x1000u, 0x2000u).empty());  // exactly WINDOW unechoed
  TEST_ASSERT(f.gs.NeedsFrameTick());
  TEST_ASSERT(f.out.active);

  f.can0.queue.capacity = SIZE_MAX;  // the fault is gone
  SofTick(f);
  while (f.in.active) f.in.Complete();
  TEST_ASSERT(f.can0.tx.size() == WINDOW);
  for (unsigned i = 0; i < WINDOW; ++i)
  {
    TEST_ASSERT(f.can0.tx[i].id == ((0x1000u + i) & 0x7FFu));
  }
  TEST_ASSERT(EchoIdsIn(f, 0x1000u, 0x2000u) == Run(0x1000u, WINDOW));
  TEST_ASSERT(!f.gs.NeedsFrameTick());
}

/// 暂存队列满（主机不遵守每通道在途上限）：多出的帧被丢弃，仍然回 echo，
/// OUT 端点不被占住；暂存的帧不受影响，之后按顺序发出，被丢弃的帧不会被发送。
/// The staging queue is full (a host that ignores the per-channel in-flight limit): the
/// extra frames are dropped and still echoed, the OUT endpoint is not held; the staged
/// frames are not affected and go out in order later, the dropped frames are never sent.
void TestStagingOverflowDropsAndEchoes()
{
  FreshTimerList();
  GsFixture f(32, 32, nullptr, 4);  // four staged frames per channel
  f.can0.queue.capacity = 0;
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f);

  for (unsigned i = 0; i < 7; ++i)
  {
    HostReadsIn(f);
    TEST_ASSERT(f.out.active);
    f.out.HostSends(HostFrame(0x1000u + i, 0));
  }
  while (f.in.active) f.in.Complete();
  TEST_ASSERT(f.out.active);
  TEST_ASSERT(f.can0.tx.empty());
  // The three frames beyond the capacity are echoed at once (and dropped).
  TEST_ASSERT((EchoIds(f) == std::vector<uint32_t>{0x1004u, 0x1005u, 0x1006u}));

  f.can0.queue.capacity = SIZE_MAX;
  SofTick(f);
  while (f.in.active) f.in.Complete();
  TEST_ASSERT(f.can0.tx.size() == 4);
  for (unsigned i = 0; i < 4; ++i)
  {
    TEST_ASSERT(f.can0.tx[i].id == ((0x1000u + i) & 0x7FFu));
  }
  TEST_ASSERT((EchoIds(f) == std::vector<uint32_t>{0x1004u, 0x1005u, 0x1006u, 0x1000u,
                                                   0x1001u, 0x1002u, 0x1003u}));
  TEST_ASSERT(!f.gs.NeedsFrameTick());

  // Channel 1 has its own staging queue: channel 0's overflow did not touch it.
  f.can1.queue.capacity = 0;
  for (unsigned i = 0; i < 4; ++i) f.out.HostSends(HostFrame(0x2000u + i, 1));
  TEST_ASSERT(f.out.active && f.can1.tx.empty());
  f.can1.queue.capacity = SIZE_MAX;
  SofTick(f);
  TEST_ASSERT(f.can1.tx.size() == 4);
}

/// 通道复位、重新启动或总线复位（解绑）时，该通道暂存的帧被丢弃：不发送，
/// 不回 echo。另一个通道的暂存帧不受影响（解绑除外：整个绑定结束）。
/// A channel reset, a restart of the channel or a bus reset (unbind) drops the frames
/// staged for it: never sent, no echo. The other channel's staged frames are not touched
/// (an unbind ends the whole binding).
void TestChannelStopDropsItsStagedFrames()
{
  for (unsigned how = 0; how < 3; ++how)  // 0: MODE RESET, 1: MODE START, 2: unbind
  {
    FreshTimerList();
    GsFixture f;
    f.can0.queue.capacity = 0;
    f.can1.queue.capacity = 0;
    f.gs.BindEndpoints(f.pool, 0, false);
    Configure(f);

    for (unsigned i = 0; i < 3; ++i)
    {
      f.out.HostSends(HostFrame(0x1000u + i, 0));
      f.out.HostSends(HostFrame(0x2000u + i, 1));
    }
    TEST_ASSERT(f.out.active && f.gs.NeedsFrameTick());

    if (how == 0) SetMode(f, 1, 0);
    if (how == 1) SetMode(f, 1, 1);
    if (how == 2)
    {
      f.gs.UnbindEndpoints(f.pool, false);
      TEST_ASSERT(!f.gs.NeedsFrameTick());
      f.gs.BindEndpoints(f.pool, 0, false);
      Configure(f);
    }
    TEST_ASSERT(f.out.active);
    TEST_ASSERT(f.gs.NeedsFrameTick() == (how != 2));

    // Both queues free up: only channel 0's frames of a surviving binding go out.
    f.can0.queue.capacity = SIZE_MAX;
    f.can1.queue.capacity = SIZE_MAX;
    SofTick(f);
    f.gs.OnService(false);
    while (f.in.active) f.in.Complete();
    TEST_ASSERT(f.can1.tx.empty());
    if (how == 2)
    {
      TEST_ASSERT(f.can0.tx.empty());
      TEST_ASSERT(EchoIds(f).empty());
    }
    else
    {
      TEST_ASSERT(f.can0.tx.size() == 3);
      TEST_ASSERT(EchoIds(f) == Run(0x1000u, 3));
    }
    TEST_ASSERT(!f.gs.NeedsFrameTick());

    // The channel works again after the stop (for MODE RESET, once it is started).
    if (how == 0) SetMode(f, 1, 1);
    f.out.HostSends(HostFrame(0x2100u, 1));
    while (f.in.active) f.in.Complete();
    TEST_ASSERT(f.can1.tx.size() == 1 && f.can1.tx[0].id == (0x2100u & 0x7FFu));
    TEST_ASSERT(EchoIdsIn(f, 0x2000u, 0x3000u) == std::vector<uint32_t>{0x2100u});
  }
}

/// echo 队列满时的配合：驱动接受了队头，但 echo 队列没有位置，这一帧保持“已接受”，
/// 下次服务只补推 echo，不会再次提交 CAN，也不会回两个 echo；此时通道复位则丢弃它，
/// 不回 echo。
/// Interplay with a full echo queue: the driver accepted the head but the echo queue has
/// no room, the frame stays "accepted" and the next service only pushes its echo; it is
/// never submitted to CAN twice nor echoed twice; a channel reset in that state drops it
/// without an echo.
/// state drops it without an echo.
void TestEchoQueueFullWhileStagedFramesAreAccepted()
{
  for (unsigned reset = 0; reset < 2; ++reset)
  {
    FreshTimerList();
    GsFixture f(32, 2);  // two echo slots
    f.can0.queue.capacity = 0;
    f.gs.BindEndpoints(f.pool, 0, false);
    Configure(f);
    constexpr unsigned FRAMES = 8;
    for (unsigned i = 0; i < FRAMES; ++i)
    {
      TEST_ASSERT(f.out.active);
      f.out.HostSends(HostFrame(0x1000u + i, 0));
    }
    TEST_ASSERT(f.can0.tx.empty());

    // The controller has room for all of them, but the host does not read the IN
    // endpoint: the echo queue fills and the head frame is accepted without an echo.
    f.can0.queue.capacity = SIZE_MAX;
    for (unsigned i = 0; i < 5; ++i) SofTick(f);
    const size_t accepted = f.can0.tx.size();
    TEST_ASSERT(accepted > 0 && accepted < FRAMES);
    TEST_ASSERT(accepted > EchoIds(f).size());
    TEST_ASSERT(f.gs.NeedsFrameTick());
    TEST_ASSERT(f.out.active);

    if (reset)
    {
      const size_t echoed = EchoIds(f).size();
      SetMode(f, 0, 0);  // the staged frames, accepted head included, are dropped
      TEST_ASSERT(!f.gs.NeedsFrameTick());
      for (unsigned i = 0; i < 64 && f.in.active; ++i) f.in.Complete();
      for (unsigned i = 0; i < 5; ++i) SofTick(f);
      TEST_ASSERT(f.can0.tx.size() == accepted);  // nothing more was submitted
      // The echoes of the reset channel that waited in the class are dropped with the
      // session (the host freed its tx contexts); the head frame's echo was never
      // queued. Only what was already in the two IN slots reaches the host.
      TEST_ASSERT(EchoIds(f).size() <= echoed + 2);
      TEST_ASSERT(EchoIds(f).size() < accepted - 1);
      continue;
    }

    for (unsigned guard = 0; guard < 200 && f.gs.NeedsFrameTick(); ++guard)
    {
      HostReadsIn(f);
      SofTick(f);
      TEST_ASSERT(f.can0.tx.size() <= FRAMES);  // never submitted twice
    }
    while (f.in.active) f.in.Complete();
    TEST_ASSERT(f.can0.tx.size() == FRAMES);
    for (unsigned i = 0; i < FRAMES; ++i)
    {
      TEST_ASSERT(f.can0.tx[i].id == ((0x1000u + i) & 0x7FFu));
    }
    TEST_ASSERT(EchoIds(f) == Run(0x1000u, FRAMES));  // one echo each, in order
    TEST_ASSERT(!f.gs.NeedsFrameTick());
  }
}

/// CAN FD 帧和不要 echo 的帧同样走暂存：提交一次，之后才有 echo（或没有）。
/// A CAN FD frame and a frame that wants no echo are staged the same way: submitted once,
/// echoed afterwards (or not at all).
void TestStagedFdFrameAndFrameWithoutEcho()
{
  FreshTimerList();
  GsFdFixture f;
  f.can0.queue.capacity = 1;
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f);
  SetMode(f, 0, 1, GsUsb::GSCAN_MODE_FD);

  auto fd_frame = [](uint32_t echo_id, uint32_t can_id, uint8_t first)
  {
    std::vector<uint8_t> frame(WIRE_HEADER + 64, 0);
    std::memcpy(&frame[0], &echo_id, 4);
    std::memcpy(&frame[4], &can_id, 4);
    frame[8] = 8;  // 8 data bytes
    frame[9] = 0;
    frame[10] = GsUsb::CAN_FLAG_FD;
    frame[WIRE_HEADER] = first;
    return frame;
  };

  f.out.HostSends(fd_frame(0x5001u, 0x301u, 0xA1));
  f.out.HostSends(fd_frame(0x5002u, 0x302u, 0xB2));  // refused: staged
  TEST_ASSERT(f.out.active && f.gs.NeedsFrameTick());
  TEST_ASSERT(f.can0.tx.size() == 1);
  HostReadsIn(f);
  TEST_ASSERT((EchoIds(f) == std::vector<uint32_t>{0x5001u}));

  f.can0.queue.Drain(1);
  SofTick(f);
  TEST_ASSERT(!f.gs.NeedsFrameTick());
  TEST_ASSERT(f.can0.tx.size() == 2 && f.can0.tx[1].id == 0x302u &&
              f.can0.tx[1].first == 0xB2);
  while (f.in.active) f.in.Complete();
  TEST_ASSERT((EchoIds(f) == std::vector<uint32_t>{0x5001u, 0x5002u}));

  // A frame that wants no echo (echo_id = ECHO_ID_RX): staged, sent, never echoed.
  f.out.HostSends(fd_frame(0xFFFFFFFFu, 0x303u, 0xC3));  // queue full again: staged
  TEST_ASSERT(f.out.active && f.gs.NeedsFrameTick());
  f.can0.queue.Drain(1);
  SofTick(f);
  TEST_ASSERT(f.can0.tx.size() == 3 && f.can0.tx[2].id == 0x303u);
  TEST_ASSERT(!f.gs.NeedsFrameTick());
  while (f.in.active) f.in.Complete();
  TEST_ASSERT((EchoIds(f) == std::vector<uint32_t>{0x5001u, 0x5002u}));
}
/// 暂存容量 0 在类的构造函数里被明确拒绝（报告的位置是 gs_usb.hpp，而不是队列内部的
/// 断言）；1 可以构造。构造在 fork 出的子进程里做，因为致命错误会结束进程。
/// A staging capacity of 0 is rejected explicitly by the class constructor (the report
/// names gs_usb.hpp, not an assertion inside the queue); 1 constructs. The construction
/// runs in a forked child because a fatal error ends the process.
void TestStagingSizeOfZeroIsRejected()
{
  for (unsigned size = 0; size < 2; ++size)
  {
    int fds[2];
    TEST_ASSERT(pipe(fds) == 0);
    const pid_t pid = fork();
    TEST_ASSERT(pid >= 0);
    if (pid == 0)
    {
      close(fds[0]);
      g_fatal_report_fd = fds[1];
      FreshTimerList();
      GsFixture f(32, 32, nullptr, size);
      _exit(0);  // constructed
    }
    close(fds[1]);
    char reported[256] = {};
    const ssize_t got = read(fds[0], reported, sizeof(reported) - 1);
    close(fds[0]);
    int status = 0;
    TEST_ASSERT(waitpid(pid, &status, 0) == pid);
    TEST_ASSERT(WIFEXITED(status));
    if (size == 0)
    {
      TEST_ASSERT(WEXITSTATUS(status) == 42 && got > 0);
      const std::string where(reported, static_cast<size_t>(got));
      TEST_ASSERT(where.find("gs_usb.hpp") != std::string::npos);
    }
    else
    {
      TEST_ASSERT(WEXITSTATUS(status) == 0 && got == 0);
    }
  }
}

}  // namespace

/// 没有启动的通道的帧不会交给 IN 端点：复位之后入队的错误帧不发，复位那一刻被 echo 队列
/// 压住的 echo 丢弃并归还 OUT 租约；另一个通道不受影响。
/// The frames of a channel that is not started are never handed to the IN endpoint: an
/// error frame queued after the reset is not sent, an echo held by echo-queue
/// back-pressure at the reset is dropped and the OUT lease goes back; the other channel
/// is not affected.
void TestStoppedChannelFramesNeverReachTheEndpoint()
{
  FreshTimerList();
  GsFixture f(32, 2);  // two echo slots: the third echo is held
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f);
  SetMode(f, 0, 1u, GsUsb::GSCAN_MODE_BERR_REPORTING);

  // Fill the IN slots with channel 1 frames so that channel 0 echoes queue up.
  f.can1.Inject(0x200u, {0x20});
  f.can1.Inject(0x201u, {0x21});
  unsigned accepted = 0;
  while (f.out.active && accepted < 8)
  {
    f.out.HostSends(ClassicFrame(0x3000u + accepted, 0x300u + accepted, 1, 0, 0, {0x5A}));
    ++accepted;
  }
  TEST_ASSERT(!f.out.active);  // the echo of the last frame is held, the lease with it

  // The host stops channel 0: the held echo is dropped and the OUT receive is armed.
  SetMode(f, 0, 0u);
  TEST_ASSERT(f.out.active);

  // An error frame of the stopped channel is queued by the CAN callback but never sent.
  f.can0.Inject(LibXR::CAN::FromErrorID(LibXR::CAN::ErrorID::CAN_ERROR_ID_BUS_OFF), {},
                LibXR::CAN::Type::ERROR);
  f.can1.Inject(0x202u, {0x22});
  for (unsigned i = 0; i < 32 && f.in.active; ++i) f.in.Complete();

  unsigned ch0_frames = 0, ch0_errors = 0;
  std::vector<uint32_t> rx1;
  for (const auto& frame : f.in.sent)
  {
    if (frame[9] == 0) ++ch0_frames;
    if (frame[9] == 0 && (ReadU32(frame, 4) & GsUsb::CAN_ERR_FLAG) != 0U) ++ch0_errors;
    if (frame[9] == 1 && ReadU32(frame, 0) == 0xFFFFFFFFu)
      rx1.push_back(ReadU32(frame, 4));
  }
  // Only the echoes that were already in the IN slots before the reset (at most two).
  TEST_ASSERT(ch0_frames <= 2);
  TEST_ASSERT(ch0_errors == 0);
  TEST_ASSERT(rx1 == std::vector<uint32_t>({0x200u, 0x201u, 0x202u}));
}

/// 最后一个通道复位时，共享 IN 端点上排队和在途的帧一起结束，所有队列清空：下一次打开
/// 之后到达主机的只有新会话的帧。
/// When the last channel is reset, the frames queued and in flight on the shared IN
/// endpoint end with it and every queue is emptied: after the next open only frames of
/// the new session reach the host.
void TestLastChannelResetFlushesTheSlots()
{
  for (unsigned asynchronous = 0; asynchronous < 2; ++asynchronous)
  {
    FreshTimerList();
    GsFixture f;
    f.in.pending_stop = asynchronous != 0U;  // the controller confirms the stop later
    f.gs.BindEndpoints(f.pool, 0, false);
    Configure(f, true, false);  // channel 0 is the only channel started

    // Six frames: one in flight, one queued in the slots, four in the RX queue.
    for (uint32_t i = 0; i < 6; ++i) f.can0.Inject(0x100u + i, {static_cast<uint8_t>(i)});
    f.can0.Inject(LibXR::CAN::FromErrorID(LibXR::CAN::ErrorID::CAN_ERROR_ID_BUS_OFF), {},
                  LibXR::CAN::Type::ERROR);
    TEST_ASSERT(f.in.active);

    SetMode(f, 0, 0u);  // the last channel goes down
    if (asynchronous != 0U)
    {
      TEST_ASSERT(f.in.stop_requested);
      f.in.ConfirmStop();
    }
    TEST_ASSERT(!f.in.active);  // nothing is armed for the next open to collect
    TEST_ASSERT(f.in.sent.empty());

    // The next open: nothing of the old session arrives, a new frame does.
    SetMode(f, 0, 1u);
    f.can0.Inject(0x500u, {0x50});
    for (unsigned i = 0; i < 32 && f.in.active; ++i) f.in.Complete();
    TEST_ASSERT(f.in.sent.size() == 1);
    TEST_ASSERT(ReadU32(f.in.sent[0], 0) == 0xFFFFFFFFu);
    TEST_ASSERT(ReadU32(f.in.sent[0], 4) == 0x500u);
  }
}

/// 只复位一个通道时共享 IN 端点不动：另一个通道已经打包的帧、排队的帧和 echo 都照常按
/// 顺序发出。
/// Resetting one channel leaves the shared IN endpoint alone: the other channel's frames
/// already packed, its queued frames and its echoes all go out, in order.
void TestOneChannelResetKeepsTheOtherChannelsFramesAndEchoes()
{
  FreshTimerList();
  GsFixture f;
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f);

  for (uint32_t i = 0; i < 6; ++i) f.can1.Inject(0x200u + i, {static_cast<uint8_t>(i)});
  for (uint32_t i = 0; i < 4; ++i) f.can0.Inject(0x100u + i, {static_cast<uint8_t>(i)});
  for (uint32_t i = 0; i < 2; ++i)
    f.out.HostSends(ClassicFrame(0x4000u + i, 0x400u + i, 1, 1, 0, {0x5B}));

  SetMode(f, 0, 0u);         // channel 1 is still started
  TEST_ASSERT(f.in.active);  // the slots are still armed

  for (unsigned i = 0; i < 64 && f.in.active; ++i) f.in.Complete();
  std::vector<uint32_t> rx1, echo1;
  for (const auto& frame : f.in.sent)
  {
    if (frame[9] != 1) continue;
    if (ReadU32(frame, 0) == 0xFFFFFFFFu)
      rx1.push_back(ReadU32(frame, 4));
    else
      echo1.push_back(ReadU32(frame, 0));
  }
  TEST_ASSERT(rx1 ==
              std::vector<uint32_t>({0x200u, 0x201u, 0x202u, 0x203u, 0x204u, 0x205u}));
  TEST_ASSERT(echo1 == std::vector<uint32_t>({0x4000u, 0x4001u}));
}

/// 一帧被接受就恰好回一个 echo（通道在运行时）；通道复位后，它的 echo 不再回，重新启动
/// 之后又恢复一帧一个 echo。
/// A frame accepted while the channel is started is echoed exactly once; after the
/// channel is reset its echoes are no longer sent, and after the restart it is one echo
/// per accepted frame again.
void TestEchoCountMatchesAcceptedFrames()
{
  FreshTimerList();
  GsFixture f;
  f.gs.BindEndpoints(f.pool, 0, false);
  Configure(f);

  auto pump = [&](unsigned frames, uint8_t ch, uint32_t base)
  {
    for (unsigned i = 0; i < frames; ++i)
    {
      for (unsigned guard = 0; guard < 64 && !f.out.active; ++guard) f.in.Complete();
      TEST_ASSERT(f.out.active);
      f.out.HostSends(ClassicFrame(base + i, 0x300u + i, 1, ch, 0, {0x5A}));
    }
    for (unsigned guard = 0; guard < 64 && f.in.active; ++guard) f.in.Complete();
  };

  auto echoes = [&](uint8_t ch)
  {
    unsigned n = 0;
    for (const auto& frame : f.in.sent)
      if (frame[9] == ch && ReadU32(frame, 0) != 0xFFFFFFFFu) ++n;
    return n;
  };

  pump(40, 0, 0x1000u);
  pump(40, 1, 0x2000u);
  TEST_ASSERT(f.can0.tx.size() == 40 && echoes(0) == 40);
  TEST_ASSERT(f.can1.tx.size() == 40 && echoes(1) == 40);

  // Channel 0 is reset with echoes pending: the accepted frames are on the bus side, the
  // echoes of the old session are not sent; channel 1 keeps its one-to-one count.
  for (unsigned i = 0; i < 3; ++i)
    f.out.HostSends(ClassicFrame(0x1100u + i, 0x310u + i, 1, 0, 0, {0x5A}));
  SetMode(f, 0, 0u);
  const unsigned accepted0 = static_cast<unsigned>(f.can0.tx.size());
  for (unsigned guard = 0; guard < 64 && f.in.active; ++guard) f.in.Complete();
  TEST_ASSERT(accepted0 == 43);
  TEST_ASSERT(echoes(0) <= 43 && echoes(0) >= 40);

  // A frame for the stopped channel is not accepted, so it is not echoed either.
  for (unsigned guard = 0; guard < 64 && !f.out.active; ++guard) f.in.Complete();
  f.out.HostSends(ClassicFrame(0x1200u, 0x320u, 1, 0, 0, {0x5A}));
  TEST_ASSERT(f.can0.tx.size() == accepted0);

  SetMode(f, 0, 1u);
  const unsigned before = echoes(0);
  pump(25, 0, 0x1300u);
  TEST_ASSERT(f.can0.tx.size() == accepted0 + 25);
  TEST_ASSERT(echoes(0) - before == 25);
  TEST_ASSERT(f.can1.tx.size() == 40 && echoes(1) == 40);
}

int main()
{
  TestBindAndConfiguration();
  TestHostFrameToCanAndEcho();
  TestCanRxToHost();
  TestChannelStartDropsStaleFrames();
  TestRebindThenStartDropsStaleFrames();
  TestRxQueueOverflowFlag();
  TestOverflowFlagRidesOnTheChannelsNextFrame();
  TestEchoAndReceivedFramesAlternate();
  TestHostBurstOnARateLimitedBus();
  TestEchoBackpressure();
  TestResumeAfterReturnedLeaseArmsNothing();
  TestUserIdRoundTripAndDatabaseWrite();
  TestMultiChannelRoundRobin();
  TestErrorFrameHasOwnQueue();
  TestFullCanQueueStagesTheFrames();
  TestNewFrameQueuesBehindStagedFrames();
  TestStagedFrameIsRetriedByTheService();
  TestStuckChannelDoesNotBlockTheOtherChannel();
  TestStagingOverflowDropsAndEchoes();
  TestChannelStopDropsItsStagedFrames();
  TestEchoQueueFullWhileStagedFramesAreAccepted();
  TestStagedFdFrameAndFrameWithoutEcho();
  TestStagingSizeOfZeroIsRejected();
  TestStoppedChannelFramesNeverReachTheEndpoint();
  TestLastChannelResetFlushesTheSlots();
  TestOneChannelResetKeepsTheOtherChannelsFramesAndEchoes();
  TestEchoCountMatchesAcceptedFrames();
  std::puts(
      "gs_usb class: queues, echo and RX alternation, burst, backpressure, overflow, "
      "USER_ID, round-robin, error queue and CAN transmit queue staging passed");
}
