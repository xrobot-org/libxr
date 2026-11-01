/**
 * @file test_gsusb.cpp
 * @brief gs_usb（CAN 适配器）设备类（槽模型）主机测试 /
 *        Host tests for the gs_usb (CAN adapter) device class.
 *
 * 覆盖 5C 迁移改动的路径：每通道一个 SPSC 接收队列 + 一个 echo 队列、服务里取槽打包提交、
 * echo 优先但设份额、echo 队列满时保留 OUT 租约作背压、RX 队列满时丢帧计数并在下一帧置
 * CAN_FLAG_OVERFLOW、以及 SET_USER_ID 只改 RAM 与待写标志、由 100 ms Timer 任务落库。
 * Covers the paths the 5C migration changed: one SPSC RX queue per channel plus one echo
 * queue, slot packing and submission in the class service, echo priority under a share,
 * the OUT lease held as backpressure while the echo queue is full, dropped frames counted
 * per channel with CAN_FLAG_OVERFLOW on that channel's next frame, and SET_USER_ID only
 * touching RAM plus a pending flag with the 100 ms Timer task writing the Database.
 */

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
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
  std::vector<Tx> tx;
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
  size_t overflow = 0;
  for (const auto& frame : f.in.sent)
  {
    if ((frame[10] & GsUsb::CAN_FLAG_OVERFLOW) != 0U) ++overflow;
  }
  // The drop is reported exactly once, on that channel's next frame.
  TEST_ASSERT(overflow == 1);
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
}  // namespace

int main()
{
  TestBindAndConfiguration();
  TestHostFrameToCanAndEcho();
  TestCanRxToHost();
  TestRxQueueOverflowFlag();
  TestEchoBackpressure();
  TestUserIdRoundTripAndDatabaseWrite();
  TestMultiChannelRoundRobin();
  TestErrorFrameHasOwnQueue();
  std::puts(
      "gs_usb class: queues, echo share, backpressure, overflow, USER_ID, round-robin "
      "and "
      "error queue passed");
}
