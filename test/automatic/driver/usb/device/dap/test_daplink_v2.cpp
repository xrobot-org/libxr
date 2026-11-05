/**
 * @file test_daplink_v2.cpp
 * @brief DAP v2 设备类（借用模式）主机测试 / Host tests for the DAP v2 device class in
 *        borrow mode.
 *
 * OUT 方向把每个收到的包借给回调、拷进命令池后立即归还，两个槽交替接收；命令按自己的长度
 * 判断结束，补零的传输整段丢弃。IN 方向把响应原地写进端点的槽再提交。命令由 Step() 线程
 * 执行，延时命令在线程里睡到到期。
 * The OUT side lends each received packet to the callback, copies it into the command
 * pool and gives it back at once, so the two slots alternate; a command ends by its own
 * length and a zero-padded transfer is dropped as a whole. The IN side writes the
 * response in place into an endpoint slot and submits it. Commands run in the Step()
 * thread, and a delayed command sleeps there until it is due.
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <string>
#include <thread>
#include <vector>

#include "../../mock_endpoint.hpp"
#include "daplink_v2.hpp"
#include "debug/swd.hpp"
#include "device_composition.hpp"
#include "lockfree_list.hpp"
#include "timer.hpp"

using namespace LibXR;
using namespace LibXR::USB;
using namespace LibXR::USB::Test;

namespace
{
static constexpr auto TEST_LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "LibXR", "USB tests", "0");

using CommandId = LibXR::USB::DapLinkV2Def::CommandId;
using InfoId = LibXR::USB::DapLinkV2Def::InfoId;
using Port = LibXR::USB::DapLinkV2Def::Port;

constexpr uint8_t INFO = static_cast<uint8_t>(CommandId::INFO);
constexpr uint8_t CONNECT = static_cast<uint8_t>(CommandId::CONNECT);
constexpr uint8_t DELAY = static_cast<uint8_t>(CommandId::DELAY);
constexpr uint8_t PACKET_SIZE_ID = static_cast<uint8_t>(InfoId::PACKET_SIZE);
constexpr uint8_t PACKET_COUNT_ID = static_cast<uint8_t>(InfoId::PACKET_COUNT);

/// Minimal SWD probe: every transfer succeeds and reads a fixed word.
class FakeSwd : public LibXR::Debug::Swd
{
 public:
  unsigned enters = 0, closes = 0;
  uint32_t clock_hz = 0;

  ErrorCode SetClockHz(uint32_t hz) override
  {
    clock_hz = hz;
    return ErrorCode::OK;
  }
  void Close() override { ++closes; }
  ErrorCode LineReset() override { return ErrorCode::OK; }
  ErrorCode EnterSwd() override
  {
    ++enters;
    return ErrorCode::OK;
  }
  ErrorCode Transfer(const LibXR::Debug::SwdProtocol::Request& req,
                     LibXR::Debug::SwdProtocol::Response& resp) override
  {
    if (req.rnw) resp.rdata = 0x12345678u;
    resp.ack = LibXR::Debug::SwdProtocol::Ack::OK;
    resp.parity_ok = true;
    return ErrorCode::OK;
  }
  void IdleClocks(uint32_t) override {}
  ErrorCode SeqWriteBits(uint32_t, const uint8_t*) override { return ErrorCode::OK; }
  ErrorCode SeqReadBits(uint32_t cycles, uint8_t* out) override
  {
    if (out != nullptr) std::memset(out, 0, (cycles + 7U) / 8U);
    return ErrorCode::OK;
  }
};

/// Give this test its own timer list so a class destroyed earlier cannot leave a
/// dangling task node behind for the next Timer::Refresh().
void FreshTimerList() { LibXR::Timer::list_ = new LibXR::LockFreeList(); }

using DapV2 = DapLinkV2Class<FakeSwd>;

class TestDap2 : public DapV2
{
 public:
  using DapV2::BindEndpoints;
  using DapV2::DapV2;
  using DapV2::OnService;
  using DapV2::UnbindEndpoints;
};

/// The class, its SWD probe and the composition that gives it a class-service pool. The
/// endpoint blocks hold two 1024 byte slots per direction, as on a real board.
struct DapFixture
{
  Block<2048> in_storage, out_storage;
  EndpointPool pool{4};
  MockEndpoint in{Number::EP1, Direction::IN, in_storage.Raw()};
  MockEndpoint out{Number::EP2, Direction::OUT, out_storage.Raw()};
  FakeSwd swd;
  TestDap2 dap;
  DeviceComposition composition;
  Callback<uint32_t> handler;

  DapFixture()
      : swd(),
        dap(Number::EP1, Number::EP2, swd),
        composition(pool, {&TEST_LANGUAGE}, {{&dap}})
  {
    TEST_ASSERT(pool.Put(&in) == ErrorCode::OK);
    TEST_ASSERT(pool.Put(&out) == ErrorCode::OK);
    // Stand-in for DeviceCore's class-service dispatch.
    handler = Callback<uint32_t>::Create(
        [](bool in_isr, DeviceClass* self, uint32_t events)
        {
          if ((events & EndpointPool::CLASS_EVENT) != 0U) self->OnService(in_isr);
        },
        static_cast<DeviceClass*>(&dap));
    pool.SetControlHandler(handler);
  }
};

/// Run every command that has arrived, the way the application's DAP thread would. A
/// command that waits for an IN slot makes Step(0) time out and ends the loop.
void Run(DapFixture& f)
{
  while (f.dap.Step(0) == ErrorCode::OK)
  {
  }
}

/// The host completes the request that is armed with these bytes; the commands that
/// completes are then executed by Step().
void Host(DapFixture& f, const std::vector<uint8_t>& bytes)
{
  f.out.HostSends(bytes);
  Run(f);
}

void Send(DapFixture& f, std::initializer_list<uint8_t> bytes)
{
  Host(f, std::vector<uint8_t>(bytes));
}

/// Complete every hardware request of one response; count bytes and zero-length tails.
void Drain(DapFixture& f, size_t& bytes, unsigned& tails)
{
  bytes = 0;
  tails = 0;
  while (f.in.active)
  {
    if (f.in.Last().length == 0)
      ++tails;
    else
      bytes += f.in.Last().length;
    f.in.Complete();
    Run(f);
  }
}

void TestCommandRoundTrip()
{
  FreshTimerList();
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);

  // Binding arms both OUT slots with one endpoint packet each; only one request is on
  // the hardware at a time.
  TEST_ASSERT(f.out.active && f.out.Last().length == 64);
  TEST_ASSERT(f.in.starts.empty());

  // DAP_Info(PACKET_COUNT) answers the effective outstanding depth.
  Send(f, {INFO, PACKET_COUNT_ID});
  TEST_ASSERT(f.in.Last().length == 3);
  TEST_ASSERT(f.in.Last().data == std::vector<uint8_t>({INFO, 1, 0x04}));
  f.in.Complete();

  // A connect command reaches the SWD probe and answers the port.
  Send(f, {CONNECT, static_cast<uint8_t>(Port::SWD)});
  TEST_ASSERT(f.in.Last().length == 2);
  TEST_ASSERT(f.in.Last().data[0] == CONNECT &&
              f.in.Last().data[1] == static_cast<uint8_t>(Port::SWD));
  TEST_ASSERT(f.swd.enters == 1 && f.swd.clock_hz == 1000000u);
  f.in.Complete();

  // The OUT side keeps two requests armed throughout.
  TEST_ASSERT(f.out.active);
}

/// Four commands in flight: the Step thread runs all of them ahead of the host. The first
/// two responses go straight into the two IN slots; the other two wait in staging slots
/// and are submitted as the host takes responses, in the order of the commands, without
/// Step having to run again.
void TestPipelineKeepsAdvertisedDepth()
{
  FreshTimerList();
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);

  for (unsigned i = 0; i < 4; ++i)
  {
    Send(f, {INFO, PACKET_SIZE_ID});
  }
  // Every command has executed already; one response is on the wire, a second is queued
  // behind it in the other IN slot, and two wait in staging slots.
  TEST_ASSERT(f.in.starts.size() == 1);
  TEST_ASSERT(f.dap.Step(0) == ErrorCode::TIMEOUT);  // nothing left to execute

  // Each response the host takes lets the USB side submit the next staged one.
  for (unsigned i = 0; i < 4; ++i)
  {
    TEST_ASSERT(f.in.active);
    f.in.Complete();
  }
  TEST_ASSERT(!f.in.active);
  TEST_ASSERT(f.in.sent.size() == 4);
  TEST_ASSERT(f.out.active);
}

/// With an IN slot free and nothing staged, Step writes the response into the IN slot and
/// submits it itself: it reaches the wire without the class service.
void TestResponseNeedsNoClassService()
{
  FreshTimerList();
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);
  f.handler = Callback<uint32_t>::Create([](bool, DapFixture*, uint32_t) {}, &f);
  f.pool.SetControlHandler(f.handler);

  Send(f, {INFO, PACKET_SIZE_ID});
  TEST_ASSERT(f.in.active && f.in.Last().length == 4);
  TEST_ASSERT(f.in.Last().data[0] == INFO);
  f.in.Complete();
}

/// Responses written into IN slots and staged ones interleave: they still go out in the
/// order of the commands. PACKET_COUNT answers 3 bytes and PACKET_SIZE 4, so the lengths
/// on the wire show the order.
void TestDirectAndStagedResponsesKeepOrder()
{
  FreshTimerList();
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);

  std::vector<size_t> expected;
  auto send = [&](uint8_t id)
  {
    Send(f, {INFO, id});
    expected.push_back(id == PACKET_COUNT_ID ? 3U : 4U);
  };
  // Two responses take both IN slots directly; the next two are staged.
  send(PACKET_COUNT_ID);
  send(PACKET_SIZE_ID);
  send(PACKET_COUNT_ID);
  send(PACKET_SIZE_ID);
  TEST_ASSERT(f.in.starts.size() == 1);
  // The host takes one; a staged response fills the slot. A new command meets a staged
  // response still waiting and is staged behind it.
  f.in.Complete();
  send(PACKET_COUNT_ID);
  while (f.in.active)
  {
    f.in.Complete();
    Run(f);
  }
  // Everything drained: the next response goes straight into an IN slot again.
  send(PACKET_SIZE_ID);
  TEST_ASSERT(f.in.active);
  f.in.Complete();

  TEST_ASSERT(f.in.sent.size() == expected.size());
  for (size_t i = 0; i < expected.size(); ++i)
  {
    TEST_ASSERT(f.in.sent[i].size() == expected[i]);
  }
}

/// An unbind while Step holds an IN slot for a delayed response: the slot is given back,
/// and after rebinding both IN slots are free.
void TestUnbindWhileHoldingInSlot()
{
  FreshTimerList();
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);

  f.out.HostSends({DELAY, 0x10, 0x27});  // 10000 us
  std::thread dap_thread([&f]() { (void)f.dap.Step(1000); });
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  f.dap.UnbindEndpoints(f.pool, false);
  dap_thread.join();
  TEST_ASSERT(f.in.starts.empty());
  // The application's loop steps again; unbound, Step gives back what it holds.
  TEST_ASSERT(f.dap.Step(0) == ErrorCode::OK);

  f.dap.BindEndpoints(f.pool, 0, false);
  Slot a, b;
  TEST_ASSERT(f.in.AcquireSlot(a, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.AcquireSlot(b, false) == ErrorCode::OK);
  f.in.ReleaseSlot(a, false);
  f.in.ReleaseSlot(b, false);
  Send(f, {INFO, PACKET_SIZE_ID});
  TEST_ASSERT(f.in.active && f.in.Last().length == 4);
  f.in.Complete();
}

void TestDelayedCommand()
{
  FreshTimerList();
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);

  // A delay command is only received by the USB side; nothing is executed or answered
  // until the DAP thread steps, and the OUT slots keep receiving.
  f.out.HostSends({DELAY, 0x10, 0x27});  // 10000 us
  TEST_ASSERT(f.in.starts.empty());
  TEST_ASSERT(f.out.active);

  // Step() sleeps the delay out in the calling thread and then answers.
  TEST_ASSERT(f.dap.Step(0) == ErrorCode::OK);
  TEST_ASSERT(f.in.active && f.in.Last().length == 2);
  TEST_ASSERT(f.in.Last().data == std::vector<uint8_t>({DELAY, 0x00}));
  f.in.Complete();

  // A zero delay answers at once.
  Send(f, {DELAY, 0x00, 0x00});
  TEST_ASSERT(f.in.active && f.in.Last().data == std::vector<uint8_t>({DELAY, 0x00}));
  f.in.Complete();
}

/// A batch that contains a delay command: the delay is slept out in the Step thread and
/// the batch response goes out whole.
void TestBatchWithDelay()
{
  FreshTimerList();
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);

  // QUEUE_COMMANDS with two commands - DAP_Info and a 2 ms DAP_Delay. The wire format is
  // plain concatenation (each command's length follows from its ID, no prefix). The
  // packet runs as EXECUTE_COMMANDS, as in the CMSIS-DAP reference implementation.
  std::vector<uint8_t> queue{
      static_cast<uint8_t>(CommandId::QUEUE_COMMANDS),
      2u,
      static_cast<uint8_t>(CommandId::INFO),
      static_cast<uint8_t>(InfoId::VENDOR),
      DELAY,
      0xD0,
      0x07,  // 2000 us little-endian
  };
  // The delay inside the batch makes Step sleep until it is due, and the whole response
  // arrives at once: [EXECUTE, count, INFO response, DELAY response], with no second
  // stray response.
  Host(f, queue);
  TEST_ASSERT(f.in.active);
  const auto& batch = f.in.Last().data;
  // INFO response: [INFO, len=7, "XRobot" + NUL], 9 bytes total (batch [2..10]); the
  // DELAY response [DELAY, 0x00] follows at [11..12], batch length 13.
  TEST_ASSERT(batch[0] == static_cast<uint8_t>(CommandId::EXECUTE_COMMANDS));
  TEST_ASSERT(batch[1] == 2U);  // number of commands
  TEST_ASSERT(batch[2] == static_cast<uint8_t>(CommandId::INFO));
  TEST_ASSERT(batch[3] == 7U);     // "XRobot" incl. NUL
  TEST_ASSERT(batch[10] == 0x00);  // NUL of "XRobot"
  TEST_ASSERT(batch[11] == DELAY);
  TEST_ASSERT(batch[12] == 0x00);  // DAP_OK
  TEST_ASSERT(f.in.Last().length == 13U);
  f.in.Complete();
  TEST_ASSERT(f.out.active);
}

/// The next command is received while the previous one executes (here: a delay) and runs
/// after it, in order.
void TestNextCommandReceivedDuringDelay()
{
  FreshTimerList();
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);

  f.out.HostSends({DELAY, 0x10, 0x27});  // 10000 us
  std::thread dap_thread([&f]() { (void)f.dap.Step(1000); });
  std::this_thread::sleep_for(std::chrono::milliseconds(2));

  // The host sends the next command while the delay is pending: it is received (the USB
  // side does not wait for the DAP thread) but not executed, and nothing answers.
  f.out.HostSends({INFO, PACKET_SIZE_ID});
  dap_thread.join();

  // The delay response goes first; the received command answers after it.
  TEST_ASSERT(f.in.active && f.in.Last().data == std::vector<uint8_t>({DELAY, 0x00}));
  f.in.Complete();
  Run(f);
  TEST_ASSERT(f.in.active && f.in.Last().data[0] == INFO);
  f.in.Complete();
  TEST_ASSERT(!f.in.active && f.out.active);
}

/// A response exactly as long as the DAP packet needs no ZLP (the host's packet-sized
/// read ends when full); one would stay armed on the endpoint and keep an IN slot
/// occupied. A response shorter than the DAP packet that is a multiple of the endpoint
/// packet size still ends with a ZLP.
void TestZlpOnlyBelowDapPacketSize()
{
  FreshTimerList();
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);
  size_t bytes;
  unsigned tails;

  // 64 bytes = one endpoint packet, shorter than the DAP packet: a ZLP follows.
  const std::string vendor(61, 'V');  // [INFO, 62, 61 chars + NUL] = 64 bytes
  DapV2::InfoStrings info{};
  info.vendor = vendor.c_str();
  f.dap.SetInfoStrings(info);
  Send(f, {INFO, static_cast<uint8_t>(InfoId::VENDOR)});
  Drain(f, bytes, tails);
  TEST_ASSERT(bytes == 64 && tails == 1);

  // 1024 bytes = the DAP packet size (a 255-word block read): no ZLP.
  Send(f, {CONNECT, static_cast<uint8_t>(Port::SWD)});
  Drain(f, bytes, tails);
  Send(f, {static_cast<uint8_t>(CommandId::TRANSFER_BLOCK), 0x00, 0xFF, 0x00, 0x0F});
  Drain(f, bytes, tails);
  TEST_ASSERT(bytes == 1024 && tails == 0);
}

/// A command whose length is a multiple of the endpoint packet size and no ZLP from the
/// host: it ends by the command's own length instead of waiting for a timeout.
void TestCommandOnPacketBoundaryEndsWithoutZlp()
{
  FreshTimerList();
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);

  // QUEUE_COMMANDS with 63 DAP_Info commands: 2 + 63 * 2 = 128 bytes, two packets.
  std::vector<uint8_t> queue{static_cast<uint8_t>(CommandId::QUEUE_COMMANDS), 63u};
  for (unsigned i = 0; i < 63; ++i)
  {
    queue.push_back(static_cast<uint8_t>(CommandId::INFO));
    queue.push_back(static_cast<uint8_t>(InfoId::VENDOR));
  }
  TEST_ASSERT(queue.size() == 128);

  TEST_ASSERT(f.out.active && f.out.Last().length == 64);
  Host(f, std::vector<uint8_t>(queue.begin(), queue.begin() + 64));
  // The first packet does not complete the command; the second slot is already armed for
  // the second packet. Nothing has been answered.
  TEST_ASSERT(f.out.active && f.out.Last().length == 64);
  TEST_ASSERT(f.in.starts.empty());
  Host(f, std::vector<uint8_t>(queue.begin() + 64, queue.end()));
  // The command is complete after the second whole packet: it answers.
  TEST_ASSERT(f.in.active);
  TEST_ASSERT(f.in.Last().data[0] == static_cast<uint8_t>(CommandId::EXECUTE_COMMANDS));
}

/// The host pads its transfer with zeros to the DAP packet size: the padding is not taken
/// for commands, it is asked for in as few requests as the content allows, and the
/// command runs once.
void TestZeroPaddedTransferIsDropped()
{
  FreshTimerList();
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);

  std::vector<uint8_t> first(64, 0);
  first[0] = INFO;
  first[1] = PACKET_SIZE_ID;
  Host(f, first);  // the command is complete inside the first packet
  TEST_ASSERT(f.in.active && f.in.starts.size() == 1);
  f.in.Complete();

  // The next whole packet is all zero: padding. The rest of the 1024 byte transfer (896
  // bytes) is asked for in one request on top of the packet that is already queued.
  Host(f, std::vector<uint8_t>(64, 0));
  TEST_ASSERT(f.out.active && f.out.Last().length == 64);  // the queued padding packet
  Host(f, std::vector<uint8_t>(64, 0));
  TEST_ASSERT(f.out.active && f.out.Last().length == 1024 - 192);  // the rest at once
  Host(f, std::vector<uint8_t>(1024 - 192, 0));
  TEST_ASSERT(f.out.active && f.out.Last().length == 64);
  TEST_ASSERT(f.in.starts.size() == 1);  // no response to padding

  // A real command still works afterwards.
  Send(f, {INFO, PACKET_SIZE_ID});
  TEST_ASSERT(f.in.active && f.in.starts.size() == 2);
  f.in.Complete();
}

/// DAP_TransferAbort has no response and does not take a command slot; reception goes on.
void TestTransferAbortHasNoResponse()
{
  FreshTimerList();
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);

  Send(f, {static_cast<uint8_t>(CommandId::TRANSFER_ABORT)});
  TEST_ASSERT(f.in.starts.empty() && !f.in.active);
  TEST_ASSERT(f.out.active);

  // A transfer started afterwards is not aborted by the stale flag.
  Send(f, {CONNECT, static_cast<uint8_t>(Port::SWD)});
  f.in.Complete();
  Send(f, {static_cast<uint8_t>(CommandId::TRANSFER), 0x00, 0x01, 0x02});  // read DP 0
  TEST_ASSERT(f.in.active);
  TEST_ASSERT(f.in.Last().data[0] == static_cast<uint8_t>(CommandId::TRANSFER));
  TEST_ASSERT(f.in.Last().data[1] == 1);  // one transfer completed
  f.in.Complete();
}

/// Every QUEUE_COMMANDS or EXECUTE_COMMANDS packet runs its own commands and answers
/// [EXECUTE, count, responses...]; nothing carries over to the next packet.
void TestQueueAndExecuteAnswerPerPacket()
{
  FreshTimerList();
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);
  const uint8_t EXECUTE = static_cast<uint8_t>(CommandId::EXECUTE_COMMANDS);

  Send(f, {static_cast<uint8_t>(CommandId::QUEUE_COMMANDS), 1, INFO, PACKET_COUNT_ID});
  TEST_ASSERT(f.in.active);
  TEST_ASSERT(f.in.Last().data == std::vector<uint8_t>({EXECUTE, 1, INFO, 1, 0x04}));
  f.in.Complete();

  Send(f, {EXECUTE, 2, INFO, PACKET_COUNT_ID, INFO, PACKET_SIZE_ID});
  TEST_ASSERT(f.in.active);
  const auto& batch = f.in.Last().data;
  TEST_ASSERT(f.in.Last().length == 9U);
  TEST_ASSERT(batch[0] == EXECUTE && batch[1] == 2);
  TEST_ASSERT(batch[2] == INFO && batch[3] == 1 && batch[4] == 0x04);
  TEST_ASSERT(batch[5] == INFO && batch[6] == 2);
  f.in.Complete();

  // A packet without the command count is malformed.
  Send(f, {EXECUTE});
  TEST_ASSERT(f.in.active);
  TEST_ASSERT(f.in.Last().data == std::vector<uint8_t>({EXECUTE, 0xFF}));
  f.in.Complete();
}

/// A batch resumed after a delay that then runs out of response space answers DAP_ERROR.
void TestResumedBatchOverflowAnswersError()
{
  FreshTimerList();
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);
  const uint8_t EXECUTE = static_cast<uint8_t>(CommandId::EXECUTE_COMMANDS);

  // A 1 ms delay, then 120 DAP_Info(vendor) answers of 9 bytes: more than one response.
  std::vector<uint8_t> req{EXECUTE, 121u, DELAY, 0xE8, 0x03};
  for (unsigned i = 0; i < 120; ++i)
  {
    req.push_back(INFO);
    req.push_back(static_cast<uint8_t>(InfoId::VENDOR));
  }
  // The host writes it in endpoint packets.
  for (size_t off = 0; off < req.size(); off += 64)
  {
    const size_t end = std::min(req.size(), off + 64);
    Host(f, std::vector<uint8_t>(req.begin() + off, req.begin() + end));
  }
  TEST_ASSERT(f.in.active);
  TEST_ASSERT(f.in.Last().data == std::vector<uint8_t>({EXECUTE, 0xFF}));
  f.in.Complete();
}

/// Padded requests with the slots full: the padding of the last admitted request is still
/// taken (it needs no command slot), so the host's write finishes, it reads the responses
/// and everything drains. A fifth command's first packet waits as a lease until a command
/// slot is free.
void TestPaddedRequestsAndAFifthCommandDrain()
{
  FreshTimerList();
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);

  auto padded = [&f]()
  {
    std::vector<uint8_t> first(64, 0);
    first[0] = INFO;
    first[1] = PACKET_SIZE_ID;
    Host(f, first);
    Host(f, std::vector<uint8_t>(64, 0));  // detects the padding
    // The rest arrives as one request on top of what is queued.
    Host(f, std::vector<uint8_t>(64, 0));
    Host(f, std::vector<uint8_t>(1024 - 192, 0));
  };
  for (unsigned i = 0; i < 4; ++i)
  {
    TEST_ASSERT(f.out.active);
    padded();
  }
  TEST_ASSERT(f.in.starts.size() == 1);  // one response on the wire, the rest wait
  TEST_ASSERT(f.out.active);

  // A fifth command: its first packet is kept as a lease, no command slot is free.
  std::vector<uint8_t> fifth(64, 0x11);
  fifth[0] = INFO;
  fifth[1] = PACKET_SIZE_ID;
  Host(f, fifth);
  TEST_ASSERT(f.in.starts.size() == 1);

  // The host reads: every response goes out, the fifth command runs last.
  unsigned answered = 0;
  while (f.in.active)
  {
    f.in.Complete();
    Run(f);
    ++answered;
  }
  TEST_ASSERT(answered == 5);
  TEST_ASSERT(f.out.active);
}

/// 按主机的方式读 MS OS 2.0 描述符集：向 BOS 能力里通告的 vendor code 发 wIndex 7
/// Read the MS OS 2.0 descriptor set the way the host does: wIndex 7 to the vendor code
/// the BOS capability advertises
ConstRawData ReadWinUsbSet(const DeviceComposition& composition)
{
  BosCapability* cap = composition.WinUsbBosCapability();
  TEST_ASSERT(cap != nullptr);
  const ConstRawData platform = cap->GetCapabilityDescriptor();
  const uint8_t vendor_code =
      static_cast<const uint8_t*>(platform.addr_)[platform.size_ - 2U];
  const SetupPacket setup{0xC0, vendor_code, 0, 0x0007, 0xFFFF};
  BosVendorResult result;
  TEST_ASSERT(cap->OnVendorRequest(false, &setup, result) == ErrorCode::OK &&
              result.handled);
  return result.in_data;
}

/// A single-function device gets the device-level MS OS 2.0 descriptor set from the
/// composition (Windows ignores the function subset header on a non-composite device),
/// and it is byte for byte the structure the device-level form defines.
void TestStandaloneDeviceUsesDeviceLevelWinUsbSet()
{
  FreshTimerList();
  DapFixture f;  // DAP is the only function of its composition
  const auto set = ReadWinUsbSet(f.composition);
  using Device = LibXR::USB::WinUsbMsOs20::DeviceScopedWinUsbMsOs20DescSet<
      LibXR::USB::WinUsbMsOs20::GUID_MULTI_SZ_UTF16_BYTES>;
  Device expected{};
  expected.Init("{CDB3B5AD-293B-4663-AA36-1AAE46463776}");
  TEST_ASSERT(set.size_ == sizeof(Device));
  TEST_ASSERT(std::memcmp(set.addr_, &expected, sizeof(Device)) == 0);
  TEST_ASSERT(f.composition.WinUsbBosCapability() != nullptr);
}

uint16_t LoadU16(const uint8_t* p)
{
  uint16_t value = 0;
  std::memcpy(&value, p, sizeof(value));
  return value;
}

/// Two WinUSB functions in one composite device: one configuration subset with one
/// function subset per function, the interface numbers following the order of the
/// classes.
void TestCompositeDeviceUsesFunctionSubsets()
{
  FreshTimerList();
  FakeSwd swd_a, swd_b;
  TestDap2 a(Number::EP1, Number::EP2, swd_a);
  TestDap2 b(Number::EP3, Number::EP4, swd_b);
  EndpointPool pool{8};
  DeviceComposition composition(pool, {&TEST_LANGUAGE}, {{&a, &b}});

  using Property = LibXR::USB::WinUsbMsOs20::DeviceInterfaceGuidsRegProperty<
      LibXR::USB::WinUsbMsOs20::GUID_MULTI_SZ_UTF16_BYTES>;
  constexpr size_t FUNCTION = 8 + 20 + sizeof(Property);  // subset header + compat + GUID
  const auto set = ReadWinUsbSet(composition);
  const auto* d = static_cast<const uint8_t*>(set.addr_);
  TEST_ASSERT(set.size_ == 10 + 8 + 2 * FUNCTION);
  TEST_ASSERT(LoadU16(d + 8) == set.size_);            // set header: wTotalLength
  TEST_ASSERT(LoadU16(d + 10 + 2) == 0x0001);          // configuration subset
  TEST_ASSERT(LoadU16(d + 10 + 6) == set.size_ - 10);  // covers both functions
  const uint8_t* first = d + 18;
  const uint8_t* second = first + FUNCTION;
  TEST_ASSERT(LoadU16(first + 2) == 0x0002 && first[4] == 0);  // function subset, itf 0
  TEST_ASSERT(LoadU16(first + 6) == FUNCTION);
  TEST_ASSERT(LoadU16(second + 2) == 0x0002 && second[4] == 1);  // function subset, itf 1
  TEST_ASSERT(LoadU16(second + 6) == FUNCTION);
}

void TestUnbind()
{
  FreshTimerList();
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);
  Send(f, {INFO, PACKET_SIZE_ID});
  TEST_ASSERT(f.in.active);
  f.dap.UnbindEndpoints(f.pool, false);
  TEST_ASSERT(f.out.closes == 1);
  // Rebinding serves the next command normally.
  f.dap.BindEndpoints(f.pool, 0, false);
  TEST_ASSERT(f.out.active && f.out.Last().length == 64);
  Send(f, {INFO, PACKET_SIZE_ID});
  TEST_ASSERT(f.in.active && f.in.Last().length == 4);
  f.in.Complete();
}
}  // namespace

int main()
{
  // Each test provides its own timer list so the constructor's Timer::Add() does not
  // start a scheduler thread; the test drives every refresh itself.
  TestCommandRoundTrip();
  TestPipelineKeepsAdvertisedDepth();
  TestResponseNeedsNoClassService();
  TestDirectAndStagedResponsesKeepOrder();
  TestUnbindWhileHoldingInSlot();
  TestDelayedCommand();
  TestBatchWithDelay();
  TestNextCommandReceivedDuringDelay();
  TestZlpOnlyBelowDapPacketSize();
  TestCommandOnPacketBoundaryEndsWithoutZlp();
  TestZeroPaddedTransferIsDropped();
  TestTransferAbortHasNoResponse();
  TestQueueAndExecuteAnswerPerPacket();
  TestResumedBatchOverflowAnswersError();
  TestPaddedRequestsAndAFifthCommandDrain();
  TestStandaloneDeviceUsesDeviceLevelWinUsbSet();
  TestCompositeDeviceUsesFunctionSubsets();
  TestUnbind();
  std::puts(
      "DAP v2 class: borrow-mode receive and submit, response depth and deferred delay "
      "passed");
}
