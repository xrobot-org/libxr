/**
 * @file test_daplink_v2.cpp
 * @brief DAP v2 设备类（借用模式）主机测试 / Host tests for the DAP v2 device class in
 *        borrow mode.
 *
 * OUT 方向把每个收到的包借给回调、拷进命令池后立即归还，两个槽交替接收；命令按自己的长度
 * 判断结束，补零的传输整段丢弃。IN 方向把响应原地写进端点的槽再提交。命令由 Step() 线程
 * 执行，延时命令在线程里等到完成。
 * The OUT side lends each received packet to the callback, copies it into the command
 * pool and gives it back at once, so the two slots alternate; a command ends by its own
 * length and a zero-padded transfer is dropped as a whole. The IN side writes the
 * response in place into an endpoint slot and submits it. Commands run in the Step()
 * thread, and a delay command waits there until it is done.
 */

#include <algorithm>
#include <array>
#include <atomic>
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

/// Minimal SWD probe: every transfer succeeds and reads a fixed word. It records the
/// thread of every call.
class FakeSwd : public LibXR::Debug::Swd
{
 public:
  unsigned enters = 0, closes = 0, sequences = 0;
  uint32_t clock_hz = 0;
  std::vector<std::thread::id> callers;

  ErrorCode SetClockHz(uint32_t hz) override
  {
    Note();
    clock_hz = hz;
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
    if (req.rnw) resp.rdata = 0x12345678u;
    resp.ack = LibXR::Debug::SwdProtocol::Ack::OK;
    resp.parity_ok = true;
    return ErrorCode::OK;
  }
  void IdleClocks(uint32_t) override { Note(); }
  ErrorCode SeqWriteBits(uint32_t, const uint8_t*) override
  {
    Note();
    ++sequences;
    return ErrorCode::OK;
  }
  ErrorCode SeqReadBits(uint32_t cycles, uint8_t* out) override
  {
    Note();
    if (out != nullptr) std::memset(out, 0, (cycles + 7U) / 8U);
    return ErrorCode::OK;
  }

 private:
  void Note() { callers.push_back(std::this_thread::get_id()); }
};

using DapV2 = DapLinkV2Class<FakeSwd>;

class TestDap2 : public DapV2
{
 public:
  using DapV2::BindEndpoints;
  using DapV2::OnControlData;
  using DapV2::OnControlRequest;
  using DapV2::ControlTransferResult;
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
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);

  // A delay command is only received by the USB side; nothing is executed or answered
  // until the DAP thread steps, and the OUT slots keep receiving.
  f.out.HostSends({DELAY, 0x10, 0x27});  // 10000 us
  TEST_ASSERT(f.in.starts.empty());
  TEST_ASSERT(f.out.active);

  // Step() waits the delay out in the calling thread and then answers.
  TEST_ASSERT(f.dap.Step(0) == ErrorCode::OK);
  TEST_ASSERT(f.in.active && f.in.Last().length == 2);
  TEST_ASSERT(f.in.Last().data == std::vector<uint8_t>({DELAY, 0x00}));
  f.in.Complete();

  // A zero delay answers at once.
  Send(f, {DELAY, 0x00, 0x00});
  TEST_ASSERT(f.in.active && f.in.Last().data == std::vector<uint8_t>({DELAY, 0x00}));
  f.in.Complete();
}

/// A batch that contains a delay command: the delay is waited out in the Step thread and
/// the batch response goes out whole.
void TestBatchWithDelay()
{
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
  // The delay inside the batch makes Step wait until it is done, and the whole response
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

/// Test vendor command 0x81 [0x81, x] answers [0x81, x + 1]; its length is 2.
constexpr uint8_t VENDOR_ID = 0x81;

bool VendorHook(void*, const uint8_t* req, uint16_t req_len, uint8_t* resp,
                uint16_t resp_cap, uint16_t& out_len)
{
  if (req_len < 2u || req[0] != VENDOR_ID || resp_cap < 2u) return false;
  resp[0] = VENDOR_ID;
  resp[1] = static_cast<uint8_t>(req[1] + 1u);
  out_len = 2u;
  return true;
}

uint16_t g_vendor_len = 2u;

bool VendorLength(void*, const uint8_t* req, uint16_t, uint16_t& len)
{
  if (req[0] != VENDOR_ID) return false;
  len = g_vendor_len;
  return true;
}

/// DAP_ExecuteCommands and DAP_QueueCommands run vendor commands in the batch when the
/// command hook gives their length, as DAP_ProcessVendorCommand does in the CMSIS-DAP
/// reference: [0x7F, count, responses...]. Without a length, or with a length of 0 or
/// past the request, the batch answers [0x7F, 0xFF] (DAP_ERROR).
void TestVendorCommandsInBatch()
{
  const uint8_t EXECUTE = static_cast<uint8_t>(CommandId::EXECUTE_COMMANDS);
  const uint8_t QUEUE = static_cast<uint8_t>(CommandId::QUEUE_COMMANDS);
  const std::vector<uint8_t> REPLY = {EXECUTE,   3,    INFO, 1, 0x04,
                                      VENDOR_ID, 0x11, INFO, 1, 0x04};
  for (const uint8_t ID : {EXECUTE, QUEUE})
  {
    DapFixture f;
    f.dap.BindEndpoints(f.pool, 0, false);

    // Command hook without a length: the batch cannot be sized.
    f.dap.SetCommandHook(&VendorHook, nullptr);
    Send(f, {ID, 3, INFO, PACKET_COUNT_ID, VENDOR_ID, 0x10, INFO, PACKET_COUNT_ID});
    TEST_ASSERT(f.in.active);
    TEST_ASSERT(f.in.Last().data == std::vector<uint8_t>({EXECUTE, 0xFF}));
    f.in.Complete();

    f.dap.SetCommandHook(&VendorHook, nullptr, &VendorLength);
    g_vendor_len = 2u;
    Send(f, {ID, 3, INFO, PACKET_COUNT_ID, VENDOR_ID, 0x10, INFO, PACKET_COUNT_ID});
    TEST_ASSERT(f.in.active);
    TEST_ASSERT(f.in.Last().data == REPLY);
    f.in.Complete();

    // A length past the request, or zero, is an error.
    g_vendor_len = 9u;
    Send(f, {ID, 3, INFO, PACKET_COUNT_ID, VENDOR_ID, 0x10, INFO, PACKET_COUNT_ID});
    TEST_ASSERT(f.in.active);
    TEST_ASSERT(f.in.Last().data == std::vector<uint8_t>({EXECUTE, 0xFF}));
    f.in.Complete();
    g_vendor_len = 0u;
    Send(f, {ID, 1, VENDOR_ID, 0x10});
    TEST_ASSERT(f.in.active);
    TEST_ASSERT(f.in.Last().data == std::vector<uint8_t>({EXECUTE, 0xFF}));
    f.in.Complete();
    g_vendor_len = 2u;
  }
}

/// WaitForWork waits for what the next Step(0) would do, without doing it: nothing
/// pending times out; a queued command, an unbind wake-up and a freed response slot each
/// return OK, and Step(0) then does that work.
class EchoPort final : public DapV2::VendorPort
{
 public:
  ErrorCode OnVendorOut(bool, uint8_t request, ConstRawData data) override
  {
    last_request = request;
    size = std::min<size_t>(data.size_, sizeof(buf));
    std::memcpy(buf, data.addr_, size);
    return ErrorCode::OK;
  }
  ConstRawData OnVendorIn(bool, uint8_t request) override
  {
    last_request = request;
    return {buf, size};
  }
  uint8_t last_request = 0;
  uint8_t buf[16] = {};
  size_t size = 0;
};

void TestVendorPort()
{
  // Vendor requests reach the port; without one they are NOT_SUPPORT; DAP commands are not
  // affected.
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);
  Run(f);
  SetupPacket out_req{0x41U, 0x10U, 0U, 0U, 3U};
  SetupPacket in_req{0xC1U, 0x11U, 0U, 0U, 16U};
  TestDap2::ControlTransferResult result{};
  TEST_ASSERT(f.dap.OnControlRequest(false, out_req, result) != ErrorCode::OK);

  EchoPort port;
  f.dap.SetVendorPort(&port);
  TEST_ASSERT(f.dap.OnControlRequest(false, out_req, result) == ErrorCode::OK);
  const uint8_t payload[3] = {1, 2, 3};
  ConstRawData data{payload, sizeof(payload)};
  TEST_ASSERT(f.dap.OnControlData(false, out_req, data) == ErrorCode::OK);
  TEST_ASSERT(port.last_request == 0x10U && port.size == 3U && port.buf[2] == 3U);
  result = {};
  TEST_ASSERT(f.dap.OnControlRequest(false, in_req, result) == ErrorCode::OK);
  TEST_ASSERT(port.last_request == 0x11U);
  TEST_ASSERT(result.write_data.size_ == 3U && result.write_data.addr_ == port.buf);

  // A class request is still not handled by the port.
  SetupPacket cls{0x21U, 0x01U, 0U, 0U, 0U};
  TEST_ASSERT(f.dap.OnControlRequest(false, cls, result) != ErrorCode::OK);
}

void TestWakeStep()
{
  // WakeStep wakes WaitForWork with no command, from another thread; Step(0) then finds
  // nothing to execute and returns OK, absorbing the spare wake-up.
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);
  Run(f);
  TEST_ASSERT(f.dap.WaitForWork(0) == ErrorCode::TIMEOUT);

  std::atomic<bool> woke{false};
  std::thread waiter(
      [&]()
      {
        woke.store(f.dap.WaitForWork(2000) == ErrorCode::OK);
      });
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  f.dap.WakeStep(false);
  waiter.join();
  TEST_ASSERT(woke.load());
  TEST_ASSERT(!f.in.active);
  TEST_ASSERT(f.dap.Step(0) == ErrorCode::OK);
  TEST_ASSERT(!f.in.active);  // nothing was executed
  TEST_ASSERT(f.dap.WaitForWork(0) == ErrorCode::TIMEOUT);  // the wake-up was absorbed

  // A command that arrives together with a wake-up is still executed by one Step(0).
  f.out.HostSends({INFO, PACKET_SIZE_ID});
  f.dap.WakeStep(false);
  TEST_ASSERT(f.dap.Step(0) == ErrorCode::OK);
  TEST_ASSERT(f.in.active && f.in.Last().data[0] == INFO);
  f.in.Complete();
  Run(f);
  (void)f.dap.Step(0);  // absorbs the spare wake-up
  TEST_ASSERT(f.dap.WaitForWork(0) == ErrorCode::TIMEOUT);
}

void TestWaitForWork()
{
  {
    DapFixture f;
    f.dap.BindEndpoints(f.pool, 0, false);
    Run(f);
    TEST_ASSERT(f.dap.WaitForWork(0) == ErrorCode::TIMEOUT);

    // A queued command: WaitForWork does not execute it, Step(0) does.
    f.out.HostSends({INFO, PACKET_SIZE_ID});
    TEST_ASSERT(f.dap.WaitForWork(0) == ErrorCode::OK);
    TEST_ASSERT(!f.in.active);
    TEST_ASSERT(f.dap.Step(0) == ErrorCode::OK);
    TEST_ASSERT(f.in.active && f.in.Last().data[0] == INFO);
    f.in.Complete();
    Run(f);
    TEST_ASSERT(f.dap.WaitForWork(0) == ErrorCode::TIMEOUT);

    // An unbind posts a wake-up with no command: Step(0) then applies the session end.
    f.dap.UnbindEndpoints(f.pool, false);
    TEST_ASSERT(f.dap.WaitForWork(0) == ErrorCode::OK);
    TEST_ASSERT(f.dap.Step(0) == ErrorCode::OK);
  }
  {
    // A seventh command with both IN slots and all four staging slots taken: Step(0)
    // times out holding it, WaitForWork blocks until the host takes a response, then
    // Step(0) executes it.
    DapFixture f;
    f.dap.BindEndpoints(f.pool, 0, false);
    for (unsigned i = 0; i < 7; ++i)
    {
      Send(f, {INFO, PACKET_SIZE_ID});
    }
    TEST_ASSERT(f.in.starts.size() == 1);
    TEST_ASSERT(f.dap.Step(0) == ErrorCode::TIMEOUT);
    TEST_ASSERT(f.dap.WaitForWork(0) == ErrorCode::TIMEOUT);

    std::atomic<bool> returned{false};
    ErrorCode result = ErrorCode::FAILED;
    std::thread waiter(
        [&]()
        {
          result = f.dap.WaitForWork(2000);
          returned.store(true);
        });
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    TEST_ASSERT(!returned.load());
    f.in.Complete();
    waiter.join();
    TEST_ASSERT(result == ErrorCode::OK);
    TEST_ASSERT(f.dap.Step(0) == ErrorCode::OK);

    size_t bytes = 0;
    unsigned tails = 0;
    Drain(f, bytes, tails);
    TEST_ASSERT(f.in.sent.size() == 7U);
  }
}

/// Session reset gate. Without a gate, or with one that returns true, a USB unbind
/// makes the Step thread close SWD and reset the DAP state. A gate that returns false
/// keeps SWD open, the debug port and the clock, so a session that another transport
/// runs through ExecuteExternal goes on working. A USB command received before the
/// binding changed is dropped either way.
bool g_gate_allows_reset = true;
bool ResetGate(void*) { return g_gate_allows_reset; }

/// Connect SWD and set a 4 MHz clock through ExecuteExternal, as a second transport does.
void ConnectThroughExternal(DapFixture& f)
{
  uint8_t resp[64] = {};
  uint16_t len = 0;
  const uint8_t CONNECT[] = {static_cast<uint8_t>(CommandId::CONNECT), 1};
  TEST_ASSERT(f.dap.ExecuteExternal(CONNECT, sizeof(CONNECT), resp, sizeof(resp), len) ==
              ErrorCode::OK);
  TEST_ASSERT(len == 2 && resp[1] == 1);  // DAP_Connect: SWD
  const uint8_t CLOCK[] = {static_cast<uint8_t>(CommandId::SWJ_CLOCK), 0x00, 0x09, 0x3D,
                           0x00};  // 4'000'000 Hz
  TEST_ASSERT(f.dap.ExecuteExternal(CLOCK, sizeof(CLOCK), resp, sizeof(resp), len) ==
              ErrorCode::OK);
  TEST_ASSERT(f.swd.clock_hz == 4'000'000u);
}

/// DAP_Transfer reading DP IDCODE through ExecuteExternal: true when it is answered OK.
bool ExternalDpReadOk(DapFixture& f)
{
  uint8_t resp[64] = {};
  uint16_t len = 0;
  const uint8_t READ[] = {static_cast<uint8_t>(CommandId::TRANSFER), 0, 1, 0x02};
  if (f.dap.ExecuteExternal(READ, sizeof(READ), resp, sizeof(resp), len) != ErrorCode::OK)
    return false;
  return len >= 3 && resp[1] == 1 && resp[2] == 0x01;  // one transfer, ACK OK
}

void TestSessionResetGate()
{
  for (const int MODE : {0, 1, 2})  // 0: no gate, 1: gate true, 2: gate false
  {
    DapFixture f;
    f.dap.BindEndpoints(f.pool, 0, false);
    Run(f);
    if (MODE != 0)
    {
      g_gate_allows_reset = (MODE == 1);
      f.dap.SetSessionResetGate(&ResetGate, nullptr);
    }
    ConnectThroughExternal(f);
    TEST_ASSERT(ExternalDpReadOk(f));
    const unsigned CLOSES = f.swd.closes;

    // The host closes the USB device: unbind, then the Step thread's wake-up.
    f.dap.UnbindEndpoints(f.pool, false);
    TEST_ASSERT(f.dap.Step(0) == ErrorCode::OK);
    if (MODE == 2)
    {
      TEST_ASSERT(f.swd.closes == CLOSES);
      TEST_ASSERT(f.swd.clock_hz == 4'000'000u);
      TEST_ASSERT(ExternalDpReadOk(f));
    }
    else
    {
      TEST_ASSERT(f.swd.closes == CLOSES + 1u);
      TEST_ASSERT(f.swd.clock_hz == 1'000'000u);
    }
    g_gate_allows_reset = true;
  }

  // A USB command from before the rebind is dropped even when the gate keeps the state.
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);
  g_gate_allows_reset = false;
  f.dap.SetSessionResetGate(&ResetGate, nullptr);
  for (unsigned i = 0; i < 6; ++i)
  {
    Send(f, {INFO, PACKET_SIZE_ID});
  }
  f.out.HostSends({static_cast<uint8_t>(CommandId::SWJ_SEQUENCE), 8, 0xFF});
  TEST_ASSERT(f.dap.Step(0) == ErrorCode::TIMEOUT);
  f.dap.UnbindEndpoints(f.pool, false);
  f.dap.BindEndpoints(f.pool, 0, false);
  Run(f);
  TEST_ASSERT(f.swd.sequences == 0);
  g_gate_allows_reset = true;
}

/// A DAP v2 class that advertises two packets in flight, for hosts that keep more.
using DapV2Count2 = DapLinkV2Class<FakeSwd, 512u, 2u, 1024u>;

class TestDap2Count2 : public DapV2Count2
{
 public:
  using DapV2Count2::BindEndpoints;
  using DapV2Count2::DapV2Count2;
  using DapV2Count2::OnService;
  using DapV2Count2::UnbindEndpoints;
};

struct DapFixtureCount2
{
  Block<2048> in_storage, out_storage;
  EndpointPool pool{4};
  MockEndpoint in{Number::EP1, Direction::IN, in_storage.Raw()};
  MockEndpoint out{Number::EP2, Direction::OUT, out_storage.Raw()};
  FakeSwd swd;
  TestDap2Count2 dap;
  DeviceComposition composition;
  Callback<uint32_t> handler;

  DapFixtureCount2()
      : swd(),
        dap(Number::EP1, Number::EP2, swd),
        composition(pool, {&TEST_LANGUAGE}, {{&dap}})
  {
    TEST_ASSERT(pool.Put(&in) == ErrorCode::OK);
    TEST_ASSERT(pool.Put(&out) == ErrorCode::OK);
    handler = Callback<uint32_t>::Create(
        [](bool in_isr, DeviceClass* self, uint32_t events)
        {
          if ((events & EndpointPool::CLASS_EVENT) != 0U) self->OnService(in_isr);
        },
        static_cast<DeviceClass*>(&dap));
    pool.SetControlHandler(handler);
  }
};

/// A host may keep more commands in flight than DAP_Info packet count advertises. The
/// class must hold the extra ones off (the OUT endpoint NAKs: the mock lets the host send
/// only while a receive is armed) and answer every command it received, in order: none
/// may be dropped, overwritten or answered twice. Commands alternate DAP_Info packet
/// count (3-byte answer) and packet size (4-byte answer), so the lengths on the wire show
/// the order.
void TestHostAboveAdvertisedPacketCount()
{
  for (unsigned depth = 3; depth <= 6; ++depth)
  {
    DapFixtureCount2 f;
    f.dap.BindEndpoints(f.pool, 0, false);
    constexpr unsigned TOTAL = 40;
    unsigned sent = 0;
    unsigned answered = 0;
    std::vector<size_t> lengths;
    unsigned spins = 0;
    while (answered < TOTAL && spins++ < 10000)
    {
      // The host sends as long as it may (depth) and the device accepts (armed OUT).
      while (sent < TOTAL && sent - answered < depth && f.out.active)
      {
        const uint8_t id = (sent % 2u == 0u) ? PACKET_COUNT_ID : PACKET_SIZE_ID;
        f.out.HostSends({INFO, id});
        ++sent;
      }
      // The DAP thread runs what it has.
      while (f.dap.Step(0) == ErrorCode::OK)
      {
      }
      // The host reads one response.
      if (f.in.active)
      {
        lengths.push_back(f.in.Last().length);
        f.in.Complete();
        ++answered;
      }
    }
    TEST_ASSERT(answered == TOTAL);
    TEST_ASSERT(lengths.size() == TOTAL);
    for (unsigned i = 0; i < TOTAL; ++i)
    {
      TEST_ASSERT(lengths[i] == ((i % 2u == 0u) ? 3U : 4U));
    }
    TEST_ASSERT(!f.in.active);
  }
}

/// The same with the DAP thread running concurrently and the traffic of a memory read
/// loop: a DAP_Transfer TAR write (3-byte answer) alternating with a 128-word
/// DAP_TransferBlock read (516-byte answer), the host keeping 4 in flight against an
/// advertised count of 2.
void TestHostAboveAdvertisedPacketCountThreaded()
{
  DapFixtureCount2 f;
  f.dap.BindEndpoints(f.pool, 0, false);
  std::atomic<bool> stop{false};
  std::thread dap_thread(
      [&]()
      {
        while (!stop.load())
        {
          (void)f.dap.Step(1);
        }
      });
  constexpr unsigned TOTAL = 2000;
  constexpr unsigned DEPTH = 4;
  const std::vector<uint8_t> TAR = {
      static_cast<uint8_t>(CommandId::TRANSFER), 0, 1, 0x05, 0x00, 0x00, 0x00, 0x20};
  const std::vector<uint8_t> BLOCK = {static_cast<uint8_t>(CommandId::TRANSFER_BLOCK), 0,
                                      128, 0, 0x0F};
  unsigned sent = 0;
  unsigned answered = 0;
  bool order_ok = true;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  while (answered < TOTAL && std::chrono::steady_clock::now() < deadline)
  {
    if (sent < TOTAL && sent - answered < DEPTH && f.out.active)
    {
      f.out.HostSends(sent % 2u == 0u ? TAR : BLOCK);
      ++sent;
      continue;
    }
    if (f.in.active)
    {
      const size_t want = (answered % 2u == 0u) ? 3U : 516U;
      if (f.in.Last().length != want) order_ok = false;
      f.in.Complete();
      ++answered;
      continue;
    }
    std::this_thread::yield();
  }
  stop.store(true);
  dap_thread.join();
  if (answered != TOTAL || !order_ok)
  {
    std::fprintf(stderr, "sent=%u answered=%u order_ok=%d\n", sent, answered, order_ok);
  }
  TEST_ASSERT(answered == TOTAL);
  TEST_ASSERT(order_ok);
}

/// With an advertised count of 2, the host sends before the DAP thread runs at all. The
/// OUT endpoint accepts at least three packets, then stops arming a receive (the host
/// sees NAK); every accepted command is answered in order once Step runs, and the OUT
/// endpoint is armed again afterwards.
void TestThirdCommandBeforeStepCount2()
{
  DapFixtureCount2 f;
  f.dap.BindEndpoints(f.pool, 0, false);
  constexpr unsigned LIMIT = 16;
  unsigned accepted = 0;
  while (accepted < LIMIT && f.out.active)
  {
    f.out.HostSends({INFO, (accepted % 2u == 0u) ? PACKET_COUNT_ID : PACKET_SIZE_ID});
    ++accepted;
  }
  TEST_ASSERT(accepted >= 3u);
  TEST_ASSERT(accepted < LIMIT);  // the endpoint NAKs before any command ran
  TEST_ASSERT(!f.out.active);
  std::vector<size_t> lengths;
  for (unsigned guard = 0; guard < 4u * LIMIT && lengths.size() < accepted; ++guard)
  {
    while (f.dap.Step(0) == ErrorCode::OK)
    {
    }
    if (f.in.active)
    {
      lengths.push_back(f.in.Last().length);
      f.in.Complete();
    }
  }
  TEST_ASSERT(lengths.size() == accepted);
  for (unsigned i = 0; i < accepted; ++i)
  {
    TEST_ASSERT(lengths[i] == ((i % 2u == 0u) ? 3U : 4U));
  }
  TEST_ASSERT(!f.in.active);
  TEST_ASSERT(f.out.active);  // armed again once command slots came back
}

/// A batch whose responses do not fit the response buffer answers DAP_ERROR, also when a
/// delay ran first.
void TestBatchOverflowAnswersError()
{
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

/// CMSIS-DAP: DAP_Delay answers [DAP_Delay, DAP_OK] after the delay, and every packet of
/// DAP_ExecuteCommands answers its own commands. An unbind and bind while a batch with a
/// delay is executing ends that session: nothing of the old batch reaches the host, and
/// the next standalone DAP_Delay of the new session answers normally.
void TestRebindDuringBatchThenDelay()
{
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);
  const uint8_t EXECUTE = static_cast<uint8_t>(CommandId::EXECUTE_COMMANDS);

  // [EXECUTE, 2, DAP_Delay 20000 us, DAP_Info(PACKET_SIZE)].
  f.out.HostSends({EXECUTE, 2, DELAY, 0x20, 0x4E, INFO, PACKET_SIZE_ID});
  std::thread dap_thread([&f]() { (void)f.dap.Step(1000); });
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  f.dap.UnbindEndpoints(f.pool, false);
  dap_thread.join();
  Run(f);  // the application's loop goes on stepping while unbound
  f.dap.BindEndpoints(f.pool, 0, false);
  Run(f);
  TEST_ASSERT(f.in.starts.empty());

  Send(f, {DELAY, 0x10, 0x00});  // 16 us
  TEST_ASSERT(f.in.active && f.in.starts.size() == 1);
  TEST_ASSERT(f.in.Last().data == std::vector<uint8_t>({DELAY, 0x00}));
  f.in.Complete();
  Run(f);
  TEST_ASSERT(!f.in.active && f.out.active);
}

/// A command that Step holds while it waits for a response buffer (Step returned on its
/// timeout) belongs to the session it was received in. After an unbind and bind it must
/// not reach the target: the next Step drops it instead of executing it.
void TestHeldCommandDroppedAfterRebind()
{
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);

  // Six answered commands fill both IN slots and the four staging slots; the host reads
  // none of them.
  for (unsigned i = 0; i < 6; ++i)
  {
    Send(f, {INFO, PACKET_SIZE_ID});
  }
  // A seventh command (DAP_SWJ_Sequence, 8 bits) finds no response buffer: Step(0) times
  // out still holding it.
  f.out.HostSends({static_cast<uint8_t>(CommandId::SWJ_SEQUENCE), 8, 0xFF});
  TEST_ASSERT(f.dap.Step(0) == ErrorCode::TIMEOUT);
  TEST_ASSERT(f.swd.sequences == 0);

  f.dap.UnbindEndpoints(f.pool, false);
  f.dap.BindEndpoints(f.pool, 0, false);
  const size_t answered = f.in.starts.size();
  Run(f);
  TEST_ASSERT(f.swd.sequences == 0);
  TEST_ASSERT(f.in.starts.size() == answered);

  // The new session works normally.
  Send(f, {INFO, PACKET_SIZE_ID});
  TEST_ASSERT(f.in.starts.size() == answered + 1);
  TEST_ASSERT(f.in.Last().data[0] == INFO);
  f.in.Complete();
}

/// SWD belongs to the thread that runs Step: bind and unbind (the USB owner context, here
/// the test's main thread) make no SWD call at all. The session end and start are applied
/// by the Step thread: it closes SWD and sets the default clock before the first command
/// of the new session, and every SWD call comes from that thread.
void TestBindAndUnbindLeaveSwdToStep()
{
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);
  f.dap.UnbindEndpoints(f.pool, false);
  f.dap.BindEndpoints(f.pool, 0, false);
  TEST_ASSERT(f.swd.callers.empty());

  f.out.HostSends({CONNECT, static_cast<uint8_t>(Port::SWD)});
  std::thread dap_thread([&f]() { (void)f.dap.Step(1000); });
  dap_thread.join();
  TEST_ASSERT(f.in.active && f.in.Last().data[0] == CONNECT);
  f.in.Complete();
  TEST_ASSERT(f.swd.enters == 1 && f.swd.clock_hz == 1000000u);

  f.dap.UnbindEndpoints(f.pool, false);
  const size_t before = f.swd.callers.size();
  TEST_ASSERT(f.swd.closes == 1);  // the session start closed SWD once
  std::thread end_thread([&f]() { (void)f.dap.Step(1000); });
  end_thread.join();
  TEST_ASSERT(f.swd.closes == 2);  // the session end, applied by Step
  TEST_ASSERT(f.swd.callers.size() > before);
  for (const auto& id : f.swd.callers)
  {
    TEST_ASSERT(id != std::this_thread::get_id());
  }
}

/// Padded requests with the slots full: the padding of the last admitted request is still
/// taken (it needs no command slot), so the host's write finishes, it reads the responses
/// and everything drains. A fifth command's first packet waits as a lease until a command
/// slot is free.
void TestPaddedRequestsAndAFifthCommandDrain()
{
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

/// 命令池满时到达的包被类持有；解绑要把这些 OUT 槽还给端点，否则下一次会话收不到
/// 命令（PR 评审发现）。
/// Packets arriving while the command pool is full are held by the class; unbinding must
/// return those OUT slots to the endpoint, or the next session receives no commands
/// (found in PR review).
void TestUnbindReturnsHeldPackets()
{
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);
  // 不运行 Step：命令池填满后，再来的包被持有，接收不再挂起。
  // Step does not run: once the command pool is full, further packets are held and no
  // receive stays armed.
  unsigned sent = 0;
  while (f.out.active && sent < 16)
  {
    f.out.HostSends({INFO, PACKET_COUNT_ID});
    ++sent;
  }
  TEST_ASSERT(!f.out.active && sent >= 3 && sent < 16);

  f.dap.UnbindEndpoints(f.pool, false);
  f.dap.BindEndpoints(f.pool, 0, false);
  Run(f);
  // 两个 OUT 槽都可用：先后两个命令都能收下 / Both OUT slots are usable: two commands in
  // a row are taken.
  TEST_ASSERT(f.out.active);
  Send(f, {INFO, PACKET_COUNT_ID});
  TEST_ASSERT(f.in.active && f.in.Last().length == 3);
  f.in.Complete();
  Run(f);
  TEST_ASSERT(f.out.active);
  Send(f, {INFO, PACKET_SIZE_ID});
  TEST_ASSERT(f.in.active && f.in.Last().length == 4);
  f.in.Complete();
}
}  // namespace

int main()
{
  TestCommandRoundTrip();
  TestPipelineKeepsAdvertisedDepth();
  TestResponseNeedsNoClassService();
  TestDirectAndStagedResponsesKeepOrder();
  TestUnbindWhileHoldingInSlot();
  TestUnbindReturnsHeldPackets();
  TestDelayedCommand();
  TestBatchWithDelay();
  TestNextCommandReceivedDuringDelay();
  TestZlpOnlyBelowDapPacketSize();
  TestCommandOnPacketBoundaryEndsWithoutZlp();
  TestZeroPaddedTransferIsDropped();
  TestTransferAbortHasNoResponse();
  TestQueueAndExecuteAnswerPerPacket();
  TestVendorCommandsInBatch();
  TestWaitForWork();
  TestWakeStep();
  TestVendorPort();
  TestSessionResetGate();
  TestHostAboveAdvertisedPacketCount();
  TestHostAboveAdvertisedPacketCountThreaded();
  TestThirdCommandBeforeStepCount2();
  TestBatchOverflowAnswersError();
  TestRebindDuringBatchThenDelay();
  TestHeldCommandDroppedAfterRebind();
  TestBindAndUnbindLeaveSwdToStep();
  TestPaddedRequestsAndAFifthCommandDrain();
  TestStandaloneDeviceUsesDeviceLevelWinUsbSet();
  TestCompositeDeviceUsesFunctionSubsets();
  TestUnbind();
  std::puts(
      "DAP v2 class: borrow-mode receive and submit, response depth and delays "
      "passed");
}
