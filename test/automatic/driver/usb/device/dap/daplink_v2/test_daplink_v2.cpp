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
#include <chrono>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <string>
#include <thread>
#include <vector>

#include "daplink_v2.hpp"
#include "debug/swd.hpp"
#include "device_composition.hpp"
#include "driver/usb/device/dap/dap_test_common.hpp"
#include "driver/usb/usb_test_common.hpp"

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

using DapV2 = DapLinkV2Class<FakeSwd>;

class TestDap2 : public DapV2
{
 public:
  using DapV2::BindEndpoints;
  using DapV2::DapV2;
  using DapV2::OnService;
  using DapV2::UnbindEndpoints;
};

/// 该类、其 SWD 探针，以及为其提供类服务池的组合。端点存储块每个方向有两个 1024 字节
/// 的槽，与真实板子一致。
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
    // 代替 DeviceCore 的类服务分发。
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

/// 按应用 DAP 线程的方式执行所有已到达的命令。等待 IN 槽的命令会使 Step(0) 超时并结束循环
/// 。
/// Run every command that has arrived, the way the application's DAP thread would. A
/// command that waits for an IN slot makes Step(0) time out and ends the loop.
void Run(DapFixture& f)
{
  while (f.dap.Step(0) == ErrorCode::OK)
  {
  }
}

/// 主机完成以这些字节挂出的请求；由此完成的命令随后由 Step() 执行。
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

/// 完成一个响应的所有硬件请求；统计字节数和零长度的尾包。
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

  // 绑定时两个 OUT 槽各挂一个端点包；硬件上同一时刻只有一个请求。
  // Binding arms both OUT slots with one endpoint packet each; only one request is on
  // the hardware at a time.
  TEST_ASSERT(f.out.active && f.out.Last().length == 64);
  TEST_ASSERT(f.in.starts.empty());

  // DAP_Info(PACKET_COUNT) 回答实际生效的未完成深度。
  // DAP_Info(PACKET_COUNT) answers the effective outstanding depth.
  Send(f, {INFO, PACKET_COUNT_ID});
  TEST_ASSERT(f.in.Last().length == 3);
  TEST_ASSERT(f.in.Last().data == std::vector<uint8_t>({INFO, 1, 0x04}));
  f.in.Complete();

  // connect 命令到达 SWD 探针并回答端口。
  // A connect command reaches the SWD probe and answers the port.
  Send(f, {CONNECT, static_cast<uint8_t>(Port::SWD)});
  TEST_ASSERT(f.in.Last().length == 2);
  TEST_ASSERT(f.in.Last().data[0] == CONNECT &&
              f.in.Last().data[1] == static_cast<uint8_t>(Port::SWD));
  TEST_ASSERT(f.swd.enters == 1 && f.swd.clock_hz == 1000000u);
  f.in.Complete();

  // OUT 方向始终保持两个请求处于挂包状态。
  // The OUT side keeps two requests armed throughout.
  TEST_ASSERT(f.out.active);
}

/// 四条命令在途：Step 线程领先于主机把它们全部执行。前两个响应直接进入两个 IN 槽；
/// 另外两个在暂存槽里等待，随着主机取走响应，按命令顺序提交，无需 Step 再次运行。
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
  // 每条命令都已执行；一个响应在总线上，第二个在另一个 IN 槽里排在它后面，
  // 另外两个在暂存槽里等待。
  // Every command has executed already; one response is on the wire, a second is queued
  // behind it in the other IN slot, and two wait in staging slots.
  TEST_ASSERT(f.in.starts.size() == 1);
  // 没有可执行的了 / nothing left to execute
  TEST_ASSERT(f.dap.Step(0) == ErrorCode::TIMEOUT);

  // 主机每取走一个响应，USB 一侧就提交下一个暂存的响应。
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

/// IN 槽空闲且没有暂存内容时，Step 把响应写进 IN 槽并自行提交：不经过类服务即可发到
/// 总线上。
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

/// 写入 IN 槽的响应与暂存的响应交错出现：它们仍按命令顺序发出。PACKET_COUNT 回答 3
/// 字节，PACKET_SIZE 回答 4 字节，所以总线上的长度能看出顺序。
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
  // 两个响应直接占用两个 IN 槽；接下来的两个被暂存。
  // Two responses take both IN slots directly; the next two are staged.
  send(PACKET_COUNT_ID);
  send(PACKET_SIZE_ID);
  send(PACKET_COUNT_ID);
  send(PACKET_SIZE_ID);
  TEST_ASSERT(f.in.starts.size() == 1);
  // 主机取走一个；暂存的响应补入该槽。新命令遇到仍在等待的暂存响应，就排在它后面暂存。
  // The host takes one; a staged response fills the slot. A new command meets a staged
  // response still waiting and is staged behind it.
  f.in.Complete();
  send(PACKET_COUNT_ID);
  while (f.in.active)
  {
    f.in.Complete();
    Run(f);
  }
  // 全部排空：下一个响应再次直接进入 IN 槽。
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

/// Step 为延时响应持有 IN 槽时发生解绑：该槽被归还，重新绑定后两个 IN 槽都空闲。
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
  // 应用循环再次 Step；处于未绑定状态时，Step 归还它持有的内容。
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

  // 延时命令只是被 USB 一侧收到；在 DAP 线程 Step 之前不执行也不应答，OUT 槽继续接收。
  // A delay command is only received by the USB side; nothing is executed or answered
  // until the DAP thread steps, and the OUT slots keep receiving.
  f.out.HostSends({DELAY, 0x10, 0x27});  // 10000 us
  TEST_ASSERT(f.in.starts.empty());
  TEST_ASSERT(f.out.active);

  // Step() 在调用线程里等完延时，然后应答。
  // Step() waits the delay out in the calling thread and then answers.
  TEST_ASSERT(f.dap.Step(0) == ErrorCode::OK);
  TEST_ASSERT(f.in.active && f.in.Last().length == 2);
  TEST_ASSERT(f.in.Last().data == std::vector<uint8_t>({DELAY, 0x00}));
  f.in.Complete();

  // 零延时立即应答。
  // A zero delay answers at once.
  Send(f, {DELAY, 0x00, 0x00});
  TEST_ASSERT(f.in.active && f.in.Last().data == std::vector<uint8_t>({DELAY, 0x00}));
  f.in.Complete();
}

/// 包含延时命令的批处理：延时在 Step 线程里等完，批处理响应整体发出。
/// A batch that contains a delay command: the delay is waited out in the Step thread and
/// the batch response goes out whole.
void TestBatchWithDelay()
{
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);

  // 带两条命令的 QUEUE_COMMANDS：DAP_Info 和一个 2 ms 的 DAP_Delay。
  // 线上格式是简单拼接（每条命令的长度由其 ID 决定，没有前缀）。
  // 该包按 EXECUTE_COMMANDS 执行，与 CMSIS-DAP 参考实现一致。
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
      // 2000 us，小端 / 2000 us little-endian
      0x07,
  };
  // 批处理里的延时使 Step 等到它完成，整个响应一次到达：[EXECUTE, count, INFO
  // 响应, DELAY 响应]，不会再有多余的第二个响应。
  // The delay inside the batch makes Step wait until it is done, and the whole response
  // arrives at once: [EXECUTE, count, INFO response, DELAY response], with no second
  // stray response.
  Host(f, queue);
  TEST_ASSERT(f.in.active);
  const auto& batch = f.in.Last().data;
  // INFO 响应：[INFO, len=7, "XRobot" + NUL]，共 9 字节（批处理 [2..10]）；DELAY 响应
  // [DELAY, 0x00] 随后位于 [11..12]，批处理总长 13。
  // INFO response: [INFO, len=7, "XRobot" + NUL], 9 bytes total (batch [2..10]); the
  // DELAY response [DELAY, 0x00] follows at [11..12], batch length 13.
  TEST_ASSERT(batch[0] == static_cast<uint8_t>(CommandId::EXECUTE_COMMANDS));
  // 命令数 / number of commands
  TEST_ASSERT(batch[1] == 2U);
  TEST_ASSERT(batch[2] == static_cast<uint8_t>(CommandId::INFO));
  // "XRobot"（含 NUL）/ "XRobot" incl. NUL
  TEST_ASSERT(batch[3] == 7U);
  // "XRobot" 的 NUL / NUL of "XRobot"
  TEST_ASSERT(batch[10] == 0x00);
  TEST_ASSERT(batch[11] == DELAY);
  TEST_ASSERT(batch[12] == 0x00);  // DAP_OK
  TEST_ASSERT(f.in.Last().length == 13U);
  f.in.Complete();
  TEST_ASSERT(f.out.active);
}

/// 上一条命令执行期间（这里是延时）就收到下一条命令，并在其之后按顺序执行。
/// The next command is received while the previous one executes (here: a delay) and runs
/// after it, in order.
void TestNextCommandReceivedDuringDelay()
{
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);

  f.out.HostSends({DELAY, 0x10, 0x27});  // 10000 us
  std::thread dap_thread([&f]() { (void)f.dap.Step(1000); });
  std::this_thread::sleep_for(std::chrono::milliseconds(2));

  // 延时挂起期间主机发来下一条命令：它被收下（USB 一侧不等待 DAP 线程），但不执行，
  // 也没有应答。
  // The host sends the next command while the delay is pending: it is received (the USB
  // side does not wait for the DAP thread) but not executed, and nothing answers.
  f.out.HostSends({INFO, PACKET_SIZE_ID});
  dap_thread.join();

  // 延时响应先发；收到的命令在其之后应答。
  // The delay response goes first; the received command answers after it.
  TEST_ASSERT(f.in.active && f.in.Last().data == std::vector<uint8_t>({DELAY, 0x00}));
  f.in.Complete();
  Run(f);
  TEST_ASSERT(f.in.active && f.in.Last().data[0] == INFO);
  f.in.Complete();
  TEST_ASSERT(!f.in.active && f.out.active);
}

/// 与 DAP 包等长的响应不需要 ZLP（主机按包大小读取，读满即结束）；若发 ZLP，它会一直
/// 挂在端点上并占用一个 IN 槽。比 DAP 包短、且是端点包大小整数倍的响应仍以 ZLP 结束。
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

  // 64 字节 = 一个端点包，比 DAP 包短：后面跟一个 ZLP。
  // 64 bytes = one endpoint packet, shorter than the DAP packet: a ZLP follows.
  // [INFO, 62, 61 个字符 + NUL] = 64 字节 / [INFO, 62, 61 chars + NUL] = 64 bytes
  const std::string vendor(61, 'V');
  DapV2::InfoStrings info{};
  info.vendor = vendor.c_str();
  f.dap.SetInfoStrings(info);
  Send(f, {INFO, static_cast<uint8_t>(InfoId::VENDOR)});
  Drain(f, bytes, tails);
  TEST_ASSERT(bytes == 64 && tails == 1);

  // 1024 字节 = DAP 包大小（读 255 个字的块）：没有 ZLP。
  // 1024 bytes = the DAP packet size (a 255-word block read): no ZLP.
  Send(f, {CONNECT, static_cast<uint8_t>(Port::SWD)});
  Drain(f, bytes, tails);
  Send(f, {static_cast<uint8_t>(CommandId::TRANSFER_BLOCK), 0x00, 0xFF, 0x00, 0x0F});
  Drain(f, bytes, tails);
  TEST_ASSERT(bytes == 1024 && tails == 0);
}

/// 命令长度是端点包大小的整数倍、且主机不发 ZLP：命令按自己的长度结束，而不是等超时。
/// A command whose length is a multiple of the endpoint packet size and no ZLP from the
/// host: it ends by the command's own length instead of waiting for a timeout.
void TestCommandOnPacketBoundaryEndsWithoutZlp()
{
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);

  // 带 63 条 DAP_Info 命令的 QUEUE_COMMANDS：2 + 63 * 2 = 128 字节，两个包。
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
  // 第一个包不能使命令完整；第二个槽已为第二个包挂好。还没有任何应答。
  // The first packet does not complete the command; the second slot is already armed for
  // the second packet. Nothing has been answered.
  TEST_ASSERT(f.out.active && f.out.Last().length == 64);
  TEST_ASSERT(f.in.starts.empty());
  Host(f, std::vector<uint8_t>(queue.begin() + 64, queue.end()));
  // 第二个整包之后命令完整：开始应答。
  // The command is complete after the second whole packet: it answers.
  TEST_ASSERT(f.in.active);
  TEST_ASSERT(f.in.Last().data[0] == static_cast<uint8_t>(CommandId::EXECUTE_COMMANDS));
}

/// 主机把传输用零补齐到 DAP 包大小：补零部分不会被当作命令，
/// 并按内容允许的最少请求数来接收，命令只执行一次。
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
  // 命令在第一个包内就完整 / the command is complete inside the first packet
  Host(f, first);
  TEST_ASSERT(f.in.active && f.in.starts.size() == 1);
  f.in.Complete();

  // 下一个整包全为零：是补零。1024 字节传输的其余部分（896 字节）用一个请求接收，叠加
  // 在已排队的包之上。
  // The next whole packet is all zero: padding. The rest of the 1024 byte transfer (896
  // bytes) is asked for in one request on top of the packet that is already queued.
  Host(f, std::vector<uint8_t>(64, 0));
  // 已排队的补零包 / the queued padding packet
  TEST_ASSERT(f.out.active && f.out.Last().length == 64);
  Host(f, std::vector<uint8_t>(64, 0));
  // 一次接收其余部分 / the rest at once
  TEST_ASSERT(f.out.active && f.out.Last().length == 1024 - 192);
  Host(f, std::vector<uint8_t>(1024 - 192, 0));
  TEST_ASSERT(f.out.active && f.out.Last().length == 64);
  // 补零没有响应 / no response to padding
  TEST_ASSERT(f.in.starts.size() == 1);

  // 之后真实命令仍能正常工作。
  // A real command still works afterwards.
  Send(f, {INFO, PACKET_SIZE_ID});
  TEST_ASSERT(f.in.active && f.in.starts.size() == 2);
  f.in.Complete();
}

/// DAP_TransferAbort 没有响应，也不占用命令槽；接收继续。
/// DAP_TransferAbort has no response and does not take a command slot; reception goes on.
void TestTransferAbortHasNoResponse()
{
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);

  Send(f, {static_cast<uint8_t>(CommandId::TRANSFER_ABORT)});
  TEST_ASSERT(f.in.starts.empty() && !f.in.active);
  TEST_ASSERT(f.out.active);

  // 之后启动的传输不会被过期标志中止。
  // A transfer started afterwards is not aborted by the stale flag.
  Send(f, {CONNECT, static_cast<uint8_t>(Port::SWD)});
  f.in.Complete();
  // 读 DP 0 / read DP 0
  Send(f, {static_cast<uint8_t>(CommandId::TRANSFER), 0x00, 0x01, 0x02});
  TEST_ASSERT(f.in.active);
  TEST_ASSERT(f.in.Last().data[0] == static_cast<uint8_t>(CommandId::TRANSFER));
  // 完成一次传输 / one transfer completed
  TEST_ASSERT(f.in.Last().data[1] == 1);
  f.in.Complete();
}

/// 每个 QUEUE_COMMANDS 或 EXECUTE_COMMANDS 包都执行自己的命令，并回答
/// [EXECUTE, count, responses...]；不会有内容延续到下一个包。
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

  // 缺少命令数的包是畸形的。
  // A packet without the command count is malformed.
  Send(f, {EXECUTE});
  TEST_ASSERT(f.in.active);
  TEST_ASSERT(f.in.Last().data == std::vector<uint8_t>({EXECUTE, 0xFF}));
  f.in.Complete();
}

/// 响应放不进响应缓冲区的批处理回答 DAP_ERROR，先执行了延时时也是如此。
/// A batch whose responses do not fit the response buffer answers DAP_ERROR, also when a
/// delay ran first.
void TestBatchOverflowAnswersError()
{
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);
  const uint8_t EXECUTE = static_cast<uint8_t>(CommandId::EXECUTE_COMMANDS);

  // 一个 1 ms 的延时，然后是 120 个 9 字节的 DAP_Info(vendor) 应答：超过一个响应。
  // A 1 ms delay, then 120 DAP_Info(vendor) answers of 9 bytes: more than one response.
  std::vector<uint8_t> req{EXECUTE, 121u, DELAY, 0xE8, 0x03};
  for (unsigned i = 0; i < 120; ++i)
  {
    req.push_back(INFO);
    req.push_back(static_cast<uint8_t>(InfoId::VENDOR));
  }
  // 主机按端点包写入。
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

/// CMSIS-DAP：DAP_Delay 在延时之后回答 [DAP_Delay, DAP_OK]，
/// DAP_ExecuteCommands 的每个包回答自己的命令。带延时的批处理执行期间发生解绑再绑定，
/// 会结束该会话：旧批处理的内容不会到达主机，新会话中下一个独立的 DAP_Delay 正常应答。
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
  // 未绑定期间应用循环继续 Step / the application's loop goes on stepping while unbound
  Run(f);
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

/// Step 等待响应缓冲区期间持有的命令（Step 因超时返回）属于收到它的会话。解绑再绑定
/// 之后它不得到达目标：下一次 Step 丢弃它而不是执行它。
/// A command that Step holds while it waits for a response buffer (Step returned on its
/// timeout) belongs to the session it was received in. After an unbind and bind it must
/// not reach the target: the next Step drops it instead of executing it.
void TestHeldCommandDroppedAfterRebind()
{
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);

  // 六条已应答的命令占满两个 IN 槽和四个暂存槽；主机一个也没有读。
  // Six answered commands fill both IN slots and the four staging slots; the host reads
  // none of them.
  for (unsigned i = 0; i < 6; ++i)
  {
    Send(f, {INFO, PACKET_SIZE_ID});
  }
  // 第七条命令（DAP_SWJ_Sequence，8 位）找不到响应缓冲区：Step(0) 超时，仍持有该命令。
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

  // 新会话正常工作。
  // The new session works normally.
  Send(f, {INFO, PACKET_SIZE_ID});
  TEST_ASSERT(f.in.starts.size() == answered + 1);
  TEST_ASSERT(f.in.Last().data[0] == INFO);
  f.in.Complete();
}

/// 命令执行期间发生解绑再绑定，而它的响应走暂存槽：响应带着旧会话的 epoch 发布，USB 侧
/// 发送时丢弃它，新会话的主机收不到。暂存槽和计数随之释放，新会话的响应照常发出。
/// An unbind and bind while a command executes whose response goes through a staging
/// slot: the response is published with the epoch of the old session and the USB side
/// drops it when it sends, so the host of the new session does not receive it. The
/// staging slot and the count are released with it and the new session answers normally.
void TestStagedResponseOfOldSessionIsDropped()
{
  DapFixture f;
  f.dap.BindEndpoints(f.pool, 0, false);

  // 两条已应答的命令占满两个 IN 槽，主机没有读：下一条命令的响应走暂存槽。
  // Two answered commands fill both IN slots and the host reads neither: the response
  // of the next command goes through a staging slot.
  Send(f, {INFO, PACKET_SIZE_ID});
  Send(f, {INFO, PACKET_SIZE_ID});
  f.swd.on_sequence = [&f]()
  {
    f.dap.UnbindEndpoints(f.pool, false);
    f.dap.BindEndpoints(f.pool, 0, false);
  };
  const size_t answered = f.in.starts.size();
  Send(f, {static_cast<uint8_t>(CommandId::SWJ_SEQUENCE), 8, 0xFF});
  TEST_ASSERT(f.swd.sequences == 1 && !f.swd.on_sequence);
  TEST_ASSERT(f.in.starts.size() == answered && !f.in.active);

  // 新会话：四条命令的响应按顺序发出，没有旧会话的响应夹在中间；第三、第四条用到暂存槽。
  // The new session: the responses of four commands go out in order with no response of
  // the old session among them; the third and the fourth use staging slots.
  for (unsigned i = 0; i < 4; ++i)
  {
    Send(f, {INFO, i % 2U == 0U ? PACKET_COUNT_ID : PACKET_SIZE_ID});
  }
  const size_t before = f.in.sent.size();
  while (f.in.active)
  {
    f.in.Complete();
    Run(f);
  }
  TEST_ASSERT(f.in.sent.size() == before + 4);
  for (unsigned i = 0; i < 4; ++i)
  {
    const auto& response = f.in.sent[before + i];
    TEST_ASSERT(response.size() == (i % 2U == 0U ? 3U : 4U) && response[0] == INFO);
  }
}

/// SWD 归运行 Step 的线程所有：绑定和解绑（USB owner 上下文，这里是测试主线程）完全
/// 不调用 SWD。会话的结束和开始由 Step 线程执行：它在新会话的第一条命令之前关闭 SWD
/// 并设置默认时钟，所有 SWD 调用都来自该线程。
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
  // 会话开始时关闭了一次 SWD / the session start closed SWD once
  TEST_ASSERT(f.swd.closes == 1);
  std::thread end_thread([&f]() { (void)f.dap.Step(1000); });
  end_thread.join();
  // 会话结束，由 Step 执行 / the session end, applied by Step
  TEST_ASSERT(f.swd.closes == 2);
  TEST_ASSERT(f.swd.callers.size() > before);
  for (const auto& id : f.swd.callers)
  {
    TEST_ASSERT(id != std::this_thread::get_id());
  }
}

/// 槽已满时的补零请求：最后一个被接纳请求的补零部分仍会被接收（它不需要命令槽），所以
/// 主机的写入能完成，随后读出响应，一切排空。第五条命令的第一个包作为租约等待，直到
/// 有命令槽空闲。
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
    // 识别出补零 / detects the padding
    Host(f, std::vector<uint8_t>(64, 0));
    // 其余部分作为一个请求，叠加在已排队的内容之上到达。
    // The rest arrives as one request on top of what is queued.
    Host(f, std::vector<uint8_t>(64, 0));
    Host(f, std::vector<uint8_t>(1024 - 192, 0));
  };
  for (unsigned i = 0; i < 4; ++i)
  {
    TEST_ASSERT(f.out.active);
    padded();
  }
  // 总线上一个响应，其余等待 / one response on the wire, the rest wait
  TEST_ASSERT(f.in.starts.size() == 1);
  TEST_ASSERT(f.out.active);

  // 第五条命令：它的第一个包作为租约保留，没有空闲的命令槽。
  // A fifth command: its first packet is kept as a lease, no command slot is free.
  std::vector<uint8_t> fifth(64, 0x11);
  fifth[0] = INFO;
  fifth[1] = PACKET_SIZE_ID;
  Host(f, fifth);
  TEST_ASSERT(f.in.starts.size() == 1);

  // 主机读取：每个响应都发出，第五条命令最后执行。
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

/// 单功能设备从组合得到设备级的 MS OS 2.0 描述符集（Windows 在非复合设备上会忽略功能子集
/// 头），并且它与设备级形式定义的结构逐字节一致。
/// A single-function device gets the device-level MS OS 2.0 descriptor set from the
/// composition (Windows ignores the function subset header on a non-composite device),
/// and it is byte for byte the structure the device-level form defines.
void TestStandaloneDeviceUsesDeviceLevelWinUsbSet()
{
  // DAP 是其组合中唯一的功能 / DAP is the only function of its composition
  DapFixture f;
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

/// 一个复合设备中的两个 WinUSB 功能：一个配置子集，每个功能一个功能子集，
/// 接口号按各类的顺序排列。
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
  // 子集头 + compat + GUID / subset header + compat + GUID
  constexpr size_t FUNCTION = 8 + 20 + sizeof(Property);
  const auto set = ReadWinUsbSet(composition);
  const auto* d = static_cast<const uint8_t*>(set.addr_);
  TEST_ASSERT(set.size_ == 10 + 8 + 2 * FUNCTION);
  // 描述符集头：wTotalLength / set header: wTotalLength
  TEST_ASSERT(LoadU16(d + 8) == set.size_);
  // 配置子集 / configuration subset
  TEST_ASSERT(LoadU16(d + 10 + 2) == 0x0001);
  // 覆盖两个功能 / covers both functions
  TEST_ASSERT(LoadU16(d + 10 + 6) == set.size_ - 10);
  const uint8_t* first = d + 18;
  const uint8_t* second = first + FUNCTION;
  // 功能子集，接口 0 / function subset, itf 0
  TEST_ASSERT(LoadU16(first + 2) == 0x0002 && first[4] == 0);
  TEST_ASSERT(LoadU16(first + 6) == FUNCTION);
  // 功能子集，接口 1 / function subset, itf 1
  TEST_ASSERT(LoadU16(second + 2) == 0x0002 && second[4] == 1);
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
  // 重新绑定后正常处理下一条命令。
  // Rebinding serves the next command normally.
  f.dap.BindEndpoints(f.pool, 0, false);
  TEST_ASSERT(f.out.active && f.out.Last().length == 64);
  Send(f, {INFO, PACKET_SIZE_ID});
  TEST_ASSERT(f.in.active && f.in.Last().length == 4);
  f.in.Complete();
}
}  // namespace

void test_usb_daplink_v2()
{
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
  TestBatchOverflowAnswersError();
  TestRebindDuringBatchThenDelay();
  TestHeldCommandDroppedAfterRebind();
  TestStagedResponseOfOldSessionIsDropped();
  TestBindAndUnbindLeaveSwdToStep();
  TestPaddedRequestsAndAFifthCommandDrain();
  TestStandaloneDeviceUsesDeviceLevelWinUsbSet();
  TestCompositeDeviceUsesFunctionSubsets();
  TestUnbind();
}
