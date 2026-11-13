/**
 * @file test_daplink_v1.cpp
 * @brief DAP v1 设备类（槽模型）主机测试 / Host tests for the DAP v1 device class.
 *
 * DAP 命令由应用线程调用 Step() 执行：owner 只把 OUT 租约交给 Step 线程，响应提交成功
 * 后才归还租约（背压）。本测试用假 SWD 探针，在测试线程里调用 Step()，覆盖收发往返、
 * 背压、IN 槽用完后的提交、两个报告背靠背到达、以及解绑/重新绑定语义。
 * DAP commands run when the application thread calls Step(): the owner only hands the
 * OUT lease to the Step thread, and the lease goes back only after the response was
 * accepted (backpressure). The test uses a fake SWD probe and calls Step() from the test
 * threads, covering the request/response round trip, backpressure, the submission after
 * the IN slots ran out, two reports arriving back to back, and unbind/rebind semantics.
 */

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <thread>
#include <vector>

#include "daplink_v1.hpp"
#include "debug/swd.hpp"
#include "driver/usb/device/dap/dap_test_common.hpp"
#include "driver/usb/usb_test_common.hpp"

using namespace LibXR;
using namespace LibXR::USB;
using namespace LibXR::USB::Test;

namespace
{
using CommandId = LibXR::USB::DapLinkV1Def::CommandId;
using InfoId = LibXR::USB::DapLinkV1Def::InfoId;
using Port = LibXR::USB::DapLinkV1Def::Port;
using SwdRequest = LibXR::Debug::SwdProtocol::Request;
using SwdResponse = LibXR::Debug::SwdProtocol::Response;

class TestDap : public DapLinkV1Class<FakeSwd>
{
 public:
  TestDap(Endpoint::EPNumber in, Endpoint::EPNumber out, FakeSwd& swd)
      : DapLinkV1Class<FakeSwd>(in, out, swd)
  {
  }
  using DapLinkV1Class<FakeSwd>::BindEndpoints;
  using DapLinkV1Class<FakeSwd>::SendInputReport;
  using DapLinkV1Class<FakeSwd>::UnbindEndpoints;
};

/// A 64-byte report with the given command prefix; the tail stays zero.
std::vector<uint8_t> Report(std::initializer_list<uint8_t> prefix)
{
  std::vector<uint8_t> data(64, 0);
  size_t i = 0;
  for (uint8_t byte : prefix) data.at(i++) = byte;
  return data;
}

std::vector<uint8_t> InfoReport(InfoId id)
{
  return Report({static_cast<uint8_t>(CommandId::INFO), static_cast<uint8_t>(id)});
}

/// Run every command that has arrived, the way the application's DAP thread would. A
/// response that waits for an IN slot makes Step(0) time out and ends the loop.
void Run(TestDap& dap)
{
  while (dap.Step(0) == ErrorCode::OK)
  {
  }
}

void TestInfoRoundTrip()
{
  FakeSwd swd;
  Fixture f;
  TestDap dap(Number::EP1, Number::EP2, swd);
  dap.BindEndpoints(f.pool, 0, false);
  // One OUT lease armed, one report per lease.
  TEST_ASSERT(f.out.active && f.out.Last().length == 64 && f.out.starts.size() == 1);
  TEST_ASSERT(f.in.starts.empty());
  // Nothing to do without a request.
  Run(dap);
  TEST_ASSERT(f.in.starts.empty());

  f.out.HostSends(InfoReport(InfoId::VENDOR));
  // The owner only takes the lease: the OUT endpoint is not rearmed (backpressure).
  TEST_ASSERT(!f.out.active && f.out.starts.size() == 1 && f.in.starts.empty());

  // The application's DAP thread executes the command.
  Run(dap);
  TEST_ASSERT(f.in.active && f.in.Last().length == 64);
  const auto& response = f.in.Last().data;
  TEST_ASSERT(response[0] == static_cast<uint8_t>(CommandId::INFO) && response[1] == 7U);
  TEST_ASSERT(std::memcmp(&response[2], "XRobot", 7) == 0);
  TEST_ASSERT(std::all_of(response.begin() + 9, response.end(),
                          [](uint8_t byte) { return byte == 0U; }));
  // The response was accepted: the lease went back and the OUT endpoint is armed again.
  TEST_ASSERT(f.out.active && f.out.starts.size() == 2 && f.out.Last().length == 64);
  // Stepping again with nothing pending does nothing.
  Run(dap);
  TEST_ASSERT(f.in.starts.size() == 1);

  f.in.Complete();
  TEST_ASSERT(!f.in.active);
}

/// HALT 之后连发两个 OUT 报告：第一条还没执行时第二条到达（RESUMED 重挂后的双完成）。
/// CMSIS-DAP 每条命令都按顺序回一个响应，所以两条都要执行、按到达顺序回答；两个租约都
/// 要归还，之后 OUT 端点的两个槽都还在接收。
/// Two OUT reports back to back after a HALT: the second one arrives before the first
/// was executed (the double completion after a RESUMED rearm). CMSIS-DAP answers every
/// command, in order, so both run and are answered in arrival order; both leases go back,
/// and afterwards both slots of the OUT endpoint are receiving again.
void TestTwoOutReportsAnsweredInOrder()
{
  FakeSwd swd;
  Fixture f;
  TestDap dap(Number::EP1, Number::EP2, swd);
  dap.BindEndpoints(f.pool, 0, false);
  Run(dap);

  // HALT 中止在途接收；ABORTED 不重挂（等 RESUMED）。
  // The HALT aborts the in-flight receive; ABORTED does not rearm (waits for the
  // RESUMED event).
  f.out.CompleteWith(0, ErrorCode::ABORTED);
  TEST_ASSERT(!f.out.active);

  dap.OnDeviceEvent(false, DeviceEvent::ENDPOINT_RESUMED,
                    static_cast<uint8_t>(Number::EP2));
  TEST_ASSERT(f.out.active);
  f.out.HostSends(InfoReport(InfoId::VENDOR));
  TEST_ASSERT(!f.out.active && f.in.starts.empty());

  dap.OnDeviceEvent(false, DeviceEvent::ENDPOINT_RESUMED,
                    static_cast<uint8_t>(Number::EP2));
  TEST_ASSERT(f.out.active);
  f.out.HostSends(InfoReport(InfoId::PRODUCT));

  // INFO 响应是 [cmd, len, 字符串...]。 / An INFO response is [cmd, len, string...].
  for (unsigned i = 0; i < 2; ++i)
  {
    Run(dap);
    TEST_ASSERT(f.in.active);
    f.in.Complete();
  }
  TEST_ASSERT(f.in.sent.size() == 2);
  TEST_ASSERT(f.in.sent[0][1] == 7U && std::memcmp(&f.in.sent[0][2], "XRobot", 7) == 0);
  TEST_ASSERT(f.in.sent[1][1] == 10U &&
              std::memcmp(&f.in.sent[1][2], "DAPLinkV1", 10) == 0);

  // Both OUT slots are receiving: after one more report the other slot is still armed.
  TEST_ASSERT(f.out.active);
  f.out.HostSends(InfoReport(InfoId::VENDOR));
  TEST_ASSERT(f.out.active);
  Run(dap);
  f.in.Complete();
}

void TestConnectDelayAndUnknown()
{
  FakeSwd swd;
  Fixture f;
  TestDap dap(Number::EP1, Number::EP2, swd);
  dap.BindEndpoints(f.pool, 0, false);

  // DAP_Connect(SWD) enters SWD and answers the port.
  f.out.HostSends(Report(
      {static_cast<uint8_t>(CommandId::CONNECT), static_cast<uint8_t>(Port::SWD)}));
  Run(dap);
  TEST_ASSERT(f.in.Last().data[0] == static_cast<uint8_t>(CommandId::CONNECT) &&
              f.in.Last().data[1] == static_cast<uint8_t>(Port::SWD));
  TEST_ASSERT(swd.enters == 1 && swd.clock_hz == 1000000u);
  f.in.Complete();

  // A delay command waits in the Step thread.
  f.out.HostSends(Report({static_cast<uint8_t>(CommandId::DELAY), 0x64, 0x00}));
  Run(dap);
  TEST_ASSERT(f.in.Last().data[0] == static_cast<uint8_t>(CommandId::DELAY) &&
              f.in.Last().data[1] == 0x00);
  f.in.Complete();

  // An unknown command answers 0xFF and does not stall the pipeline.
  f.out.HostSends(Report({0x42}));
  Run(dap);
  TEST_ASSERT(f.in.Last().data[0] == 0xFF);
  f.in.Complete();
  TEST_ASSERT(f.out.active && f.out.starts.size() == 4);
}

void TestPipelineAndSlotExhaustion()
{
  FakeSwd swd;
  Fixture f;
  TestDap dap(Number::EP1, Number::EP2, swd);
  dap.BindEndpoints(f.pool, 0, false);
  Run(dap);

  const auto info = InfoReport(InfoId::PRODUCT);

  // First command: its response goes on the wire and the lease goes back.
  f.out.HostSends(info);
  Run(dap);
  TEST_ASSERT(f.in.starts.size() == 1 && f.in.active && f.out.starts.size() == 2);

  // Second command: the second IN slot is taken, the response queues behind the first.
  f.out.HostSends(info);
  Run(dap);
  TEST_ASSERT(f.in.starts.size() == 1 && f.out.starts.size() == 3);

  // Third command: no IN slot left, so Step times out holding the response and the
  // lease.
  f.out.HostSends(info);
  TEST_ASSERT(dap.Step(0) == ErrorCode::TIMEOUT);
  TEST_ASSERT(f.in.starts.size() == 1 && f.out.starts.size() == 3 && !f.out.active);
  TEST_ASSERT(dap.Step(0) == ErrorCode::TIMEOUT);
  TEST_ASSERT(f.in.starts.size() == 1);

  // The host taking the first response frees a slot; the next Step submits the third.
  f.in.Complete();
  TEST_ASSERT(f.in.starts.size() == 2);
  Run(dap);
  TEST_ASSERT(f.out.active && f.out.starts.size() == 4);
  f.in.Complete();
  f.in.Complete();
  TEST_ASSERT(!f.in.active);
  TEST_ASSERT(f.in.sent.size() == 3);
}

void TestUnbindWhilePending()
{
  FakeSwd swd;
  Fixture f;
  TestDap dap(Number::EP1, Number::EP2, swd);
  dap.BindEndpoints(f.pool, 0, false);
  f.out.HostSends(InfoReport(InfoId::VENDOR));
  TEST_ASSERT(!f.out.active);

  // Unbinding ends the session: the next Step drops the held command and gives its
  // stale lease back.
  dap.UnbindEndpoints(f.pool, false);
  TEST_ASSERT(f.out.closes == 1);
  Run(dap);
  TEST_ASSERT(f.in.starts.empty());

  // A fresh binding serves the next command normally.
  dap.BindEndpoints(f.pool, 0, false);
  TEST_ASSERT(f.out.active && f.out.Last().length == 64);
  const size_t armed = f.out.starts.size();
  f.out.HostSends(Report(
      {static_cast<uint8_t>(CommandId::CONNECT), static_cast<uint8_t>(Port::SWD)}));
  Run(dap);
  TEST_ASSERT(f.in.active &&
              f.in.Last().data[0] == static_cast<uint8_t>(CommandId::CONNECT));
  TEST_ASSERT(f.out.active && f.out.starts.size() == armed + 1);
  f.in.Complete();
}

/// A command that is executing while the device is unbound and bound again belongs to
/// the session that ended: its response must not be submitted into the new binding.
void TestNoStaleResponseAfterRebind()
{
  FakeSwd swd;
  Fixture f;
  TestDap dap(Number::EP1, Number::EP2, swd);
  dap.BindEndpoints(f.pool, 0, false);
  Run(dap);

  // DAP_Delay 10000 us.
  f.out.HostSends(Report({static_cast<uint8_t>(CommandId::DELAY), 0x10, 0x27}));
  std::thread dap_thread([&dap]() { (void)dap.Step(1000); });
  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  dap.UnbindEndpoints(f.pool, false);
  dap.BindEndpoints(f.pool, 0, false);
  dap_thread.join();
  Run(dap);
  TEST_ASSERT(f.in.starts.empty());

  // The new session answers normally.
  f.out.HostSends(InfoReport(InfoId::VENDOR));
  Run(dap);
  TEST_ASSERT(f.in.active && f.in.starts.size() == 1);
  TEST_ASSERT(f.in.Last().data[0] == static_cast<uint8_t>(CommandId::INFO));
  f.in.Complete();
}

/// SWD belongs to the thread that runs Step: construction, bind and unbind (the USB owner
/// context, here the test's main thread) make no SWD call. The Step thread applies the
/// session start before the first command and the session end after the unbind.
void TestBindAndUnbindLeaveSwdToStep()
{
  FakeSwd swd;
  Fixture f;
  TestDap dap(Number::EP1, Number::EP2, swd);
  dap.BindEndpoints(f.pool, 0, false);
  dap.UnbindEndpoints(f.pool, false);
  dap.BindEndpoints(f.pool, 0, false);
  TEST_ASSERT(swd.callers.empty());

  f.out.HostSends(Report(
      {static_cast<uint8_t>(CommandId::CONNECT), static_cast<uint8_t>(Port::SWD)}));
  std::thread dap_thread([&dap]() { Run(dap); });
  dap_thread.join();
  TEST_ASSERT(f.in.active &&
              f.in.Last().data[0] == static_cast<uint8_t>(CommandId::CONNECT));
  TEST_ASSERT(swd.enters == 1 && swd.clock_hz == 1000000u && swd.closes == 1);
  f.in.Complete();

  dap.UnbindEndpoints(f.pool, false);
  TEST_ASSERT(swd.closes == 1);
  std::thread end_thread([&dap]() { Run(dap); });
  end_thread.join();
  TEST_ASSERT(swd.closes == 2);
  for (const auto& id : swd.callers)
  {
    TEST_ASSERT(id != std::this_thread::get_id());
  }
}

void TestResponseNeverSentWithoutABinding()
{
  // The class answers FAILED while unbound instead of touching the endpoints.
  FakeSwd swd;
  Fixture f;
  TestDap dap(Number::EP1, Number::EP2, swd);
  const std::array<uint8_t, 8> report{};
  TEST_ASSERT(dap.SendInputReport({report.data(), report.size()}, false) ==
              ErrorCode::FAILED);
  TEST_ASSERT(f.in.starts.empty());
}
}  // namespace

void test_usb_daplink_v1()
{
  TestInfoRoundTrip();
  TestTwoOutReportsAnsweredInOrder();
  TestConnectDelayAndUnknown();
  TestPipelineAndSlotExhaustion();
  TestUnbindWhilePending();
  TestNoStaleResponseAfterRebind();
  TestBindAndUnbindLeaveSwdToStep();
  TestResponseNeverSentWithoutABinding();
}
