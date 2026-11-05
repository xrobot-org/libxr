/**
 * @file test_daplink_v1.cpp
 * @brief DAP v1 设备类（槽模型）主机测试 / Host tests for the DAP v1 device class.
 *
 * DAP 命令在类自己的上下文执行（MODEL_USB §3.1/§10.3-a）：owner 只登记 OUT 租约并置
 * 待处理标志，命令由构造期注册的 1 ms Timer 任务调用的 ProcessPending() 推进，响应
 * 提交成功后才归还租约（背压）。本测试用假 SWD 探针和一个预先挂好的 Timer 列表（不启
 * 动调度线程）手动驱动，覆盖收发往返、背压、响应槽耗尽后的补提交、以及解绑语义。
 * DAP commands run in the class's own context (MODEL_USB §3.1/§10.3-a): the owner only
 * records the OUT lease and raises the pending flag, the command is driven by
 * ProcessPending(), which the 1 ms Timer task registered in the constructor calls, and
 * the lease goes back only after the response was accepted (backpressure). The test uses
 * a fake SWD probe and a pre-provided Timer list (no scheduler thread) so it drives the
 * command path by hand, covering the request/response round trip, backpressure, the
 * resubmission after the response slots ran out, and unbind semantics.
 */

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <vector>

#include "../../mock_endpoint.hpp"
#include "daplink_v1.hpp"
#include "debug/swd.hpp"
#include "lockfree_list.hpp"
#include "timer.hpp"

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

/// Minimal SWD probe: every transfer succeeds and reads a fixed word.
class FakeSwd : public LibXR::Debug::Swd
{
 public:
  unsigned enters = 0, closes = 0, line_resets = 0;
  uint32_t clock_hz = 0;
  unsigned writes = 0, reads = 0;

  ErrorCode SetClockHz(uint32_t hz) override
  {
    clock_hz = hz;
    return ErrorCode::OK;
  }
  void Close() override { ++closes; }
  ErrorCode LineReset() override
  {
    ++line_resets;
    return ErrorCode::OK;
  }
  ErrorCode EnterSwd() override
  {
    ++enters;
    return ErrorCode::OK;
  }
  ErrorCode Transfer(const SwdRequest& req, SwdResponse& resp) override
  {
    if (req.rnw)
    {
      ++reads;
      resp.rdata = 0x12345678u;
    }
    else
    {
      ++writes;
    }
    resp.ack = LibXR::Debug::SwdProtocol::Ack::OK;
    resp.parity_ok = true;
    return ErrorCode::OK;
  }
  void IdleClocks(uint32_t) override {}
  ErrorCode SeqWriteBits(uint32_t cycles, const uint8_t* data) override
  {
    if (cycles != 0U && data != nullptr) ++writes;
    return ErrorCode::OK;
  }
  ErrorCode SeqReadBits(uint32_t cycles, uint8_t* out) override
  {
    if (out != nullptr) std::memset(out, 0, (cycles + 7U) / 8U);
    return ErrorCode::OK;
  }
};

class TestDap : public DapLinkV1Class<FakeSwd>
{
 public:
  TestDap(Endpoint::EPNumber in, Endpoint::EPNumber out, FakeSwd& swd)
      : DapLinkV1Class<FakeSwd>(in, out, swd)
  {
  }
  using DapLinkV1Class<FakeSwd>::BindEndpoints;
  using DapLinkV1Class<FakeSwd>::ProcessPending;
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
  dap.ProcessPending(false);
  TEST_ASSERT(f.in.starts.empty());

  f.out.HostSends(Report(
      {static_cast<uint8_t>(CommandId::INFO), static_cast<uint8_t>(InfoId::VENDOR)}));
  // The owner only takes the lease: the OUT endpoint is not rearmed (backpressure).
  TEST_ASSERT(!f.out.active && f.out.starts.size() == 1 && f.in.starts.empty());

  // The BSP Timer task drives the command in DAP's own context.
  dap.ProcessPending(false);
  TEST_ASSERT(f.in.active && f.in.Last().length == 64);
  const auto& response = f.in.Last().data;
  TEST_ASSERT(response[0] == static_cast<uint8_t>(CommandId::INFO) && response[1] == 7U);
  TEST_ASSERT(std::memcmp(&response[2], "XRobot", 7) == 0);
  TEST_ASSERT(std::all_of(response.begin() + 9, response.end(),
                          [](uint8_t byte) { return byte == 0U; }));
  // The response was accepted: the lease went back and the OUT endpoint is armed again.
  TEST_ASSERT(f.out.active && f.out.starts.size() == 2 && f.out.Last().length == 64);
  // A second pump with nothing pending does nothing.
  dap.ProcessPending(false);
  TEST_ASSERT(f.in.starts.size() == 1);

  f.in.Complete();
  TEST_ASSERT(!f.in.active);
}

/// HALT 之后连发两个 OUT 报告：第一条还在 PENDING 时第二条到达（模拟 RESUMED
/// 重挂后的双完成），旧实现会 REQUIRE fatal；现在后到者胜、响应正常发出。
/// Two OUT reports back to back after a HALT: the second one arrives while the
/// first is still PENDING (the double completion after a RESUMED rearm). The old
/// implementation died on REQUIRE; the newest report now wins and the response is
/// sent normally.
void TestBusyOutReportsAfterHalt()
{
  FakeSwd swd;
  Fixture f;
  TestDap dap(Number::EP1, Number::EP2, swd);
  dap.BindEndpoints(f.pool, 0, false);

  // HALT 中止在途接收；ABORTED 不重挂（等 RESUMED）。
  // The HALT aborts the in-flight receive; ABORTED does not rearm (waits for the
  // RESUMED event).
  f.out.CompleteWith(0, ErrorCode::ABORTED);
  TEST_ASSERT(!f.out.active);

  // RESUMED 重挂接收，主机发来第一条报告并被登记（PENDING，租约在手）。
  // The RESUMED rearm fires and the first report is recorded (PENDING, lease
  // held).
  dap.OnDeviceEvent(false, DeviceEvent::ENDPOINT_RESUMED,
                    static_cast<uint8_t>(Number::EP2));
  TEST_ASSERT(f.out.active);
  const auto vendor = Report(
      {static_cast<uint8_t>(CommandId::INFO), static_cast<uint8_t>(InfoId::VENDOR)});
  f.out.HostSends(vendor);
  TEST_ASSERT(!f.out.active && f.in.starts.empty());

  // 第二条报告（再次 RESUMED 重挂后完成）：旧代码在这里 REQUIRE fatal。
  // A second report (completed after another RESUMED rearm): the old code died on
  // REQUIRE right here.
  dap.OnDeviceEvent(false, DeviceEvent::ENDPOINT_RESUMED,
                    static_cast<uint8_t>(Number::EP2));
  TEST_ASSERT(f.out.active);
  const auto product = Report(
      {static_cast<uint8_t>(CommandId::INFO), static_cast<uint8_t>(InfoId::PRODUCT)});
  f.out.HostSends(product);

  // 后到者胜：处理的是第二条（PRODUCT，"DAPLinkV1"），而不是第一条（VENDOR，
  // "XRobot"）。INFO 响应是 [cmd, len, 字符串...]，data[1] 是长度字节。
  // Last one wins: the second report (PRODUCT, "DAPLinkV1") is processed, not the
  // first one (VENDOR, "XRobot"). An INFO response is [cmd, len, string...], so
  // data[1] is the length byte.
  dap.ProcessPending(false);
  TEST_ASSERT(f.in.Last().data[0] == static_cast<uint8_t>(CommandId::INFO));
  TEST_ASSERT(f.in.Last().data[1] == 10U);  // "DAPLinkV1" = 9 chars + NUL
  TEST_ASSERT(std::memcmp(&f.in.Last().data[2], "DAPLinkV1", 10) == 0);
  f.in.Complete();
  TEST_ASSERT(f.out.active && f.out.starts.size() == 4);
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
  dap.ProcessPending(false);
  TEST_ASSERT(f.in.Last().data[0] == static_cast<uint8_t>(CommandId::CONNECT) &&
              f.in.Last().data[1] == static_cast<uint8_t>(Port::SWD));
  TEST_ASSERT(swd.enters == 1 && swd.clock_hz == 1000000u);
  f.in.Complete();

  // A delay command runs in DAP's own (non-ISR) context.
  f.out.HostSends(Report({static_cast<uint8_t>(CommandId::DELAY), 0x64, 0x00}));
  dap.ProcessPending(false);
  TEST_ASSERT(f.in.Last().data[0] == static_cast<uint8_t>(CommandId::DELAY) &&
              f.in.Last().data[1] == 0x00);
  f.in.Complete();

  // An unknown command answers 0xFF and does not stall the pipeline.
  f.out.HostSends(Report({0x42}));
  dap.ProcessPending(false);
  TEST_ASSERT(f.in.Last().data[0] == 0xFF);
  f.in.Complete();
  TEST_ASSERT(f.out.active && f.out.starts.size() == 4);
}

void TestPipelineAndSlotExhaustionRetry()
{
  FakeSwd swd;
  Fixture f;
  TestDap dap(Number::EP1, Number::EP2, swd);
  dap.BindEndpoints(f.pool, 0, false);

  const auto info = Report(
      {static_cast<uint8_t>(CommandId::INFO), static_cast<uint8_t>(InfoId::PRODUCT)});

  // First command: its response goes on the wire and the lease goes back.
  f.out.HostSends(info);
  dap.ProcessPending(false);
  TEST_ASSERT(f.in.starts.size() == 1 && f.in.active && f.out.starts.size() == 2);

  // Second command: the second IN slot is taken, the response queues behind the first.
  f.out.HostSends(info);
  dap.ProcessPending(false);
  TEST_ASSERT(f.in.starts.size() == 1 && f.out.starts.size() == 3);

  // Third command: no slot left, so it waits in RESPONDING and keeps the lease.
  f.out.HostSends(info);
  dap.ProcessPending(false);
  TEST_ASSERT(f.in.starts.size() == 1 && f.out.starts.size() == 3 && !f.out.active);
  // Pumping again while waiting for a slot changes nothing.
  dap.ProcessPending(false);
  TEST_ASSERT(f.in.starts.size() == 1);

  // The host taking the first response frees a slot; the completion retries the submit.
  f.in.Complete();
  TEST_ASSERT(f.in.starts.size() == 2);
  TEST_ASSERT(f.out.active && f.out.starts.size() == 4);
  f.in.Complete();
  f.in.Complete();
  TEST_ASSERT(!f.in.active);
}

void TestUnbindWhilePending()
{
  FakeSwd swd;
  Fixture f;
  TestDap dap(Number::EP1, Number::EP2, swd);
  dap.BindEndpoints(f.pool, 0, false);
  f.out.HostSends(Report(
      {static_cast<uint8_t>(CommandId::INFO), static_cast<uint8_t>(InfoId::VENDOR)}));
  TEST_ASSERT(!f.out.active);

  // Unbinding drops the held command: the stale lease is returned and the state is idle.
  dap.UnbindEndpoints(f.pool, false);
  TEST_ASSERT(f.out.closes == 1);
  dap.ProcessPending(false);
  TEST_ASSERT(f.in.starts.empty());

  // A fresh binding serves the next command normally.
  dap.BindEndpoints(f.pool, 0, false);
  TEST_ASSERT(f.out.active && f.out.Last().length == 64);
  const size_t armed = f.out.starts.size();
  f.out.HostSends(Report(
      {static_cast<uint8_t>(CommandId::CONNECT), static_cast<uint8_t>(Port::SWD)}));
  dap.ProcessPending(false);
  TEST_ASSERT(f.in.active &&
              f.in.Last().data[0] == static_cast<uint8_t>(CommandId::CONNECT));
  TEST_ASSERT(f.out.active && f.out.starts.size() == armed + 1);
  f.in.Complete();
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

int main()
{
  // Provide the timer list so the constructor's Timer::Add() does not start a scheduler
  // thread; the test drives ProcessPending() itself (MODEL_USB §10.3-a).
  LibXR::LockFreeList timer_list;
  LibXR::Timer::list_ = &timer_list;

  TestInfoRoundTrip();
  TestBusyOutReportsAfterHalt();
  TestConnectDelayAndUnknown();
  TestPipelineAndSlotExhaustionRetry();
  TestUnbindWhilePending();
  TestResponseNeverSentWithoutABinding();
  std::puts("DAP v1 class: lease backpressure, own-context pump and unbind passed");
}
