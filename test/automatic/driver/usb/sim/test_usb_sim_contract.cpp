/**
 * @file test_usb_sim_contract.cpp
 * @brief 后端契约测试，跑在仿真后端上 / Backend contract tests on the simulation backend
 *
 * 每个用例走真实的端点核心、设备核心和组合层，主机按包驱动。判据是 ep.hpp、
 * usb_service.hpp 和 dev_core.hpp 写下的契约：结尾零长度包、硬件上的多个请求、停止的
 * 三种结束方式（当场、按帧、超时后复位控制器）、线上的包与 actual 的下界、挂起和总线
 * 复位结束停止、STALL 与数据翻转位、SETUP 取代进行中的阶段（规则 1 和 2）、别的线程
 * 持有 owner 时中断只屏蔽并返回、多线程提交、等时区间、高带宽 PID 序列、直接读写槽的
 * 端点在停止期间扣住被停的槽、Stop 等停稳后断开。
 * Every case runs the real endpoint core, device core and composition, with the host
 * driving packets. The oracle is the contract written in ep.hpp, usb_service.hpp and
 * dev_core.hpp: the closing zero-length packet, several requests on the hardware, the
 * three ways a stop ends (at once, after frames, after a timeout and a controller
 * reset), the packet on the wire and actual as a lower bound, suspend and bus reset
 * ending stops, STALL and the data toggle, a SETUP superseding the stage in progress
 * (rules 1 and 2), the interrupt only masking and returning while another thread owns,
 * submissions from several threads, isochronous intervals, the high-bandwidth PID
 * sequence, an endpoint whose hardware accesses the slot keeping the stopped slot
 * during the stop, and Stop detaching once the stops have ended.
 */

#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "sim_probe.hpp"
#include "usb_sim.hpp"

using namespace LibXR;
using namespace LibXR::USB;
using namespace LibXR::USB::Sim;

namespace
{
constexpr auto LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "LibXR", "USB sim", "0001");

using Dir = Endpoint::Direction;
using Type = Endpoint::Type;
using Bytes = std::vector<uint8_t>;

/// EP1 批量 IN、EP1 批量 OUT / Bulk IN on EP1, bulk OUT on EP1
std::vector<ProbeEndpoint> BulkPair()
{
  return {{1, Dir::IN, Type::BULK, 64, 1, 0}, {1, Dir::OUT, Type::BULK, 64, 1, 0}};
}

Bytes Pattern(size_t n, uint8_t seed)
{
  Bytes b(n);
  for (size_t i = 0; i < n; ++i)
  {
    b[i] = static_cast<uint8_t>(seed + i * 7U);
  }
  return b;
}

/// 一台仿真设备、一个探针类和主机 / One simulated device, one probe class and a host
struct Rig
{
  Rig(const Profile& profile, std::vector<ProbeEndpoint> defs, Speed speed = Speed::FULL)
      : probe(std::move(defs)),
        dev(profile, DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x0001, 0x0100,
            {&LANGUAGE}, {{&probe}}, speed),
        host(dev)
  {
  }

  /// Init、Start、枚举 / Init, Start and enumerate
  void Up(Speed speed = Speed::FULL)
  {
    dev.Init(false);
    dev.Start(false);
    const bool ok = host.Enumerate(1, 1, speed);
    if (!ok)
    {
      std::fprintf(stderr, "enumeration failed at %s\n", host.last_failure);
    }
    TEST_ASSERT(ok);
  }

  Probe probe;
  SimDevice dev;
  Host host;
};

void Sofs(Controller& dev, unsigned n)
{
  for (unsigned i = 0; i < n; ++i)
  {
    dev.Sof();
  }
}

/// 两线程间的一次性信号 / One-shot signal between two threads
struct Signal
{
  void Set()
  {
    std::lock_guard<std::mutex> lock(m);
    on = true;
    cv.notify_all();
  }
  void Wait()
  {
    std::unique_lock<std::mutex> lock(m);
    cv.wait(lock, [this] { return on; });
  }
  std::mutex m;
  std::condition_variable cv;
  bool on = false;
};

// ---------------------------------------------------------------------------

/// 枚举：地址在配置的阶段写入，SOF 报告给设备核心 / Enumeration: the address is written
/// at the configured stage, SOFs are reported to the device core
void TestEnumeration()
{
  for (auto stage : {DeviceCore::AddressStage::STATUS_COMPLETE,
                     DeviceCore::AddressStage::BEFORE_STATUS})
  {
    Profile profile;
    profile.address_stage = stage;
    Rig rig(profile, BulkPair());
    rig.Up();
    TEST_ASSERT(rig.dev.Address() == 1U);
    TEST_ASSERT(rig.probe.binds == 1U);
    TEST_ASSERT(rig.dev.Out(1)->OnHardware() == 1U);  // 绑定时挂的接收 / armed at bind
    const unsigned before = rig.dev.sofs;
    Sofs(rig.dev, 3);
    TEST_ASSERT(rig.dev.sofs == before + 3U);
  }
}

/// IN 与结尾零长度包：两次 StartHardware、一次完成 / IN with a closing zero-length
/// packet: two StartHardware calls, one completion
void TestInWithClosingZlp()
{
  Rig rig(Profile{}, BulkPair());
  rig.Up();
  SimEndpoint* in = rig.dev.In(1);
  const unsigned starts = in->starts;
  const Bytes data = Pattern(64, 1);
  TEST_ASSERT(rig.probe.Send(0, data, true) == ErrorCode::OK);
  InResult first = rig.host.In(1);
  TEST_ASSERT(first.handshake == Handshake::ACK && first.data == data);
  TEST_ASSERT(rig.probe.Count(0) == 0U);  // 零长度包还欠着 / the closing packet is owed
  InResult zlp = rig.host.In(1);
  TEST_ASSERT(zlp.handshake == Handshake::ACK && zlp.data.empty());
  TEST_ASSERT(in->starts == starts + 2U);
  TEST_ASSERT(rig.probe.Count(0) == 1U);
  const ProbeDone done = rig.probe.At(0, 0);
  TEST_ASSERT(done.result == ErrorCode::OK && done.actual == 64U);
  // 不带 zlp 的整包没有零长度包 / A full packet without zlp gets no closing packet.
  TEST_ASSERT(rig.probe.Send(0, data, false) == ErrorCode::OK);
  TEST_ASSERT(rig.host.In(1).data == data);
  TEST_ASSERT(rig.probe.Count(0) == 2U);
  TEST_ASSERT(rig.dev.InToken(1).handshake == Handshake::NAK);
}

/// OUT：收满或短包结束请求，数据在槽里 / OUT: a full request or a short packet ends it,
/// the data in the slot
void TestOutReceive()
{
  Rig rig(Profile{}, BulkPair());
  rig.probe.receive_bytes = 128;
  rig.Up();
  const Bytes data = Pattern(128, 3);
  TEST_ASSERT(rig.host.BulkOut(1, 64, data, false));
  TEST_ASSERT(rig.probe.Count(1) == 1U);
  TEST_ASSERT(rig.probe.At(1, 0).actual == 128U && rig.probe.At(1, 0).data == data);
  const Bytes small = Pattern(10, 9);
  TEST_ASSERT(rig.host.BulkOut(1, 64, small));
  TEST_ASSERT(rig.probe.Count(1) == 2U && rig.probe.At(1, 1).data == small);
  TEST_ASSERT(rig.dev.Out(1)->OnHardware() == 1U);
}

/// 重发的 OUT 包（翻转位相同）确认但丢弃 / A retransmitted OUT packet (same toggle) is
/// acknowledged and dropped
void TestOutDuplicateDropped()
{
  Rig rig(Profile{}, BulkPair());
  rig.Up();
  const Bytes a = Pattern(8, 1);
  const Bytes b = Pattern(8, 2);
  TEST_ASSERT(rig.dev.OutToken(1, a, Pid::DATA0) == Handshake::ACK);
  // 主机没收到 ACK，以同一个 PID 重发 / The host lost the ACK and resends with the same
  // PID.
  TEST_ASSERT(rig.dev.OutToken(1, a, Pid::DATA0) == Handshake::ACK);
  TEST_ASSERT(rig.dev.OutToken(1, b, Pid::DATA1) == Handshake::ACK);
  TEST_ASSERT(rig.probe.Count(1) == 2U);
  TEST_ASSERT(rig.probe.At(1, 0).data == a && rig.probe.At(1, 1).data == b);
}

/// HwLimits::max_requests 决定硬件上同时几个请求，完成按启动顺序 / max_requests sets
/// how many requests are on the hardware; completions in start order
void TestRequestsOnHardware()
{
  for (uint8_t max : {uint8_t{1}, uint8_t{2}})
  {
    Profile profile;
    profile.endpoints = {{1, Dir::BOTH, 1024, 2, 0, max}};
    Rig rig(profile, BulkPair());
    rig.Up();
    const Bytes a = Pattern(64, 1);
    const Bytes b = Pattern(32, 2);
    TEST_ASSERT(rig.probe.Send(0, a) == ErrorCode::OK);
    TEST_ASSERT(rig.probe.Send(0, b) == ErrorCode::OK);
    TEST_ASSERT(rig.dev.In(1)->OnHardware() == max);
    TEST_ASSERT(rig.host.In(1).data == a);
    TEST_ASSERT(rig.host.In(1).data == b);
    TEST_ASSERT(rig.probe.Count(0) == 2U);
    TEST_ASSERT(rig.probe.At(0, 0).actual == 64U && rig.probe.At(0, 1).actual == 32U);
  }
}

/// 解绑：在途请求恰好回调一次 ABORTED；按帧停稳时 CloseHardware 推迟到停稳，之后不再回调
/// Unbind: the in-flight request gets exactly one ABORTED; with a stop that ends after
/// frames CloseHardware waits for it and no callback follows
void TestUnbindEndsRequestsOnce()
{
  for (StopMode mode : {StopMode::IMMEDIATE, StopMode::FRAMES})
  {
    Profile profile;
    profile.stop_mode = mode;
    Rig rig(profile, BulkPair());
    rig.host.auto_frames = false;
    rig.Up();
    SimEndpoint* in = rig.dev.In(1);
    TEST_ASSERT(rig.probe.Send(0, Pattern(32, 1)) == ErrorCode::OK);
    const unsigned closes = in->closes;
    TEST_ASSERT(rig.host.SetConfiguration(0));
    TEST_ASSERT(rig.probe.Count(0) == 1U);
    TEST_ASSERT(rig.probe.At(0, 0).result == ErrorCode::ABORTED);
    TEST_ASSERT(in->GetState() == Endpoint::State::DISABLED);
    if (mode == StopMode::FRAMES)
    {
      TEST_ASSERT(in->pending_stops == 1U && in->closes == closes);
      TEST_ASSERT(rig.dev.InToken(1).handshake == Handshake::NAK);
      Sofs(rig.dev, profile.stop_frames);
    }
    TEST_ASSERT(in->closes == closes + 1U && !in->StopPending());
    TEST_ASSERT(rig.probe.Count(0) == 1U);
  }
}

/// 停止未结束时重新配置：Configure 当场 OK，硬件配置和请求启动推迟到停稳 / Configure
/// while a stop is pending: OK at once, the hardware configuration and the start wait
/// for the end of the stop
void TestConfigureDuringPendingStop()
{
  Profile profile;
  profile.stop_mode = StopMode::FRAMES;
  profile.stop_frames = 3;
  Rig rig(profile, BulkPair());
  rig.host.auto_frames = false;
  rig.Up();
  SimEndpoint* in = rig.dev.In(1);
  TEST_ASSERT(rig.probe.Send(0, Pattern(32, 1)) == ErrorCode::OK);
  TEST_ASSERT(rig.host.SetConfiguration(0));
  const unsigned configures = in->configures;
  const unsigned starts = in->starts;
  TEST_ASSERT(rig.host.SetConfiguration(1));
  TEST_ASSERT(in->GetState() == Endpoint::State::IDLE);
  TEST_ASSERT(in->configures == configures);
  const Bytes data = Pattern(20, 5);
  TEST_ASSERT(rig.probe.Send(0, data) == ErrorCode::OK);
  TEST_ASSERT(in->starts == starts);
  TEST_ASSERT(rig.dev.InToken(1).handshake == Handshake::NAK);
  Sofs(rig.dev, profile.stop_frames);
  TEST_ASSERT(in->configures == configures + 1U && in->starts == starts + 1U);
  TEST_ASSERT(rig.host.In(1).data == data);
  TEST_ASSERT(rig.probe.Count(0) == 2U);
  TEST_ASSERT(rig.probe.At(0, 1).result == ErrorCode::OK);
}

/// 停止发出后线上的包仍到达主机：回调当场以 ABORTED、actual 为当时已知的字节数交付，
/// 停稳时的真实结果被丢弃 / A packet on the wire still reaches the host after the stop:
/// the callback comes at once with ABORTED and the bytes known then, and the real
/// outcome at the end of the stop is dropped
void TestPacketOnWireAfterStop()
{
  Profile profile;
  profile.stop_mode = StopMode::FRAMES;
  profile.stop_grace_tokens = 1;
  Rig rig(profile, BulkPair());
  rig.host.auto_frames = false;
  rig.Up();
  const Bytes data = Pattern(128, 4);
  TEST_ASSERT(rig.probe.Send(0, data) == ErrorCode::OK);
  TEST_ASSERT(rig.host.SetConfiguration(0));
  TEST_ASSERT(rig.probe.Count(0) == 1U);
  TEST_ASSERT(rig.probe.At(0, 0).result == ErrorCode::ABORTED);
  TEST_ASSERT(rig.probe.At(0, 0).actual == 0U);
  InResult late = rig.dev.InToken(1);
  TEST_ASSERT(late.handshake == Handshake::ACK && late.data.size() == 64U);
  TEST_ASSERT(rig.dev.InToken(1).handshake == Handshake::NAK);
  Sofs(rig.dev, profile.stop_frames);
  TEST_ASSERT(!rig.dev.In(1)->StopPending());
  TEST_ASSERT(rig.probe.Count(0) == 1U);
}

/// 停止超时：TIMEOUT、控制器复位、所有端点关闭，主机重新枚举后照常工作 / Stop
/// timeout: TIMEOUT, controller reset, every endpoint closed; after the host enumerates
/// again everything works
void TestStopTimeoutResetsController()
{
  Profile profile;
  profile.stop_mode = StopMode::NEVER;
  profile.stop_timeout_frames = 8;
  Rig rig(profile, BulkPair());
  rig.host.auto_frames = false;
  rig.Up();
  TEST_ASSERT(rig.probe.Send(0, Pattern(32, 1)) == ErrorCode::OK);
  TEST_ASSERT(rig.host.SetConfiguration(0));
  TEST_ASSERT(rig.dev.In(1)->StopPending());
  Sofs(rig.dev, profile.stop_timeout_frames);
  TEST_ASSERT(rig.dev.ControllerResets() == 1U);
  TEST_ASSERT(!rig.dev.In(1)->StopPending() && !rig.dev.Out(1)->StopPending());
  TEST_ASSERT(rig.dev.Visible());
  TEST_ASSERT(rig.probe.Count(0) == 1U);
  rig.host.auto_frames = true;
  TEST_ASSERT(rig.host.Enumerate());
  const Bytes data = Pattern(40, 8);
  TEST_ASSERT(rig.probe.Send(0, data) == ErrorCode::OK);
  TEST_ASSERT(rig.host.In(1).data == data);
}

/// 挂起时总线空闲，停止照常结束 / While suspended the bus is idle and stops end as usual
void TestSuspendEndsStops()
{
  Profile profile;
  profile.stop_mode = StopMode::NEVER;
  Rig rig(profile, BulkPair());
  rig.host.auto_frames = false;
  rig.Up();
  SimEndpoint* in = rig.dev.In(1);
  TEST_ASSERT(rig.probe.Send(0, Pattern(32, 1)) == ErrorCode::OK);
  TEST_ASSERT(rig.host.SetConfiguration(0));
  TEST_ASSERT(in->StopPending());
  const unsigned closes = in->closes;
  rig.dev.Suspend();
  TEST_ASSERT(rig.dev.suspends == 1U);
  TEST_ASSERT(!in->StopPending() && in->closes == closes + 1U);
  rig.dev.Resume();
  TEST_ASSERT(rig.dev.resumes == 1U);
  TEST_ASSERT(rig.host.SetConfiguration(1));
  const Bytes data = Pattern(16, 2);
  TEST_ASSERT(rig.probe.Send(0, data) == ErrorCode::OK);
  TEST_ASSERT(rig.host.In(1).data == data);
  TEST_ASSERT(rig.dev.ControllerResets() == 0U);
}

/// 总线复位和断开之前，后端先结束并报告未结束的停止 / Before a bus reset or a
/// disconnect the backend ends and reports every pending stop
void TestResetAndDisconnectEndStops()
{
  for (bool disconnect : {false, true})
  {
    Profile profile;
    profile.stop_mode = StopMode::NEVER;
    Rig rig(profile, BulkPair());
    rig.host.auto_frames = false;
    rig.Up();
    TEST_ASSERT(rig.probe.Send(0, Pattern(32, 1)) == ErrorCode::OK);
    TEST_ASSERT(rig.host.SetConfiguration(0));
    TEST_ASSERT(rig.dev.In(1)->StopPending());
    if (disconnect)
    {
      rig.dev.Unplug();
      TEST_ASSERT(rig.dev.disconnects == 1U);
      rig.dev.Plug();
    }
    TEST_ASSERT(rig.host.Enumerate());
    TEST_ASSERT(!rig.dev.In(1)->StopPending());
    const Bytes data = Pattern(16, 3);
    TEST_ASSERT(rig.probe.Send(0, data) == ErrorCode::OK);
    TEST_ASSERT(rig.host.In(1).data == data);
    TEST_ASSERT(rig.dev.ControllerResets() == 0U);
  }
}

/// SET_FEATURE、CLEAR_FEATURE(ENDPOINT_HALT)：STALL，清除后两边的翻转位都回到 DATA0 /
/// SET_FEATURE and CLEAR_FEATURE(ENDPOINT_HALT): STALL, and both toggles back to DATA0
void TestHaltAndToggle()
{
  Rig rig(Profile{}, BulkPair());
  rig.Up();
  const Bytes a = Pattern(16, 1);
  TEST_ASSERT(rig.probe.Send(0, a) == ErrorCode::OK);
  TEST_ASSERT(rig.host.In(1).data == a);  // 翻转位到 DATA1 / toggles now DATA1
  TEST_ASSERT(rig.host.SetHalt(0x81));
  TEST_ASSERT(rig.dev.In(1)->HardwareStalled());
  TEST_ASSERT(rig.dev.InToken(1).handshake == Handshake::STALL);
  const Bytes b = Pattern(16, 2);
  TEST_ASSERT(rig.probe.Send(0, b) == ErrorCode::OK);  // 排队到清除 / queued until clear
  TEST_ASSERT(rig.host.ClearHalt(0x81));
  TEST_ASSERT(!rig.dev.In(1)->HardwareStalled());
  InResult got = rig.host.In(1);
  TEST_ASSERT(got.handshake == Handshake::ACK && got.data == b);
  TEST_ASSERT(rig.host.duplicates == 0U);
}

/// 对没有 STALL 的端点 CLEAR_FEATURE：硬件上的请求留着，翻转位回到 DATA0 / CLEAR_FEATURE
/// on an endpoint that is not halted: the request on the hardware stays and the toggle
/// returns to DATA0
void TestClearHaltOnIdleEndpoint()
{
  Rig rig(Profile{}, BulkPair());
  rig.Up();
  SimEndpoint* out = rig.dev.Out(1);
  TEST_ASSERT(rig.host.BulkOut(1, 64, Pattern(8, 1)));
  TEST_ASSERT(rig.probe.Count(1) == 1U && out->Toggle() == 1U);
  const unsigned clears = out->clears;
  TEST_ASSERT(rig.host.ClearHalt(0x01));
  TEST_ASSERT(out->clears == clears + 1U);
  TEST_ASSERT(out->OnHardware() == 1U && rig.probe.Count(1) == 1U);
  const Bytes b = Pattern(8, 2);
  TEST_ASSERT(rig.host.BulkOut(1, 64, b));
  TEST_ASSERT(rig.probe.Count(1) == 2U && rig.probe.At(1, 1).data == b);
}

/// SETUP 取代进行中的控制传输；EP0 的停止在 SETUP 报告之后当场给出结果 / A SETUP
/// supersedes the control transfer in progress; a stop of EP0 gives its outcome at once
/// after the SETUP is reported
void TestSetupSupersedesStage()
{
  for (StopMode mode : {StopMode::IMMEDIATE, StopMode::FRAMES})
  {
    Profile profile;
    profile.stop_mode = mode;
    Rig rig(profile, BulkPair());
    rig.host.auto_frames = false;
    rig.Up();
    // 主机发了 SETUP，没取数据阶段就发下一个 / The host sends a SETUP and the next one
    // without taking the data stage.
    rig.dev.Setup(MakeSetup(0x80, 6, 0x0100, 0, 18));
    TEST_ASSERT(rig.dev.In(0)->OnHardware() == 1U);
    bool ok = false;
    const Bytes head = rig.host.ControlIn(MakeSetup(0x80, 6, 0x0200, 0, 9), ok);
    TEST_ASSERT(ok && head.size() == 9U && head[1] == 2U);
    TEST_ASSERT(!rig.dev.In(0)->IsStalled());
  }
}

/// 控制端点规则 2：装入时锁存了未报告的 SETUP，后端撤回，旧应答被忽略，新传输照常 /
/// Control endpoint rule 2: an unreported SETUP is latched at the load, the backend
/// retracts it, the old reply is ignored and the new transfer runs as usual
void TestSetupDuringLoad()
{
  for (SetupRule rule : {SetupRule::NAK, SetupRule::RETRACT})
  {
    Profile profile;
    profile.setup_rule = rule;
    Rig* rig_ptr = nullptr;
    bool armed = false;
    bool probe_in = false;
    Handshake stale = Handshake::NAK;
    profile.before_start = [&](uint8_t number, bool in)
    {
      // 旧应答正要装入时，主机已超时，发下一个 SETUP / The old reply is about to be
      // loaded when the host has timed out and sends the next SETUP.
      probe_in = armed && number == 0U && in;
      if (probe_in)
      {
        armed = false;
        rig_ptr->dev.Setup(MakeSetup(0x80, 6, 0x0200, 0, 9));
      }
    };
    profile.on_start = [&](uint8_t number, bool in)
    {
      // 旧应答留在硬件上时主机的 IN 令牌立刻到：不得拿到旧数据（撤回时不会到这里）。
      // The host's IN token arrives while the old reply stays on the hardware: it must
      // not get the old data (a retracted load never gets here).
      if (probe_in && number == 0U && in)
      {
        probe_in = false;
        stale = rig_ptr->dev.InToken(0).handshake;
      }
    };
    Rig rig(profile, BulkPair());
    rig_ptr = &rig;
    rig.Up();
    armed = true;
    rig.dev.Setup(MakeSetup(0x80, 6, 0x0100, 0, 18));
    TEST_ASSERT(!armed);
    TEST_ASSERT(stale == Handshake::NAK);
    bool ok = false;
    const Bytes head = rig.host.ControlInData(9, ok);
    TEST_ASSERT(ok && head.size() == 9U && head[1] == 2U);
    if (rule == SetupRule::RETRACT)
    {
      TEST_ASSERT(rig.dev.In(0)->retracts == 1U);
    }
  }
}

/// 线程持有 owner 时中断只屏蔽并返回，事件由该线程在它这一轮处理 / While a thread owns,
/// the interrupt only masks and returns, and that thread handles the event in its pass
void TestInterruptWhileThreadOwns()
{
  Rig rig(Profile{}, BulkPair());
  rig.Up();
  Signal entered;
  Signal go;
  std::thread::id owner_thread;
  rig.probe.on_service = [&](bool in_isr)
  {
    TEST_ASSERT(!in_isr);
    owner_thread = std::this_thread::get_id();
    entered.Set();
    go.Wait();
  };
  std::thread worker([&rig] { rig.probe.RequestService(false); });
  entered.Wait();
  const unsigned entries = rig.dev.interrupts;
  const Bytes data = Pattern(12, 6);
  TEST_ASSERT(rig.dev.OutToken(1, data, Pid::DATA0) == Handshake::ACK);
  // 中断进来、屏蔽了线就返回，完成留给 owner / The interrupt came, masked the line and
  // returned; the completion is left to the owner.
  TEST_ASSERT(rig.dev.interrupts == entries + 1U && rig.dev.LineMasked());
  TEST_ASSERT(rig.probe.Count(1) == 0U);
  go.Set();
  worker.join();
  TEST_ASSERT(rig.probe.Count(1) == 1U);
  const ProbeDone done = rig.probe.At(1, 0);
  TEST_ASSERT(done.data == data && done.thread == owner_thread && !done.in_isr);
  TEST_ASSERT(!rig.dev.LineMasked());
}

/// owner 正忙时主机取走了 IN：之后的 Flush 拿到硬件已做完的结果 OK / The host took an
/// IN while the owner was busy: the later Flush gets the finished outcome, OK
void TestStopReturnsFinishedRequest()
{
  Rig rig(Profile{}, BulkPair());
  rig.Up();
  const Bytes data = Pattern(64, 2);
  TEST_ASSERT(rig.probe.Send(0, data) == ErrorCode::OK);
  Signal entered;
  Signal go;
  rig.probe.on_service = [&](bool in_isr)
  {
    entered.Set();
    go.Wait();
    rig.probe.eps[0]->Flush(in_isr);
  };
  std::thread worker([&rig] { rig.probe.RequestService(false); });
  entered.Wait();
  TEST_ASSERT(rig.dev.InToken(1).data == data);
  go.Set();
  worker.join();
  TEST_ASSERT(rig.probe.Count(0) == 1U);
  TEST_ASSERT(rig.probe.At(0, 0).result == ErrorCode::OK);
  TEST_ASSERT(rig.probe.At(0, 0).actual == 64U);
}

/// 两个线程同时提交，主机按序收齐；有无 PendInterrupt 都一样 / Two threads submit at
/// once and the host receives everything in order; with and without PendInterrupt
void TestSubmitFromThreads()
{
  for (bool pend : {false, true})
  {
    Profile profile;
    profile.pend_interrupt = pend;
    profile.endpoints = {{1, Dir::BOTH, 1024, 4, 0, 2}};
    Rig rig(profile, BulkPair());
    rig.Up();
    constexpr unsigned PER_THREAD = 300;
    std::atomic<bool> isr_seen{false};
    auto sender = [&rig](uint8_t tag)
    {
      for (unsigned i = 0; i < PER_THREAD;)
      {
        Bytes b(8, tag);
        b[1] = static_cast<uint8_t>(i);
        b[2] = static_cast<uint8_t>(i >> 8U);
        const ErrorCode result = rig.probe.Send(0, b);
        if (result == ErrorCode::OK)
        {
          ++i;
        }
        else
        {
          TEST_ASSERT(result == ErrorCode::BUSY);
          std::this_thread::yield();
        }
      }
    };
    std::thread a(sender, uint8_t{0xA0});
    std::thread b(sender, uint8_t{0xB0});
    unsigned next[2] = {0, 0};
    for (unsigned got = 0; got < 2U * PER_THREAD;)
    {
      InResult in = rig.dev.InToken(1);
      if (in.handshake != Handshake::ACK)
      {
        std::this_thread::yield();
        continue;
      }
      TEST_ASSERT(in.data.size() == 8U);
      const unsigned which = in.data[0] == 0xA0 ? 0U : 1U;
      TEST_ASSERT(in.data[0] == (which == 0U ? 0xA0 : 0xB0));
      TEST_ASSERT((in.data[1] | (in.data[2] << 8U)) == next[which]);
      ++next[which];
      ++got;
    }
    a.join();
    b.join();
    for (size_t n = 0; n < rig.probe.Count(0); ++n)
    {
      TEST_ASSERT(rig.probe.At(0, n).result == ErrorCode::OK);
      isr_seen = isr_seen || rig.probe.At(0, n).in_isr;
    }
    TEST_ASSERT(rig.probe.Count(0) == 2U * PER_THREAD);
    // PendInterrupt 时回调都在中断里 / With PendInterrupt every callback runs in the
    // interrupt.
    if (pend)
    {
      for (size_t n = 0; n < rig.probe.Count(0); ++n)
      {
        TEST_ASSERT(rig.probe.At(0, n).in_isr);
      }
    }
  }
}

/// 等时 IN：没被取走的区间以 FAILED 结束，或留在硬件上等主机 / Isochronous IN: an
/// interval the host did not take ends with FAILED, or stays on the hardware
void TestIsochronousIn()
{
  for (bool expire : {true, false})
  {
    Profile profile;
    profile.iso_in_expire = expire;
    Rig rig(profile, {{1, Dir::IN, Type::ISOCHRONOUS, 64, 1, 1}});
    rig.host.auto_frames = false;
    rig.Up();
    const Bytes data = Pattern(48, 7);
    TEST_ASSERT(rig.probe.Send(0, data) == ErrorCode::OK);
    Sofs(rig.dev, 3);
    if (expire)
    {
      TEST_ASSERT(rig.probe.Count(0) == 1U);
      TEST_ASSERT(rig.probe.At(0, 0).result == ErrorCode::FAILED);
      InResult none = rig.dev.InToken(1);
      TEST_ASSERT(none.handshake == Handshake::ACK && none.data.empty());
    }
    else
    {
      TEST_ASSERT(rig.probe.Count(0) == 0U);
      InResult got = rig.dev.InToken(1);
      TEST_ASSERT(got.data == data && got.pid == Pid::DATA0);
      TEST_ASSERT(rig.probe.Count(0) == 1U);
      TEST_ASSERT(rig.probe.At(0, 0).result == ErrorCode::OK);
    }
  }
}

/// 高速高带宽等时 OUT：一个区间三个包，PID 序列 MDATA、MDATA、DATA2；序列不对是 FAILED /
/// High-speed high-bandwidth isochronous OUT: three packets in one interval, PIDs
/// MDATA, MDATA, DATA2; a wrong sequence is FAILED
void TestHighBandwidthIsoOut()
{
  Profile profile;
  profile.max_packet_limit = 1024;
  profile.max_packets_per_interval = 3;
  profile.endpoints = {{1, Dir::OUT, 8192, 2, 0, 1}};
  Rig rig(profile, {{1, Dir::OUT, Type::ISOCHRONOUS, 1024, 3, 1}}, Speed::HIGH);
  rig.host.auto_frames = false;
  rig.Up(Speed::HIGH);
  Endpoint* ep = rig.dev.Out(1);
  TEST_ASSERT(ep->MaxPacketSize() == 1024U && ep->PacketsPerInterval() == 3U);
  const Bytes a = Pattern(1024, 1);
  const Bytes c = Pattern(1000, 3);
  TEST_ASSERT(rig.dev.OutToken(1, a, Pid::MDATA) == Handshake::NONE);
  TEST_ASSERT(rig.dev.OutToken(1, a, Pid::MDATA) == Handshake::NONE);
  TEST_ASSERT(rig.dev.OutToken(1, c, Pid::DATA2) == Handshake::NONE);
  TEST_ASSERT(rig.probe.Count(0) == 1U);
  TEST_ASSERT(rig.probe.At(0, 0).result == ErrorCode::OK);
  TEST_ASSERT(rig.probe.At(0, 0).actual == 2048U + 1000U);
  rig.dev.Sof();
  TEST_ASSERT(rig.dev.OutToken(1, a, Pid::MDATA) == Handshake::NONE);
  TEST_ASSERT(rig.dev.OutToken(1, c, Pid::DATA0) == Handshake::NONE);
  TEST_ASSERT(rig.probe.Count(0) == 2U);
  TEST_ASSERT(rig.probe.At(0, 1).result == ErrorCode::FAILED);
  rig.dev.Sof();
  TEST_ASSERT(rig.dev.OutToken(1, c, Pid::DATA0) == Handshake::NONE);
  TEST_ASSERT(rig.probe.Count(0) == 3U && rig.probe.At(0, 2).actual == 1000U);
}

/// 硬件直接读写槽的端点在停止期间扣住被停的槽；不直接读写的当场放回 / An endpoint
/// whose hardware accesses the slot keeps the stopped slot during the stop; one that
/// does not gives it back at once
void TestStoppedSlotHeld()
{
  for (bool direct : {true, false})
  {
    Profile profile;
    profile.stop_mode = StopMode::FRAMES;
    profile.direct_memory = direct;
    Rig rig(profile, BulkPair());
    rig.host.auto_frames = false;
    rig.Up();
    TEST_ASSERT(rig.probe.Send(0, Pattern(32, 1)) == ErrorCode::OK);
    TEST_ASSERT(rig.host.SetConfiguration(0));
    TEST_ASSERT(rig.host.SetConfiguration(1));
    Endpoint* in = rig.dev.In(1);
    Endpoint::Slot a;
    Endpoint::Slot b;
    TEST_ASSERT(in->AcquireSlot(a) == ErrorCode::OK);
    TEST_ASSERT(in->AcquireSlot(b) == (direct ? ErrorCode::BUSY : ErrorCode::OK));
    if (!direct)
    {
      TEST_ASSERT(in->ReleaseSlot(b) == ErrorCode::OK);
    }
    Sofs(rig.dev, profile.stop_frames);
    TEST_ASSERT(in->AcquireSlot(b) == ErrorCode::OK);
    TEST_ASSERT(in->ReleaseSlot(a) == ErrorCode::OK);
    TEST_ASSERT(in->ReleaseSlot(b) == ErrorCode::OK);
  }
}

/// Stop：在途请求以 ABORTED 回调，所有停止结束后才断开总线；Start 之后重新枚举 / Stop:
/// requests in flight are called back with ABORTED and the bus is detached only once
/// every stop has ended; after Start the host enumerates again
void TestStopDetachesAfterStops()
{
  Profile profile;
  profile.stop_mode = StopMode::FRAMES;
  Rig rig(profile, BulkPair());
  rig.host.auto_frames = false;
  rig.Up();
  TEST_ASSERT(rig.probe.Send(0, Pattern(32, 1)) == ErrorCode::OK);
  rig.dev.Stop(false);
  TEST_ASSERT(rig.probe.Count(0) == 1U);
  TEST_ASSERT(rig.probe.At(0, 0).result == ErrorCode::ABORTED);
  TEST_ASSERT(rig.dev.Visible() && rig.dev.ControllerStops() == 0U);
  Sofs(rig.dev, profile.stop_frames);
  TEST_ASSERT(!rig.dev.Visible() && rig.dev.ControllerStops() == 1U);
  rig.dev.Start(false);
  TEST_ASSERT(rig.dev.Visible());
  rig.host.auto_frames = true;
  TEST_ASSERT(rig.host.Enumerate());
  const Bytes data = Pattern(16, 5);
  TEST_ASSERT(rig.probe.Send(0, data) == ErrorCode::OK);
  TEST_ASSERT(rig.host.In(1).data == data);
}

/// 线程持有 owner 时主机拔出、接上、复位：断开、复位在同一次读取里按先后报告；复位
/// 报告之前启动的请求被接受，复位处理里当场停掉 / The host unplugs, plugs and resets
/// while a thread owns: disconnect and reset come in one read and are reported in order;
/// a request started before the reset is reported is accepted and stopped at once
void TestResetWhileThreadOwns()
{
  Profile profile;
  profile.stop_mode = StopMode::NEVER;
  Rig rig(profile, BulkPair());
  rig.host.auto_frames = false;
  rig.Up();
  Signal entered;
  Signal go;
  rig.probe.on_service = [&](bool)
  {
    entered.Set();
    go.Wait();
    // 这一轮的端点阶段在复位报告之前 / This pass's endpoint step precedes the report
    // of the reset.
    TEST_ASSERT(rig.probe.Send(0, Pattern(16, 1)) == ErrorCode::OK);
  };
  std::thread worker([&rig] { rig.probe.RequestService(false); });
  entered.Wait();
  rig.dev.Unplug();
  rig.dev.Plug();
  rig.dev.BusReset();
  go.Set();
  worker.join();
  TEST_ASSERT(rig.dev.disconnects == 1U && rig.dev.resets == 2U);
  TEST_ASSERT(rig.probe.Count(0) == 1U);
  TEST_ASSERT(rig.probe.At(0, 0).result == ErrorCode::ABORTED);
  TEST_ASSERT(rig.dev.In(1)->OnHardware() == 0U);
  // 不再复位，直接控制传输：设备已初始化 / No further reset, straight to a control
  // transfer: the device is initialized.
  bool ok = false;
  const Bytes desc = rig.host.ControlIn(MakeSetup(0x80, 6, 0x0100, 0, 18), ok);
  TEST_ASSERT(ok && desc.size() == 18U);
}

struct Case
{
  const char* name;
  void (*run)();
};

const Case CASES[] = {
    {"enumeration", TestEnumeration},
    {"in_closing_zlp", TestInWithClosingZlp},
    {"out_receive", TestOutReceive},
    {"out_duplicate", TestOutDuplicateDropped},
    {"requests_on_hardware", TestRequestsOnHardware},
    {"unbind_once", TestUnbindEndsRequestsOnce},
    {"configure_pending_stop", TestConfigureDuringPendingStop},
    {"packet_on_wire", TestPacketOnWireAfterStop},
    {"stop_timeout", TestStopTimeoutResetsController},
    {"suspend_ends_stops", TestSuspendEndsStops},
    {"reset_disconnect_end_stops", TestResetAndDisconnectEndStops},
    {"halt_toggle", TestHaltAndToggle},
    {"clear_halt_idle", TestClearHaltOnIdleEndpoint},
    {"setup_supersedes", TestSetupSupersedesStage},
    {"setup_during_load", TestSetupDuringLoad},
    {"interrupt_while_thread_owns", TestInterruptWhileThreadOwns},
    {"stop_finished_request", TestStopReturnsFinishedRequest},
    {"submit_threads", TestSubmitFromThreads},
    {"iso_in", TestIsochronousIn},
    {"hs_iso_out", TestHighBandwidthIsoOut},
    {"stopped_slot_held", TestStoppedSlotHeld},
    {"stop_detach", TestStopDetachesAfterStops},
    {"reset_while_thread_owns", TestResetWhileThreadOwns},
};

}  // namespace

int main(int argc, char** argv)
{
  // 参数：只跑名字相同的用例 / Argument: run only the case of that name
  const std::string only = argc > 1 ? argv[1] : "";
  unsigned ran = 0;
  for (const Case& c : CASES)
  {
    if (!only.empty() && only != c.name)
    {
      continue;
    }
    std::printf("usb_sim_contract: %s\n", c.name);
    std::fflush(stdout);
    c.run();
    ++ran;
  }
  TEST_ASSERT(ran != 0U);
  std::printf("usb_sim_contract: %u cases passed\n", ran);
  return 0;
}
