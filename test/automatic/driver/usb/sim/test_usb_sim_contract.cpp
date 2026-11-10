/**
 * @file test_usb_sim_contract.cpp
 * @brief 后端契约测试，跑在仿真后端上 / Backend contract tests on the simulation
 *        backend.
 *
 * 每个用例走真实的端点核心与 DeviceCore，主机按包驱动。用例检查的是 ep.hpp 和
 * dev_core.hpp 对后端和端点的契约：枚举、IN/OUT 借用、槽句柄的借出戳、槽在解绑后作废、
 * 在途请求恰好结束一次、停止的三种确认方式、停止超时、挂起与断开、STALL、帧中断只在
 * 后端打开时才到核心、守卫内发起的中断挂起执行，以及多线程取槽提交。
 * Every case runs the real endpoint core and DeviceCore with the host driving packets.
 * The cases check the contract that ep.hpp and dev_core.hpp set for backends and
 * endpoints: enumeration, IN and OUT borrowing, the lend stamp of slot handles, slots
 * voided by an unbind, in-flight requests ending exactly once, the three ways a stop is
 * confirmed, the stop timeout, suspend and disconnect, STALL, the frame interrupt
 * reaching the core only while the backend has it on, interrupts raised inside the guard
 * running when it is released, and slots acquired and submitted from another thread.
 */

#include <atomic>
#include <cstdio>
#include <mutex>
#include <thread>
#include <vector>

#include "device_class.hpp"
#include "usb_sim.hpp"

using namespace LibXR;
using namespace LibXR::USB;
using namespace LibXR::USB::Sim;

namespace
{
constexpr auto TEST_LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "LibXR", "USB sim", "0");

constexpr uint16_t PACKET = 64;

using Caps = FullSpeedCapabilities;
using SimDevice = Sim::Device<Caps>;
using SimHost = Sim::Host<Caps>;

/// 一个接口、一对批量端点的探针类 / Probe class: one interface, one bulk endpoint pair
class Probe : public DeviceClass
{
 public:
  explicit Probe(uint8_t number, bool arm_out = true, bool wants_time = false)
      : number_(number), arm_out_(arm_out), wants_time_(wants_time)
  {
  }

  /// 要帧时间的类：每个到达核心的 SOF 都在 owner 里调用 OnBusTime
  /// A class that wants bus time: every SOF that reaches the core calls OnBusTime in the
  /// owner
  bool WantsBusTime() const override { return wants_time_; }
  void OnBusTime(bool, const BusTime&) override { ++bus_times; }
  std::atomic<unsigned> bus_times{0};

  Endpoint* in = nullptr;
  Endpoint* out = nullptr;
  std::mutex lock;
  std::vector<Endpoint::Completion> in_done, out_done;
  std::vector<DeviceEvent> events;
  bool bound = false;    ///< 真正绑定（非规划阶段）/ Really bound (not a planning pass)
  unsigned unbinds = 0;  ///< 真正解绑的次数 / Real unbinds

  bool Bound()
  {
    std::lock_guard<std::mutex> guard(lock);
    return bound;
  }

  size_t GetInterfaceCount() override { return 1; }
  bool HasIAD() override { return false; }
  size_t GetMaxConfigSize() override { return sizeof(desc_); }

  bool OwnsEndpoint(uint8_t addr) const override
  {
    return (in && in->GetAddress() == addr) || (out && out->GetAddress() == addr);
  }

  void BindEndpoints(EndpointPool& pool, uint8_t itf, bool in_isr) override
  {
    TEST_ASSERT(pool.Get(in, Endpoint::Direction::IN,
                         static_cast<Endpoint::EPNumber>(number_)) == ErrorCode::OK);
    TEST_ASSERT(pool.Get(out, Endpoint::Direction::OUT,
                         static_cast<Endpoint::EPNumber>(number_)) == ErrorCode::OK);
    TEST_ASSERT(in->Configure({Endpoint::Direction::IN, Endpoint::Type::BULK, PACKET},
                              in_isr) == ErrorCode::OK);
    TEST_ASSERT(out->Configure({Endpoint::Direction::OUT, Endpoint::Type::BULK, PACKET},
                               in_isr) == ErrorCode::OK);
    in->SetOnComplete(Callback<const Endpoint::Completion&>::Create(
        [](bool, Probe* self, const Endpoint::Completion& c)
        {
          std::lock_guard<std::mutex> guard(self->lock);
          self->in_done.push_back(c);
        },
        this));
    out->SetOnComplete(Callback<const Endpoint::Completion&>::Create(
        [](bool, Probe* self, const Endpoint::Completion& c)
        {
          std::lock_guard<std::mutex> guard(self->lock);
          self->out_done.push_back(c);
        },
        this));
    desc_.intf = {9, 4, itf, 0, 2, 0xFF, 0, 0, 0};
    desc_.ep_in = {7, 5, in->GetAddress(), 2, PACKET, 0};
    desc_.ep_out = {7, 5, out->GetAddress(), 2, PACKET, 0};
    SetData({&desc_, sizeof(desc_)});
    // 描述符规划阶段只绑定不启用 / The descriptor planning pass binds without enabling.
    if (!pool.IsPlanning())
    {
      std::lock_guard<std::mutex> guard(lock);
      bound = true;
    }
    if (arm_out_ && !pool.IsPlanning())
    {
      TEST_ASSERT(out->Receive(PACKET, in_isr) == ErrorCode::OK);
      TEST_ASSERT(out->Receive(PACKET, in_isr) == ErrorCode::OK);
    }
  }

  void UnbindEndpoints(EndpointPool& pool, bool in_isr) override
  {
    in->Close(in_isr);
    out->Close(in_isr);
    if (!pool.IsPlanning())
    {
      std::lock_guard<std::mutex> guard(lock);
      bound = false;
      ++unbinds;
    }
    TEST_ASSERT(pool.Release(in) == ErrorCode::OK);
    TEST_ASSERT(pool.Release(out) == ErrorCode::OK);
  }

  void OnDeviceEvent(bool, DeviceEvent event, uint8_t) override
  {
    std::lock_guard<std::mutex> guard(lock);
    events.push_back(event);
  }

  bool Saw(DeviceEvent event)
  {
    std::lock_guard<std::mutex> guard(lock);
    for (auto e : events)
      if (e == event) return true;
    return false;
  }

 private:
  LIBXR_PACKED_BEGIN
  struct Desc
  {
    InterfaceDescriptor intf;
    EndpointDescriptor ep_in;
    EndpointDescriptor ep_out;
  } desc_{};
  LIBXR_PACKED_END
  uint8_t number_;
  bool arm_out_;
  bool wants_time_;
};

/// 仿真设备加探针类，已上电、复位并完成枚举 / Simulated device with the probe class,
/// powered, reset and enumerated
struct Bench
{
  Probe probe;
  SimDevice device;
  SimHost host;

  explicit Bench(const Profile& profile = {}, bool arm_out = true,
                 bool wants_time = false)
      : probe(1, arm_out, wants_time),
        device(profile, DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x0001, 0x0100,
               {&TEST_LANGUAGE}, {{&probe}}),
        host(device)
  {
    device.Init(false);
    device.Start(false);
    device.BusReset();
    TEST_ASSERT(probe.Saw(DeviceEvent::RESET));
    TEST_ASSERT(host.Enumerate());
    TEST_ASSERT(probe.Bound());
  }

  /// IN 借用：取槽、填充、提交 / IN borrowing: acquire, fill, submit
  Endpoint::Slot SubmitIn(const std::vector<uint8_t>& data, bool zlp = false)
  {
    Endpoint::Slot slot;
    TEST_ASSERT(probe.in->AcquireSlot(slot, false) == ErrorCode::OK);
    TEST_ASSERT(slot.data.size_ >= data.size());
    std::memcpy(slot.data.addr_, data.data(), data.size());
    TEST_ASSERT(probe.in->Submit(slot, data.size(), zlp, false) == ErrorCode::OK);
    return slot;
  }

  bool Reconfigure(uint8_t n) { return host.ControlOut(MakeSetup(0x00, 9, n, 0, 0)); }
};

std::vector<uint8_t> Bytes(size_t n, uint8_t seed)
{
  std::vector<uint8_t> v(n);
  for (size_t i = 0; i < n; ++i) v[i] = static_cast<uint8_t>(seed + i);
  return v;
}

// ---------------------------------------------------------------------------

void TestEnumeration()
{
  Bench b;
  TEST_ASSERT(b.device.Address() == 1);
  // 配置描述符：配置头 + 接口 + 两个端点 / configuration + interface + two endpoints
  TEST_ASSERT(b.host.ConfigDescriptor().size() == 9U + 9U + 7U + 7U);
}

/// 两个槽排队：主机按提交顺序取走，短包结束一次传输；整包长度的传输带 zlp 时补零长包。
/// Two queued slots are taken in submit order and a short packet ends a transfer; a
/// packet-multiple transfer with zlp ends with a zero-length packet.
void TestInBorrowOrderAndZlp()
{
  Bench b;
  const auto a = Bytes(100, 1), c = Bytes(64, 50);
  b.SubmitIn(a);
  b.SubmitIn(c, true);
  TEST_ASSERT(b.host.BulkIn(1, PACKET, 1024) == a);
  // 第二个传输：64 字节后跟零长包 / the second transfer: 64 bytes, then a ZLP
  TEST_ASSERT(b.host.BulkIn(1, PACKET, 1024) == c);
  TEST_ASSERT(b.probe.in_done.size() == 2);
  TEST_ASSERT(b.probe.in_done[0].result == ErrorCode::OK &&
              b.probe.in_done[0].actual == 100);
  TEST_ASSERT(b.probe.in_done[1].result == ErrorCode::OK &&
              b.probe.in_done[1].actual == 64);
  // 没有请求时 NAK / NAK with nothing submitted
  TEST_ASSERT(b.device.InToken(1).handshake == Handshake::NAK);
}

/// 包长整数倍的传输带 zlp 时，主机先取到满包，再取到一个单独的零长包，传输在零长包
/// 之后才完成；不带 zlp 时满包取走就完成，之后是 NAK。
/// A packet-multiple transfer with zlp: the host gets the full packet and then a separate
/// zero-length packet, and the transfer completes only after it; without zlp it completes
/// with the full packet and the next token is NAKed.
void TestZeroLengthPacketAfterPacketMultiple()
{
  Bench b;
  b.SubmitIn(Bytes(PACKET, 50), true);
  const InResult full = b.device.InToken(1);
  TEST_ASSERT(full.handshake == Handshake::ACK && full.data.size() == PACKET);
  TEST_ASSERT(b.probe.in_done.empty());
  const InResult zlp = b.device.InToken(1);
  TEST_ASSERT(zlp.handshake == Handshake::ACK && zlp.data.empty());
  TEST_ASSERT(b.probe.in_done.size() == 1 && b.probe.in_done[0].actual == PACKET);

  b.SubmitIn(Bytes(PACKET, 7), false);
  TEST_ASSERT(b.device.InToken(1).data.size() == PACKET);
  TEST_ASSERT(b.probe.in_done.size() == 2);
  TEST_ASSERT(b.device.InToken(1).handshake == Handshake::NAK);
}

/// OUT 借用：两个槽交替接收，租约归还后重新挂接收。
/// OUT borrowing: the two slots alternate, and a returned lease is armed again.
void TestOutBorrowAlternates()
{
  Bench b;
  const auto data = Bytes(100, 7);
  TEST_ASSERT(b.host.BulkOut(1, PACKET, data));
  TEST_ASSERT(b.probe.out_done.size() == 2);
  const auto first = b.probe.out_done[0], second = b.probe.out_done[1];
  TEST_ASSERT(first.result == ErrorCode::OK && first.actual == 64 && first.slot.Valid());
  TEST_ASSERT(second.result == ErrorCode::OK && second.actual == 36);
  TEST_ASSERT(first.slot.index != second.slot.index);
  TEST_ASSERT(std::memcmp(first.slot.data.addr_, data.data(), 64) == 0);
  TEST_ASSERT(std::memcmp(second.slot.data.addr_, data.data() + 64, 36) == 0);
  // 两个租约都没归还：端点没有可用的槽，主机得到 NAK（背压）
  // Neither lease returned: no slot on the endpoint, the host gets NAK (backpressure).
  TEST_ASSERT(b.device.OutPacket(1, Bytes(10, 0)) == Handshake::NAK);
  TEST_ASSERT(b.probe.out->ReturnSlot(first.slot, false) == ErrorCode::OK);
  TEST_ASSERT(b.device.OutPacket(1, Bytes(10, 0)) == Handshake::ACK);
  TEST_ASSERT(b.probe.out_done.size() == 3 && b.probe.out_done[2].actual == 10);
}

/// 解绑：在途请求以 ABORTED 恰好结束一次并带已传字节；解绑前借出的槽之后提交得到
/// ABORTED。
/// Unbind: the in-flight request ends exactly once as ABORTED with the bytes moved; a
/// slot lent before the unbind gets ABORTED when submitted later.
void TestUnbindEndsRequestsOnce()
{
  Bench b;
  b.SubmitIn(Bytes(200, 3));
  TEST_ASSERT(b.device.InToken(1).data.size() == 64);  // 一个包已经发出 / one packet out
  Endpoint::Slot held;
  TEST_ASSERT(b.probe.in->AcquireSlot(held, false) == ErrorCode::OK);

  TEST_ASSERT(b.Reconfigure(0));
  TEST_ASSERT(!b.probe.Bound());
  TEST_ASSERT(b.probe.in_done.size() == 1);
  TEST_ASSERT(b.probe.in_done[0].result == ErrorCode::ABORTED &&
              b.probe.in_done[0].actual == 64);

  TEST_ASSERT(b.Reconfigure(1));
  TEST_ASSERT(b.probe.in->Submit(held, 10, false, false) == ErrorCode::ABORTED);
  TEST_ASSERT(b.probe.in_done.size() == 1);  // 没有第二次完成 / no second completion
  // 新绑定照常工作 / the new binding works
  b.SubmitIn(Bytes(5, 9));
  TEST_ASSERT(b.host.BulkIn(1, PACKET, 64) == Bytes(5, 9));
}

/// 停止要等若干帧确认：SET_CONFIGURATION 的状态阶段等停止确认后才完成。
/// A stop confirmed after some frames: the SET_CONFIGURATION status stage completes
/// only once the stop is confirmed.
void TestStopConfirmedAfterFrames()
{
  Profile profile;
  profile.stop_mode = StopMode::FRAMES;
  profile.stop_frames = 3;
  Bench b(profile);
  b.SubmitIn(Bytes(200, 3));
  TEST_ASSERT(b.device.InToken(1).data.size() == 64);

  b.device.Setup(MakeSetup(0x00, 9, 0, 0, 0));
  TEST_ASSERT(b.device.In(1)->StopPending());
  TEST_ASSERT(b.device.InToken(0).handshake == Handshake::NAK);  // 状态阶段未就绪
  b.device.Sof();
  b.device.Sof();
  TEST_ASSERT(b.device.InToken(0).handshake == Handshake::NAK);
  b.device.Sof();  // 第三帧确认 / the third frame confirms
  const auto status = b.device.InToken(0);
  TEST_ASSERT(status.handshake == Handshake::ACK && status.data.empty());
  TEST_ASSERT(b.probe.in_done.size() == 1 &&
              b.probe.in_done[0].result == ErrorCode::ABORTED &&
              b.probe.in_done[0].actual == 64);
  TEST_ASSERT(b.device.StopTimeouts() == 0);
}

/// 重新 SET_CONFIGURATION 时旧绑定上有在途数据：停止待决期间状态阶段不就绪，新绑定要等
/// 停止确认之后才建立，之后端点照常工作。
/// SET_CONFIGURATION again with data in flight on the old binding: the status stage is
/// not ready while the stop is pending, the new binding is made only after the stop is
/// confirmed, and the endpoints work as usual afterwards.
void TestReconfigureWaitsForPendingStop()
{
  Profile profile;
  profile.stop_mode = StopMode::FRAMES;
  profile.stop_frames = 3;
  // 类要帧时间：待决停止期间每一帧都让 owner 处理一次事件，重新绑定不能趁机提前发生。
  // The class wants bus time: every frame during the pending stop makes the owner handle
  // an event, and the new binding must not happen early because of it.
  Bench b(profile, true, true);
  b.SubmitIn(Bytes(200, 3));
  TEST_ASSERT(b.device.InToken(1).data.size() == 64);

  b.device.Setup(MakeSetup(0x00, 9, 1, 0, 0));
  TEST_ASSERT(b.device.In(1)->StopPending());
  TEST_ASSERT(b.device.InToken(0).handshake == Handshake::NAK);
  b.device.Sof();
  b.device.Sof();
  TEST_ASSERT(b.device.InToken(0).handshake == Handshake::NAK);
  TEST_ASSERT(!b.probe.Bound());
  b.device.Sof();  // 第三帧确认停止 / the third frame confirms the stop
  const InResult status = b.device.InToken(0);
  TEST_ASSERT(status.handshake == Handshake::ACK && status.data.empty());
  TEST_ASSERT(b.probe.Bound());
  TEST_ASSERT(b.probe.in_done.size() == 1 &&
              b.probe.in_done[0].result == ErrorCode::ABORTED);
  b.SubmitIn(Bytes(5, 9));
  TEST_ASSERT(b.host.BulkIn(1, PACKET, 64) == Bytes(5, 9));
}

/// 停止一直不确认：第 STOP_FRAME_LIMIT 帧触发停止超时，设备断开，复位后重新枚举可用。
/// A stop that is never confirmed: frame STOP_FRAME_LIMIT triggers the stop timeout, the
/// device disconnects, and after a reset it enumerates again.
void TestStopTimeoutRecovers()
{
  Profile profile;
  profile.stop_mode = StopMode::NEVER;
  Bench b(profile);
  b.SubmitIn(Bytes(200, 3));
  b.device.Setup(MakeSetup(0x00, 9, 0, 0, 0));
  for (uint32_t i = 1; i < EndpointPool::STOP_FRAME_LIMIT; ++i)
  {
    b.device.Sof();
    TEST_ASSERT(b.device.StopTimeouts() == 0);
  }
  b.device.Sof();
  TEST_ASSERT(b.device.StopTimeouts() == 1);
  TEST_ASSERT(!b.probe.Bound());
  TEST_ASSERT(b.probe.in_done.size() == 1 &&
              b.probe.in_done[0].result == ErrorCode::ABORTED);

  b.device.Attach();
  b.device.BusReset();
  TEST_ASSERT(b.host.Enumerate());
  b.SubmitIn(Bytes(5, 9));
  TEST_ASSERT(b.host.BulkIn(1, PACKET, 64) == Bytes(5, 9));
}

/// 挂起期间待决停止不计帧，恢复后继续计帧。
/// A pending stop counts no frames while suspended and goes on after resume.
void TestSuspendPausesStopCounting()
{
  Profile profile;
  profile.stop_mode = StopMode::NEVER;
  Bench b(profile);
  b.SubmitIn(Bytes(200, 3));
  b.device.Setup(MakeSetup(0x00, 9, 0, 0, 0));
  for (int i = 0; i < 3; ++i) b.device.Sof();
  b.device.Suspend();
  b.device.Resume();
  const uint32_t rest = EndpointPool::STOP_FRAME_LIMIT - 3U;
  for (uint32_t i = 1; i < rest; ++i)
  {
    b.device.Sof();
    TEST_ASSERT(b.device.StopTimeouts() == 0);
  }
  b.device.Sof();
  TEST_ASSERT(b.device.StopTimeouts() == 1);
}

/// 挂起中有待决停止时断开，断开立即生效，请求恰好结束一次。
/// A disconnect while suspended with a pending stop takes effect at once, and
/// the request ends exactly once.
void TestDisconnectWhileSuspendedWithPendingStop()
{
  Profile profile;
  profile.stop_mode = StopMode::NEVER;
  Bench b(profile);
  b.SubmitIn(Bytes(200, 3));
  b.device.Setup(MakeSetup(0x00, 9, 0, 0, 0));
  TEST_ASSERT(b.device.In(1)->StopPending());
  b.device.Suspend();
  b.device.Disconnect();
  TEST_ASSERT(!b.probe.Bound());
  TEST_ASSERT(!b.device.In(1)->StopPending());
  TEST_ASSERT(b.probe.in_done.size() == 1 &&
              b.probe.in_done[0].result == ErrorCode::ABORTED);
  TEST_ASSERT(b.device.StopTimeouts() == 0);

  b.device.Attach();
  b.device.BusReset();
  TEST_ASSERT(b.host.Enumerate());
}

/// 端点 HALT：SET_FEATURE 后主机得到 STALL，CLEAR_FEATURE 后恢复。
/// Endpoint halt: STALL after SET_FEATURE, back to normal after CLEAR_FEATURE.
void TestEndpointHalt()
{
  Bench b;
  TEST_ASSERT(b.host.ControlOut(MakeSetup(0x02, 3, 0, 0x81, 0)));
  TEST_ASSERT(b.device.InToken(1).handshake == Handshake::STALL);
  TEST_ASSERT(b.host.ControlOut(MakeSetup(0x02, 1, 0, 0x81, 0)));
  TEST_ASSERT(b.device.InToken(1).handshake == Handshake::NAK);
  b.SubmitIn(Bytes(5, 9));
  TEST_ASSERT(b.host.BulkIn(1, PACKET, 64) == Bytes(5, 9));
}

/// 槽句柄带借出戳：重复归还、提交后再归还、用旧句柄提交都得到 STATE_ERR，并且不影响槽的
/// 当前持有者（契约见 ep.hpp）。
/// Slot handles carry a lend stamp: a double return, a release after submit and a submit
/// through an older handle all get STATE_ERR and leave the slot's current holder alone
/// (the contract is described in ep.hpp).
void TestStaleSlotHandles()
{
  Bench b;
  Endpoint::Slot first;
  TEST_ASSERT(b.probe.in->AcquireSlot(first, false) == ErrorCode::OK);
  TEST_ASSERT(b.probe.in->ReleaseSlot(first, false) == ErrorCode::OK);
  TEST_ASSERT(b.probe.in->ReleaseSlot(first, false) == ErrorCode::STATE_ERR);

  // 同一个槽被再次借出：旧句柄失效，新句柄不受影响。
  // The slot is lent again: the old handle is dead, the new one is unaffected.
  Endpoint::Slot second;
  TEST_ASSERT(b.probe.in->AcquireSlot(second, false) == ErrorCode::OK);
  TEST_ASSERT(second.index == first.index && second.token != first.token);
  TEST_ASSERT(b.probe.in->Submit(first, 4, false, false) == ErrorCode::STATE_ERR);
  std::memcpy(second.data.addr_, "abcd", 4);
  TEST_ASSERT(b.probe.in->Submit(second, 4, false, false) == ErrorCode::OK);
  TEST_ASSERT(b.probe.in->Submit(second, 4, false, false) == ErrorCode::STATE_ERR);
  TEST_ASSERT(b.probe.in->ReleaseSlot(second, false) == ErrorCode::STATE_ERR);
  TEST_ASSERT(b.host.BulkIn(1, PACKET, 64) == (std::vector<uint8_t>{'a', 'b', 'c', 'd'}));

  // OUT 租约同理 / The same for an OUT lease.
  TEST_ASSERT(b.host.BulkOut(1, PACKET, Bytes(10, 5)));
  TEST_ASSERT(b.probe.out_done.size() == 1);
  const Endpoint::Slot lease = b.probe.out_done[0].slot;
  TEST_ASSERT(b.probe.out->ReturnSlot(lease, false) == ErrorCode::OK);
  TEST_ASSERT(b.probe.out->ReturnSlot(lease, false) == ErrorCode::STATE_ERR);
}

/// 帧中断只在后端打开它时才送到 DeviceCore：没有类需要帧时间、也没有待决停止时，主机的
/// SOF 到不了核心；有待决停止时到；停止确认后（OnStopsSettled）重新关闭。
/// The frame interrupt reaches DeviceCore only while the backend has it on: with no class
/// wanting bus time and no pending stop the host's SOF does not reach the core, with a
/// pending stop it does, and it is off again once the stops settled (OnStopsSettled).
void TestFrameInterruptFollowsTheBackend()
{
  Profile profile;
  profile.stop_mode = StopMode::FRAMES;
  profile.stop_frames = 2;
  Bench b(profile);
  b.device.Sof();
  b.device.Sof();
  TEST_ASSERT(b.device.SofDelivered() == 0U && !b.device.FrameInterruptOn());

  b.SubmitIn(Bytes(200, 3));
  TEST_ASSERT(b.device.InToken(1).data.size() == 64);
  b.device.Setup(MakeSetup(0x00, 9, 0, 0, 0));
  TEST_ASSERT(b.device.In(1)->StopPending());
  TEST_ASSERT(b.device.FrameInterruptOn());
  b.device.Sof();
  TEST_ASSERT(b.device.SofDelivered() == 1U);
  b.device.Sof();  // 第二帧确认停止，这一帧也送到核心 / the second frame confirms the
                   // stop and still reaches the core
  TEST_ASSERT(!b.device.In(1)->StopPending());
  TEST_ASSERT(b.device.SofDelivered() == 2U && !b.device.FrameInterruptOn());
  b.device.Sof();
  TEST_ASSERT(b.device.SofDelivered() == 2U);
}

/// 守卫内发起的中断挂起到释放时执行：每次启动硬件时主机立刻取走数据，完成在启动返回后
/// 才到，两个槽的顺序和内容都对。
/// An interrupt raised inside the guard runs when it is released: the host takes the
/// data the moment the hardware starts, the completion arrives after the start
/// returned, and both slots keep their order and content.
void TestInterruptPendedInsideGuard()
{
  Profile profile;
  SimDevice* device = nullptr;
  std::vector<std::vector<uint8_t>> taken;
  profile.on_start = [&](uint8_t number, bool in)
  {
    if (number == 1 && in)
      device->InjectIn(1, [&](InResult r) { taken.push_back(r.data); });
  };
  Bench b(profile);
  device = &b.device;
  b.SubmitIn(Bytes(20, 1));
  b.SubmitIn(Bytes(30, 2));
  TEST_ASSERT(taken.size() == 2);
  TEST_ASSERT(taken[0] == Bytes(20, 1) && taken[1] == Bytes(30, 2));
  TEST_ASSERT(b.probe.in_done.size() == 2 && b.probe.in_done[0].actual == 20 &&
              b.probe.in_done[1].actual == 30);
}

/// 另一个线程取槽、填充、提交，主机线程同时取数据：顺序和内容不乱（TSan 下运行）。
/// Another thread acquires, fills and submits while the host thread takes the data:
/// order and content stay intact (runs under TSan).
void TestSubmitFromAnotherThread()
{
  Bench b;
  constexpr unsigned COUNT = 2000;
  std::thread producer(
      [&b]
      {
        for (unsigned i = 0; i < COUNT; ++i)
        {
          Endpoint::Slot slot;
          while (b.probe.in->AcquireSlot(slot, false) != ErrorCode::OK)
            std::this_thread::yield();
          const uint8_t value[4] = {static_cast<uint8_t>(i), static_cast<uint8_t>(i >> 8),
                                    0xA5, 0x5A};
          std::memcpy(slot.data.addr_, value, sizeof(value));
          TEST_ASSERT(b.probe.in->Submit(slot, sizeof(value), false, false) ==
                      ErrorCode::OK);
        }
      });
  for (unsigned i = 0; i < COUNT;)
  {
    const auto in = b.device.InToken(1);
    if (in.handshake == Handshake::NAK)
    {
      std::this_thread::yield();
      continue;
    }
    TEST_ASSERT(in.handshake == Handshake::ACK && in.data.size() == 4);
    TEST_ASSERT(in.data[0] == static_cast<uint8_t>(i) &&
                in.data[1] == static_cast<uint8_t>(i >> 8) && in.data[2] == 0xA5);
    ++i;
  }
  producer.join();
  std::lock_guard<std::mutex> guard(b.probe.lock);
  TEST_ASSERT(b.probe.in_done.size() == COUNT);
}

}  // namespace

int main()
{
  TestEnumeration();
  TestInBorrowOrderAndZlp();
  TestZeroLengthPacketAfterPacketMultiple();
  TestOutBorrowAlternates();
  TestUnbindEndsRequestsOnce();
  TestStopConfirmedAfterFrames();
  TestReconfigureWaitsForPendingStop();
  TestStopTimeoutRecovers();
  TestSuspendPausesStopCounting();
  TestDisconnectWhileSuspendedWithPendingStop();
  TestEndpointHalt();
  TestStaleSlotHandles();
  TestFrameInterruptFollowsTheBackend();
  TestInterruptPendedInsideGuard();
  TestSubmitFromAnotherThread();
  std::puts("usb sim: backend contract");
  return 0;
}
