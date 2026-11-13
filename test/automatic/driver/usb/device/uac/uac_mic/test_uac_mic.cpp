/**
 * @file test_uac_mic.cpp
 * @brief UAC1 麦克风设备类（槽模型）主机测试 / Host tests for the UAC1 microphone device
 *        class.
 *
 * 覆盖 alt 0/1 切换时的激活与补零投帧、槽被占满时的丢帧计数、等时请求 FAILED 后的
 * 恢复、经设备核心的 SET_INTERFACE 流程（含停止未确认时的延迟）、解绑后速度清除、
 * 后端不支持等时传输时的绑定失败、采样率与 Feature Unit 类请求，以及高速下微帧债务
 * 累积成整个服务区间的节拍。
 * Covers the activation and zero-filled frame submission on alt 0/1 switches, the drop
 * count when the slots are taken, the recovery after an isochronous request FAILED, the
 * SET_INTERFACE flow through the device core (including the deferral while a stop is
 * unconfirmed), the speed cleared on unbind, the bind failure on a backend without
 * isochronous support, the sampling-frequency and feature-unit class requests, and the
 * high-speed pacing where microframe debt accrues into whole service intervals.
 */

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <vector>

#include "dev_core.hpp"
#include "driver/usb/usb_test_common.hpp"
#include "ep_pool.hpp"
#include "test_assert.hpp"
#include "uac_mic.hpp"

using namespace LibXR;
using namespace LibXR::USB;
using namespace LibXR::USB::Test;

namespace
{

/// 重新导出受保护的控制传输结果类型和诊断接口，供测试使用
/// Re-exports the protected control-transfer result type and the diagnostics so a test
/// can use them.
class Uac : public UAC1MicrophoneQ<1, 16>
{
 public:
  using UAC1MicrophoneQ<1, 16>::UAC1MicrophoneQ;
  using ControlTransferResult = DeviceClass::ControlTransferResult;
  using UAC1MicrophoneQ<1, 16>::K_SPEED_UNBOUND;
  using UAC1MicrophoneQ<1, 16>::UnderrunCount;
  using UAC1MicrophoneQ<1, 16>::MissedTransferCount;
  using UAC1MicrophoneQ<1, 16>::DroppedFrameCount;
  using UAC1MicrophoneQ<1, 16>::CurrentSpeed;
};
// 16000 Hz 单声道 16 bit：每音频帧 2 B，每个 1 ms 服务区间 16 帧 = 32 B，余数为 0。
// 16000 Hz mono 16 bit: one audio frame is 2 B, one 1 ms service interval carries
// 16 frames = 32 B, with a zero remainder.

bool IsBytes(const ConstRawData& data, std::initializer_list<uint8_t> expected)
{
  return data.size_ == expected.size() &&
         std::equal(expected.begin(), expected.end(),
                    static_cast<const uint8_t*>(data.addr_));
}

std::vector<uint8_t> Zeros(size_t size) { return std::vector<uint8_t>(size, 0); }

static constexpr auto TEST_LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "LibXR", "USB tests", "0");

/// 类级夹具：端点池、一个 ISO IN 端点和 UAC 麦克风
/// Class-level fixture: the pool, one ISO IN endpoint and the UAC microphone.
struct UacFixture
{
  // 2 个 64 B 的槽；一个 32 B 的包可放两次。
  // 2 slots of 64 B; one 32 B packet fits twice.
  Block<128> iso_storage;
  EndpointPool pool{4};
  MockEndpoint iso{Number::EP1, Direction::IN, iso_storage.Raw()};
  Uac uac{Number::EP1, 16000};
  // 把类连接到池的服务 / Connects the class to the pool's service.
  DeviceComposition composition;
  UacFixture() : composition(pool, {&TEST_LANGUAGE}, {{&uac}})
  {
    TEST_ASSERT(pool.Put(&iso) == ErrorCode::OK);
    pool.SetControlHandler(Callback<uint32_t>::Create(
        [](bool in_isr, Uac* uac, uint32_t events)
        {
          if (events & EndpointPool::CLASS_EVENT) uac->OnService(in_isr);
        },
        &uac));
    pool.SetDataEnabled(true, false);
  }
};

class TestUacDevice : public DeviceCore<FullSpeedCapabilities>
{
 public:
  TestUacDevice(EndpointPool& pool, DeviceClass& klass)
      : DeviceCore(pool, USBSpec::USB_2_0, Speed::FULL,
                   DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x4321, 0x0100,
                   {&TEST_LANGUAGE}, {{&klass}})
  {
  }
  void Start(bool) override {}
  void Stop(bool) override {}

 protected:
  ErrorCode SetAddress(uint8_t, ControlContext) override { return ErrorCode::OK; }
  void OnStopTimeout(bool) override {}
};

/// 设备级夹具：EP0 替身、一个 ISO IN 端点和设备核心。核心执行标准的
/// SET_CONFIGURATION / SET_INTERFACE 流程。
/// Device-level fixture: EP0 mocks, one ISO IN endpoint and the device core. The core
/// runs the standard SET_CONFIGURATION / SET_INTERFACE flows.
struct UacDeviceFixture
{
  Block<64> ep0_in_storage, ep0_out_storage;
  Block<128> iso_storage;
  EndpointPool pool{4};
  MockEndpoint in0{Number::EP0, Direction::IN, ep0_in_storage.Raw()};
  MockEndpoint out0{Number::EP0, Direction::OUT, ep0_out_storage.Raw()};
  MockEndpoint iso{Number::EP1, Direction::IN, iso_storage.Raw()};
  Uac uac{Number::EP1, 16000};
  TestUacDevice device{pool, uac};
  UacDeviceFixture()
  {
    TEST_ASSERT(pool.Put(&iso) == ErrorCode::OK);
    pool.SetEndpoint0(&in0, &out0);
  }
  void Setup(uint8_t kind, uint8_t request, uint16_t value, uint16_t length,
             uint16_t index = 0)
  {
    // SETUP 会独立于软件取消旧的硬件控制传输。
    // A SETUP cancels the old hardware control transfer independently from software.
    in0.active = false;
    out0.active = false;
    SetupPacket packet{kind, request, value, index, length};
    device.OnSetupPacket(true, &packet);
  }
};

// ---------------------------------------------------------------------------
// KickOneFrame：取槽、补零出队、提交
// KickOneFrame: slot, pop with zero fill, submit
// ---------------------------------------------------------------------------

void TestUacActivateAndFill()
{
  UacFixture f;
  f.uac.BindEndpoints(f.pool, 0, false);
  // 绑定后停在 alt 0：端点已配置又关闭。
  // After the bind the device sits at alt 0: the endpoint is configured, then closed.
  TEST_ASSERT(f.iso.GetState() == State::DISABLED && f.iso.closes == 1);
  TEST_ASSERT(f.uac.CurrentSpeed() == Speed::FULL);
  uint8_t alt = 0xFF;
  TEST_ASSERT(f.uac.GetAltSetting(1, alt) == ErrorCode::OK && alt == 0);
  TEST_ASSERT(f.uac.GetAltSetting(0, alt) == ErrorCode::NOT_SUPPORT);

  // Alt 0 → 1：配置端点并投第一帧；队列为空，整帧补零。
  // Alt 0 → 1: configure the endpoint and submit the first frame; the queue is empty,
  // the whole frame is zero-filled.
  TEST_ASSERT(f.uac.SetAltSetting(1, 1) == ErrorCode::OK);
  TEST_ASSERT(f.iso.GetState() == State::IDLE && f.iso.starts.size() == 1);
  TEST_ASSERT(f.iso.Last().length == 32 && f.uac.UnderrunCount() == 1);
  f.iso.Complete();
  TEST_ASSERT(f.iso.sent.back() == Zeros(32));

  // 完成不再自动投帧：下一帧来自 SOF 节拍。
  // A completion does not submit the next frame by itself: the SOF tick paces.
  TEST_ASSERT(f.iso.starts.size() == 1 && !f.iso.active);

  // 16 字节（8 个音频帧）+ 16 字节补零。
  // 16 bytes (8 audio frames) followed by 16 bytes of zero fill.
  const uint8_t pcm[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
  TEST_ASSERT(f.uac.WritePcm(pcm, sizeof(pcm)) == ErrorCode::OK);
  BusTime tick;  // elapsed_intervals 为 0：按一拍处理 / zero delta: one interval
  f.uac.OnBusTime(false, tick);
  TEST_ASSERT(f.iso.starts.size() == 2 && f.iso.Last().length == 32 &&
              f.uac.UnderrunCount() == 2);
  f.iso.Complete();
  std::vector<uint8_t> expected(pcm, pcm + sizeof(pcm));
  expected.insert(expected.end(), 16, 0);
  TEST_ASSERT(f.iso.sent.back() == expected);

  // 整帧 PCM：32 字节正好一个服务区间，不补零。
  // A full interval of PCM: 32 bytes fit exactly, no zero fill.
  uint8_t full[32];
  for (size_t i = 0; i < sizeof(full); ++i) full[i] = static_cast<uint8_t>(40 + i);
  TEST_ASSERT(f.uac.WritePcm(full, sizeof(full)) == ErrorCode::OK);
  f.uac.OnBusTime(false, tick);
  TEST_ASSERT(f.iso.Last().length == 32 && f.uac.UnderrunCount() == 2);
  f.iso.Complete();
  TEST_ASSERT(f.iso.sent.back() == std::vector<uint8_t>(full, full + sizeof(full)));

  // 队列里剩余的非整帧字节（1 B）留在队列里，不拆散采样帧。
  // A trailing partial audio frame (1 B) stays queued; sample frames are not split.
  const uint8_t tail[3] = {7, 7, 7};
  TEST_ASSERT(f.uac.WritePcm(tail, sizeof(tail)) == ErrorCode::OK);
  f.uac.OnBusTime(false, tick);
  f.iso.Complete();
  std::vector<uint8_t> tail_expected(tail, tail + 2);
  tail_expected.insert(tail_expected.end(), 30, 0);
  TEST_ASSERT(f.iso.sent.back() == tail_expected && f.uac.QueueSize() == 1);
  f.uac.ResetQueue();

  // Alt 1 → 0：端点关闭，节拍不再投帧。
  // Alt 1 → 0: the endpoint closes and the tick submits nothing.
  TEST_ASSERT(f.uac.SetAltSetting(1, 0) == ErrorCode::OK);
  TEST_ASSERT(f.iso.GetState() == State::DISABLED);
  TEST_ASSERT(f.uac.GetAltSetting(1, alt) == ErrorCode::OK && alt == 0);
  f.uac.OnBusTime(false, tick);
  TEST_ASSERT(f.iso.starts.size() == 4 && !f.iso.active);

  // 非法 alt 与外来接口号。
  // An illegal alt and a foreign interface number.
  TEST_ASSERT(f.uac.SetAltSetting(1, 2) == ErrorCode::ARG_ERR);
  TEST_ASSERT(f.uac.SetAltSetting(0, 1) == ErrorCode::NOT_SUPPORT);
  TEST_ASSERT(f.uac.SetAltSetting(0, 0) == ErrorCode::NOT_SUPPORT);
}

void TestUacFrameDropCount()
{
  UacFixture f;
  f.uac.BindEndpoints(f.pool, 0, false);
  TEST_ASSERT(f.uac.SetAltSetting(1, 1) == ErrorCode::OK);  // 槽 0 上线 / slot 0 on wire
  TEST_ASSERT(f.iso.active && f.iso.starts.size() == 1);
  BusTime tick;
  f.uac.OnBusTime(false, tick);  // 槽 1 排队 / slot 1 queued behind
  TEST_ASSERT(f.iso.starts.size() == 1);
  // 两个槽都被占用：本帧丢弃并计数。
  // Both slots are taken: the frame is dropped and counted.
  f.uac.OnBusTime(false, tick);
  TEST_ASSERT(f.uac.DroppedFrameCount() == 1 && f.iso.starts.size() == 1);
  f.iso.Complete();  // 槽 0 完成，槽 1 上线 / slot 0 done, slot 1 starts
  TEST_ASSERT(f.iso.starts.size() == 2 && f.iso.active);
  f.iso.Complete();
  TEST_ASSERT(!f.iso.active);
  f.uac.OnBusTime(false, tick);  // 有空闲槽了：继续投帧 / a slot is free again
  TEST_ASSERT(f.iso.starts.size() == 3 && f.uac.DroppedFrameCount() == 1);
  f.iso.Complete();
}

// ---------------------------------------------------------------------------
// FAILED 恢复：结束待决请求，重新 Configure，从下一拍继续
// FAILED recovery: end pending requests, re-Configure, resume from the next tick
// ---------------------------------------------------------------------------

void TestUacFailedRecovery()
{
  UacFixture f;
  f.uac.BindEndpoints(f.pool, 0, false);
  TEST_ASSERT(f.uac.SetAltSetting(1, 1) == ErrorCode::OK);
  TEST_ASSERT(f.iso.active && f.iso.starts.size() == 1);

  // 等时请求 FAILED：完成路径只置待决标志（完成回调里不做生命周期操作），
  // Close 与重新 Configure 由下一拍非中断 SOF 的 ActivateStreaming 执行。
  // The isochronous request FAILED: the completion only raises the pending flag
  // (no lifecycle ops in a completion callback); the next non-ISR SOF runs Close and the
  // re-Configure via ActivateStreaming.
  BusTime tick;
  f.iso.CompleteWith(10, ErrorCode::FAILED);
  TEST_ASSERT(f.uac.MissedTransferCount() == 1);
  f.uac.OnBusTime(false, tick);
  TEST_ASSERT(f.iso.GetState() == State::IDLE && f.iso.active &&
              f.iso.starts.size() == 2);
  f.iso.Complete();
  TEST_ASSERT(f.iso.sent.back() == Zeros(32));
  TEST_ASSERT(!f.iso.active);

  // 完成不会自动投下一帧：先用一拍把下一帧放上线，才有请求可以 FAILED。
  // A completion does not submit the next frame: arm one with a tick, otherwise
  // there is no request left to fail.
  f.uac.OnBusTime(false, tick);
  TEST_ASSERT(f.iso.active);
  f.iso.CompleteWith(0, ErrorCode::FAILED);
  TEST_ASSERT(f.uac.MissedTransferCount() == 2);
  // 恢复同样延迟到下一拍非中断 SOF。
  // The recovery likewise defers to the next non-ISR SOF tick.
  f.uac.OnBusTime(false, tick);
  TEST_ASSERT(f.iso.GetState() == State::IDLE && f.iso.active);
  f.iso.Complete();

  // ABORTED（复位 / 切配置的请求结束）不触发恢复。
  // ABORTED (a request ended by a reset / config switch) does not trigger recovery.
  TEST_ASSERT(f.uac.SetAltSetting(1, 0) == ErrorCode::OK);
  TEST_ASSERT(f.uac.SetAltSetting(1, 1) == ErrorCode::OK);
  f.iso.CompleteWith(0, ErrorCode::ABORTED);
  TEST_ASSERT(f.uac.MissedTransferCount() == 2);
  TEST_ASSERT(f.iso.GetState() == State::IDLE);
}

// ---------------------------------------------------------------------------
// 经设备核心的 SET_INTERFACE
// SET_INTERFACE through the device core
// ---------------------------------------------------------------------------

void TestUacSetInterfaceFlow()
{
  UacDeviceFixture f;
  f.device.Init(false);
  TEST_ASSERT(f.device.IsInited());
  f.Setup(0x00, 9, 1, 0);  // SET_CONFIGURATION 1
  TEST_ASSERT(f.in0.active && f.in0.Last().length == 0);
  f.in0.Complete();
  TEST_ASSERT(f.iso.GetState() == State::DISABLED && f.uac.OwnsEndpoint(0x81));

  // GET_INTERFACE：alt 0。
  // GET_INTERFACE: alt 0.
  f.Setup(0x81, 10, 0, 1, 1);
  TEST_ASSERT(f.in0.Last().data == std::vector<uint8_t>{0});
  f.in0.Complete();
  f.out0.HostSends({});

  // SET_INTERFACE alt=1：端点开启、第一帧上槽、状态阶段 ZLP。
  // SET_INTERFACE alt=1: the endpoint opens, the first frame takes a slot, the status
  // stage is a ZLP.
  f.Setup(0x01, 11, 1, 0, 1);
  TEST_ASSERT(f.in0.active && f.in0.Last().length == 0);
  TEST_ASSERT(f.iso.GetState() == State::IDLE && f.iso.starts.size() == 1);
  f.in0.Complete();
  uint8_t alt = 0xFF;
  TEST_ASSERT(f.uac.GetAltSetting(1, alt) == ErrorCode::OK && alt == 1);

  // GET_INTERFACE：alt 1。
  f.Setup(0x81, 10, 0, 1, 1);
  TEST_ASSERT(f.in0.Last().data == std::vector<uint8_t>{1});
  f.in0.Complete();
  f.out0.HostSends({});

  // alt=2 不存在：请求 STALL。
  // alt=2 does not exist: the request stalls.
  f.Setup(0x01, 11, 2, 0, 1);
  TEST_ASSERT(f.in0.IsStalled() && f.out0.IsStalled());
  f.Setup(0x81, 10, 0, 1, 1);  // 新 SETUP 清掉 STALL / a new SETUP clears the stall
  TEST_ASSERT(f.in0.Last().data == std::vector<uint8_t>{1});
  f.in0.Complete();
  f.out0.HostSends({});

  // SET_INTERFACE alt=0：端点关闭。
  // SET_INTERFACE alt=0: the endpoint closes.
  f.Setup(0x01, 11, 0, 0, 1);
  TEST_ASSERT(f.in0.active && f.in0.Last().length == 0);
  TEST_ASSERT(f.iso.GetState() == State::DISABLED && !f.iso.active);
  f.in0.Complete();
  f.Setup(0x81, 10, 0, 1, 1);
  TEST_ASSERT(f.in0.Last().data == std::vector<uint8_t>{0});
  f.in0.Complete();
  f.out0.HostSends({});
}

void TestUacSetInterfaceSlowStop()
{
  UacDeviceFixture f;
  f.device.Init(false);
  f.Setup(0x00, 9, 1, 0);
  f.in0.Complete();
  f.Setup(0x01, 11, 1, 0, 1);  // alt 1，流已开启 / alt 1, streaming
  f.in0.Complete();
  TEST_ASSERT(f.iso.active && f.iso.starts.size() == 1);

  // 流传输在途时重开 alt=1：停止未确认，状态阶段与激活都推迟。
  // Re-selecting alt=1 with a transfer on the wire: while the stop is unconfirmed both
  // the status stage and the activation wait.
  f.iso.pending_stop = true;
  f.Setup(0x01, 11, 1, 0, 1);
  TEST_ASSERT(f.iso.stop_requested && f.iso.IsStopping());
  TEST_ASSERT(!f.in0.active);

  // 停止确认后：状态阶段先发出；激活仍挂起，由下一拍（SOF）完成。
  // After the stop confirmation the status stage goes out; the activation is still
  // pending and completes on the next tick.
  f.iso.ConfirmStop();
  TEST_ASSERT(f.in0.active && f.in0.Last().length == 0);
  f.in0.Complete();
  TEST_ASSERT(f.iso.GetState() == State::DISABLED && !f.iso.active);
  BusTime tick;
  f.uac.OnBusTime(false, tick);
  TEST_ASSERT(f.iso.GetState() == State::IDLE && f.iso.active &&
              f.iso.starts.size() == 2);
  f.iso.pending_stop = false;
  f.iso.Complete();
  // 完成不会自动投帧：先补一拍把下一帧放上线，切换 alt 时才有在途请求要停。
  // A completion does not re-arm: tick once so the alt switch has an in-flight
  // request to stop.
  f.uac.OnBusTime(false, tick);
  TEST_ASSERT(f.iso.active);

  // 流传输在途时切 alt=0：同样等停止确认。
  // Switching to alt=0 with a transfer on the wire also waits for the stop.
  f.iso.pending_stop = true;
  f.Setup(0x01, 11, 0, 0, 1);
  TEST_ASSERT(f.iso.stop_requested && !f.in0.active);
  f.iso.ConfirmStop();
  TEST_ASSERT(f.in0.active && f.in0.Last().length == 0);
  f.in0.Complete();
  TEST_ASSERT(f.iso.GetState() == State::DISABLED);
}

// ---------------------------------------------------------------------------
// 解绑时清除 speed_
// speed_ is cleared on unbind
// ---------------------------------------------------------------------------

void TestUacSpeedClearedOnUnbind()
{
  UacFixture f;
  f.uac.BindEndpoints(f.pool, 0, false);
  TEST_ASSERT(f.uac.CurrentSpeed() == Speed::FULL);
  TEST_ASSERT(f.uac.SetAltSetting(1, 1) == ErrorCode::OK);
  TEST_ASSERT(f.iso.active);
  f.uac.UnbindEndpoints(f.pool, false);
  TEST_ASSERT(f.uac.CurrentSpeed() == Uac::K_SPEED_UNBOUND);
  TEST_ASSERT(!f.uac.OwnsEndpoint(0x81));
  TEST_ASSERT(f.iso.GetState() == State::DISABLED);

  // 重新绑定：速度重新从端点池读取。
  // A new bind re-reads the speed from the pool.
  f.uac.BindEndpoints(f.pool, 0, false);
  TEST_ASSERT(f.uac.CurrentSpeed() == Speed::FULL);
  TEST_ASSERT(f.uac.SetAltSetting(1, 1) == ErrorCode::OK);
  TEST_ASSERT(f.iso.GetState() == State::IDLE);
  f.uac.UnbindEndpoints(f.pool, false);
  TEST_ASSERT(f.uac.CurrentSpeed() == Uac::K_SPEED_UNBOUND);
}

// ---------------------------------------------------------------------------
// 不支持等时传输的后端使绑定失败，并给出明确的运行时错误
// A backend without isochronous support fails the bind with a clear runtime error
// ---------------------------------------------------------------------------

void TestUacUnsupportedBackend()
{
  UacFixture f;
  f.iso.configure_result = ErrorCode::NOT_SUPPORT;  // FSDEV / CH32 DevFS
  f.uac.BindEndpoints(f.pool, 0, false);
  TEST_ASSERT(f.iso.GetState() == State::ERROR);
  TEST_ASSERT(!f.pool.ConfigurationValid());
  TEST_ASSERT(!f.uac.OwnsEndpoint(0x81));
  // 类自身不进入流态。
  // The class itself never enters the streaming state.
  TEST_ASSERT(f.uac.SetAltSetting(1, 1) == ErrorCode::NOT_SUPPORT);

  UacDeviceFixture d;
  d.iso.configure_result = ErrorCode::NOT_SUPPORT;
  d.device.Init(false);
  // SET_CONFIGURATION 1：绑定失败 → INIT_ERR → STALL
  // SET_CONFIGURATION 1: the bind fails -> INIT_ERR -> STALL
  d.Setup(0x00, 9, 1, 0);
  TEST_ASSERT(d.in0.IsStalled() && d.out0.IsStalled());
  TEST_ASSERT(!d.uac.OwnsEndpoint(0x81));
}

// ---------------------------------------------------------------------------
// 类请求：端点采样率与 Feature Unit
// Class requests: EP sampling frequency and the feature unit
// ---------------------------------------------------------------------------

void TestUacClassRequests()
{
  UacFixture f;
  f.uac.BindEndpoints(f.pool, 0, false);
  Uac::ControlTransferResult result;

  // 端点采样率：GET_CUR/MIN/MAX 都返回当前频点（16000 Hz，小端 3 字节）。
  // EP sampling frequency: GET_CUR/MIN/MAX all report the current rate (16000 Hz,
  // 3 bytes little-endian).
  TEST_ASSERT(f.uac.OnClassRequest(false, Uac::GET_CUR, 0x0100, 3, 0x81, result) ==
              ErrorCode::OK);
  TEST_ASSERT(IsBytes(result.write_data, {0x80, 0x3E, 0x00}));
  TEST_ASSERT(f.uac.OnClassRequest(false, Uac::GET_MIN, 0x0100, 3, 0x81, result) ==
              ErrorCode::OK);
  TEST_ASSERT(IsBytes(result.write_data, {0x80, 0x3E, 0x00}));
  TEST_ASSERT(f.uac.OnClassRequest(false, Uac::GET_RES, 0x0100, 3, 0x81, result) ==
              ErrorCode::OK);
  TEST_ASSERT(IsBytes(result.write_data, {1, 0, 0}));
  TEST_ASSERT(f.uac.OnClassRequest(false, Uac::GET_CUR, 0x0100, 2, 0x81, result) ==
              ErrorCode::ARG_ERR);

  // SET_CUR 采样率：接受当前频点，拒绝其他。
  // SET_CUR of the rate: the current rate is accepted, others are refused.
  TEST_ASSERT(f.uac.OnClassRequest(false, Uac::SET_CUR, 0x0100, 3, 0x81, result) ==
              ErrorCode::OK);
  uint8_t rate[3] = {0x80, 0x3E, 0x00};
  ConstRawData stage{rate, sizeof(rate)};
  TEST_ASSERT(f.uac.OnClassData(false, Uac::SET_CUR, stage) == ErrorCode::OK);
  // 被拒的那次也要先发 SETUP：数据阶段的类型由上一次请求登记，消费后即清空。
  // 被拒的那次也要先发 SETUP：数据阶段的类型由上一次请求登记，消费后即清空。
  // A refused attempt needs its own SETUP too: the data stage is typed by the
  // previous request and cleared once consumed.
  const uint8_t wrong[3] = {0x80, 0xBB, 0x00};  // 48000 Hz
  TEST_ASSERT(f.uac.OnClassRequest(false, Uac::SET_CUR, 0x0100, 3, 0x81, result) ==
              ErrorCode::OK);
  stage = ConstRawData{wrong, sizeof(wrong)};
  TEST_ASSERT(f.uac.OnClassData(false, Uac::SET_CUR, stage) == ErrorCode::ARG_ERR);

  // Feature Unit：静音与音量。
  // Feature unit: mute and volume.
  // wIndex 高字节 / High byte of wIndex.
  const uint16_t fu = static_cast<uint16_t>(Uac::ID_FU << 8U);
  TEST_ASSERT(f.uac.OnClassRequest(false, Uac::GET_MIN, Uac::FU_VOLUME << 8U, 2, fu,
                                   result) == ErrorCode::OK);
  TEST_ASSERT(IsBytes(result.write_data, {0, 0}));
  TEST_ASSERT(f.uac.OnClassRequest(false, Uac::SET_CUR, Uac::FU_VOLUME << 8U, 2, fu,
                                   result) == ErrorCode::OK);
  const uint8_t volume[2] = {0x34, 0x12};
  stage = ConstRawData{volume, sizeof(volume)};
  TEST_ASSERT(f.uac.OnClassData(false, Uac::SET_CUR, stage) == ErrorCode::OK);
  TEST_ASSERT(f.uac.OnClassRequest(false, Uac::GET_CUR, Uac::FU_VOLUME << 8U, 2, fu,
                                   result) == ErrorCode::OK);
  TEST_ASSERT(IsBytes(result.write_data, {0x34, 0x12}));
  TEST_ASSERT(f.uac.OnClassRequest(false, Uac::SET_CUR, Uac::FU_MUTE << 8U, 1, fu,
                                   result) == ErrorCode::OK);
  const uint8_t mute[1] = {1};
  stage = ConstRawData{mute, sizeof(mute)};
  TEST_ASSERT(f.uac.OnClassData(false, Uac::SET_CUR, stage) == ErrorCode::OK);
  TEST_ASSERT(f.uac.OnClassRequest(false, Uac::GET_CUR, Uac::FU_MUTE << 8U, 1, fu,
                                   result) == ErrorCode::OK);
  TEST_ASSERT(IsBytes(result.write_data, {1}));
  // SET_RES 音量：接受并忽略（分辨率固定）。
  // SET_RES of the volume: accepted and ignored (the resolution is fixed).
  TEST_ASSERT(f.uac.OnClassRequest(false, Uac::SET_RES, Uac::FU_VOLUME << 8U, 2, fu,
                                   result) == ErrorCode::OK);
  stage = ConstRawData{volume, sizeof(volume)};
  TEST_ASSERT(f.uac.OnClassData(false, Uac::SET_CUR, stage) == ErrorCode::OK);

  // 错误的实体与通道。
  // A wrong entity and channel.
  const uint16_t wrong_entity = static_cast<uint16_t>(Uac::ID_IT_MIC << 8U);
  TEST_ASSERT(f.uac.OnClassRequest(false, Uac::GET_CUR, Uac::FU_VOLUME << 8U, 2,
                                   wrong_entity, result) == ErrorCode::NOT_SUPPORT);
  TEST_ASSERT(f.uac.OnClassRequest(false, Uac::GET_CUR, (Uac::FU_VOLUME << 8U) | 9U, 2,
                                   fu, result) == ErrorCode::ARG_ERR);
}

// ---------------------------------------------------------------------------
// 高速节拍：微帧债务累积成整个服务区间
// High-speed pacing: microframe debt accrues into whole service intervals
// ---------------------------------------------------------------------------

void TestUacHighSpeedPacing()
{
  // interval=1：每个微帧一个服务区间（4 B 一帧：16000 Hz / 8000 微帧 × 2 B）。
  // interval=1: one service interval per microframe (4 B: 16000 Hz over 8000
  // microframes at 2 B per audio frame).
  UacFixture f;
  f.pool.SetSpeed(Speed::HIGH);
  f.uac.BindEndpoints(f.pool, 0, false);
  TEST_ASSERT(f.uac.SetAltSetting(1, 1) == ErrorCode::OK);
  TEST_ASSERT(f.iso.starts.size() == 1 && f.iso.Last().length == 4);
  f.iso.Complete();

  // 3 个微帧 elapsed，超出的债务丢弃 / 3 microframes elapsed; the excess debt is dropped.
  BusTime micro{3, 3};
  f.uac.OnBusTime(false, micro);
  TEST_ASSERT(f.iso.starts.size() == 2 && f.iso.Last().length == 4);
  f.iso.Complete();
  // 帧号节拍的后端（无微帧）：1 帧 = 8 微帧。
  // A frame-only backend: one frame equals 8 microframes.
  BusTime frame{0xFF, 1};
  f.uac.OnBusTime(false, frame);
  TEST_ASSERT(f.iso.starts.size() == 3);
  f.iso.Complete();
  f.uac.UnbindEndpoints(f.pool, false);

  // interval=3：一个服务区间 = 4 个微帧，债务凑满才踢（8000 Hz × 4 微帧 = 4 帧 = 8 B）。
  // interval=3: one service interval is 4 microframes; the kick waits for a full one
  // (8000 Hz over 4 microframes = 4 audio frames = 8 B).
  Uac slow{Number::EP1, 8000, 0, 0, 1, Speed::HIGH, 1024, 3};
  slow.BindEndpoints(f.pool, 0, false);
  TEST_ASSERT(slow.SetAltSetting(1, 1) == ErrorCode::OK);
  TEST_ASSERT(f.iso.starts.size() == 4 && f.iso.Last().length == 8);
  f.iso.Complete();
  BusTime partial{1, 3};  // 3 < 4 微帧：不踢 / 3 < 4 microframes: no kick
  slow.OnBusTime(false, partial);
  TEST_ASSERT(f.iso.starts.size() == 4);
  BusTime rest{3, 2};  // 债务 5 ≥ 4：踢一次，余 1 / debt 5 ≥ 4: one kick
  slow.OnBusTime(false, rest);
  TEST_ASSERT(f.iso.starts.size() == 5 && f.iso.Last().length == 8);
  f.iso.Complete();
}

}  // namespace

void test_usb_uac_mic()
{
  TestUacActivateAndFill();
  TestUacFrameDropCount();
  TestUacFailedRecovery();
  TestUacSetInterfaceFlow();
  TestUacSetInterfaceSlowStop();
  TestUacSpeedClearedOnUnbind();
  TestUacUnsupportedBackend();
  TestUacClassRequests();
  TestUacHighSpeedPacing();
}
