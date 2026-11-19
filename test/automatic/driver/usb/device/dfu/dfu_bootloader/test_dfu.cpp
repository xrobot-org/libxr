/**
 * @file test_dfu.cpp
 * @brief DFU 设备类（槽模型）主机测试 / Host tests for the DFU device classes.
 *
 * 覆盖槽模型迁移改动的路径：控制传输的状态阶段改由核心完成（删除 SendStatusInZLP 后
 * 请求仍返回 OK 且不产生 IN 数据），以及 DNLOAD 的 OUT 数据阶段由核心收进自己的缓冲
 * 区、完整后经 OnControlData -> OnClassData 交给类，再由 GetControlReceiveCapacity
 * 覆盖 wTransferSize 块长；以及慢工作（runtime 的 detach 跳转、bootloader 的擦写）只在
 * 应用调用 Process() 时执行，不需要 Timer 任务。
 * Covers the paths the slot-model migration changed: a control transfer's status stage is
 * now finished by the core (with SendStatusInZLP gone the request still returns OK and
 * produces no IN data), and the DNLOAD OUT data stage is received into the core's own
 * buffer and handed to the class via OnControlData -> OnClassData, with
 * GetControlReceiveCapacity covering the wTransferSize block length; and the slow work
 * (the runtime's detach jump, the bootloader's erase and program) runs only when the
 * application calls Process(), with no Timer task.
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

#include "device_composition.hpp"
#include "dfu_bootloader.hpp"
#include "dfu_runtime.hpp"
#include "driver/usb/device/usb_device_test_common.hpp"
#include "driver/usb/usb_test_common.hpp"
#include "ep_pool.hpp"
#include "flash.hpp"
#include "test_assert.hpp"

using namespace LibXR;
using namespace LibXR::USB;
using LibXR::USB::Test::RamFlash;

namespace
{
static constexpr auto TEST_LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "LibXR", "USB tests", "0");

constexpr uint8_t DFU_DETACH = 0x00;
constexpr uint8_t DFU_DNLOAD = 0x01;
constexpr uint8_t DFU_UPLOAD = 0x02;
constexpr uint8_t DFU_GETSTATUS = 0x03;
constexpr uint8_t DFU_CLRSTATUS = 0x04;
constexpr uint8_t DFU_GETSTATE = 0x05;

unsigned g_jumps = 0;
void JumpToApp(void*) { ++g_jumps; }
void JumpToBootloader(void*) { ++g_jumps; }

class TestRuntime : public DfuRuntimeClass
{
 public:
  using DfuRuntimeClass::BindEndpoints;
  using DfuRuntimeClass::DfuRuntimeClass;
  using DfuRuntimeClass::OnClassRequest;
  using DfuRuntimeClass::OnControlComplete;
  using DfuRuntimeClass::UnbindEndpoints;
  using ControlTransferResult = DeviceClass::ControlTransferResult;
};

using BootClass = DfuBootloaderClassT<256>;

class TestBoot : public BootClass
{
 public:
  using BootClass::BootClass;
  using DFUClass<DfuBootloaderBackend, 256>::BindEndpoints;
  using DFUClass<DfuBootloaderBackend, 256>::OnClassData;
  using DFUClass<DfuBootloaderBackend, 256>::OnClassInDataStatusComplete;
  using DFUClass<DfuBootloaderBackend, 256>::OnClassRequest;
  using DFUClass<DfuBootloaderBackend, 256>::OnControlRequest;
  using DFUClass<DfuBootloaderBackend, 256>::UnbindEndpoints;
  using ControlTransferResult = DeviceClass::ControlTransferResult;
};

/// 控制响应的字节视图 / Byte view of a control response.
const uint8_t* Bytes(const ConstRawData& data)
{
  return static_cast<const uint8_t*>(data.addr_);
}

void TestRuntimeDetach()
{
  TestRuntime rt(&JumpToBootloader, nullptr, 20u);
  EndpointPool pool{2};
  rt.BindEndpoints(pool, 3, false);
  TestRuntime::ControlTransferResult result;
  // DeviceCore 为每个 SETUP 提供一个新的 result；类只填写请求产生的内容，所以复用
  // result 会保留上一次的响应。
  // DeviceCore hands a fresh result to every SETUP; the class only fills what the
  // request produces, so a reused result would keep the previous response.
  auto ask = [&](uint8_t req, uint16_t val, uint16_t len, uint16_t idx)
  {
    result.write_data = {};
    return rt.OnClassRequest(false, req, val, len, idx, result);
  };

  // 针对其他接口的请求被拒绝 / A request for another interface is refused.
  TEST_ASSERT(ask(DFU_GETSTATUS, 0, 6, 2) == ErrorCode::NOT_FOUND);

  // DETACH 没有数据阶段：请求成功且不产生 IN 数据，状态阶段由核心自己完成（已删除的
  // SendStatusInZLP 过去会请求一个 ZLP）。
  // DETACH has no data stage: the request succeeds without IN data and the core finishes
  // the status stage itself (the removed SendStatusInZLP used to ask for a ZLP).
  TEST_ASSERT(ask(DFU_DETACH, 0, 0, 3) == ErrorCode::OK);
  TEST_ASSERT(result.write_data.addr_ == nullptr && result.write_data.size_ == 0U);

  // GETSTATUS 报告 detach 状态和剩余超时。
  // GETSTATUS reports the detach state and the remaining timeout.
  TEST_ASSERT(ask(DFU_GETSTATUS, 0, 6, 3) == ErrorCode::OK);
  TEST_ASSERT(result.write_data.size_ == 6U);
  const uint8_t* poll = Bytes(result.write_data);
  TEST_ASSERT(poll[0] == 0U);
  TEST_ASSERT(poll[4] == static_cast<uint8_t>(DFUState::APP_DETACH));
  TEST_ASSERT(poll[5] == 0U);
  const uint32_t remain = static_cast<uint32_t>(poll[1]) |
                          (static_cast<uint32_t>(poll[2]) << 8) |
                          (static_cast<uint32_t>(poll[3]) << 16);
  TEST_ASSERT(remain <= 20U);

  // GETSTATE 回答同一个状态 / GETSTATE answers the same state.
  TEST_ASSERT(ask(DFU_GETSTATE, 0, 1, 3) == ErrorCode::OK);
  TEST_ASSERT(result.write_data.size_ == 1U &&
              Bytes(result.write_data)[0] == static_cast<uint8_t>(DFUState::APP_DETACH));

  // 带数据阶段的 DETACH 被拒绝 / DETACH with a data stage is refused.
  rt.UnbindEndpoints(pool, false);
  rt.BindEndpoints(pool, 3, false);
  TEST_ASSERT(ask(DFU_DETACH, 0, 4, 3) == ErrorCode::ARG_ERR);
  // 未知请求被拒绝 / An unknown request is refused.
  TEST_ASSERT(ask(0x7F, 0, 0, 3) == ErrorCode::ARG_ERR);

  // 没有跳转回调时，runtime 类不提供 DETACH。
  // Without a jump callback the runtime class does not offer DETACH.
  TestRuntime plain(nullptr);
  plain.BindEndpoints(pool, 3, false);
  TEST_ASSERT(plain.OnClassRequest(false, DFU_DETACH, 0, 0, 3, result) ==
              ErrorCode::NOT_SUPPORT);
  // 绑定之前请求被拒绝 / Before binding, requests are refused.
  TestRuntime unbound(&JumpToBootloader);
  TEST_ASSERT(unbound.OnClassRequest(false, DFU_GETSTATUS, 0, 6, 0, result) ==
              ErrorCode::INIT_ERR);
}

/// 设置 bitWillDetach 时，设备在 DFU_DETACH 之后自行 detach（DFU 1.1，表 4.2）。
/// runtime 类在通告的超时（从状态阶段开始计）到期后，由 Process() 完成此动作；不涉及
/// Timer 任务或类服务。
/// With bitWillDetach the device detaches on its own after DFU_DETACH (DFU 1.1, table
/// 4.2). The runtime class does so from Process() once the advertised timeout, counted
/// from the status stage, has expired; no Timer task or class service is involved.
void TestRuntimeJumpsFromProcess()
{
  // 20 ms 的超时本身是被测内容，所以有两处要小心处理：
  // - “超时从状态阶段开始”：DETACH 与状态阶段之间经过 30 ms（比超时长），Process() 不得
  //   跳转。这里要的就是时间流逝，停得更久也不会破坏它。
  // - “状态阶段刚结束时还没到期”：OnControlComplete 之后紧接着运行 Process()，不得跳转。
  //   这只在两者之间线程没有被调度出去超过超时的情况下成立，而负载高的主机做得到这一点。
  //   用单调时钟量这段间隔；达到 10 ms 时这次尝试说明不了类的行为，换一个新对象重来（最多
  //   20 次；始终得不到一次干净的尝试时测试失败）。
  // 跳转本身靠轮询 Process() 等到，不用固定时长的睡眠。
  // The 20 ms timeout is part of what is tested, so two things are handled with care:
  // - "The timeout starts at the status stage": 30 ms (more than the timeout) pass
  //   between DETACH and the status stage, and Process() must not jump. Time passing is
  //   the point here, and a longer pause cannot break it.
  // - "Not expired right after the status stage": Process() runs a moment after
  //   OnControlComplete and must not jump. That holds only if the thread was not
  //   descheduled for longer than the timeout in between, which a loaded host can do. The
  //   steady clock measures the gap; when it was 10 ms or more the attempt says nothing
  //   about the class and is repeated with a fresh object (up to 20 times, and a test
  //   that never gets a clean attempt fails).
  // The jump itself is waited for by polling Process(), not by a fixed sleep.
  bool clean = false;
  for (unsigned attempt = 0; attempt < 20U && !clean; ++attempt)
  {
    g_jumps = 0;
    TestRuntime rt(&JumpToBootloader, nullptr, 20u);
    EndpointPool pool{2};
    rt.BindEndpoints(pool, 3, false);
    TestRuntime::ControlTransferResult result;

    rt.Process();
    TEST_ASSERT(g_jumps == 0);
    TEST_ASSERT(rt.OnClassRequest(false, DFU_DETACH, 0, 0, 3, result) == ErrorCode::OK);
    // 超时从状态阶段开始 / The timeout starts at the status stage.
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    rt.Process();
    TEST_ASSERT(g_jumps == 0);
    const auto begin = std::chrono::steady_clock::now();
    rt.OnControlComplete(false, SetupPacket{0x21, DFU_DETACH, 0, 3, 0});
    rt.Process();
    const auto gap = std::chrono::steady_clock::now() - begin;
    if (gap >= std::chrono::milliseconds(10)) continue;
    clean = true;
    TEST_ASSERT(g_jumps == 0);
    // 超时到期后跳转 / The jump comes once the timeout has expired.
    TEST_ASSERT(LibXR::USB::Test::WaitUntil(
        [&]()
        {
          rt.Process();
          return g_jumps != 0;
        }));
    TEST_ASSERT(g_jumps == 1);
    // 跳转只发生一次 / The jump happens once.
    rt.Process();
    TEST_ASSERT(g_jumps == 1);
  }
  TEST_ASSERT(clean);
}

/// bootloader 类、其 RAM flash，以及为其提供服务池的组合
/// The bootloader class, its RAM flash and the composition that gives it a service pool.
struct BootFixture
{
  std::array<uint8_t, 4096> flash_mem{};
  RamFlash flash{256, 8, {flash_mem.data(), flash_mem.size()}};
  TestBoot boot{flash, 0, 2048, 2048, &JumpToApp, nullptr, false};
  EndpointPool pool{2};
  DeviceComposition composition{pool, {&TEST_LANGUAGE}, {{&boot}}};
  Callback<uint32_t> handler;

  BootFixture()
  {
    // 代替 DeviceCore 的类服务分发（后端在 OnService 里提交其工作结果）。
    // Stand-in for DeviceCore's class-service dispatch (the backend commits its work
    // result from OnService).
    handler = Callback<uint32_t>::Create(
        [](bool in_isr, DeviceClass* self, uint32_t events)
        {
          if ((events & EndpointPool::CLASS_EVENT) != 0U) self->OnService(in_isr);
        },
        static_cast<DeviceClass*>(&boot));
    pool.SetControlHandler(handler);
  }
};

void TestBootloaderDownload()
{
  BootFixture f;
  TestBoot& boot = f.boot;
  boot.BindEndpoints(f.pool, 0, false);

  // 核心接收整个 wTransferSize 块：类报告其容量。
  // The core receives a whole wTransferSize block: the class reports its capacity.
  TEST_ASSERT(boot.GetControlReceiveCapacity() == 256U);

  TestBoot::ControlTransferResult result;
  auto ask = [&](uint8_t req, uint16_t val, uint16_t len, uint16_t idx)
  {
    result.write_data = {};
    return boot.OnClassRequest(false, req, val, len, idx, result);
  };

  // GETSTATUS：六个字节，仍为空闲 / GETSTATUS: six bytes, still idle.
  TEST_ASSERT(ask(DFU_GETSTATUS, 0, 6, 0) == ErrorCode::OK);
  TEST_ASSERT(result.write_data.size_ == 6U);
  TEST_ASSERT(Bytes(result.write_data)[4] == static_cast<uint8_t>(DFUState::DFU_IDLE));

  // flash 中没有有效映像时（已擦除的 RAM flash 没有）UPLOAD 被拒绝，并锁存错误状态。
  // UPLOAD is refused while no valid image is stored in the flash (the erased RAM flash
  // has none), which latches the error state.
  TEST_ASSERT(ask(DFU_UPLOAD, 0, 32, 0) != ErrorCode::OK);
  TEST_ASSERT(ask(DFU_GETSTATE, 0, 1, 0) == ErrorCode::OK);
  TEST_ASSERT(Bytes(result.write_data)[0] == static_cast<uint8_t>(DFUState::DFU_ERROR));

  // CLRSTATUS 同样没有数据阶段：请求成功且不产生 IN 数据，状态阶段由核心完成，
  // 之后设备重新空闲。
  // CLRSTATUS has no data stage either: it succeeds without IN data and the core
  // finishes the status stage, then the device is idle again.
  TEST_ASSERT(ask(DFU_CLRSTATUS, 0, 0, 0) == ErrorCode::OK);
  TEST_ASSERT(result.write_data.addr_ == nullptr && result.write_data.size_ == 0U);
  TEST_ASSERT(ask(DFU_GETSTATE, 0, 1, 0) == ErrorCode::OK);
  TEST_ASSERT(Bytes(result.write_data)[0] == static_cast<uint8_t>(DFUState::DFU_IDLE));

  // DNLOAD：请求只登记该块，OUT 数据阶段稍后到达。
  // DNLOAD: the request only registers the block; the OUT data stage arrives later.
  TEST_ASSERT(ask(DFU_DNLOAD, 0, 16, 0) == ErrorCode::OK);
  TEST_ASSERT(result.write_data.addr_ == nullptr && result.write_data.size_ == 0U);

  std::array<uint8_t, 16> payload{};
  for (size_t i = 0; i < payload.size(); ++i) payload[i] = static_cast<uint8_t>(0x5A + i);
  ConstRawData data{payload.data(), payload.size()};
  // 核心已收到整个阶段；交给类后状态机前进。
  // The core received the whole stage; handing it over advances the state machine.
  TEST_ASSERT(boot.OnClassData(false, DFU_DNLOAD, data) == ErrorCode::OK);
  TEST_ASSERT(ask(DFU_GETSTATE, 0, 1, 0) == ErrorCode::OK);
  TEST_ASSERT(Bytes(result.write_data)[0] ==
              static_cast<uint8_t>(DFUState::DFU_DNLOAD_SYNC));

  // 主机完成 GETSTATUS 的状态阶段之前，不允许执行暂存的 flash 写入；此前一直报告
  // DNBUSY 状态。
  // The staged flash write is not allowed to run until the host completed the GETSTATUS
  // status stage; the reported state is DNBUSY until then.
  TEST_ASSERT(ask(DFU_GETSTATUS, 0, 6, 0) == ErrorCode::OK);
  TEST_ASSERT(Bytes(result.write_data)[4] == static_cast<uint8_t>(DFUState::DFU_DNBUSY));
  boot.OnClassInDataStatusComplete(false, DFU_GETSTATUS);
  // 仍然不会自行发生任何事：写入在应用调用 Process() 时执行，随后通过类服务提交结果。
  // Still nothing happens on its own: the write runs when the application calls
  // Process(), which then commits the result through the class service.
  TEST_ASSERT(ask(DFU_GETSTATUS, 0, 6, 0) == ErrorCode::OK);
  TEST_ASSERT(Bytes(result.write_data)[4] == static_cast<uint8_t>(DFUState::DFU_DNBUSY));
  TEST_ASSERT(std::all_of(f.flash_mem.begin(), f.flash_mem.begin() + 16,
                          [](uint8_t b) { return b == 0U; }));
  boot.Process();
  TEST_ASSERT(ask(DFU_GETSTATUS, 0, 6, 0) == ErrorCode::OK);
  TEST_ASSERT(Bytes(result.write_data)[4] ==
              static_cast<uint8_t>(DFUState::DFU_DNLOAD_IDLE));
  // 该块确实写到了 flash 中的映像基址。
  // The block really landed in the flash at the image base.
  TEST_ASSERT(std::equal(payload.begin(), payload.end(), f.flash_mem.begin()));

  // 接受一个块之后的零长度 DNLOAD 请求 manifest。
  // A zero-length DNLOAD after one accepted block asks for manifest.
  TEST_ASSERT(ask(DFU_DNLOAD, 0, 0, 0) == ErrorCode::OK);
  TEST_ASSERT(ask(DFU_GETSTATE, 0, 1, 0) == ErrorCode::OK);
  TEST_ASSERT(Bytes(result.write_data)[0] ==
              static_cast<uint8_t>(DFUState::DFU_MANIFEST_SYNC));

  // 长于 wTransferSize 的块被拒绝 / A block longer than wTransferSize is refused.
  TEST_ASSERT(ask(DFU_DNLOAD, 0, 257, 0) != ErrorCode::OK);
  // DETACH 属于 runtime 类，而不属于 bootloader。
  // DETACH belongs to the runtime class, not to the bootloader.
  TEST_ASSERT(ask(DFU_DETACH, 0, 0, 0) != ErrorCode::OK);
  // 针对其他接口的请求被拒绝 / A request for another interface is refused.
  TEST_ASSERT(ask(DFU_GETSTATUS, 0, 6, 9) == ErrorCode::NOT_FOUND);

  boot.UnbindEndpoints(f.pool, false);
}

/// 写块期间，主机必须等待 bwPollTimeout 才能发下一个 GETSTATUS。该等待从 GETSTATUS
/// 的应答开始计，而不是从它的状态阶段开始：在硬件上，flash 操作关中断运行时，状态阶段
/// 的中断可能迟到，恰好等待 bwPollTimeout 的主机曾因此被 STALL。真正过早到来的
/// GETSTATUS 仍然被 STALL（DFU 1.1，dfuDNBUSY）。报告的超时跟随同类写入步骤上一次的
/// 实测耗时。
/// While a block is written the host must wait bwPollTimeout before the next GETSTATUS.
/// That wait counts from the GETSTATUS answer, not from its status stage: on hardware the
/// status-stage interrupt can come late while flash work runs with interrupts off, and a
/// host that waited exactly bwPollTimeout was stalled. A GETSTATUS that really comes
/// early is still stalled (DFU 1.1, dfuDNBUSY). The reported timeout follows the measured
/// duration of the last write step of the same kind.
void TestPollWaitCountsFromTheAnswer()
{
  BootFixture f;
  TestBoot& boot = f.boot;
  boot.BindEndpoints(f.pool, 0, false);

  TestBoot::ControlTransferResult result;
  auto ask = [&](uint8_t req, uint16_t val, uint16_t len)
  {
    result.write_data = {};
    return boot.OnClassRequest(false, req, val, len, 0, result);
  };
  // 通过类的控制入口发 GETSTATUS，该入口会应用轮询截止时间。
  // GETSTATUS through the class's control entry, which applies the poll deadline.
  auto get_status = [&]()
  {
    result.write_data = {};
    const SetupPacket setup{0xA1, DFU_GETSTATUS, 0, 0, 6};
    return boot.OnControlRequest(false, setup, result);
  };
  auto poll_ms = [&]()
  {
    const uint8_t* s = Bytes(result.write_data);
    return static_cast<uint32_t>(s[1]) | static_cast<uint32_t>(s[2]) << 8 |
           static_cast<uint32_t>(s[3]) << 16;
  };
  std::array<uint8_t, 256> payload{};
  payload.fill(0xA5);
  auto download_block0 = [&]()
  {
    TEST_ASSERT(ask(DFU_DNLOAD, 0, 256) == ErrorCode::OK);
    ConstRawData data{payload.data(), payload.size()};
    TEST_ASSERT(boot.OnClassData(false, DFU_DNLOAD, data) == ErrorCode::OK);
    TEST_ASSERT(get_status() == ErrorCode::OK);
    TEST_ASSERT(Bytes(result.write_data)[4] ==
                static_cast<uint8_t>(DFUState::DFU_DNBUSY));
    return poll_ms();
  };
  // 状态阶段迟到完成（如 flash 操作阻塞了中断），随后主机在应答之后恰好
  // bwPollTimeout 时回来：不 STALL，且已完成。
  // The status stage completes late (as when flash work blocks the interrupt), then the
  // host comes back exactly bwPollTimeout after the answer: not stalled, and done.
  auto late_status_stage = [&](uint32_t reported)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(reported / 2));
    boot.OnClassInDataStatusComplete(false, DFU_GETSTATUS);
    boot.Process();  // 执行写入步骤 / the write step runs
    std::this_thread::sleep_for(std::chrono::milliseconds(reported - reported / 2));
    TEST_ASSERT(get_status() == ErrorCode::OK);
    TEST_ASSERT(Bytes(result.write_data)[4] ==
                static_cast<uint8_t>(DFUState::DFU_DNLOAD_IDLE));
  };

  // 在任何测量之前，擦除步骤报告 25 ms。
  // Before any measurement a step that erases reports 25 ms.
  TEST_ASSERT(download_block0() == 25u);
  // 立即回来太早：被 STALL，设备处于 dfuERROR。
  // Coming back at once is too early: stalled, the device is in dfuERROR.
  TEST_ASSERT(get_status() != ErrorCode::OK);
  TEST_ASSERT(ask(DFU_CLRSTATUS, 0, 0) == ErrorCode::OK);

  TEST_ASSERT(download_block0() == 25u);
  late_status_stage(25u);

  // abort 之后该块需要重新擦除，此时报告实测耗时（RAM flash 上瞬时完成，所以取 10 ms
  // 的下限）。
  // After an abort the block needs its erase again, now reported with the measured
  // duration (instant on the RAM flash, so the 10 ms floor).
  TEST_ASSERT(ask(0x06 /* ABORT */, 0, 0) == ErrorCode::OK);
  TEST_ASSERT(download_block0() == 10u);
  late_status_stage(10u);
  boot.UnbindEndpoints(f.pool, false);
}

uint16_t LoadU16(const uint8_t* p)
{
  uint16_t value = 0;
  std::memcpy(&value, p, sizeof(value));
  return value;
}

/// DFU 类只声明其 WinUSB GUID；MS OS 2.0 描述符集由组合提供。单独的 bootloader 得到
/// 设备级形式，与其他功能并存的 runtime DFU 则在设备报告的唯一描述符集中得到一个功能
/// 子集，所以 BOS 只带一个 MS OS 2.0 能力。
/// The DFU classes only declare their WinUSB GUID; the MS OS 2.0 descriptor set comes
/// from the composition. A bootloader on its own gets the device-level form, and a
/// runtime DFU next to another function gets a function subset in the one set the
/// device reports, so the BOS carries a single MS OS 2.0 capability.
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

void TestWinUsbSetComesFromComposition()
{
  using Device = WinUsbMsOs20::DeviceScopedWinUsbMsOs20DescSet<
      WinUsbMsOs20::GUID_MULTI_SZ_UTF16_BYTES>;
  using Property = WinUsbMsOs20::DeviceInterfaceGuidsRegProperty<
      WinUsbMsOs20::GUID_MULTI_SZ_UTF16_BYTES>;

  BootFixture f;
  Device expected{};
  expected.Init(TestBoot::DEFAULT_WINUSB_DEVICE_INTERFACE_GUID);
  const auto standalone = ReadWinUsbSet(f.composition);
  TEST_ASSERT(standalone.size_ == sizeof(Device));
  TEST_ASSERT(std::memcmp(standalone.addr_, &expected, sizeof(Device)) == 0);
  TEST_ASSERT(static_cast<DeviceClass&>(f.boot).GetBosCapabilityCount() == 0U);

  TestRuntime rt(&JumpToBootloader);
  std::array<uint8_t, 4096> flash_mem{};
  RamFlash flash{256, 8, {flash_mem.data(), flash_mem.size()}};
  TestBoot boot{flash, 0, 2048, 2048, &JumpToApp, nullptr, false};
  EndpointPool pool{2};
  DeviceComposition composite(pool, {&TEST_LANGUAGE}, {{&boot, &rt}});
  TEST_ASSERT(static_cast<DeviceClass&>(rt).GetBosCapabilityCount() == 0U);
  TEST_ASSERT(composite.WinUsbBosCapability() != nullptr);

  constexpr size_t FUNCTION = 8 + 20 + sizeof(Property);
  const auto set = ReadWinUsbSet(composite);
  const auto* d = static_cast<const uint8_t*>(set.addr_);
  TEST_ASSERT(set.size_ == 10 + 8 + 2 * FUNCTION);
  TEST_ASSERT(LoadU16(d + 10 + 2) == 0x0001);  // 配置子集 / configuration subset
  const uint8_t* second = d + 18 + FUNCTION;
  // runtime DFU，接口 1 / runtime DFU, itf 1.
  TEST_ASSERT(LoadU16(second + 2) == 0x0002 && second[4] == 1);
}
}  // namespace

void test_usb_dfu()
{
  TestRuntimeDetach();
  TestRuntimeJumpsFromProcess();
  TestBootloaderDownload();
  TestPollWaitCountsFromTheAnswer();
  TestWinUsbSetComesFromComposition();
}
