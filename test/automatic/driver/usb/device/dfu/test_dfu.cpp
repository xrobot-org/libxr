/**
 * @file test_dfu.cpp
 * @brief DFU 设备类（槽模型）主机测试 / Host tests for the DFU device classes.
 *
 * 覆盖 5B 迁移改动的路径：控制传输的状态阶段改由核心完成（删除 SendStatusInZLP 后
 * 请求仍返回 OK 且不产生 IN 数据），以及 DNLOAD 的 OUT 数据阶段由核心收进自己的缓冲
 * 区、完整后经 OnControlData -> OnClassData 交给类，再由 GetControlReceiveCapacity
 * 覆盖 wTransferSize 块长。
 * Covers the paths the 5B migration changed: a control transfer's status stage is now
 * finished by the core (with SendStatusInZLP gone the request still returns OK and
 * produces no IN data), and the DNLOAD OUT data stage is received into the core's own
 * buffer and handed to the class via OnControlData -> OnClassData, with
 * GetControlReceiveCapacity covering the wTransferSize block length.
 */

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <thread>

#include "device_composition.hpp"
#include "dfu_bootloader.hpp"
#include "dfu_runtime.hpp"
#include "ep_pool.hpp"
#include "flash.hpp"
#include "lockfree_list.hpp"
#include "test_assert.hpp"
#include "timer.hpp"

using namespace LibXR;
using namespace LibXR::USB;

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

/// Give this test its own timer list so a class destroyed earlier cannot leave a dangling
/// task node behind for the next Timer::Refresh().
void FreshTimerList() { LibXR::Timer::list_ = new LibXR::LockFreeList(); }

/// RAM-backed Flash: erase fills with 0xFF, write copies, read is the base default.
class RamFlash : public LibXR::Flash
{
 public:
  RamFlash(size_t min_erase, size_t min_write, RawData area)
      : Flash(min_erase, min_write, area), area_(area)
  {
  }
  ErrorCode Erase(size_t offset, size_t size) override
  {
    TEST_ASSERT(offset + size <= area_.size_);
    std::memset(static_cast<uint8_t*>(area_.addr_) + offset, 0xFF, size);
    return ErrorCode::OK;
  }
  ErrorCode Write(size_t offset, ConstRawData data) override
  {
    TEST_ASSERT(offset + data.size_ <= area_.size_);
    std::memcpy(static_cast<uint8_t*>(area_.addr_) + offset, data.addr_, data.size_);
    return ErrorCode::OK;
  }

 private:
  RawData area_;
};

unsigned g_jumps = 0;
void JumpToApp(void*) { ++g_jumps; }
void JumpToBootloader(void*) { ++g_jumps; }

class TestRuntime : public DfuRuntimeClass
{
 public:
  using DfuRuntimeClass::BindEndpoints;
  using DfuRuntimeClass::DfuRuntimeClass;
  using DfuRuntimeClass::OnClassRequest;
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

/// Byte view of a control response.
const uint8_t* Bytes(const ConstRawData& data)
{
  return static_cast<const uint8_t*>(data.addr_);
}

void TestRuntimeDetach()
{
  FreshTimerList();
  TestRuntime rt(&JumpToBootloader, nullptr, 20u);
  EndpointPool pool{2};
  rt.BindEndpoints(pool, 3, false);
  TestRuntime::ControlTransferResult result;
  // DeviceCore hands a fresh result to every SETUP; the class only fills what the
  // request produces, so a reused result would keep the previous response.
  auto ask = [&](uint8_t req, uint16_t val, uint16_t len, uint16_t idx)
  {
    result.write_data = {};
    return rt.OnClassRequest(false, req, val, len, idx, result);
  };

  // A request for another interface is refused.
  TEST_ASSERT(ask(DFU_GETSTATUS, 0, 6, 2) == ErrorCode::NOT_FOUND);

  // DETACH has no data stage: the request succeeds without IN data and the core finishes
  // the status stage itself (the removed SendStatusInZLP used to ask for a ZLP).
  TEST_ASSERT(ask(DFU_DETACH, 0, 0, 3) == ErrorCode::OK);
  TEST_ASSERT(result.write_data.addr_ == nullptr && result.write_data.size_ == 0U);

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

  // GETSTATE answers the same state.
  TEST_ASSERT(ask(DFU_GETSTATE, 0, 1, 3) == ErrorCode::OK);
  TEST_ASSERT(result.write_data.size_ == 1U &&
              Bytes(result.write_data)[0] == static_cast<uint8_t>(DFUState::APP_DETACH));

  // DETACH with a data stage is refused.
  rt.UnbindEndpoints(pool, false);
  rt.BindEndpoints(pool, 3, false);
  TEST_ASSERT(ask(DFU_DETACH, 0, 4, 3) == ErrorCode::ARG_ERR);
  // An unknown request is refused.
  TEST_ASSERT(ask(0x7F, 0, 0, 3) == ErrorCode::ARG_ERR);

  // Without a jump callback the runtime class does not offer DETACH.
  TestRuntime plain(nullptr);
  plain.BindEndpoints(pool, 3, false);
  TEST_ASSERT(plain.OnClassRequest(false, DFU_DETACH, 0, 0, 3, result) ==
              ErrorCode::NOT_SUPPORT);
  // Before binding, requests are refused.
  TestRuntime unbound(&JumpToBootloader);
  TEST_ASSERT(unbound.OnClassRequest(false, DFU_GETSTATUS, 0, 6, 0, result) ==
              ErrorCode::INIT_ERR);
}

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
  FreshTimerList();
  BootFixture f;
  TestBoot& boot = f.boot;
  boot.BindEndpoints(f.pool, 0, false);

  // The core receives a whole wTransferSize block: the class reports its capacity.
  TEST_ASSERT(boot.GetControlReceiveCapacity() == 256U);

  TestBoot::ControlTransferResult result;
  auto ask = [&](uint8_t req, uint16_t val, uint16_t len, uint16_t idx)
  {
    result.write_data = {};
    return boot.OnClassRequest(false, req, val, len, idx, result);
  };

  // GETSTATUS: six bytes, still idle.
  TEST_ASSERT(ask(DFU_GETSTATUS, 0, 6, 0) == ErrorCode::OK);
  TEST_ASSERT(result.write_data.size_ == 6U);
  TEST_ASSERT(Bytes(result.write_data)[4] == static_cast<uint8_t>(DFUState::DFU_IDLE));

  // UPLOAD is refused while no valid image is stored in the flash (the erased RAM flash
  // has none), which latches the error state.
  TEST_ASSERT(ask(DFU_UPLOAD, 0, 32, 0) != ErrorCode::OK);
  TEST_ASSERT(ask(DFU_GETSTATE, 0, 1, 0) == ErrorCode::OK);
  TEST_ASSERT(Bytes(result.write_data)[0] == static_cast<uint8_t>(DFUState::DFU_ERROR));

  // CLRSTATUS has no data stage either: it succeeds without IN data and the core
  // finishes the status stage, then the device is idle again.
  TEST_ASSERT(ask(DFU_CLRSTATUS, 0, 0, 0) == ErrorCode::OK);
  TEST_ASSERT(result.write_data.addr_ == nullptr && result.write_data.size_ == 0U);
  TEST_ASSERT(ask(DFU_GETSTATE, 0, 1, 0) == ErrorCode::OK);
  TEST_ASSERT(Bytes(result.write_data)[0] == static_cast<uint8_t>(DFUState::DFU_IDLE));

  // DNLOAD: the request only registers the block; the OUT data stage arrives later.
  TEST_ASSERT(ask(DFU_DNLOAD, 0, 16, 0) == ErrorCode::OK);
  TEST_ASSERT(result.write_data.addr_ == nullptr && result.write_data.size_ == 0U);

  std::array<uint8_t, 16> payload{};
  for (size_t i = 0; i < payload.size(); ++i) payload[i] = static_cast<uint8_t>(0x5A + i);
  ConstRawData data{payload.data(), payload.size()};
  // The core received the whole stage; handing it over advances the state machine.
  TEST_ASSERT(boot.OnClassData(false, DFU_DNLOAD, data) == ErrorCode::OK);
  TEST_ASSERT(ask(DFU_GETSTATE, 0, 1, 0) == ErrorCode::OK);
  TEST_ASSERT(Bytes(result.write_data)[0] ==
              static_cast<uint8_t>(DFUState::DFU_DNLOAD_SYNC));

  // The staged flash write is not allowed to run until the host completed the GETSTATUS
  // status stage; the reported state is DNBUSY until then.
  TEST_ASSERT(ask(DFU_GETSTATUS, 0, 6, 0) == ErrorCode::OK);
  TEST_ASSERT(Bytes(result.write_data)[4] == static_cast<uint8_t>(DFUState::DFU_DNBUSY));
  boot.OnClassInDataStatusComplete(false, DFU_GETSTATUS);
  // The write itself runs in the backend's own (Timer) context, which then commits the
  // result through the class service.
  Timer::Refresh();
  TEST_ASSERT(ask(DFU_GETSTATUS, 0, 6, 0) == ErrorCode::OK);
  TEST_ASSERT(Bytes(result.write_data)[4] ==
              static_cast<uint8_t>(DFUState::DFU_DNLOAD_IDLE));
  // The block really landed in the flash at the image base.
  TEST_ASSERT(std::equal(payload.begin(), payload.end(), f.flash_mem.begin()));

  // A zero-length DNLOAD after one accepted block asks for manifest.
  TEST_ASSERT(ask(DFU_DNLOAD, 0, 0, 0) == ErrorCode::OK);
  TEST_ASSERT(ask(DFU_GETSTATE, 0, 1, 0) == ErrorCode::OK);
  TEST_ASSERT(Bytes(result.write_data)[0] ==
              static_cast<uint8_t>(DFUState::DFU_MANIFEST_SYNC));

  // A block longer than wTransferSize is refused.
  TEST_ASSERT(ask(DFU_DNLOAD, 0, 257, 0) != ErrorCode::OK);
  // DETACH belongs to the runtime class, not to the bootloader.
  TEST_ASSERT(ask(DFU_DETACH, 0, 0, 0) != ErrorCode::OK);
  // A request for another interface is refused.
  TEST_ASSERT(ask(DFU_GETSTATUS, 0, 6, 9) == ErrorCode::NOT_FOUND);

  boot.UnbindEndpoints(f.pool, false);
}

/// While a block is written the host must wait bwPollTimeout before the next GETSTATUS.
/// That wait counts from the GETSTATUS answer, not from its status stage: on hardware the
/// status-stage interrupt can come late while flash work runs with interrupts off, and a
/// host that waited exactly bwPollTimeout was stalled. A GETSTATUS that really comes
/// early is still stalled (DFU 1.1, dfuDNBUSY). The reported timeout follows the measured
/// duration of the last write step of the same kind.
void TestPollWaitCountsFromTheAnswer()
{
  FreshTimerList();
  BootFixture f;
  TestBoot& boot = f.boot;
  boot.BindEndpoints(f.pool, 0, false);

  TestBoot::ControlTransferResult result;
  auto ask = [&](uint8_t req, uint16_t val, uint16_t len)
  {
    result.write_data = {};
    return boot.OnClassRequest(false, req, val, len, 0, result);
  };
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
  // The status stage completes late (as when flash work blocks the interrupt), then the
  // host comes back exactly bwPollTimeout after the answer: not stalled, and done.
  auto late_status_stage = [&](uint32_t reported)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(reported / 2));
    boot.OnClassInDataStatusComplete(false, DFU_GETSTATUS);
    Timer::Refresh();  // the write step runs
    std::this_thread::sleep_for(std::chrono::milliseconds(reported - reported / 2));
    TEST_ASSERT(get_status() == ErrorCode::OK);
    TEST_ASSERT(Bytes(result.write_data)[4] ==
                static_cast<uint8_t>(DFUState::DFU_DNLOAD_IDLE));
  };

  // Before any measurement a step that erases reports 25 ms.
  TEST_ASSERT(download_block0() == 25u);
  // Coming back at once is too early: stalled, the device is in dfuERROR.
  TEST_ASSERT(get_status() != ErrorCode::OK);
  TEST_ASSERT(ask(DFU_CLRSTATUS, 0, 0) == ErrorCode::OK);

  TEST_ASSERT(download_block0() == 25u);
  late_status_stage(25u);

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

  FreshTimerList();
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
  TEST_ASSERT(LoadU16(d + 10 + 2) == 0x0001);  // configuration subset
  const uint8_t* second = d + 18 + FUNCTION;
  TEST_ASSERT(LoadU16(second + 2) == 0x0002 && second[4] == 1);  // runtime DFU, itf 1
}
}  // namespace

int main()
{
  TestRuntimeDetach();
  TestBootloaderDownload();
  TestPollWaitCountsFromTheAnswer();
  TestWinUsbSetComesFromComposition();
  std::puts(
      "DFU classes: status stage in the core, DNLOAD data stage, capacity and WinUSB "
      "descriptor set passed");
}
