/**
 * @file mock_endpoint.hpp
 * @brief 设备类主机测试共用的端点替身 / Shared endpoint double for device-class host
 * tests.
 *
 * 与 core/test_endpoint.cpp 里的 MockEndpoint 行为一致：记录每次硬件启动、由测试扮演
 * 控制器推进完成与停止。设备类测试（HID/DAP/UAC/gs_usb...）用它替代真实后端。
 * Behaves like the MockEndpoint in core/test_endpoint.cpp: it records every hardware
 * start and lets the test play the controller for completions and stops. Device-class
 * tests use it in place of a real backend.
 */

#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <vector>

#include "ep.hpp"
#include "ep_pool.hpp"
#include "test_assert.hpp"

namespace LibXR::USB::Test
{
using Direction = Endpoint::Direction;
using Number = Endpoint::EPNumber;
using Type = Endpoint::Type;
using State = Endpoint::State;
using Completion = Endpoint::Completion;
using Slot = Endpoint::Slot;

/// Cache-line aligned storage block; the host build's line is 64 B.
template <size_t N>
struct alignas(64) Block
{
  std::array<uint8_t, N> bytes{};
  RawData Raw() { return {bytes.data(), bytes.size()}; }
};

/// Backend double: records every start and lets the test play the hardware.
class MockEndpoint : public Endpoint
{
 public:
  struct Start
  {
    uint32_t tag;
    uint8_t* address;
    size_t capacity;
    size_t length;
    std::vector<uint8_t> data;
  };

  MockEndpoint(Number number, Direction dir, RawData storage, uint8_t slot_count = 0)
      : Endpoint(number, dir, storage, slot_count)
  {
  }

  std::vector<Start> starts;
  std::vector<std::vector<uint8_t>> sent;
  bool active = false;
  bool pending_stop = false;  // StopHardware answers PENDING.
  bool stop_requested = false;
  size_t reap_actual = 0;
  bool reap_retired = false;
  unsigned closes = 0, stalls = 0, clear_stalls = 0;
  size_t limit = UINT16_MAX;
  ErrorCode configure_result = ErrorCode::OK;
  bool fail_next_start = false;
  // 只有多线程测试会在另一个线程里扮演硬件/主机。StartHardware/StopHardware 在 lock 下
  // 改状态，对应真实 MCU 上屏蔽中断的临界区；主机侧读写同一状态也必须取这把锁。
  // Only multi-threaded tests play the hardware/host from another thread.
  // StartHardware/StopHardware change state under lock, which models the interrupt-masked
  // region of a real MCU; the host side takes the same lock to read or write that state.
  std::mutex lock;

  const Start& Last() const { return starts.back(); }

  /// 在锁内读取 active，供另一个线程扮演主机时使用 / Read `active` under the lock, for a
  /// host played from another thread.
  bool IsActive()
  {
    std::lock_guard<std::mutex> guard(lock);
    return active;
  }
  /// 在锁内读取最近一次启动的长度 / Length of the last start, read under the lock.
  size_t LastLength()
  {
    std::lock_guard<std::mutex> guard(lock);
    return starts.back().length;
  }

  /// IN: the host takes the whole request.
  void Complete(bool in_isr = true)
  {
    uint32_t tag;
    size_t length;
    {
      std::lock_guard<std::mutex> guard(lock);
      TEST_ASSERT(active);
      active = false;
      const Start& start = starts.back();
      if (start.length != 0U)
        TEST_ASSERT(std::equal(start.data.begin(), start.data.end(), start.address));
      sent.push_back(start.data);
      tag = start.tag;
      length = start.length;
    }
    OnHardwareComplete(in_isr, tag, length);
  }
  /// IN: the host took fewer bytes than requested.
  void CompleteWith(size_t actual, ErrorCode result = ErrorCode::OK)
  {
    TEST_ASSERT(active);
    active = false;
    OnHardwareComplete(true, starts.back().tag, actual, result);
  }
  /// OUT: the host sends bytes into the armed slot.
  void HostSends(const std::vector<uint8_t>& bytes)
  {
    uint8_t* address;
    uint32_t tag;
    {
      // 与 Complete 一样：状态在锁内，OnHardwareComplete 在锁外（它可能就地运行 owner，
      // 进而在 StartHardware 里取同一把锁）。
      // As in Complete: state under the lock, OnHardwareComplete outside it (it may run
      // the owner in place, which takes the same lock in StartHardware).
      std::lock_guard<std::mutex> guard(lock);
      TEST_ASSERT(active && bytes.size() <= starts.back().capacity);
      active = false;
      address = starts.back().address;
      tag = starts.back().tag;
    }
    if (!bytes.empty()) std::memcpy(address, bytes.data(), bytes.size());
    OnHardwareComplete(true, tag, bytes.size());
  }
  void ConfirmStop()
  {
    TEST_ASSERT(stop_requested);
    stop_requested = false;
    active = false;
    OnHardwareStopped(true);
  }
  void Late(uint32_t tag, size_t actual) { OnHardwareComplete(true, tag, actual); }

 protected:
  ErrorCode ConfigureHardware(const Config&) override { return configure_result; }
  void CloseHardware(bool) override
  {
    TEST_ASSERT(!active);
    ++closes;
  }
  ErrorCode StartHardware(const HwRequest& request, bool) override
  {
    std::lock_guard<std::mutex> guard(lock);
    if (fail_next_start)
    {
      fail_next_start = false;
      return ErrorCode::FAILED;
    }
    TEST_ASSERT(!active && !stop_requested);
    TEST_ASSERT(request.length <= request.slot.size_ || request.slot.size_ == 0U);
    Start start{request.tag,
                static_cast<uint8_t*>(request.slot.addr_),
                request.slot.size_,
                request.length,
                {}};
    if (GetConfig().direction == Direction::IN && request.length)
      start.data.assign(start.address, start.address + request.length);
    starts.push_back(std::move(start));
    active = true;
    return ErrorCode::OK;
  }
  StopResult StopHardware(bool) override
  {
    // A SETUP may already have cancelled an EP0 transfer in hardware.
    std::lock_guard<std::mutex> guard(lock);
    if (!pending_stop)
    {
      active = false;
      return StopResult::DONE;
    }
    stop_requested = true;
    return StopResult::PENDING;
  }
  size_t ReapProgress(const HwRequest&, bool& retired) override
  {
    retired = reap_retired;
    return reap_actual;
  }
  ErrorCode StallHardware(bool) override
  {
    ++stalls;
    return ErrorCode::OK;
  }
  ErrorCode ClearStallHardware(bool) override
  {
    ++clear_stalls;
    return ErrorCode::OK;
  }
  size_t MaxPacketSizeLimit(Type) const override { return limit; }
};

/// Records completions delivered to one endpoint binding.
struct Recorder
{
  std::vector<Completion> list;
  Callback<const Completion&> Bind()
  {
    return Callback<const Completion&>::Create(
        [](bool, Recorder* self, const Completion& completion)
        { self->list.push_back(completion); }, this);
  }
};

inline std::vector<uint8_t> Bytes(size_t size, uint8_t seed)
{
  std::vector<uint8_t> data(size);
  for (size_t i = 0; i != size; ++i) data[i] = static_cast<uint8_t>(seed + i);
  return data;
}

/// Everything the controller put on the wire, in start order.
inline std::vector<uint8_t> Wire(const MockEndpoint& ep)
{
  std::vector<uint8_t> data;
  for (const auto& packet : ep.sent)
    data.insert(data.end(), packet.begin(), packet.end());
  return data;
}

/// Three data endpoints plus their pool: EP1 IN, EP2 OUT, EP3 IN.
struct Fixture
{
  Block<128> in_storage;
  Block<128> out_storage;
  Block<128> comm_storage;
  EndpointPool pool{4};
  MockEndpoint in{Number::EP1, Direction::IN, in_storage.Raw()};
  MockEndpoint out{Number::EP2, Direction::OUT, out_storage.Raw()};
  MockEndpoint comm{Number::EP3, Direction::IN, comm_storage.Raw()};
  Fixture()
  {
    TEST_ASSERT(pool.Put(&in) == ErrorCode::OK);
    TEST_ASSERT(pool.Put(&out) == ErrorCode::OK);
    TEST_ASSERT(pool.Put(&comm) == ErrorCode::OK);
  }
};

}  // namespace LibXR::USB::Test
