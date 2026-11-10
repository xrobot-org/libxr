/**
 * @file usb_backend_cases.hpp
 * @brief 两个 DWC2 后端共用的契约用例 / Contract cases shared by the DWC2 backends.
 *
 * 每个用例是一个以夹具类型为参数的模板。夹具提供控制器模型 hw、主机脚本 host、探针设备类
 * probe、被测设备 device、Service()（运行中断处理）、Up()（枚举到配置 1）、
 * VendorRead/VendorWrite 和批量包长 bulk；用例只通过端点 API、DeviceCore 和总线事件
 * 驱动后端。每个夹具在析构时检查控制器模型没有记下违反手册的操作。夹具的常量
 * EXACT_IN_PROGRESS 为假时，被停止的 IN 请求报告的已发送长度不作检查；
 * CLOSE_DEACTIVATES 为假时，只检查关闭的端点不再传数据，不检查它不应答。
 * Each case is a template over the fixture type. A fixture provides the controller model
 * hw, the host script host, the probe class probe, the device under test device,
 * Service() (runs the interrupt handler), Up() (enumerates to configuration 1),
 * VendorRead/VendorWrite and the bulk packet size bulk; a case drives the backend only
 * through the endpoint API, DeviceCore and bus events. Every fixture checks on
 * destruction that the controller model recorded no operation the manual forbids. With
 * the fixture constant EXACT_IN_PROGRESS false, the sent length a stopped IN request
 * reports is not checked; with CLOSE_DEACTIVATES false, a closed endpoint is only
 * checked to move no data, not to stay silent.
 */

#pragma once

#include <cstdio>
#include <cstring>
#include <vector>

#include "dwc2_host.hpp"
#include "test_assert.hpp"
#include "usb_probe_class.hpp"

namespace UsbBackendCases
{
using namespace LibXR;
using namespace LibXR::USB;
using Dwc2::Resp;
using UsbProbe::Log;
using Result = Dwc2::Host::Result;

inline std::vector<uint8_t> Bytes(size_t count, uint8_t seed)
{
  std::vector<uint8_t> data(count);
  for (size_t i = 0; i < count; ++i) data[i] = static_cast<uint8_t>(seed + i * 3U);
  return data;
}

/// 借一个 IN 槽、拷入并提交 / Borrow an IN slot, copy the bytes in and submit
inline ErrorCode SubmitBytes(Endpoint* in, const std::vector<uint8_t>& bytes,
                             bool zlp = false)
{
  Endpoint::Slot slot;
  const ErrorCode acquire = in->AcquireSlot(slot, false);
  if (acquire != ErrorCode::OK) return acquire;
  if (!bytes.empty()) std::memcpy(slot.data.addr_, bytes.data(), bytes.size());
  return in->Submit(slot, bytes.size(), zlp, false);
}

/// 枚举：复位后在地址 0 读设备描述符；SET_ADDRESS 的状态阶段仍在地址 0 完成，之后设备只
/// 在新地址应答；读配置描述符；SET_CONFIGURATION 绑定设备类，GET_CONFIGURATION 读回。
/// Enumeration: the device descriptor is read at address 0 after the reset; the status
/// stage of SET_ADDRESS still completes at address 0 and from then on the device answers
/// only at the new address; the configuration descriptor is read; SET_CONFIGURATION binds
/// the class and GET_CONFIGURATION reads it back.
template <typename Fixture>
void TestEnumeration(const typename Fixture::ModeType& mode)
{
  Fixture f(mode);
  f.host.Reset();
  TEST_ASSERT(f.probe.resets == 1U);
  TEST_ASSERT(f.hw.HighSpeed() == Fixture::HighSpeed(mode));

  std::vector<uint8_t> data;
  TEST_ASSERT(f.host.GetDescriptor(1, 0, 64, data) == Result::OK);
  TEST_ASSERT(data.size() == 18U && data[0] == 18U && data[1] == 1U && data[7] == 64U);
  TEST_ASSERT(data[8] == 0x09U && data[9] == 0x12U);

  TEST_ASSERT(f.host.SetAddress(7) == Result::OK);
  TEST_ASSERT(f.hw.Address() == 7U);
  f.host.address = 0;
  TEST_ASSERT(f.host.GetDescriptor(1, 0, 18, data) == Result::TIMEOUT);
  f.host.address = 7;
  TEST_ASSERT(f.host.GetDescriptor(1, 0, 18, data) == Result::OK && data.size() == 18U);

  // 配置描述符：先读 9 字节头，再读全长 / Header first, then the whole descriptor
  TEST_ASSERT(f.host.GetDescriptor(2, 0, 9, data) == Result::OK && data.size() == 9U);
  const uint16_t total = static_cast<uint16_t>(data[2] | (data[3] << 8U));
  TEST_ASSERT(total == 9U + 9U + 5U * 7U);
  TEST_ASSERT(f.host.GetDescriptor(2, 0, 255, data) == Result::OK);
  TEST_ASSERT(data.size() == total && data[4] == 1U);
  // 批量端点的包长跟随速度 / The bulk packet size follows the speed
  TEST_ASSERT((data[22] | (data[23] << 8U)) == (Fixture::HighSpeed(mode) ? 512 : 64));

  // 字符串描述符 / String descriptors
  TEST_ASSERT(f.host.GetDescriptor(3, 0, 255, data) == Result::OK && data.size() == 4U);
  TEST_ASSERT(f.host.GetDescriptor(3, 2, 255, data, 0x0409) == Result::OK);
  TEST_ASSERT(data.size() > 2U && data.size() == data[0] && data[1] == 3U);

  TEST_ASSERT(!f.device.IsConfigured());
  TEST_ASSERT(f.host.SetConfiguration(1) == Result::OK);
  TEST_ASSERT(f.device.IsConfigured() && f.probe.binds == 1U);
  TEST_ASSERT(f.host.ControlRead(Dwc2::Host::Setup(0x80, 8, 0, 0, 1), data) ==
              Result::OK);
  TEST_ASSERT(data.size() == 1U && data[0] == 1U);
  // 未配置的端点不应答，配置后的端点应答 / Unconfigured endpoints stay silent
  TEST_ASSERT(f.hw.HostIn(7, 1).resp == Resp::NAK);
  TEST_ASSERT(f.hw.HostIn(7, 4).resp == Resp::TIMEOUT);
}

// ---------------------------------------------------------------------------
// Control transfers
// ---------------------------------------------------------------------------

/// 控制读的各种长度：短于一个包、恰好一个包、多包、长度是包长整数倍且短于 wLength 时以
/// 零长包结束、等于 wLength 时不加零长包、响应长于 wLength 时截断、零长数据阶段。
/// Control reads of every shape: shorter than a packet, exactly one packet, several
/// packets, a multiple of the packet size shorter than wLength ended by a zero-length
/// packet, none when it equals wLength, a response longer than wLength cut off, and an
/// empty data stage.
template <typename Fixture>
void TestControlRead(const typename Fixture::ModeType& mode)
{
  Fixture f(mode);
  f.Up();
  for (size_t i = 0; i < f.probe.input.size(); ++i)
    f.probe.input[i] = static_cast<uint8_t>(i * 7U + 1U);
  struct Shape
  {
    size_t response;
    uint16_t length;
    size_t expected;
    unsigned packets;  ///< 数据阶段的包数，含结尾的零长包 / Including a final ZLP
  };
  const Shape shapes[] = {{1, 64, 1, 1},      {63, 64, 63, 1},    {64, 64, 64, 1},
                          {64, 255, 64, 2},   {65, 255, 65, 2},   {128, 255, 128, 3},
                          {128, 128, 128, 2}, {200, 255, 200, 4}, {200, 100, 100, 2},
                          {0, 64, 0, 1},      {300, 512, 300, 5}};
  unsigned completed = 0;
  for (const Shape& shape : shapes)
  {
    f.probe.response_size = shape.response;
    const auto setup = Dwc2::Host::Setup(0xC1, 0x70, 0, 0, shape.length);
    TEST_ASSERT(f.host.SendSetup(setup) == Result::OK);
    std::vector<uint8_t> data;
    uint8_t pid = 1;
    unsigned packets = 0;
    while (data.size() < shape.length)
    {
      std::vector<uint8_t> packet;
      TEST_ASSERT(f.host.InPacket(0, packet, &pid) == Result::OK);
      ++packets;
      data.insert(data.end(), packet.begin(), packet.end());
      if (packet.size() < 64U) break;
    }
    TEST_ASSERT(packets == shape.packets && data.size() == shape.expected);
    TEST_ASSERT(data.empty() ||
                std::memcmp(data.data(), f.probe.input.data(), data.size()) == 0);
    // 数据阶段结束后设备不再发数据 / Nothing more follows the data stage
    TEST_ASSERT(f.hw.HostIn(7, 0).resp == Resp::NAK);
    f.Service();
    TEST_ASSERT(f.host.StatusOut() == Result::OK);
    TEST_ASSERT(f.probe.complete_count == ++completed);
  }
  TEST_ASSERT(f.probe.abort_count == 0U);
}

/// 控制写：1 字节到多包的数据阶段，数据完整交给设备类一次，状态阶段是零长 IN 包。
/// Control writes from one byte to several packets: the class gets the complete data
/// once and the status stage is a zero-length IN packet.
template <typename Fixture>
void TestControlWrite(const typename Fixture::ModeType& mode)
{
  Fixture f(mode);
  f.Up();
  unsigned count = 0;
  for (size_t size : {size_t{1}, size_t{7}, size_t{64}, size_t{65}, size_t{128},
                      size_t{200}, size_t{512}})
  {
    const auto data = Bytes(size, static_cast<uint8_t>(size));
    TEST_ASSERT(f.VendorWrite(data) == Result::OK);
    TEST_ASSERT(f.probe.data_count == ++count && f.probe.received == size);
    TEST_ASSERT(std::memcmp(f.probe.output.data(), data.data(), size) == 0);
    TEST_ASSERT(f.probe.complete_count == count);
  }
  // 无数据阶段 / No data stage
  TEST_ASSERT(f.host.ControlNoData(Dwc2::Host::Setup(0x41, 0x72, 0, 0, 0)) == Result::OK);
  TEST_ASSERT(f.probe.complete_count == count + 1U && f.probe.data_count == count);
  TEST_ASSERT(f.probe.abort_count == 0U);
}

/// 不支持的请求：数据阶段和状态阶段都回 STALL，直到下一个 SETUP；下一个 SETUP 照常处理。
/// 无数据和 IN 数据两种请求都检查。
/// An unsupported request stalls its data and status stages until the next SETUP, which
/// is handled normally; checked for a request without data and with IN data.
template <typename Fixture>
void TestControlStall(const typename Fixture::ModeType& mode)
{
  Fixture f(mode);
  f.Up();
  std::vector<uint8_t> data;

  // IN 数据阶段 / IN data stage
  TEST_ASSERT(f.host.ControlRead(Dwc2::Host::Setup(0xC1, 0x7F, 0, 0, 8), data) ==
              Result::STALL);
  TEST_ASSERT(f.hw.HostIn(7, 0).resp == Resp::STALL);  // 保持 STALL / Stays stalled
  TEST_ASSERT(f.VendorRead(10, 64, data) == Result::OK && data.size() == 10U);

  // 无数据阶段 / No data stage
  TEST_ASSERT(f.host.ControlNoData(Dwc2::Host::Setup(0x41, 0x7F, 0, 0, 0)) ==
              Result::STALL);
  TEST_ASSERT(f.VendorRead(10, 64, data) == Result::OK && data.size() == 10U);

  // 标准请求的错误参数 / A standard request with a bad argument
  TEST_ASSERT(f.host.GetDescriptor(2, 5, 9, data) == Result::STALL);
  TEST_ASSERT(f.host.GetDescriptor(1, 0, 18, data) == Result::OK && data.size() == 18U);
}

/// 带 OUT 数据阶段的请求被拒绝：数据阶段回 STALL，下一个 SETUP 照常处理。
/// A refused request with an OUT data stage: the data stage is stalled and the next
/// SETUP is handled normally.
template <typename Fixture>
void TestControlStallOutData(const typename Fixture::ModeType& mode)
{
  Fixture f(mode);
  f.Up();
  std::vector<uint8_t> data;
  TEST_ASSERT(f.host.ControlWrite(Dwc2::Host::Setup(0x41, 0x7F, 0, 0, 4), Bytes(4, 1)) ==
              Result::STALL);
  TEST_ASSERT(f.VendorWrite(Bytes(4, 9)) == Result::OK && f.probe.data_count == 1U);
  TEST_ASSERT(f.VendorRead(10, 64, data) == Result::OK && data.size() == 10U);
}

/// 背靠背的 SETUP：主机连发几个 SETUP，中间没有 IN/OUT 令牌，设备应答最后一个。三个
/// SETUP 在同一次中断里被看到，和每个 SETUP 之后都进了中断，两种情况都检查。
/// Back-to-back SETUPs: the host sends several SETUPs with no IN or OUT token in between
/// and the device answers the last one, both when one interrupt sees all three and when
/// an interrupt follows each of them.
template <typename Fixture>
void TestControlBackToBackSetups(const typename Fixture::ModeType& mode)
{
  Fixture f(mode);
  f.Up();
  f.host.auto_service = false;
  f.probe.response_size = 5;
  const auto first = Dwc2::Host::Setup(0xC1, 0x70, 1, 0, 64);
  const auto second = Dwc2::Host::Setup(0x41, 0x7F, 2, 0, 0);
  const auto third = Dwc2::Host::Setup(0xC1, 0x70, 3, 0, 5);
  TEST_ASSERT(f.host.SendSetup(first) == Result::OK);
  TEST_ASSERT(f.host.SendSetup(second) == Result::OK);
  TEST_ASSERT(f.host.SendSetup(third) == Result::OK);
  f.host.auto_service = true;
  uint8_t pid = 1;
  std::vector<uint8_t> packet;
  TEST_ASSERT(f.host.InPacket(0, packet, &pid) == Result::OK && packet.size() == 5U);
  TEST_ASSERT(f.probe.last_value == 3U && f.probe.requests == 1U);
  TEST_ASSERT(f.host.StatusOut() == Result::OK);
  TEST_ASSERT(f.probe.complete_count == 1U);

  const auto fourth = Dwc2::Host::Setup(0xC1, 0x70, 4, 0, 5);
  TEST_ASSERT(f.host.SendSetup(first) == Result::OK);
  TEST_ASSERT(f.host.SendSetup(fourth) == Result::OK);
  pid = 1;
  TEST_ASSERT(f.host.InPacket(0, packet, &pid) == Result::OK && packet.size() == 5U);
  TEST_ASSERT(f.probe.last_value == 4U && f.probe.requests == 2U);
  TEST_ASSERT(f.host.StatusOut() == Result::OK);
  TEST_ASSERT(f.probe.complete_count == 2U);
}

/// SETUP 打断多包的 IN 数据阶段：旧传输结束（设备类收到一次 abort），新请求的数据完整，
/// 旧请求的包不会再出现。
/// A SETUP interrupts a multi-packet IN data stage: the old transfer ends (one abort for
/// the class), the data of the new request is complete and no packet of the old request
/// shows up again.
template <typename Fixture>
void TestControlSetupDuringInData(const typename Fixture::ModeType& mode)
{
  Fixture f(mode);
  f.Up();
  for (size_t i = 0; i < f.probe.input.size(); ++i)
    f.probe.input[i] = static_cast<uint8_t>(i + 1U);
  std::vector<uint8_t> data;
  std::vector<uint8_t> packet;
  f.probe.response_size = 200;
  TEST_ASSERT(f.host.SendSetup(Dwc2::Host::Setup(0xC1, 0x70, 0, 0, 255)) == Result::OK);
  uint8_t pid = 1;
  TEST_ASSERT(f.host.InPacket(0, packet, &pid) == Result::OK && packet.size() == 64U);
  for (size_t i = 0; i < f.probe.input.size(); ++i)
    f.probe.input[i] = static_cast<uint8_t>(0x80U + i);
  TEST_ASSERT(f.VendorRead(70, 255, data) == Result::OK && data.size() == 70U);
  TEST_ASSERT(std::memcmp(data.data(), f.probe.input.data(), 70) == 0);
  TEST_ASSERT(f.probe.abort_count == 1U && f.probe.complete_count == 1U);
  TEST_ASSERT(f.VendorWrite(Bytes(3, 5)) == Result::OK && f.probe.received == 3U);
}

/// SETUP 打断 OUT 数据阶段：已经收到的第一个包被丢弃，新请求的数据完整交给设备类一次。
/// A SETUP interrupts an OUT data stage: the first packet already received is dropped
/// and the data of the new request reaches the class once, complete.
template <typename Fixture>
void TestControlSetupDuringOutData(const typename Fixture::ModeType& mode)
{
  Fixture f(mode);
  f.Up();
  std::vector<uint8_t> data;
  const auto payload = Bytes(100, 3);
  TEST_ASSERT(f.host.SendSetup(Dwc2::Host::Setup(0x41, 0x71, 0, 0, 100)) == Result::OK);
  TEST_ASSERT(f.host.OutPacket(0, payload.data(), 64, 1) == Result::OK);
  const auto fresh = Bytes(30, 5);
  TEST_ASSERT(f.VendorWrite(fresh) == Result::OK);
  TEST_ASSERT(f.probe.data_count == 1U && f.probe.received == 30U);
  TEST_ASSERT(std::memcmp(f.probe.output.data(), fresh.data(), 30) == 0);
  TEST_ASSERT(f.probe.abort_count == 1U);
  TEST_ASSERT(f.VendorRead(9, 64, data) == Result::OK && data.size() == 9U);

  // 打断数据阶段的是另一种请求：应答的是新请求，不是旧请求 / A request of another kind
  // interrupts the data stage: the new request is answered, not the old one
  TEST_ASSERT(f.host.SendSetup(Dwc2::Host::Setup(0x41, 0x71, 0, 0, 100)) == Result::OK);
  TEST_ASSERT(f.host.OutPacket(0, payload.data(), 64, 1) == Result::OK);
  f.probe.response_size = 7;
  TEST_ASSERT(f.host.ControlRead(Dwc2::Host::Setup(0xC1, 0x70, 5, 0, 64), data) ==
              Result::OK);
  TEST_ASSERT(data.size() == 7U && f.probe.last_value == 5U);
  TEST_ASSERT(f.probe.abort_count == 2U);
}

/// 主机不取控制读的状态阶段就发下一个 SETUP：后面的控制读和控制写都正常。
/// The host sends the next SETUP without taking the status stage of a control read: the
/// control read and the control write that follow work.
template <typename Fixture>
void TestControlSetupSkipsStatus(const typename Fixture::ModeType& mode)
{
  Fixture f(mode);
  f.Up();
  std::vector<uint8_t> data;
  std::vector<uint8_t> packet;
  f.probe.response_size = 8;
  TEST_ASSERT(f.host.SendSetup(Dwc2::Host::Setup(0xC1, 0x70, 0, 0, 8)) == Result::OK);
  uint8_t pid = 1;
  TEST_ASSERT(f.host.InPacket(0, packet, &pid) == Result::OK && packet.size() == 8U);
  TEST_ASSERT(f.VendorRead(9, 64, data) == Result::OK && data.size() == 9U);
  TEST_ASSERT(f.VendorWrite(Bytes(3, 5)) == Result::OK && f.probe.received == 3U);
}

/// 主机不取控制写的状态阶段就发下一个 SETUP：数据阶段已经交给设备类，新请求照常处理。
/// The host sends the next SETUP without taking the status stage of a control write: the
/// data stage was already handed to the class and the new request is handled normally.
template <typename Fixture>
void TestControlSetupAfterWriteData(const typename Fixture::ModeType& mode)
{
  Fixture f(mode);
  f.Up();
  const auto payload = Bytes(10, 1);
  TEST_ASSERT(f.host.SendSetup(Dwc2::Host::Setup(0x41, 0x71, 0, 0, 10)) == Result::OK);
  TEST_ASSERT(f.host.OutPacket(0, payload.data(), payload.size(), 1) == Result::OK);
  TEST_ASSERT(f.probe.data_count == 1U);
  std::vector<uint8_t> data;
  TEST_ASSERT(f.VendorRead(9, 64, data) == Result::OK && data.size() == 9U);
}

// ---------------------------------------------------------------------------
// Bulk and interrupt data
// ---------------------------------------------------------------------------

/// 批量 IN：单个短包；恰好一个包且不要求零长包时主机下一次 IN 得到 NAK；多包以短包结束；
/// 要求零长包时在包长整数倍之后多发一个零长包；单独的零长请求。每个请求完成一次，长度
/// 等于提交长度。主机不轮询时请求一直挂着，不会完成。
/// Bulk IN: a single short packet; exactly one packet without a requested ZLP leaves the
/// next IN with a NAK; several packets end with a short one; a requested ZLP follows a
/// length that is a multiple of the packet size; a zero-length request on its own. Each
/// request completes once with the submitted length. While the host does not poll, a
/// request stays armed and does not complete.
template <typename Fixture>
void TestBulkIn(const typename Fixture::ModeType& mode)
{
  Fixture f(mode);
  f.Up();
  Log& log = f.probe.bulk_in_log;
  const size_t packet = f.bulk;
  std::vector<uint8_t> data;

  // 没有提交时 NAK / NAK with nothing submitted
  TEST_ASSERT(f.hw.HostIn(7, 1).resp == Resp::NAK);
  f.Service();

  auto bytes = Bytes(10, 1);
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, bytes) == ErrorCode::OK);
  // 主机不轮询：几帧过去请求仍未完成 / Not polled: frames pass, nothing completes
  f.host.Sof(4);
  TEST_ASSERT(log.items.empty());
  TEST_ASSERT(f.host.ReadTransfer(1, data, 4096, packet) == Result::OK && data == bytes);
  TEST_ASSERT(log.items.size() == 1U && log.items[0].result == ErrorCode::OK &&
              log.items[0].actual == 10U);

  bytes = Bytes(packet, 2);
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, bytes) == ErrorCode::OK);
  std::vector<uint8_t> one;
  TEST_ASSERT(f.host.Read(1, one) == Result::OK && one == bytes);
  TEST_ASSERT(log.items.size() == 2U && log.items[1].actual == packet);
  TEST_ASSERT(f.hw.HostIn(7, 1).resp == Resp::NAK);
  f.Service();

  // 槽里放得下的几个整包加一个短包 / As many full packets as the slot holds, less one,
  // and a short one
  const size_t capacity = f.probe.bulk_in->GetGranted().slot_capacity;
  bytes = Bytes(capacity - packet + 8U, 3);
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, bytes) == ErrorCode::OK);
  TEST_ASSERT(f.host.ReadTransfer(1, data, 4096, packet) == Result::OK && data == bytes);
  TEST_ASSERT(log.items.size() == 3U && log.items[2].actual == bytes.size());

  bytes = Bytes(2U * packet, 4);
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, bytes, true) == ErrorCode::OK);
  TEST_ASSERT(f.host.ReadTransfer(1, data, 4096, packet) == Result::OK && data == bytes);
  TEST_ASSERT(log.items.size() == 4U && log.items[3].actual == bytes.size());
  TEST_ASSERT(f.hw.HostIn(7, 1).resp == Resp::NAK);
  f.Service();

  TEST_ASSERT(f.probe.bulk_in->SubmitZeroLength(false) == ErrorCode::OK);
  TEST_ASSERT(f.host.Read(1, one) == Result::OK && one.empty());
  TEST_ASSERT(log.items.size() == 5U && log.items[4].actual == 0U);

  // 两个槽排队：按提交顺序发出 / Two slots queued leave in submission order
  const auto a = Bytes(packet + 1U, 5);
  const auto b = Bytes(7, 6);
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, a) == ErrorCode::OK);
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, b) == ErrorCode::OK);
  Endpoint::Slot none;
  TEST_ASSERT(f.probe.bulk_in->AcquireSlot(none, false) == ErrorCode::BUSY);
  TEST_ASSERT(f.host.ReadTransfer(1, data, 4096, packet) == Result::OK && data == a);
  TEST_ASSERT(f.host.ReadTransfer(1, data, 4096, packet) == Result::OK && data == b);
  TEST_ASSERT(log.items.size() == 7U && log.Count(ErrorCode::OK) == 7U);
  TEST_ASSERT(f.hw.TxFifoWords(1) == 0U);
}

/// 批量 OUT：短包结束一个请求；恰好收满请求长度；多包加短包；零长包；主机发来的数据比
/// 挂的接收长，多出的包被 NAK 到下一次挂接收，数据不丢不重；没有挂接收时 NAK。
/// Bulk OUT: a short packet ends a request; a request filled exactly; several packets and
/// a short one; a zero-length packet; more data than was armed, where the extra packets
/// are NAKed until the next request is armed and nothing is lost or repeated; NAK with
/// nothing armed.
template <typename Fixture>
void TestBulkOut(const typename Fixture::ModeType& mode)
{
  Fixture f(mode);
  f.Up();
  Log& log = f.probe.bulk_out_log;
  log.rearm = false;
  const size_t packet = f.bulk;
  const size_t capacity = f.probe.bulk_out->GetGranted().slot_capacity;
  TEST_ASSERT(capacity >= 2U * packet);

  // 没有挂接收：NAK，数据不被收下 / Nothing armed: NAK, the data is not taken
  auto bytes = Bytes(10, 1);
  TEST_ASSERT(f.hw.HostOut(7, 1, 0, bytes.data(), bytes.size()) == Resp::NAK);
  f.Service();
  TEST_ASSERT(log.items.empty());

  TEST_ASSERT(f.probe.bulk_out->Receive(capacity, false) == ErrorCode::OK);
  TEST_ASSERT(f.host.Write(1, bytes, packet) == Result::OK);
  TEST_ASSERT(log.items.size() == 1U && log.items[0].result == ErrorCode::OK &&
              log.items[0].data == bytes);

  // 恰好收满 / Filled exactly
  bytes = Bytes(2U * packet, 2);
  TEST_ASSERT(f.probe.bulk_out->Receive(2U * packet, false) == ErrorCode::OK);
  TEST_ASSERT(f.host.Write(1, bytes, packet) == Result::OK);
  TEST_ASSERT(log.items.size() == 2U && log.items[1].data == bytes);

  // 两个槽都已借出：归还一个再挂 / Both slots are lent: return one, then arm
  TEST_ASSERT(f.probe.bulk_out->Receive(packet, false) == ErrorCode::BUSY);
  TEST_ASSERT(f.probe.bulk_out->ReleaseSlot(log.leases[0], false) == ErrorCode::OK);
  TEST_ASSERT(f.probe.bulk_out->ReleaseSlot(log.leases[1], false) == ErrorCode::OK);
  log.leases.clear();

  // 多包加短包 / Several packets and a short one
  bytes = Bytes(packet + 9U, 3);
  TEST_ASSERT(f.probe.bulk_out->Receive(capacity, false) == ErrorCode::OK);
  TEST_ASSERT(f.host.Write(1, bytes, packet) == Result::OK);
  TEST_ASSERT(log.items.size() == 3U && log.items[2].data == bytes);

  // 零长包 / Zero-length packet
  TEST_ASSERT(f.probe.bulk_out->Receive(capacity, false) == ErrorCode::OK);
  TEST_ASSERT(f.host.Write(1, {}, packet) == Result::OK);
  TEST_ASSERT(log.items.size() == 4U && log.items[3].result == ErrorCode::OK &&
              log.items[3].actual == 0U);
  for (const auto& lease : log.leases)
    TEST_ASSERT(f.probe.bulk_out->ReleaseSlot(lease, false) == ErrorCode::OK);
  log.leases.clear();

  // 主机发来的比挂的长：每次只挂一个包 / More than armed: one packet armed at a time
  bytes = Bytes(2U * packet + 20U, 4);
  TEST_ASSERT(f.probe.bulk_out->Receive(packet, false) == ErrorCode::OK);
  TEST_ASSERT(f.hw.HostOut(7, 1, 0, bytes.data(), packet) == Resp::ACK);
  f.Service();
  TEST_ASSERT(log.items.size() == 5U && log.items[4].actual == packet);
  TEST_ASSERT(f.hw.HostOut(7, 1, 1, bytes.data() + packet, packet) == Resp::NAK);
  f.Service();
  TEST_ASSERT(log.items.size() == 5U);
  TEST_ASSERT(f.probe.bulk_out->Receive(packet, false) == ErrorCode::OK);
  TEST_ASSERT(f.hw.HostOut(7, 1, 1, bytes.data() + packet, packet) == Resp::ACK);
  f.Service();
  TEST_ASSERT(log.items.size() == 6U);
  // 归还租约：按配置的接收长度（槽容量）重新挂 / Returning a lease arms the
  // configured receive size (the slot capacity)
  TEST_ASSERT(f.probe.bulk_out->ReturnSlot(log.leases[0], false) == ErrorCode::OK);
  TEST_ASSERT(f.hw.HostOut(7, 1, 0, bytes.data() + 2U * packet, 20) == Resp::ACK);
  f.Service();
  TEST_ASSERT(log.items.size() == 7U && log.items[6].actual == 20U);
  std::vector<uint8_t> joined;
  for (size_t i = 4; i < 7; ++i)
    joined.insert(joined.end(), log.items[i].data.begin(), log.items[i].data.end());
  TEST_ASSERT(joined == bytes);
  TEST_ASSERT(log.Count(ErrorCode::OK) == 7U);
}

/// 中断端点：16 字节的报告，IN 和 OUT 各发几个，包长不是 4 的倍数的也检查。
/// Interrupt endpoints: 16-byte reports in both directions, including lengths that are
/// not a multiple of four.
template <typename Fixture>
void TestInterrupt(const typename Fixture::ModeType& mode)
{
  Fixture f(mode);
  f.Up();
  Log& in = f.probe.int_in_log;
  Log& out = f.probe.int_out_log;
  TEST_ASSERT(f.probe.int_out->Receive(16, false) == ErrorCode::OK);
  std::vector<uint8_t> expected;
  unsigned reports = 0;
  for (size_t size : {size_t{16}, size_t{1}, size_t{13}, size_t{16}, size_t{2}})
  {
    const auto report = Bytes(size, static_cast<uint8_t>(0x40U + size));
    TEST_ASSERT(SubmitBytes(f.probe.int_in, report) == ErrorCode::OK);
    std::vector<uint8_t> packet;
    TEST_ASSERT(f.host.Read(2, packet) == Result::OK && packet == report);
    TEST_ASSERT(in.items.size() == ++reports && in.items.back().actual == size);
    TEST_ASSERT(f.host.Write(2, report, 16) == Result::OK);
    TEST_ASSERT(out.items.size() == reports && out.items.back().data == report);
  }
  // 主机不轮询的中断 IN 不完成 / An interrupt IN that is not polled stays armed
  TEST_ASSERT(SubmitBytes(f.probe.int_in, Bytes(4, 1)) == ErrorCode::OK);
  f.host.Sof(3);
  TEST_ASSERT(in.items.size() == reports);
}

/// 完成回调里重新提交：OUT 回调归还租约（重新挂接收），IN 回调提交下一个包。50 个来回，
/// 每个请求恰好完成一次，数据按顺序。
/// Re-arming inside the completion callback: the OUT callback returns its lease (which
/// arms the next receive) and the IN callback submits the next packet. Fifty round
/// trips, every request completes exactly once and the data stays in order.
template <typename Fixture>
void TestCallbackRearm(const typename Fixture::ModeType& mode)
{
  Fixture f(mode);
  f.Up();
  Log& out = f.probe.bulk_out_log;
  Log& in = f.probe.bulk_in_log;
  const size_t packet = f.bulk;
  const size_t capacity = f.probe.bulk_out->GetGranted().slot_capacity;
  TEST_ASSERT(f.probe.bulk_out->Receive(capacity, false) == ErrorCode::OK);
  in.chain = 49;
  in.chain_length = 12;
  in.chain_seed = 0;
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, std::vector<uint8_t>(12, 0)) == ErrorCode::OK);
  for (unsigned i = 0; i < 50; ++i)
  {
    // 交替：收满整个请求（多包）和一个短包 / Alternately a request filled exactly
    // (several packets) and one short packet
    const auto bytes = Bytes(i % 2U == 0U ? capacity : 5U, static_cast<uint8_t>(i));
    TEST_ASSERT(f.host.Write(1, bytes, packet) == Result::OK);
    TEST_ASSERT(out.items.size() == i + 1U && out.items[i].data == bytes);
    std::vector<uint8_t> got;
    TEST_ASSERT(f.host.Read(1, got) == Result::OK);
    TEST_ASSERT(got == std::vector<uint8_t>(12, static_cast<uint8_t>(i)));
    TEST_ASSERT(in.items.size() == i + 1U);
  }
  TEST_ASSERT(in.Count(ErrorCode::OK) == 50U && out.Count(ErrorCode::OK) == 50U);
  TEST_ASSERT(f.hw.HostIn(7, 1).resp == Resp::NAK);
  f.Service();
}

// ---------------------------------------------------------------------------
// Stall, stop, reconfiguration, reset
// ---------------------------------------------------------------------------

/// 端点 STALL：SET_FEATURE(ENDPOINT_HALT) 后令牌得到 STALL，GET_STATUS 读到 halted，在途
/// 请求以 ABORTED 结束一次；CLEAR_FEATURE 后恢复，数据翻转回到 DATA0，设备类收到
/// ENDPOINT_RESUMED。
/// Endpoint halt: after SET_FEATURE(ENDPOINT_HALT) tokens get STALL, GET_STATUS reads
/// halted and an in-flight request ends once with ABORTED; CLEAR_FEATURE restores the
/// endpoint with the data toggle back at DATA0 and the class gets ENDPOINT_RESUMED.
template <typename Fixture>
void TestEndpointHalt(const typename Fixture::ModeType& mode)
{
  Fixture f(mode);
  f.Up();
  const size_t packet = f.bulk;
  std::vector<uint8_t> data;
  Log& in = f.probe.bulk_in_log;
  Log& out = f.probe.bulk_out_log;

  // 先各传一个包，让翻转位停在 DATA1 / One packet each way leaves the toggles at DATA1
  TEST_ASSERT(f.probe.bulk_out->Receive(packet, false) == ErrorCode::OK);
  TEST_ASSERT(f.host.Write(1, Bytes(packet, 1), packet) == Result::OK);
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, Bytes(packet, 2)) == ErrorCode::OK);
  TEST_ASSERT(f.host.Read(1, data) == Result::OK);

  // IN：有在途请求时置 STALL / IN: halted with a request in flight
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, Bytes(20, 3)) == ErrorCode::OK);
  TEST_ASSERT(f.host.SetHalt(0x81) == Result::OK);
  f.host.Sof(2);
  TEST_ASSERT(in.items.size() == 2U && in.items[1].result == ErrorCode::ABORTED);
  TEST_ASSERT(f.hw.HostIn(7, 1).resp == Resp::STALL);
  TEST_ASSERT(f.host.ControlRead(Dwc2::Host::Setup(0x82, 0, 0, 0x81, 2), data) ==
              Result::OK);
  TEST_ASSERT(data.size() == 2U && data[0] == 1U);
  Endpoint::Slot slot;
  TEST_ASSERT(f.probe.bulk_in->IsStalled());
  TEST_ASSERT(f.host.ClearHalt(0x81) == Result::OK);
  TEST_ASSERT(f.probe.resumed.size() == 1U && f.probe.resumed[0] == 0x81U);
  TEST_ASSERT(f.host.ControlRead(Dwc2::Host::Setup(0x82, 0, 0, 0x81, 2), data) ==
              Result::OK);
  TEST_ASSERT(data[0] == 0U);
  // 恢复后第一个包是 DATA0，且是新数据 / The first packet is DATA0 and new data
  const auto fresh = Bytes(30, 4);
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, fresh) == ErrorCode::OK);
  TEST_ASSERT(f.host.Read(1, data) == Result::OK && data == fresh);
  TEST_ASSERT(in.items.size() == 3U && in.items[2].result == ErrorCode::OK);

  // OUT：有在途接收时置 STALL / OUT: halted with a receive in flight
  TEST_ASSERT(f.host.SetHalt(0x01) == Result::OK);
  f.host.Sof(2);
  TEST_ASSERT(out.Count(ErrorCode::ABORTED) == 1U);
  const auto bytes = Bytes(10, 5);
  TEST_ASSERT(f.hw.HostOut(7, 1, 0, bytes.data(), bytes.size()) == Resp::STALL);
  f.Service();
  TEST_ASSERT(f.host.ClearHalt(0x01) == Result::OK);
  TEST_ASSERT(f.probe.resumed.size() == 2U && f.probe.resumed[1] == 0x01U);
  TEST_ASSERT(f.probe.bulk_out->Receive(packet, false) == ErrorCode::OK);
  TEST_ASSERT(f.host.Write(1, bytes, packet) == Result::OK);
  TEST_ASSERT(out.items.back().result == ErrorCode::OK && out.items.back().data == bytes);
  // 另一个端点不受影响 / Another endpoint is not affected
  TEST_ASSERT(SubmitBytes(f.probe.int_in, Bytes(8, 6)) == ErrorCode::OK);
  TEST_ASSERT(f.host.Read(2, data) == Result::OK && data.size() == 8U);
  (void)slot;
}

/// 停止在途的 IN 请求：Flush 之后请求以 ABORTED 结束一次，发送 FIFO 被清空，主机得到
/// NAK；之后提交的新数据原样发出，没有旧包。多包传输发到一半时停止，报告的长度不超过
/// 主机已取走的。
/// Stopping an IN request in flight: after Flush it ends once with ABORTED, the transmit
/// FIFO is empty and the host gets NAK; data submitted afterwards goes out as it is,
/// with no packet of the old request. Stopped halfway through a multi-packet transfer,
/// the reported length does not exceed what the host took.
template <typename Fixture>
void TestStopIn(const typename Fixture::ModeType& mode)
{
  Fixture f(mode);
  f.Up();
  const size_t packet = f.bulk;
  Log& log = f.probe.bulk_in_log;
  std::vector<uint8_t> data;

  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, Bytes(packet, 1)) == ErrorCode::OK);
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, Bytes(9, 2)) == ErrorCode::OK);  // 排队的
  f.probe.bulk_in->Flush(false);
  f.host.Sof(3);
  TEST_ASSERT(!f.probe.bulk_in->IsStopping());
  TEST_ASSERT(log.items.size() == 2U && log.Count(ErrorCode::ABORTED) == 2U);
  // 主机一个包也没取走 / The host took no packet
  if (Fixture::EXACT_IN_PROGRESS)
    TEST_ASSERT(log.items[0].actual == 0U && log.items[1].actual == 0U);
  TEST_ASSERT(f.hw.TxFifoWords(1) == 0U);
  TEST_ASSERT(f.hw.HostIn(7, 1).resp == Resp::NAK);
  f.Service();
  // 帧中断一直开着 / The frame interrupt stays on
  TEST_ASSERT(!f.hw.SofMasked());

  auto fresh = Bytes(packet + 3U, 3);
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, fresh) == ErrorCode::OK);
  TEST_ASSERT(f.host.ReadTransfer(1, data, 4096, packet) == Result::OK && data == fresh);
  TEST_ASSERT(log.items.size() == 3U && log.items[2].result == ErrorCode::OK);

  // 几个包发走一个之后停止 / Stopped after one of several packets left
  const size_t capacity = f.probe.bulk_in->GetGranted().slot_capacity;
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, Bytes(capacity, 4)) == ErrorCode::OK);
  std::vector<uint8_t> one;
  TEST_ASSERT(f.host.Read(1, one) == Result::OK && one.size() == packet);
  f.probe.bulk_in->Flush(false);
  f.host.Sof(3);
  TEST_ASSERT(log.items.size() == 4U && log.items[3].result == ErrorCode::ABORTED);
  if (Fixture::EXACT_IN_PROGRESS)
    TEST_ASSERT(log.items[3].actual <= packet && log.items[3].actual % packet == 0U);
  TEST_ASSERT(f.hw.TxFifoWords(1) == 0U);
  // 主机的数据翻转已经前进了一个包；下一次传输从设备当前的翻转继续 / The host's toggle
  // advanced by one packet and the next transfer continues from the device's toggle
  fresh = Bytes(11, 5);
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, fresh) == ErrorCode::OK);
  TEST_ASSERT(f.host.Read(1, one) == Result::OK && one == fresh);

  // 没有在途请求时 Flush 不产生完成 / Flush with nothing in flight completes nothing
  f.probe.bulk_in->Flush(false);
  f.host.Sof(2);
  TEST_ASSERT(log.items.size() == 5U);
}

/// 停止时主机恰好取走最后一个包：SNAK 写进去之前传输已经完成，请求按主机实际取走的
/// 结果结束一次（OK，全长），不会报成 ABORTED 让上层重发。
/// The host takes the last packet just as the stop begins: the transfer is complete
/// before SNAK lands, and the request ends once with what the host really took (OK, full
/// length) instead of ABORTED, which would make the upper layer send it again.
template <typename Fixture>
void TestStopInRace(const typename Fixture::ModeType& mode)
{
  Fixture f(mode);
  f.Up();
  Log& log = f.probe.bulk_in_log;
  const auto bytes = Bytes(20, 7);
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, bytes) == ErrorCode::OK);
  Dwc2::InResult taken;
  f.hw.before_write = [&](uint32_t, uint32_t) { taken = f.hw.HostIn(7, 1); };
  f.probe.bulk_in->Flush(false);
  f.host.Sof(3);
  TEST_ASSERT(taken.resp == Resp::DATA && taken.data == bytes);
  TEST_ASSERT(log.items.size() == 1U && log.items[0].result == ErrorCode::OK &&
              log.items[0].actual == 20U);
  TEST_ASSERT(!f.probe.bulk_in->IsStopping());
  // 主机的翻转跟上这一个包 / The host's toggle follows that packet
  std::vector<uint8_t> data;
  uint8_t pid = 1;
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, Bytes(5, 8)) == ErrorCode::OK);
  TEST_ASSERT(f.host.InPacket(1, data, &pid) == Result::OK && data.size() == 5U);
}

/// 停止在途的 OUT 请求：Flush 之后请求以 ABORTED 结束一次，端点 NAK；停止结束后全局
/// OUT NAK 被撤销，别的 OUT 端点照常收数据；重新挂接收后数据完整。已经收到一部分数据时
/// 停止也只完成一次。
/// Stopping an OUT request in flight: after Flush it ends once with ABORTED and the
/// endpoint NAKs; the global OUT NAK is released when the stop is done, so other OUT
/// endpoints receive as before; a new receive gets complete data. A stop after part of
/// the data arrived also completes once.
template <typename Fixture>
void TestStopOut(const typename Fixture::ModeType& mode)
{
  Fixture f(mode);
  f.Up();
  const size_t packet = f.bulk;
  Log& log = f.probe.bulk_out_log;
  log.rearm = false;
  TEST_ASSERT(f.probe.int_out->Receive(16, false) == ErrorCode::OK);

  TEST_ASSERT(f.probe.bulk_out->Receive(2U * packet, false) == ErrorCode::OK);
  f.probe.bulk_out->Flush(false);
  f.host.Sof(3);
  TEST_ASSERT(!f.probe.bulk_out->IsStopping());
  TEST_ASSERT(log.items.size() == 1U && log.items[0].result == ErrorCode::ABORTED);
  auto bytes = Bytes(10, 1);
  TEST_ASSERT(f.hw.HostOut(7, 1, 0, bytes.data(), bytes.size()) == Resp::NAK);
  f.Service();
  TEST_ASSERT((f.hw.Peek(Dwc2::DCTL) & Dwc2::DCTL_GONSTS) == 0U);
  TEST_ASSERT(!f.hw.SofMasked());
  // 别的 OUT 端点照常 / Another OUT endpoint works as before
  TEST_ASSERT(f.host.Write(2, Bytes(16, 2), 16) == Result::OK);
  TEST_ASSERT(f.probe.int_out_log.items.size() == 1U);

  TEST_ASSERT(f.probe.bulk_out->Receive(packet, false) == ErrorCode::OK);
  TEST_ASSERT(f.host.Write(1, bytes, packet) == Result::OK);
  TEST_ASSERT(log.items.size() == 2U && log.items[1].data == bytes);
  TEST_ASSERT(f.probe.bulk_out->ReleaseSlot(log.leases[0], false) == ErrorCode::OK);

  // 收到一个整包之后停止 / Stopped after one full packet arrived
  bytes = Bytes(packet, 3);
  TEST_ASSERT(f.probe.bulk_out->Receive(2U * packet, false) == ErrorCode::OK);
  TEST_ASSERT(f.host.Write(1, bytes, packet) == Result::OK);
  TEST_ASSERT(log.items.size() == 2U);
  f.probe.bulk_out->Flush(false);
  f.host.Sof(3);
  TEST_ASSERT(log.items.size() == 3U && log.items[2].result == ErrorCode::ABORTED);
  TEST_ASSERT((f.hw.Peek(Dwc2::DCTL) & Dwc2::DCTL_GONSTS) == 0U);
  TEST_ASSERT(f.hw.RxFifoWords() == 0U);

  // 两个 OUT 端点同时停止 / Two OUT endpoints stopped together
  TEST_ASSERT(f.probe.bulk_out->Receive(packet, false) == ErrorCode::OK);
  f.probe.bulk_out->Flush(false);
  f.probe.int_out->Flush(false);
  f.host.Sof(3);
  TEST_ASSERT(log.items.size() == 4U && f.probe.int_out_log.items.size() == 2U);
  TEST_ASSERT((f.hw.Peek(Dwc2::DCTL) & Dwc2::DCTL_GONSTS) == 0U);
  TEST_ASSERT(f.probe.int_out->Receive(16, false) == ErrorCode::OK);
  TEST_ASSERT(f.host.Write(2, Bytes(16, 4), 16) == Result::OK);
  TEST_ASSERT(f.probe.int_out_log.items.size() == 3U);
}

/// 重新配置：配置 1 → 0 → 1。取消配置时在途和排队的请求各以 ABORTED 结束一次，端点不再
/// 应答；重新配置后端点从 DATA0 开始，旧的数据不会出现。
/// Reconfiguration 1 → 0 → 1. Deconfiguring ends in-flight and queued requests once each
/// with ABORTED and the endpoints stop answering; after configuring again the endpoints
/// start at DATA0 and no old data shows up.
template <typename Fixture>
void TestReconfigure(const typename Fixture::ModeType& mode)
{
  Fixture f(mode);
  f.Up();
  const size_t packet = f.bulk;
  std::vector<uint8_t> data;
  // 翻转位留在 DATA1 / Leave the toggles at DATA1
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, Bytes(5, 1)) == ErrorCode::OK);
  TEST_ASSERT(f.host.Read(1, data) == Result::OK);
  TEST_ASSERT(f.probe.bulk_out->Receive(packet, false) == ErrorCode::OK);
  TEST_ASSERT(f.host.Write(1, Bytes(5, 1), packet) == Result::OK);

  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, Bytes(packet, 2)) == ErrorCode::OK);
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, Bytes(6, 3)) == ErrorCode::OK);
  TEST_ASSERT(SubmitBytes(f.probe.int_in, Bytes(6, 4)) == ErrorCode::OK);
  TEST_ASSERT(f.probe.int_out->Receive(16, false) == ErrorCode::OK);
  const size_t in_done = f.probe.bulk_in_log.items.size();
  const size_t out_done = f.probe.bulk_out_log.items.size();

  // 状态阶段要等端点停下来：先让几帧过去 / The status stage waits for the stops
  TEST_ASSERT(f.host.SendSetup(Dwc2::Host::Setup(0x00, 9, 0, 0, 0)) == Result::OK);
  f.host.Sof(4);
  TEST_ASSERT(f.host.StatusIn() == Result::OK);
  TEST_ASSERT(!f.device.IsConfigured() && f.probe.unbinds == 1U);
  TEST_ASSERT(f.probe.bulk_in_log.items.size() == in_done + 2U &&
              f.probe.bulk_in_log.Count(ErrorCode::ABORTED) == 2U);
  TEST_ASSERT(f.probe.bulk_out_log.items.size() == out_done + 1U &&
              f.probe.bulk_out_log.Count(ErrorCode::ABORTED) == 1U);
  TEST_ASSERT(f.probe.int_in_log.Count(ErrorCode::ABORTED) == 1U);
  TEST_ASSERT(f.probe.int_out_log.Count(ErrorCode::ABORTED) == 1U);
  // 取消配置后的端点不再应答 / A deconfigured endpoint no longer answers
  if (Fixture::CLOSE_DEACTIVATES)
  {
    TEST_ASSERT(f.hw.HostIn(7, 1).resp == Resp::TIMEOUT);
    TEST_ASSERT(f.hw.HostOut(7, 1, 0, nullptr, 0) == Resp::TIMEOUT);
    TEST_ASSERT(f.hw.HostIn(7, 2).resp == Resp::TIMEOUT);
  }
  else
  {
    TEST_ASSERT(f.hw.HostIn(7, 1).resp != Resp::DATA);
    TEST_ASSERT(f.hw.HostOut(7, 1, 0, nullptr, 0) != Resp::ACK);
  }
  TEST_ASSERT(!f.hw.SofMasked());
  TEST_ASSERT(f.host.ControlRead(Dwc2::Host::Setup(0x80, 8, 0, 0, 1), data) ==
              Result::OK);
  TEST_ASSERT(data[0] == 0U);

  TEST_ASSERT(f.host.SetConfiguration(1) == Result::OK);
  TEST_ASSERT(f.device.IsConfigured() && f.probe.binds == 2U);
  const auto fresh = Bytes(packet + 2U, 9);
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, fresh) == ErrorCode::OK);
  TEST_ASSERT(f.host.ReadTransfer(1, data, 4096, packet) == Result::OK && data == fresh);
  TEST_ASSERT(f.probe.bulk_out->Receive(packet, false) == ErrorCode::OK);
  const auto small = Bytes(10, 8);
  TEST_ASSERT(f.host.Write(1, small, packet) == Result::OK);
  TEST_ASSERT(f.probe.bulk_out_log.items.back().result == ErrorCode::OK &&
              f.probe.bulk_out_log.items.back().data == small);
}

/// 传输进行中总线复位：在途的 IN、OUT 和控制传输各结束一次，设备回到地址 0；重新枚举后
/// 第一个 IN 包是新数据，FIFO 里没有旧包。控制读发到一半时复位也检查。
/// Bus reset in the middle of transfers: the IN, OUT and control transfers in flight end
/// once each and the device is back at address 0; after enumerating again the first IN
/// packet is new data and no old packet is left in a FIFO. A reset halfway through a
/// control read is checked too.
template <typename Fixture>
void TestBusResetMidTransfer(const typename Fixture::ModeType& mode)
{
  Fixture f(mode);
  f.Up();
  const size_t packet = f.bulk;
  std::vector<uint8_t> data;
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, Bytes(2U * packet, 1)) == ErrorCode::OK);
  std::vector<uint8_t> one;
  TEST_ASSERT(f.host.Read(1, one) == Result::OK);
  TEST_ASSERT(f.probe.bulk_out->Receive(2U * packet, false) == ErrorCode::OK);
  TEST_ASSERT(f.host.Write(1, Bytes(packet, 2), packet) == Result::OK);
  TEST_ASSERT(SubmitBytes(f.probe.int_in, Bytes(8, 3)) == ErrorCode::OK);
  // 控制读发出第一个包 / A control read with its first packet out
  f.probe.response_size = 200;
  TEST_ASSERT(f.host.SendSetup(Dwc2::Host::Setup(0xC1, 0x70, 0, 0, 255)) == Result::OK);
  uint8_t pid = 1;
  TEST_ASSERT(f.host.InPacket(0, one, &pid) == Result::OK && one.size() == 64U);

  f.hw.HostResetBegin();
  f.Service();
  f.hw.HostResetEnd();
  f.Service();
  // 复位里的停止靠帧推进 / Stops started by the reset advance on frames
  f.host.address = 0;
  TEST_ASSERT(f.host.SofUntil([&] { return f.probe.resets == 2U; }));
  TEST_ASSERT(f.hw.Address() == 0U);
  TEST_ASSERT(!f.device.IsConfigured() && f.probe.resets == 2U);
  TEST_ASSERT(f.probe.bulk_in_log.items.size() == 1U &&
              f.probe.bulk_in_log.items[0].result == ErrorCode::ABORTED);
  TEST_ASSERT(f.probe.bulk_out_log.items.size() == 1U &&
              f.probe.bulk_out_log.items[0].result == ErrorCode::ABORTED);
  TEST_ASSERT(f.probe.int_in_log.items.size() == 1U &&
              f.probe.int_in_log.items[0].result == ErrorCode::ABORTED);
  TEST_ASSERT(f.probe.abort_count == 1U);
  for (uint8_t i = 0; i < 4; ++i) TEST_ASSERT(f.hw.TxFifoWords(i) == 0U);

  f.host.ResetToggles();
  TEST_ASSERT(f.host.GetDescriptor(1, 0, 18, data) == Result::OK && data.size() == 18U);
  TEST_ASSERT(f.host.SetAddress(9) == Result::OK);
  TEST_ASSERT(f.host.SetConfiguration(1) == Result::OK);
  TEST_ASSERT(f.probe.binds == 2U);
  const auto fresh = Bytes(10, 9);
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, fresh) == ErrorCode::OK);
  TEST_ASSERT(f.host.Read(1, data) == Result::OK && data == fresh);
  TEST_ASSERT(f.probe.bulk_out->Receive(packet, false) == ErrorCode::OK);
  TEST_ASSERT(f.host.Write(1, fresh, packet) == Result::OK);
  TEST_ASSERT(f.probe.bulk_out_log.items.back().data == fresh);
  TEST_ASSERT(f.VendorRead(70, 255, data) == Result::OK && data.size() == 70U);

  // 复位和复位结束在同一次中断里被看到 / Reset and its end seen by one interrupt
  f.hw.HostResetBegin();
  f.hw.HostResetEnd();
  f.Service();
  f.host.address = 0;
  f.host.ResetToggles();
  f.host.Sof(4);
  TEST_ASSERT(f.probe.resets == 3U);
  TEST_ASSERT(f.host.GetDescriptor(1, 0, 18, data) == Result::OK && data.size() == 18U);
}

/// 总线复位之后端点不再处于 STALL：两个方向的 EP1 都被 SET_FEATURE(ENDPOINT_HALT) 置了
/// STALL，复位后所有 OUT 端点处于 NAK（手册"USB 复位时的端点初始化"第 1 步；控制器模型在
/// 复位后第一个令牌到来时检查全部 5 步）。重新枚举并配置后，两个方向的 EP1 都正常传数据，
/// 不应答 STALL，类的端点也不再是 halted。手册没有要求复位清 STALL，这里检查的是主机在新
/// 会话里看到的结果。
/// After a bus reset the endpoints are no longer stalled: EP1 is halted in both
/// directions with SET_FEATURE(ENDPOINT_HALT); after the reset every OUT endpoint is
/// NAKed (step 1 of "Endpoint initialization on USB reset" in the manual; the controller
/// model checks all five steps when the first token after the reset arrives). After
/// enumerating and configuring again, EP1 moves data in both directions, answers no STALL
/// and the class endpoints are not halted. The manual does not ask for the reset to clear
/// STALL; what is checked is what the host sees in the new session.
template <typename Fixture>
void TestBusResetClearsHalt(const typename Fixture::ModeType& mode)
{
  Fixture f(mode);
  f.Up();
  const size_t packet = f.bulk;
  std::vector<uint8_t> data;
  TEST_ASSERT(f.host.SetHalt(0x81) == Result::OK);
  TEST_ASSERT(f.host.SetHalt(0x01) == Result::OK);
  const auto bytes = Bytes(10, 5);
  TEST_ASSERT(f.hw.HostIn(7, 1).resp == Resp::STALL);
  TEST_ASSERT(f.hw.HostOut(7, 1, 0, bytes.data(), bytes.size()) == Resp::STALL);
  TEST_ASSERT((f.hw.InCtl(1) & Dwc2::CTL_STALL) != 0U);
  TEST_ASSERT((f.hw.OutCtl(1) & Dwc2::CTL_STALL) != 0U);

  f.hw.HostResetBegin();
  f.Service();
  f.hw.HostResetEnd();
  f.Service();
  f.host.address = 0;
  f.host.ResetToggles();
  f.host.Sof(4);
  // 复位之后 OUT 端点都在 NAK / After the reset every OUT endpoint is NAKed
  for (uint8_t i = 1; i < 4; ++i) TEST_ASSERT((f.hw.OutCtl(i) & Dwc2::CTL_NAKSTS) != 0U);

  TEST_ASSERT(f.host.GetDescriptor(1, 0, 18, data) == Result::OK && data.size() == 18U);
  TEST_ASSERT(f.host.SetAddress(9) == Result::OK);
  TEST_ASSERT(f.host.SetConfiguration(1) == Result::OK);
  TEST_ASSERT(f.probe.binds == 2U);
  TEST_ASSERT(!f.probe.bulk_in->IsStalled() && !f.probe.bulk_out->IsStalled());
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, bytes) == ErrorCode::OK);
  TEST_ASSERT(f.host.Read(1, data) == Result::OK && data == bytes);
  TEST_ASSERT(f.probe.bulk_out->Receive(packet, false) == ErrorCode::OK);
  TEST_ASSERT(f.host.Write(1, bytes, packet) == Result::OK);
  TEST_ASSERT(f.probe.bulk_out_log.items.back().result == ErrorCode::OK &&
              f.probe.bulk_out_log.items.back().data == bytes);
}

/// 挂起与恢复：挂起期间提交的请求不启动硬件，恢复后发出；挂起中复位照常枚举。帧中断挂起
/// 时关闭、恢复时打开，挂起中复位后也是开的。
/// Suspend and resume: a request submitted while suspended does not start the hardware
/// and goes out after the resume; a reset during suspend enumerates as usual. The frame
/// interrupt is off while suspended, on after the resume and on after a reset during
/// suspend.
template <typename Fixture>
void TestSuspendResume(const typename Fixture::ModeType& mode)
{
  Fixture f(mode);
  f.Up();
  TEST_ASSERT(!f.hw.SofMasked());
  std::vector<uint8_t> data;
  f.hw.HostSuspend();
  f.Service();
  TEST_ASSERT(f.hw.SofMasked());
  const auto bytes = Bytes(12, 1);
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, bytes) == ErrorCode::OK);
  TEST_ASSERT((f.hw.InCtl(1) & Dwc2::CTL_EPENA) == 0U);
  f.hw.HostResume();
  f.Service();
  TEST_ASSERT(!f.hw.SofMasked());
  TEST_ASSERT((f.hw.InCtl(1) & Dwc2::CTL_EPENA) != 0U);
  TEST_ASSERT(f.host.Read(1, data) == Result::OK && data == bytes);
  TEST_ASSERT(f.device.IsConfigured());

  f.hw.HostSuspend();
  f.Service();
  TEST_ASSERT(f.hw.SofMasked());
  TEST_ASSERT(f.host.Enumerate(3, 1) == Result::OK);
  TEST_ASSERT(!f.hw.SofMasked());
  TEST_ASSERT(f.probe.binds == 2U);
  TEST_ASSERT(SubmitBytes(f.probe.bulk_in, bytes) == ErrorCode::OK);
  TEST_ASSERT(f.host.Read(1, data) == Result::OK && data == bytes);
}

// ---------------------------------------------------------------------------
// Frame interrupt and isochronous data
// ---------------------------------------------------------------------------

/// 帧节拍：帧中断枚举后就是开的。设备类需要帧节拍时，每个 SOF 恰好送一次 OnFrameTick，
/// 间隔为 1，高速时每个微帧一次；没有设备类需要时 SOF 照常中断，但类收不到节拍；需求
/// 变化立刻生效。
/// Frame ticks: the frame interrupt is on after enumeration. While a class needs frame
/// ticks every SOF delivers exactly one OnFrameTick with an interval of one, once per
/// microframe at high speed; with no class wanting it a SOF still interrupts but the
/// class gets no tick; a change of need takes effect at once.
template <typename Fixture>
void TestFrameTicks(const typename Fixture::ModeType& mode)
{
  {
    Fixture f(mode, true);
    f.Up();
    TEST_ASSERT(!f.hw.SofMasked());
    f.probe.ticks.clear();
    f.host.Sof(40);
    TEST_ASSERT(f.probe.ticks.size() == 40U);
    for (size_t i = 1; i < f.probe.ticks.size(); ++i)
    {
      TEST_ASSERT(f.probe.ticks[i].elapsed_intervals == 1U);
      if (Fixture::HighSpeed(mode))
        TEST_ASSERT(f.probe.ticks[i].microframe == ((f.hw.Frame() - 39U + i) & 7U));
      else
        TEST_ASSERT(f.probe.ticks[i].microframe == 0xFFU);
    }
    // 两个 SOF 在同一次中断里：只看到最后一个，间隔为 2 / Two SOFs seen by one
    // interrupt: only the last one is seen, with an interval of two
    f.hw.HostSof();
    f.hw.HostSof();
    f.Service();
    TEST_ASSERT(f.probe.ticks.size() == 41U);
    TEST_ASSERT(f.probe.ticks.back().elapsed_intervals == 2U);
  }
  {
    Fixture f(mode, false);
    f.Up();
    TEST_ASSERT(!f.hw.SofMasked());
    const unsigned before = f.interrupts;
    f.host.Sof(10);
    TEST_ASSERT(f.interrupts > before && f.probe.ticks.empty());
    f.probe.needs_frame_tick = true;
    f.host.Sof(5);
    TEST_ASSERT(f.probe.ticks.size() == 5U);
    f.probe.needs_frame_tick = false;
    f.host.Sof(5);
    TEST_ASSERT(f.probe.ticks.size() == 5U);
  }
}

/// 核心对同一个 SOF 令牌把帧中断置位两次（STM32H723 上测得，两次的 DSTS.FNSOF 相同）：
/// 每帧（高速时每微帧）仍恰好送一次 OnFrameTick，间隔为 1；高速的各个微帧帧号不同，
/// 一个都不能丢。总线复位之后，同一帧号的第一个 SOF 同样要送出。
/// The core flags the same SOF token twice (measured on an STM32H723, both times with the
/// same DSTS.FNSOF): every frame (microframe at high speed) still delivers exactly one
/// OnFrameTick with an interval of one; the microframes of high speed have different
/// numbers and none may be dropped. After a bus reset the first SOF with the same
/// number is delivered as well.
template <typename Fixture>
void TestFrameTicksWhenSofIsRaisedTwice(const typename Fixture::ModeType& mode)
{
  Fixture f(mode, true);
  f.Up();
  f.hw.sof_twice = true;
  f.probe.ticks.clear();
  const unsigned before = f.interrupts;
  f.host.Sof(40);
  TEST_ASSERT(f.probe.ticks.size() == 40U);
  TEST_ASSERT(f.interrupts >=
              before + 80U);  // 每个令牌两次中断 / Two interrupts per token
  for (size_t i = 0; i < f.probe.ticks.size(); ++i)
  {
    if (i != 0U) TEST_ASSERT(f.probe.ticks[i].elapsed_intervals == 1U);
    if (Fixture::HighSpeed(mode))
      TEST_ASSERT(f.probe.ticks[i].microframe == ((f.hw.Frame() - 39U + i) & 7U));
    else
      TEST_ASSERT(f.probe.ticks[i].microframe == 0xFFU);
  }
  // 复位之后帧号从刚处理过的值重新开始 / After a reset the number restarts at the one
  // just handled
  const uint16_t handled = f.hw.Frame();
  f.host.frame_per_retry = false;  // 枚举期间没有 SOF / No SOF during the enumeration
  TEST_ASSERT(f.host.Enumerate(7, 1) == Result::OK);
  f.probe.ticks.clear();
  f.hw.HostSetNextFrame(handled);
  f.host.Sof();
  TEST_ASSERT(f.probe.ticks.size() == 1U);
}

/// 等时 IN 的奇偶帧：请求在第 N 帧提交，安排在第 N+1 帧发送；第 N 帧里的 IN 令牌得到
/// 零长包，第 N+1 帧发出数据并完成一次。偶数帧和奇数帧提交各检查一次。
/// Even and odd frames of isochronous IN: a request submitted in frame N is scheduled
/// for frame N+1; an IN token in frame N gets a zero-length packet, frame N+1 sends the
/// data and completes once. Checked for a submission in an even and in an odd frame.
/// 依赖模型的假设：未到安排的那一帧时等时 IN 端点以零长包应答（手册没有写，见
/// fake_dwc2.hpp 的 ASSUMPTION）。
/// Depends on a model ASSUMPTION: before its scheduled frame the isochronous IN endpoint
/// answers a zero-length packet (not stated in the manual, see fake_dwc2.hpp).
template <typename Fixture>
void TestIsochronousFrames(const typename Fixture::ModeType& mode)
{
  Fixture f(mode, true);
  f.Up();
  Log& log = f.probe.iso_in_log;
  for (unsigned round = 0; round < 4; ++round)
  {
    const auto bytes = Bytes(40U + round, static_cast<uint8_t>(round));
    const uint16_t frame = f.hw.Frame();
    TEST_ASSERT(SubmitBytes(f.probe.iso_in, bytes) == ErrorCode::OK);
    Dwc2::InResult in = f.hw.HostIn(7, 3);
    f.Service();
    TEST_ASSERT(in.resp == Resp::DATA && in.data.empty());
    TEST_ASSERT(log.items.size() == round);
    f.host.Sof();
    TEST_ASSERT(f.hw.Frame() ==
                ((frame + 1U) & (Fixture::HighSpeed(mode) ? 0x3FFFU : 0x7FFU)));
    in = f.hw.HostIn(7, 3);
    f.Service();
    TEST_ASSERT(in.resp == Resp::DATA && in.data == bytes);
    TEST_ASSERT(log.items.size() == round + 1U &&
                log.items[round].result == ErrorCode::OK &&
                log.items[round].actual == bytes.size());
    if (round % 2U == 1U) f.host.Sof();  // 换一个奇偶 / Switch the parity
  }
}

/// 等时 IN 错过了它的帧：主机在安排的那一帧没有取数据，帧结束时控制器报告未完成的等时
/// IN 传输。请求以 FAILED 结束一次，发送 FIFO 被清空，之后提交的包正常发出。停止与错过帧
/// 同时发生时请求以 ABORTED 结束一次。
/// An isochronous IN request misses its frame: the host does not take the data in the
/// scheduled frame and the controller reports an incomplete isochronous IN transfer at
/// the end of that frame. The request ends once with FAILED, the transmit FIFO is
/// emptied and a packet submitted afterwards is sent normally. A stop that coincides
/// with the missed frame ends the request once with ABORTED.
/// 依赖模型的假设：中断挂起期间 DSTS 仍显示刚结束的那一帧（手册没有写 FNSOF 的更新时间，
/// 见 fake_dwc2.hpp 的 ASSUMPTION）。
/// Depends on a model ASSUMPTION: while the interrupt is pending DSTS still shows the
/// frame that just ended (the manual does not say when FNSOF updates, see fake_dwc2.hpp).
template <typename Fixture>
void TestIsochronousMissedFrame(const typename Fixture::ModeType& mode)
{
  Fixture f(mode, true);
  f.Up();
  Log& log = f.probe.iso_in_log;
  TEST_ASSERT(SubmitBytes(f.probe.iso_in, Bytes(48, 1)) == ErrorCode::OK);
  f.host.Sof();  // 安排的帧开始 / The scheduled frame starts
  TEST_ASSERT(log.items.empty());
  f.host.Sof();  // 它结束了，主机没有取 / It ended and the host took nothing
  f.host.Sof(3);
  TEST_ASSERT(log.items.size() == 1U && log.items[0].result == ErrorCode::FAILED &&
              log.items[0].actual == 0U);
  TEST_ASSERT(f.hw.TxFifoWords(3) == 0U);
  TEST_ASSERT((f.hw.InCtl(3) & Dwc2::CTL_EPENA) == 0U);

  const auto bytes = Bytes(50, 2);
  TEST_ASSERT(SubmitBytes(f.probe.iso_in, bytes) == ErrorCode::OK);
  f.host.Sof();
  Dwc2::InResult in = f.hw.HostIn(7, 3);
  f.Service();
  TEST_ASSERT(in.resp == Resp::DATA && in.data == bytes);
  TEST_ASSERT(log.items.size() == 2U && log.items[1].result == ErrorCode::OK);

  // 错过帧之后、禁用完成之前应用停止端点 / The application stops the endpoint after the
  // missed frame and before the disable is complete
  TEST_ASSERT(SubmitBytes(f.probe.iso_in, Bytes(48, 3)) == ErrorCode::OK);
  f.host.Sof();
  f.host.Sof();
  f.probe.iso_in->Flush(false);
  f.host.Sof(4);
  TEST_ASSERT(log.items.size() == 3U);
  TEST_ASSERT(log.items[2].result == ErrorCode::ABORTED ||
              log.items[2].result == ErrorCode::FAILED);
  TEST_ASSERT(!f.probe.iso_in->IsStopping());
  TEST_ASSERT(SubmitBytes(f.probe.iso_in, bytes) == ErrorCode::OK);
  f.host.Sof();
  in = f.hw.HostIn(7, 3);
  f.Service();
  TEST_ASSERT(in.resp == Resp::DATA && in.data == bytes);
}

/// 两个等时 IN 流：A 错过了它的帧，B 已经为下一帧挂好请求。帧结束时只有 A 以 FAILED
/// 结束，B 的请求保持，在下一帧发出数据并以 OK 结束。
/// Two isochronous IN streams: A misses its frame while B is already armed for the next
/// one. At the end of the frame only A is aborted and ends with FAILED; the request of B
/// stays and its data goes out in the next frame, ending with OK.
/// 依赖的模型假设同 TestIsochronousMissedFrame（DSTS 在中断挂起期间的帧号）。
/// Depends on the same model ASSUMPTION as TestIsochronousMissedFrame (the frame number
/// DSTS shows while the interrupt is pending).
template <typename Fixture>
void TestTwoIsochronousStreams(const typename Fixture::ModeType& mode)
{
  Fixture f(mode, true, false, true);
  f.Up();
  TEST_ASSERT(f.probe.iso_in != nullptr && f.probe.iso_in2 != nullptr);
  Log& log_a = f.probe.iso_in_log;
  Log& log_b = f.probe.iso_in2_log;
  const auto bytes_a = Bytes(48, 1);
  const auto bytes_b = Bytes(52, 2);
  for (unsigned round = 0; round < 2U; ++round)
  {
    const size_t failed_a = log_a.Count(ErrorCode::FAILED);
    const size_t ok_b = log_b.Count(ErrorCode::OK);
    TEST_ASSERT(SubmitBytes(f.probe.iso_in, bytes_a) == ErrorCode::OK);
    f.host.Sof();  // A 的帧开始 / The frame of A starts
    TEST_ASSERT(SubmitBytes(f.probe.iso_in2, bytes_b) == ErrorCode::OK);
    f.host.Sof();  // A 的帧结束，主机没有取 / It ends and the host took nothing of A
    TEST_ASSERT(log_b.Count(ErrorCode::FAILED) == 0U);
    TEST_ASSERT((f.hw.InCtl(4) & Dwc2::CTL_EPENA) != 0U);
    const Dwc2::InResult in = f.hw.HostIn(7, 4);  // B 的帧 / The frame of B
    f.Service();
    TEST_ASSERT(in.resp == Resp::DATA && in.data == bytes_b);
    TEST_ASSERT(log_b.Count(ErrorCode::OK) == ok_b + 1U);
    TEST_ASSERT(log_b.Count(ErrorCode::FAILED) == 0U);
    f.host.Sof(4);  // A 的禁用走完 / The disable of A runs to its end
    TEST_ASSERT(log_a.Count(ErrorCode::FAILED) == failed_a + 1U);
    TEST_ASSERT(log_a.Count(ErrorCode::OK) == 0U);
    TEST_ASSERT(log_b.Count(ErrorCode::FAILED) == 0U);
    if (round == 0U) f.host.Sof();  // 换一个奇偶 / Switch the parity
  }
}

// ---------------------------------------------------------------------------
// UAC1 microphone
// ---------------------------------------------------------------------------

/// UAC1 麦克风的流接口号（控制接口是 0）/ Streaming interface of the UAC1 microphone
/// (the control interface is 0)
inline constexpr uint8_t UAC_STREAMING_INTERFACE = 1;

/// 正常枚举后的 UAC1 麦克风连续出流：选中备用设置 1 后 3000 帧取到 3000 个 32 字节的包
/// （16 kHz 单声道 16 bit，每毫秒 16 个采样），不是发一个包就停。在 STM32H723 板上测到
/// 的也是连续出流。
/// A UAC1 microphone streams continuously after a normal enumeration: with alternate
/// setting 1 selected, 3000 frames give 3000 packets of 32 bytes (16 kHz mono 16 bit,
/// 16 samples per millisecond) instead of one packet and silence. Continuous streaming
/// is also what was measured on an STM32H723 board.
/// 依赖模型的假设：未到安排的帧时等时 IN 端点以零长包应答，见 TestIsochronousFrames。
/// Depends on the model ASSUMPTION that an isochronous IN endpoint answers a zero-length
/// packet before its scheduled frame, see TestIsochronousFrames.
template <typename UacFixture>
void TestUacStreams(const typename UacFixture::ModeType& mode)
{
  UacFixture f(mode);
  TEST_ASSERT(f.host.Enumerate(7, 1) == Result::OK);
  TEST_ASSERT(f.host.SetInterface(UAC_STREAMING_INTERFACE, 1) == Result::OK);
  // 启动的头几帧：SET_INTERFACE 时提交的包没有被主机取走，以 FAILED 结束后重新开始
  // Start-up: the packet submitted at SET_INTERFACE is not taken by the host, ends with
  // FAILED and the stream starts over
  (void)f.Stream(8);
  const std::vector<uint8_t> pcm(32U * 16U, 0x5A);
  unsigned packets = 0;
  for (unsigned block = 0; block < 300; ++block)
  {
    (void)f.mic.WritePcm(pcm.data(), 32U * 10U);
    packets += f.Stream(10);
  }
  if (packets != 3000U)
    std::fprintf(stderr, "UAC1 microphone: %u packets in 3000 frames\n", packets);
  TEST_ASSERT(packets == 3000U);
  // 关流后不再出包，重新开流后继续 / No packets with the stream off, then on again
  TEST_ASSERT(f.host.SendSetup(Dwc2::Host::Setup(0x01, 11, 0, 1, 0)) == Result::OK);
  f.host.Sof(4);
  TEST_ASSERT(f.host.StatusIn() == Result::OK);
  TEST_ASSERT(f.Stream(10) == 0U);
  TEST_ASSERT(f.host.SetInterface(UAC_STREAMING_INTERFACE, 1) == Result::OK);
  (void)f.Stream(8);
  TEST_ASSERT(f.Stream(100) == 100U);
}

/// 帧中断不随配置变化：枚举、SET_CONFIGURATION 0 和再配置之后都是开的。
/// The frame interrupt does not follow the configuration: it is on after enumeration,
/// after SET_CONFIGURATION 0 and after configuring again.
template <typename UacFixture>
void TestUacFrameInterruptIgnoresConfiguration(const typename UacFixture::ModeType& mode)
{
  UacFixture f(mode);
  f.host.Reset();
  std::vector<uint8_t> data;
  TEST_ASSERT(f.host.GetDescriptor(1, 0, 18, data) == Result::OK);
  TEST_ASSERT(f.host.SetAddress(7) == Result::OK);
  TEST_ASSERT(f.host.SetConfiguration(1) == Result::OK);
  f.host.Sof(2);
  TEST_ASSERT(!f.hw.SofMasked());
  TEST_ASSERT(f.host.SendSetup(Dwc2::Host::Setup(0x00, 9, 0, 0, 0)) == Result::OK);
  f.host.Sof(4);
  TEST_ASSERT(f.host.StatusIn() == Result::OK);
  f.host.Sof(2);
  TEST_ASSERT(!f.hw.SofMasked());
  TEST_ASSERT(f.host.SetConfiguration(1) == Result::OK);
  f.host.Sof(2);
  TEST_ASSERT(!f.hw.SofMasked());
}

/// 帧中断被库外的代码关掉时，流在一个包之后停止（数据来自每帧的节拍）；在 STM32H723
/// 板上测到的相同。
/// With the frame interrupt turned off by code outside the library the stream stops
/// after one packet (the data comes from the per-frame tick), as measured on an
/// STM32H723 board.
template <typename UacFixture>
void TestUacStopsWithoutFrameInterrupt(const typename UacFixture::ModeType& mode)
{
  UacFixture f(mode);
  TEST_ASSERT(f.host.Enumerate(7, 1) == Result::OK);
  TEST_ASSERT(f.host.SetInterface(UAC_STREAMING_INTERFACE, 1) == Result::OK);
  (void)f.Stream(8);
  TEST_ASSERT(f.Stream(20) == 20U);
  f.MaskFrameInterrupt();
  TEST_ASSERT(f.Stream(20) <= 1U);
}

}  // namespace UsbBackendCases
