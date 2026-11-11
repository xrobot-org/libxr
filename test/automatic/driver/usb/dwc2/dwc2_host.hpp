/**
 * @file dwc2_host.hpp
 * @brief 接在 DWC2 模型上的主机脚本 / Host script on the DWC2 model.
 *
 * 主机按 USB 2.0 的事务规则发令牌：控制传输是 SETUP、数据阶段、状态阶段；收到 NAK 就
 * 重试；批量和中断端点的 DATA0/DATA1 由主机自己维护。每个总线事件之后调用 service
 * 运行被测驱动的中断处理（中断线为高时），测试也可以把 service 关掉，让几个事件在同一次
 * 中断里被看到。
 *
 * The host sends tokens by the transaction rules of USB 2.0: a control transfer is SETUP,
 * data stage and status stage; a NAK is retried; DATA0/DATA1 of bulk and interrupt
 * endpoints are kept by the host. After every bus event service runs the interrupt
 * handler of the driver under test (while the interrupt line is high); a test can switch
 * service off so that several events are seen by one interrupt.
 */

#pragma once

#include <array>
#include <cstring>
#include <functional>
#include <vector>

#include "fake_dwc2.hpp"

namespace Dwc2
{

class Host
{
 public:
  enum class Result : uint8_t
  {
    OK,
    STALL,
    NAK,     ///< 重试次数用完仍是 NAK / Still NAK after every retry
    TIMEOUT  ///< 设备不应答 / The device does not answer
  };

  Host(Controller& hw, std::function<void()> service)
      : hw_(hw), service_(std::move(service))
  {
  }

  uint8_t address = 0;
  unsigned retries = 8;      ///< NAK 重试次数 / NAK retries
  bool auto_service = true;  ///< 每个总线事件后运行中断处理 / Service after each event
  /// 被 NAK 的事务在下一帧重试：重试之前先发一个 SOF / A NAKed transaction is retried
  /// in the next frame: a SOF goes out before the retry
  bool frame_per_retry = true;
  unsigned toggle_errors = 0;  ///< 设备发来的 DATA PID 与预期不符 / Unexpected DATA PID
  uint8_t ep0_packet = 64;

  void Service()
  {
    if (auto_service) service_();
  }
  /// 不管 auto_service，运行中断处理 / Run the interrupt handler regardless
  void ServiceNow() { service_(); }

  /// 总线复位：复位信号开始和结束之间驱动处理 USBRST / Bus reset: the driver handles
  /// USBRST between the start and the end of the reset signalling
  void Reset()
  {
    hw_.HostResetBegin();
    Service();
    hw_.HostResetEnd();
    Service();
    // 复位信号还在（核心重新检测到复位）：再结束一次 / The reset signalling is still on
    // (the core detected the reset again): end it once more
    if (hw_.InReset())
    {
      hw_.HostResetEnd();
      Service();
    }
    address = 0;
    out_toggle_.fill(0);
    in_toggle_.fill(0);
  }

  void Sof()
  {
    hw_.HostSof();
    Service();
  }
  void Sof(unsigned count)
  {
    for (unsigned i = 0; i < count; ++i) Sof();
  }

  /// 推进帧直到条件成立，最多 limit 帧；返回条件的值。停止序列走完需要的帧数取决于冲刷
  /// 和禁用的时间，手册没有给出，所以等条件而不是等固定的帧数。
  /// Advance frames until the condition holds, at most limit; returns the condition. The
  /// number of frames a stop sequence needs depends on the flush and disable times, which
  /// the manual does not give, so the case waits for the condition instead of a fixed
  /// number of frames.
  template <typename Condition>
  bool SofUntil(Condition condition, unsigned limit = 16)
  {
    for (unsigned i = 0; i < limit && !condition(); ++i) Sof();
    return condition();
  }

  static std::array<uint8_t, 8> Setup(uint8_t type, uint8_t request, uint16_t value,
                                      uint16_t index, uint16_t length)
  {
    return {type,
            request,
            static_cast<uint8_t>(value),
            static_cast<uint8_t>(value >> 8U),
            static_cast<uint8_t>(index),
            static_cast<uint8_t>(index >> 8U),
            static_cast<uint8_t>(length),
            static_cast<uint8_t>(length >> 8U)};
  }

  /// 只发 SETUP 包 / The SETUP packet alone
  Result SendSetup(const std::array<uint8_t, 8>& setup)
  {
    const Resp resp = hw_.HostSetup(address, setup.data());
    Service();
    return resp == Resp::ACK ? Result::OK : Result::TIMEOUT;
  }

  /// 控制读：数据阶段读到 wLength 或短包为止，然后发状态阶段的 OUT 零长包 / Control
  /// read: the data stage runs to wLength or a short packet, then the zero-length status
  /// OUT
  Result ControlRead(const std::array<uint8_t, 8>& setup, std::vector<uint8_t>& data)
  {
    data.clear();
    const size_t length = setup[6] | (setup[7] << 8U);
    if (SendSetup(setup) != Result::OK) return Result::TIMEOUT;
    uint8_t pid = 1;
    while (data.size() < length)
    {
      std::vector<uint8_t> packet;
      const Result result = InPacket(0, packet, &pid);
      if (result != Result::OK) return result;
      data.insert(data.end(), packet.begin(), packet.end());
      if (packet.size() < ep0_packet) break;
    }
    return StatusOut();
  }

  /// 控制写：数据阶段之后取状态阶段的 IN 零长包 / Control write: after the data stage
  /// the zero-length status IN is taken
  Result ControlWrite(const std::array<uint8_t, 8>& setup,
                      const std::vector<uint8_t>& data)
  {
    if (SendSetup(setup) != Result::OK) return Result::TIMEOUT;
    uint8_t pid = 1;
    size_t offset = 0;
    while (offset < data.size())
    {
      const size_t chunk = std::min<size_t>(ep0_packet, data.size() - offset);
      const Result result = OutPacket(0, data.data() + offset, chunk, pid);
      if (result != Result::OK) return result;
      pid ^= 1U;
      offset += chunk;
    }
    return StatusIn();
  }

  Result ControlNoData(const std::array<uint8_t, 8>& setup)
  {
    return ControlWrite(setup, {});
  }

  Result StatusOut() { return OutPacket(0, nullptr, 0, 1); }

  Result StatusIn()
  {
    std::vector<uint8_t> packet;
    uint8_t pid = 1;
    const Result result = InPacket(0, packet, &pid);
    if (result != Result::OK) return result;
    return packet.empty() ? Result::OK : Result::TIMEOUT;
  }

  /// 一个 OUT 包，NAK 时重试 / One OUT packet, retried on NAK
  Result OutPacket(uint8_t num, const uint8_t* data, size_t length, uint8_t pid)
  {
    for (unsigned i = 0; i <= retries; ++i)
    {
      const Resp resp = hw_.HostOut(address, num, pid, data, length);
      Service();
      if (resp == Resp::ACK) return Result::OK;
      if (resp == Resp::STALL) return Result::STALL;
      if (resp == Resp::TIMEOUT) return Result::TIMEOUT;
      NextFrame();
    }
    return Result::NAK;
  }

  /// 一个 IN 包，NAK 时重试；expected 给出并更新预期的 DATA PID / One IN packet, retried
  /// on NAK; expected holds and advances the DATA PID the host expects
  Result InPacket(uint8_t num, std::vector<uint8_t>& packet, uint8_t* expected)
  {
    for (unsigned i = 0; i <= retries; ++i)
    {
      InResult in = hw_.HostIn(address, num);
      Service();
      if (in.resp == Resp::STALL) return Result::STALL;
      if (in.resp == Resp::TIMEOUT) return Result::TIMEOUT;
      if (in.resp != Resp::DATA)
      {
        NextFrame();
        continue;
      }
      if (expected != nullptr)
      {
        if (in.pid != *expected)
        {
          ++toggle_errors;  // A resend of the previous packet: acknowledged, dropped.
          continue;
        }
        *expected ^= 1U;
      }
      packet = std::move(in.data);
      return Result::OK;
    }
    return Result::NAK;
  }

  /// 批量/中断 OUT：按包长切开发送，DATA PID 由主机维护 / Bulk or interrupt OUT, split
  /// at the packet size, with the DATA PID kept by the host
  Result Write(uint8_t num, const std::vector<uint8_t>& data, size_t packet_size)
  {
    size_t offset = 0;
    do
    {
      const size_t chunk = std::min(packet_size, data.size() - offset);
      const Result result = OutPacket(num, data.data() + offset, chunk, out_toggle_[num]);
      if (result != Result::OK) return result;
      out_toggle_[num] ^= 1U;
      offset += chunk;
    } while (offset < data.size());
    return Result::OK;
  }

  /// 批量/中断 IN：取一个包 / Bulk or interrupt IN: one packet
  Result Read(uint8_t num, std::vector<uint8_t>& packet)
  {
    return InPacket(num, packet, &in_toggle_[num]);
  }

  /// 取到短包或 total 字节为止 / Read until a short packet or total bytes
  Result ReadTransfer(uint8_t num, std::vector<uint8_t>& data, size_t total,
                      size_t packet_size)
  {
    data.clear();
    while (data.size() < total)
    {
      std::vector<uint8_t> packet;
      const Result result = Read(num, packet);
      if (result != Result::OK) return result;
      data.insert(data.end(), packet.begin(), packet.end());
      if (packet.size() < packet_size) break;
    }
    return Result::OK;
  }

  /// 端点的 DATA PID 回到 DATA0（配置、CLEAR_FEATURE(ENDPOINT_HALT) 之后）/ Back to
  /// DATA0 (after a configuration or CLEAR_FEATURE(ENDPOINT_HALT))
  void ResetToggle(uint8_t num)
  {
    out_toggle_[num] = 0;
    in_toggle_[num] = 0;
  }
  void ResetToggles()
  {
    out_toggle_.fill(0);
    in_toggle_.fill(0);
  }

  // Standard requests.
  Result GetDescriptor(uint8_t type, uint8_t index, uint16_t length,
                       std::vector<uint8_t>& data, uint16_t language = 0)
  {
    return ControlRead(
        Setup(0x80, 6, static_cast<uint16_t>((type << 8U) | index), language, length),
        data);
  }
  Result SetAddress(uint8_t value)
  {
    const Result result = ControlNoData(Setup(0x00, 5, value, 0, 0));
    if (result == Result::OK) address = value;
    return result;
  }
  Result SetConfiguration(uint8_t value)
  {
    const Result result = ControlNoData(Setup(0x00, 9, value, 0, 0));
    if (result == Result::OK) ResetToggles();
    return result;
  }
  Result SetInterface(uint8_t interface, uint8_t alternate)
  {
    return ControlNoData(Setup(0x01, 11, alternate, interface, 0));
  }
  Result ClearHalt(uint8_t endpoint_address)
  {
    const Result result = ControlNoData(Setup(0x02, 1, 0, endpoint_address, 0));
    if (result == Result::OK) ResetToggle(endpoint_address & 0x0FU);
    return result;
  }
  Result SetHalt(uint8_t endpoint_address)
  {
    return ControlNoData(Setup(0x02, 3, 0, endpoint_address, 0));
  }

  /// 复位、读设备描述符、设地址、选配置 / Reset, read the device descriptor, set the
  /// address, select the configuration
  Result Enumerate(uint8_t new_address, uint8_t configuration)
  {
    Reset();
    std::vector<uint8_t> descriptor;
    Result result = GetDescriptor(1, 0, 18, descriptor);
    if (result != Result::OK) return result;
    if (descriptor.size() != 18U) return Result::TIMEOUT;
    result = SetAddress(new_address);
    if (result != Result::OK) return result;
    return SetConfiguration(configuration);
  }

 private:
  void NextFrame()
  {
    if (!frame_per_retry) return;
    hw_.HostSof();
    Service();
  }

  Controller& hw_;
  std::function<void()> service_;
  std::array<uint8_t, 16> out_toggle_{};
  std::array<uint8_t, 16> in_toggle_{};
};

}  // namespace Dwc2
