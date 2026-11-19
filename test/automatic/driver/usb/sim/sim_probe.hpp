/**
 * @file sim_probe.hpp
 * @brief 仿真测试用的探针类 / Probe class for the simulation tests
 *
 * 一个接口，若干个端点（批量、中断、等时都行），每个端点的完成按到达顺序记下：结果、
 * 字节数、收到的数据、是否在中断里、所在线程。OUT 端点在绑定时挂接收，完成后交回槽并
 * 重新挂。
 * One interface with any number of endpoints (bulk, interrupt or isochronous); each
 * endpoint's completions are recorded in arrival order: result, byte count, received
 * data, whether in an interrupt, and the thread. OUT endpoints arm a receive at bind,
 * and hand the slot back and arm again after each completion.
 */

#pragma once

#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "device_class.hpp"
#include "test_assert.hpp"

namespace LibXR::USB::Sim
{

/// 探针的一个端点 / One endpoint of the probe
struct ProbeEndpoint
{
  uint8_t number = 1;
  Endpoint::Direction direction = Endpoint::Direction::IN;
  Endpoint::Type type = Endpoint::Type::BULK;
  uint16_t max_packet = 512;  ///< 包长上限 / Packet-size bound
  uint8_t packets = 1;        ///< 一个服务区间的包数上限 / Packets-per-interval bound
  uint8_t interval = 1;       ///< bInterval
};

/// 一次完成 / One completion
struct ProbeDone
{
  ErrorCode result = ErrorCode::OK;
  size_t actual = 0;
  std::vector<uint8_t> data;
  bool in_isr = false;
  std::thread::id thread;
};

/**
 * @class Probe
 * @brief 探针类 / Probe class
 */
class Probe : public DeviceClass
{
 public:
  explicit Probe(std::vector<ProbeEndpoint> defs) : defs_(std::move(defs))
  {
    eps.assign(defs_.size(), nullptr);
    done.resize(defs_.size());
    fields_.assign(defs_.size(), 0U);
    for (size_t i = 0; i < defs_.size(); ++i)
    {
      bindings_.emplace_back(new Binding{this, i});
      callbacks_.push_back(Callback<const Endpoint::Completion&>::Create(
          [](bool in_isr, Binding* b, const Endpoint::Completion& c)
          { b->probe->OnDone(b->index, in_isr, c); }, bindings_.back().get()));
    }
  }

  using DeviceClass::RequestService;

  std::vector<Endpoint*> eps;  ///< 绑定时的端点，按 defs 顺序 / Bound endpoints
  bool arm_out = true;         ///< 绑定时挂接收 / Arm a receive at bind
  bool rearm = true;           ///< 完成后重新挂 / Arm again after a completion
  size_t receive_bytes = 0;    ///< 0：一个服务区间 / 0: one service interval
  /// OnService 里执行，参数是 in_isr / Run in OnService, with in_isr
  std::function<void(bool)> on_service;
  std::atomic<unsigned> binds{0};
  std::atomic<unsigned> unbinds{0};

  /// 端点 i 的完成个数 / Completions of endpoint i
  size_t Count(size_t i)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return done[i].size();
  }

  /// 端点 i 的第 n 个完成 / Completion n of endpoint i
  ProbeDone At(size_t i, size_t n)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    TEST_ASSERT(n < done[i].size());
    return done[i][n];
  }

  /// 端点 i 收到的全部 OK 数据 / Every OK byte endpoint i received
  std::vector<uint8_t> Received(size_t i)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<uint8_t> all;
    for (const ProbeDone& d : done[i])
    {
      all.insert(all.end(), d.data.begin(), d.data.end());
    }
    return all;
  }

  /// 在端点 i 上发 data；任何上下文 / Send data on endpoint i; any context
  ErrorCode Send(size_t i, const std::vector<uint8_t>& data, bool zlp = false,
                 bool in_isr = false)
  {
    Endpoint* ep = eps[i];
    if (ep == nullptr)
    {
      return ErrorCode::STATE_ERR;
    }
    Endpoint::Slot slot;
    ErrorCode result = ep->AcquireSlot(slot);
    if (result != ErrorCode::OK)
    {
      return result;
    }
    TEST_ASSERT(data.size() <= slot.data.size_);
    if (!data.empty())
    {
      std::memcpy(slot.data.addr_, data.data(), data.size());
    }
    result = ep->Submit(slot, data.size(), zlp, in_isr);
    if (result == ErrorCode::SIZE_ERR)
    {
      (void)ep->ReleaseSlot(slot);
    }
    return result;
  }

  /// 在端点 i 上挂接收 / Arm a receive on endpoint i
  ErrorCode Arm(size_t i, bool in_isr)
  {
    Endpoint* ep = eps[i];
    if (ep == nullptr)
    {
      return ErrorCode::STATE_ERR;
    }
    size_t bytes = receive_bytes;
    if (bytes == 0U)
    {
      bytes = static_cast<size_t>(ep->MaxPacketSize()) * ep->PacketsPerInterval();
    }
    return ep->Receive(bytes, in_isr);
  }

  std::vector<std::vector<ProbeDone>> done;

 protected:
  size_t GetInterfaceCount() const override { return 1U; }
  bool HasIAD() const override { return false; }
  size_t GetMaxConfigSize() const override { return 9U + 7U * defs_.size(); }

  ErrorCode WriteDescriptors(const EndpointPool& pool, Speed speed,
                             uint8_t first_interface, ConstRawData& block) override
  {
    desc_.assign(9U, 0U);
    desc_[0] = 9U;
    desc_[1] = 4U;
    desc_[2] = first_interface;
    desc_[4] = static_cast<uint8_t>(defs_.size());
    desc_[5] = 0xFFU;
    for (size_t i = 0; i < defs_.size(); ++i)
    {
      const ProbeEndpoint& d = defs_[i];
      EndpointDescriptor ep{};
      if (!WriteEndpointDescriptor(pool, speed, static_cast<Endpoint::EPNumber>(d.number),
                                   Cfg(d), d.interval, ep))
      {
        return ErrorCode::NOT_SUPPORT;
      }
      fields_[i] = ep.wMaxPacketSize;
      const auto* bytes = reinterpret_cast<const uint8_t*>(&ep);
      desc_.insert(desc_.end(), bytes, bytes + 7U);
    }
    block = {desc_.data(), desc_.size()};
    return ErrorCode::OK;
  }

  ErrorCode BindEndpoints(EndpointPool& pool, uint8_t first_interface,
                          bool in_isr) override
  {
    UNUSED(first_interface);
    ++binds;
    for (size_t i = 0; i < defs_.size(); ++i)
    {
      const ProbeEndpoint& d = defs_[i];
      const ErrorCode result =
          BindEndpoint(pool, static_cast<Endpoint::EPNumber>(d.number), Cfg(d),
                       fields_[i], in_isr, eps[i]);
      if (result != ErrorCode::OK)
      {
        return result;
      }
      eps[i]->SetOnComplete(callbacks_[i]);
    }
    for (size_t i = 0; i < defs_.size(); ++i)
    {
      if (arm_out && defs_[i].direction == Endpoint::Direction::OUT)
      {
        const ErrorCode result = Arm(i, in_isr);
        if (result != ErrorCode::OK)
        {
          return result;
        }
      }
    }
    return ErrorCode::OK;
  }

  void UnbindEndpoints(EndpointPool& pool, bool in_isr) override
  {
    ++unbinds;
    for (Endpoint*& ep : eps)
    {
      UnbindEndpoint(pool, ep, in_isr);
    }
  }

  void OnService(bool in_isr) override
  {
    if (on_service)
    {
      on_service(in_isr);
    }
  }

 private:
  struct Binding
  {
    Probe* probe;
    size_t index;
  };

  static Endpoint::Config Cfg(const ProbeEndpoint& d)
  {
    return {d.direction, d.type, d.max_packet, d.packets};
  }

  void OnDone(size_t i, bool in_isr, const Endpoint::Completion& c)
  {
    ProbeDone d;
    d.result = c.result;
    d.actual = c.actual;
    d.in_isr = in_isr;
    d.thread = std::this_thread::get_id();
    Endpoint* ep = eps[i];
    if (c.slot.Valid())
    {
      const auto* p = static_cast<const uint8_t*>(c.slot.data.addr_);
      d.data.assign(p, p + c.slot.data.size_);
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      done[i].push_back(d);
    }
    if (c.slot.Valid() && ep != nullptr)
    {
      TEST_ASSERT(ep->ReleaseSlot(c.slot) == ErrorCode::OK);
    }
    const bool failed =
        c.result == ErrorCode::FAILED || c.result == ErrorCode::OUT_OF_RANGE;
    if (rearm && ep != nullptr && defs_[i].direction == Endpoint::Direction::OUT &&
        (c.slot.Valid() || failed))
    {
      (void)Arm(i, in_isr);
    }
  }

  std::vector<ProbeEndpoint> defs_;
  std::vector<uint16_t> fields_;
  std::vector<uint8_t> desc_;
  std::vector<std::unique_ptr<Binding>> bindings_;
  std::vector<Callback<const Endpoint::Completion&>> callbacks_;
  std::mutex mutex_;
};

}  // namespace LibXR::USB::Sim
