/**
 * @file test_usb_ep_pool.cpp
 * @brief 端点池的假后端测试 / Fake-backend tests of the endpoint pool
 *
 * 登记、分配、回收、按地址查找和端点 0 的用例按头文件的文档检查返回码与可见性；包长
 * 查询的用例检查它与 Configure 对同一配置、同一速度选出的包长一致，并且不改变端点。
 * The registry cases check return codes and visibility of registration, allocation,
 * release, lookup by address and endpoint 0 against the header documentation; the
 * packet-size cases check that the query agrees with the size Configure chooses for
 * the same configuration and speed, and that it changes no endpoint.
 */

#include <cstddef>
#include <cstdint>
#include <functional>
#include <utility>

#include "ep.hpp"
#include "ep_pool.hpp"
#include "test_assert.hpp"
#include "usb_service.hpp"

namespace
{
using LibXR::Callback;
using LibXR::ErrorCode;
using LibXR::RawData;
using LibXR::USB::DeviceService;
using LibXR::USB::Endpoint;
using LibXR::USB::EndpointPool;
using LibXR::USB::Speed;
using Config = Endpoint::Config;
using Direction = Endpoint::Direction;
using EPNumber = Endpoint::EPNumber;
using State = Endpoint::State;
using Type = Endpoint::Type;

static_assert(LibXR::HW_CACHE_LINE_SIZE <= 64U, "test memory is aligned to 64 bytes");

/// 端点内存 / Endpoint memory
struct alignas(64) Memory
{
  uint8_t bytes[2048];
};

/// 只计数配置和上限查询的后端端点 / Backend endpoint that counts configures and limit
/// queries
class PoolEndpoint final : public Endpoint
{
 public:
  using Endpoint::HwLimits;

  PoolEndpoint(EPNumber number, Direction avail, RawData memory, uint8_t slots = 0U)
      : Endpoint(number, avail, memory, slots)
  {
  }

  HwLimits limits;
  size_t configures = 0U;  ///< ConfigureHardware 调用次数 / ConfigureHardware calls
  mutable size_t limit_queries = 0U;  ///< HardwareLimits 调用次数 / HardwareLimits calls
  mutable Direction limits_direction = Direction::BOTH;
  mutable Type limits_type = Type::CONTROL;

 protected:
  void ConfigureHardware(const Config&) override { ++configures; }

  void CloseHardware(bool) override {}

  ErrorCode StartHardware(const HwRequest&, bool) override { return ErrorCode::OK; }

  StopOutcome StopHardware(bool) override { return {ErrorCode::ABORTED, 0U}; }

  ErrorCode StallHardware(bool) override { return ErrorCode::OK; }

  ErrorCode ClearStallHardware(bool) override { return ErrorCode::OK; }

  HwLimits HardwareLimits(Direction direction, Type type) const override
  {
    ++limit_queries;
    limits_direction = direction;
    limits_type = type;
    return limits;
  }
};

/// 在 owner 中运行任务的设备，没有挂起函数 / Device that runs jobs in the owner,
/// without a pend function
class PoolDevice final : public DeviceService
{
 public:
  static constexpr uint32_t JOB_EVENT = 1U;  ///< 在 owner 中运行任务 / Run a job

  PoolDevice()
  {
    SetDeviceHandler(
        Callback<uint32_t>::Create([](bool in_isr, PoolDevice* self, uint32_t events)
                                   { self->OnEvents(events, in_isr); }, this));
  }

  /// 在 owner 中运行 job / Run job in the owner
  void AsOwner(std::function<void(bool)> job)
  {
    job_ = std::move(job);
    Request(JOB_EVENT, false);
    TEST_ASSERT(!job_);
  }

  void SetBusSpeed(Speed speed)
  {
    AsOwner([&](bool) { SetSpeed(speed); });
  }

  ErrorCode Configure(Endpoint& ep, const Config& cfg)
  {
    ErrorCode result = ErrorCode::PENDING;
    AsOwner([&](bool in_isr) { result = ep.Configure(cfg, in_isr); });
    return result;
  }

  void Close(Endpoint& ep)
  {
    AsOwner([&](bool in_isr) { ep.Close(in_isr); });
  }

 protected:
  void ServiceHardware(bool) override {}
  void MaskInterrupt() override {}
  void UnmaskInterrupt() override {}

 private:
  void OnEvents(uint32_t events, bool in_isr)
  {
    if ((events & JOB_EVENT) != 0U && job_)
    {
      std::function<void(bool)> job = std::move(job_);
      job_ = nullptr;
      job(in_isr);
    }
  }

  std::function<void(bool)> job_;
};

// ---------------------------------------------------------------------------
// 登记、分配、回收 / Registration, allocation, release
// ---------------------------------------------------------------------------

/// Put 的返回码和占位 / Return codes and placement of Put
void CheckPut()
{
  Memory mem;
  EndpointPool pool;
  PoolEndpoint in1(EPNumber::EP1, Direction::IN, {mem.bytes, 128U});
  PoolEndpoint in1_again(EPNumber::EP1, Direction::IN, {mem.bytes, 128U});
  PoolEndpoint out1(EPNumber::EP1, Direction::OUT, {mem.bytes, 128U});
  PoolEndpoint both1(EPNumber::EP1, Direction::BOTH, {mem.bytes, 128U});
  PoolEndpoint both2(EPNumber::EP2, Direction::BOTH, {mem.bytes, 128U});
  PoolEndpoint in2(EPNumber::EP2, Direction::IN, {mem.bytes, 128U});
  PoolEndpoint out15(EPNumber::EP15, Direction::OUT, {mem.bytes, 128U});

  TEST_ASSERT(pool.Put(nullptr) == ErrorCode::ARG_ERR);
  TEST_ASSERT(pool.Put(&in1) == ErrorCode::OK);
  // 同号同方向已占 / Same number and direction taken
  TEST_ASSERT(pool.Put(&in1_again) == ErrorCode::FULL);
  // 同号另一方向可登记 / The other direction of the same number is free
  TEST_ASSERT(pool.Put(&out1) == ErrorCode::OK);
  // BOTH 要两个方向都空 / BOTH needs both directions free
  TEST_ASSERT(pool.Put(&both1) == ErrorCode::FULL);
  TEST_ASSERT(pool.Put(&both2) == ErrorCode::OK);
  TEST_ASSERT(pool.Put(&in2) == ErrorCode::FULL);
  TEST_ASSERT(pool.Put(&out15) == ErrorCode::OK);

  // 失败的 Put 不改变已登记的端点 / A failed Put changes no registered endpoint
  Endpoint* ep = nullptr;
  TEST_ASSERT(pool.Get(ep, Direction::IN, EPNumber::EP1) == ErrorCode::OK);
  TEST_ASSERT(ep == &in1);
  TEST_ASSERT(pool.Get(ep, Direction::OUT, EPNumber::EP1) == ErrorCode::OK);
  TEST_ASSERT(ep == &out1);
  TEST_ASSERT(pool.Get(ep, Direction::IN, EPNumber::EP2) == ErrorCode::OK);
  TEST_ASSERT(ep == &both2);
  TEST_ASSERT(pool.Get(ep, Direction::OUT, EPNumber::EP15) == ErrorCode::OK);
  TEST_ASSERT(ep == &out15);
}

/// Get 和 Release 的返回码与占用 / Return codes and usage of Get and Release
void CheckGetRelease()
{
  Memory mem;
  EndpointPool pool;
  PoolEndpoint in1(EPNumber::EP1, Direction::IN, {mem.bytes, 128U});
  PoolEndpoint out1(EPNumber::EP1, Direction::OUT, {mem.bytes, 128U});
  PoolEndpoint both2(EPNumber::EP2, Direction::BOTH, {mem.bytes, 128U});
  PoolEndpoint lone(EPNumber::EP3, Direction::IN, {mem.bytes, 128U});
  TEST_ASSERT(pool.Put(&in1) == ErrorCode::OK);
  TEST_ASSERT(pool.Put(&out1) == ErrorCode::OK);
  TEST_ASSERT(pool.Put(&both2) == ErrorCode::OK);

  Endpoint* ep = nullptr;
  // 没登记、BOTH 方向、号越界 / Not registered, BOTH direction, number out of range
  TEST_ASSERT(pool.Get(ep, Direction::IN, EPNumber::EP3) == ErrorCode::NOT_FOUND);
  TEST_ASSERT(pool.Get(ep, Direction::OUT, EPNumber::EP0) == ErrorCode::NOT_FOUND);
  TEST_ASSERT(pool.Get(ep, Direction::BOTH, EPNumber::EP1) == ErrorCode::NOT_FOUND);
  TEST_ASSERT(pool.Get(ep, Direction::IN, EPNumber::EP_INVALID) == ErrorCode::NOT_FOUND);
  TEST_ASSERT(ep == nullptr);

  // 已分配的不能再分配，另一方向不受影响 / An allocated one cannot be allocated again;
  // the other direction is unaffected
  TEST_ASSERT(pool.Get(ep, Direction::IN, EPNumber::EP1) == ErrorCode::OK);
  TEST_ASSERT(ep == &in1);
  Endpoint* again = nullptr;
  TEST_ASSERT(pool.Get(again, Direction::IN, EPNumber::EP1) == ErrorCode::NOT_FOUND);
  TEST_ASSERT(again == nullptr);
  TEST_ASSERT(pool.Get(again, Direction::OUT, EPNumber::EP1) == ErrorCode::OK);
  TEST_ASSERT(again == &out1);

  // BOTH 端点分配一个方向后两个方向都占用 / Allocating a BOTH endpoint in one
  // direction takes both
  TEST_ASSERT(pool.Get(ep, Direction::OUT, EPNumber::EP2) == ErrorCode::OK);
  TEST_ASSERT(ep == &both2);
  TEST_ASSERT(pool.Get(again, Direction::IN, EPNumber::EP2) == ErrorCode::NOT_FOUND);

  // Release：空指针、没登记、没分配 / Release: null, not registered, not allocated
  TEST_ASSERT(pool.Release(nullptr) == ErrorCode::NOT_FOUND);
  TEST_ASSERT(pool.Release(&lone) == ErrorCode::NOT_FOUND);
  TEST_ASSERT(pool.Release(&in1) == ErrorCode::OK);
  TEST_ASSERT(pool.Release(&in1) == ErrorCode::NOT_FOUND);
  // 回收后可再分配，另一方向仍占用 / Allocatable again after release; the other
  // direction stays taken
  TEST_ASSERT(pool.Get(ep, Direction::IN, EPNumber::EP1) == ErrorCode::OK);
  TEST_ASSERT(ep == &in1);
  TEST_ASSERT(pool.Get(again, Direction::OUT, EPNumber::EP1) == ErrorCode::NOT_FOUND);

  // BOTH 端点回收后两个方向都可分配 / After release a BOTH endpoint is free in both
  // directions
  TEST_ASSERT(pool.Release(&both2) == ErrorCode::OK);
  TEST_ASSERT(pool.Get(ep, Direction::IN, EPNumber::EP2) == ErrorCode::OK);
  TEST_ASSERT(ep == &both2);
  TEST_ASSERT(pool.Release(&both2) == ErrorCode::OK);
  TEST_ASSERT(pool.Get(ep, Direction::OUT, EPNumber::EP2) == ErrorCode::OK);
  TEST_ASSERT(ep == &both2);
}

/// 端点 0 的访问与按地址查找 / Endpoint 0 accessors and lookup by address
void CheckEndpoint0()
{
  Memory mem;
  EndpointPool pool;
  PoolEndpoint ep0_in(EPNumber::EP0, Direction::IN, {mem.bytes, 64U});
  PoolEndpoint ep0_out(EPNumber::EP0, Direction::OUT, {mem.bytes + 64, 64U});

  Endpoint* ep = nullptr;
  TEST_ASSERT(pool.GetEndpoint0In() == nullptr);
  TEST_ASSERT(pool.GetEndpoint0Out() == nullptr);
  TEST_ASSERT(pool.FindEndpoint(0x80U, ep) == ErrorCode::NOT_FOUND);
  TEST_ASSERT(pool.FindEndpoint(0x00U, ep) == ErrorCode::NOT_FOUND);

  pool.SetEndpoint0(&ep0_in, &ep0_out);
  TEST_ASSERT(pool.GetEndpoint0In() == &ep0_in);
  TEST_ASSERT(pool.GetEndpoint0Out() == &ep0_out);
  // EP0 不需分配也不需配置即可查到 / EP0 is found without allocation or configuration
  TEST_ASSERT(pool.FindEndpoint(0x80U, ep) == ErrorCode::OK);
  TEST_ASSERT(ep == &ep0_in);
  TEST_ASSERT(pool.FindEndpoint(0x00U, ep) == ErrorCode::OK);
  TEST_ASSERT(ep == &ep0_out);
  // EP0 不在分配数组里 / EP0 is not in the allocation array
  TEST_ASSERT(pool.Get(ep, Direction::IN, EPNumber::EP0) == ErrorCode::NOT_FOUND);
  TEST_ASSERT(pool.Release(&ep0_in) == ErrorCode::NOT_FOUND);
}

/// 按地址查找只看已分配且已配置成该方向的端点 / Lookup by address sees only
/// allocated endpoints configured for that direction
void CheckFindEndpoint()
{
  Memory mem;
  PoolDevice dev;
  EndpointPool pool;
  PoolEndpoint in1(EPNumber::EP1, Direction::IN, {mem.bytes, 256U});
  PoolEndpoint out2(EPNumber::EP2, Direction::OUT, {mem.bytes + 256, 256U});
  PoolEndpoint both3(EPNumber::EP3, Direction::BOTH, {mem.bytes + 512, 256U});
  for (PoolEndpoint* ep : {&in1, &out2, &both3})
  {
    TEST_ASSERT(dev.Register(*ep) == ErrorCode::OK);
    TEST_ASSERT(pool.Put(ep) == ErrorCode::OK);
  }

  Endpoint* ep = nullptr;
  // 已登记但没分配 / Registered but not allocated
  TEST_ASSERT(pool.FindEndpoint(0x81U, ep) == ErrorCode::NOT_FOUND);
  TEST_ASSERT(pool.FindEndpoint(0x02U, ep) == ErrorCode::NOT_FOUND);
  // 号越界 / Number out of range
  TEST_ASSERT(pool.FindEndpoint(0x90U, ep) == ErrorCode::NOT_FOUND);

  TEST_ASSERT(pool.Get(ep, Direction::IN, EPNumber::EP1) == ErrorCode::OK);
  TEST_ASSERT(pool.Get(ep, Direction::OUT, EPNumber::EP2) == ErrorCode::OK);
  TEST_ASSERT(pool.Get(ep, Direction::IN, EPNumber::EP3) == ErrorCode::OK);

  // 已分配、未配置：地址只有端点号，所以 IN 端点查不到，OUT 端点按可用方向查得到。
  // Allocated, not configured: the address is just the number, so an IN endpoint is not
  // found and an OUT endpoint is found by its available direction.
  ep = nullptr;
  TEST_ASSERT(pool.FindEndpoint(0x81U, ep) == ErrorCode::NOT_FOUND);
  TEST_ASSERT(pool.FindEndpoint(0x02U, ep) == ErrorCode::OK);
  TEST_ASSERT(ep == &out2);
  TEST_ASSERT(pool.FindEndpoint(0x83U, ep) == ErrorCode::NOT_FOUND);
  TEST_ASSERT(pool.FindEndpoint(0x03U, ep) == ErrorCode::NOT_FOUND);

  TEST_ASSERT(dev.Configure(in1, {Direction::IN, Type::BULK, 64U}) == ErrorCode::OK);
  TEST_ASSERT(dev.Configure(both3, {Direction::IN, Type::INTR, 16U}) == ErrorCode::OK);
  ep = nullptr;
  TEST_ASSERT(pool.FindEndpoint(0x81U, ep) == ErrorCode::OK);
  TEST_ASSERT(ep == &in1);
  TEST_ASSERT(pool.FindEndpoint(0x01U, ep) == ErrorCode::NOT_FOUND);
  // 配置成 IN 的 BOTH 端点只按 IN 地址查到 / A BOTH endpoint configured as IN is found
  // by its IN address only
  TEST_ASSERT(pool.FindEndpoint(0x83U, ep) == ErrorCode::OK);
  TEST_ASSERT(ep == &both3);
  TEST_ASSERT(pool.FindEndpoint(0x03U, ep) == ErrorCode::NOT_FOUND);

  // 关闭不影响分配，回收后查不到 / Close leaves the allocation; after release it is
  // not found
  dev.Close(in1);
  TEST_ASSERT(pool.FindEndpoint(0x81U, ep) == ErrorCode::NOT_FOUND);
  TEST_ASSERT(dev.Configure(in1, {Direction::IN, Type::BULK, 64U}) == ErrorCode::OK);
  TEST_ASSERT(pool.FindEndpoint(0x81U, ep) == ErrorCode::OK);
  TEST_ASSERT(pool.Release(&in1) == ErrorCode::OK);
  TEST_ASSERT(pool.FindEndpoint(0x81U, ep) == ErrorCode::NOT_FOUND);
  // Release 不关闭端点 / Release does not close the endpoint
  TEST_ASSERT(in1.GetState() == State::IDLE);
}

// ---------------------------------------------------------------------------
// 包长查询 / Packet-size query
// ---------------------------------------------------------------------------

/// 查询的固定值：速度、类型、上限、槽和后端上限 / Fixed answers of the query: speed,
/// type, bound, slot and backend limit
void CheckQueryValues()
{
  Memory mem;
  EndpointPool pool;
  // 两个 1024 字节的槽 / Two slots of 1024 bytes
  PoolEndpoint big(EPNumber::EP1, Direction::IN, {mem.bytes, 2048U});
  // 两个 64 字节的槽，可配置为任一方向 / Two slots of 64 bytes, either direction
  PoolEndpoint small(EPNumber::EP2, Direction::BOTH, {mem.bytes, 128U});
  // 后端最大包长 256 / Backend largest packet 256
  PoolEndpoint limited(EPNumber::EP3, Direction::OUT, {mem.bytes, 2048U});
  limited.limits.max_packet_size = 256U;
  PoolEndpoint ep0_in(EPNumber::EP0, Direction::IN, {mem.bytes, 64U});
  PoolEndpoint ep0_out(EPNumber::EP0, Direction::OUT, {mem.bytes, 64U});
  TEST_ASSERT(pool.Put(&big) == ErrorCode::OK);
  TEST_ASSERT(pool.Put(&small) == ErrorCode::OK);
  TEST_ASSERT(pool.Put(&limited) == ErrorCode::OK);
  pool.SetEndpoint0(&ep0_in, &ep0_out);

  struct Row
  {
    EPNumber number;
    Direction direction;
    Type type;
    uint16_t bound;
    Speed speed;
    uint16_t packet;
  };
  constexpr uint16_t ANY = UINT16_MAX;
  const Row rows[] = {
      {EPNumber::EP1, Direction::IN, Type::BULK, ANY, Speed::HIGH, 512U},
      {EPNumber::EP1, Direction::IN, Type::BULK, ANY, Speed::FULL, 64U},
      {EPNumber::EP1, Direction::IN, Type::BULK, ANY, Speed::LOW, 0U},
      {EPNumber::EP1, Direction::IN, Type::ISOCHRONOUS, ANY, Speed::FULL, 1023U},
      {EPNumber::EP1, Direction::IN, Type::ISOCHRONOUS, ANY, Speed::HIGH, 1024U},
      {EPNumber::EP1, Direction::IN, Type::INTR, 700U, Speed::HIGH, 700U},
      {EPNumber::EP1, Direction::IN, Type::BULK, 63U, Speed::FULL, 32U},
      {EPNumber::EP1, Direction::IN, Type::BULK, 511U, Speed::HIGH, 0U},
      // 方向不可用、没登记 / Direction not available, not registered
      {EPNumber::EP1, Direction::OUT, Type::BULK, ANY, Speed::FULL, 0U},
      {EPNumber::EP4, Direction::IN, Type::BULK, ANY, Speed::FULL, 0U},
      {EPNumber::EP_INVALID, Direction::IN, Type::BULK, ANY, Speed::FULL, 0U},
      {EPNumber::EP1, Direction::BOTH, Type::BULK, ANY, Speed::FULL, 0U},
      // 槽只有 64 字节 / Slots of 64 bytes only
      {EPNumber::EP2, Direction::IN, Type::BULK, ANY, Speed::HIGH, 0U},
      {EPNumber::EP2, Direction::OUT, Type::BULK, ANY, Speed::FULL, 64U},
      {EPNumber::EP2, Direction::OUT, Type::ISOCHRONOUS, ANY, Speed::FULL, 64U},
      // 后端上限 / Backend limit
      {EPNumber::EP3, Direction::OUT, Type::INTR, ANY, Speed::HIGH, 256U},
      {EPNumber::EP3, Direction::OUT, Type::BULK, ANY, Speed::HIGH, 0U},
      {EPNumber::EP3, Direction::OUT, Type::BULK, ANY, Speed::FULL, 64U},
      // 端点 0 / Endpoint 0
      {EPNumber::EP0, Direction::IN, Type::CONTROL, ANY, Speed::LOW, 8U},
      {EPNumber::EP0, Direction::OUT, Type::CONTROL, ANY, Speed::HIGH, 64U},
      {EPNumber::EP0, Direction::IN, Type::CONTROL, 40U, Speed::FULL, 32U},
  };
  for (const Row& row : rows)
  {
    const Config cfg{row.direction, row.type, row.bound};
    TEST_ASSERT(pool.PacketSizeFor(row.number, cfg, row.speed) == row.packet);
  }

  // 用的是该号该方向的端点自己的后端上限 / The query asks the backend limits of the
  // endpoint at that number and direction
  small.limit_queries = 0U;
  TEST_ASSERT(pool.PacketSizeFor(EPNumber::EP2, {Direction::OUT, Type::INTR, ANY},
                                 Speed::FULL) == 64U);
  TEST_ASSERT(small.limit_queries == 1U);
  TEST_ASSERT(small.limits_direction == Direction::OUT);
  TEST_ASSERT(small.limits_type == Type::INTR);

  // 查询不配置端点 / The query configures nothing
  for (const PoolEndpoint* ep : {&big, &small, &limited, &ep0_in, &ep0_out})
  {
    TEST_ASSERT(ep->configures == 0U);
    TEST_ASSERT(ep->GetState() == State::DISABLED);
    TEST_ASSERT(ep->MaxPacketSize() == 0U);
  }
}

/// 查询等于 Configure 之后选出的包长 / The query equals the size a later Configure
/// chooses
void CheckQueryMatchesConfigure()
{
  Memory mem;
  PoolDevice dev;
  EndpointPool pool;
  PoolEndpoint ep0_in(EPNumber::EP0, Direction::IN, {mem.bytes, 64U});
  // 两个 512 字节的槽 / Two slots of 512 bytes
  PoolEndpoint in1(EPNumber::EP1, Direction::IN, {mem.bytes, 1024U});
  // 一个 64 字节的槽 / One slot of 64 bytes
  PoolEndpoint both2(EPNumber::EP2, Direction::BOTH, {mem.bytes, 64U}, 1U);
  // 两个 1024 字节的槽，后端最大包长 200 / Two slots of 1024 bytes, backend largest
  // packet 200
  PoolEndpoint out3(EPNumber::EP3, Direction::OUT, {mem.bytes, 2048U});
  out3.limits.max_packet_size = 200U;
  // 两个 1024 字节的槽 / Two slots of 1024 bytes
  PoolEndpoint both4(EPNumber::EP4, Direction::BOTH, {mem.bytes, 2048U});
  // 后端一个服务区间的上限：2、默认的 1、3 / Backend limits per service interval: 2, the
  // default 1, 3
  in1.limits.max_packets_per_interval = 2U;
  both4.limits.max_packets_per_interval = 3U;
  // 两个 3072 字节的槽，后端一个区间 3 个包 / Two slots of 3072 bytes, a backend limit
  // of 3 packets per interval
  alignas(64) static uint8_t wide_memory[6144];
  PoolEndpoint in5(EPNumber::EP5, Direction::IN, {wide_memory, sizeof(wide_memory)});
  in5.limits.max_packets_per_interval = 3U;
  PoolEndpoint* const eps[] = {&ep0_in, &in1, &both2, &out3, &both4, &in5};
  for (PoolEndpoint* ep : eps)
  {
    TEST_ASSERT(dev.Register(*ep) == ErrorCode::OK);
    if (ep != &ep0_in)
    {
      TEST_ASSERT(pool.Put(ep) == ErrorCode::OK);
    }
  }
  pool.SetEndpoint0(&ep0_in, nullptr);

  const Speed speeds[] = {Speed::LOW, Speed::FULL, Speed::HIGH};
  const Type types[] = {Type::CONTROL, Type::ISOCHRONOUS, Type::BULK, Type::INTR};
  const Direction directions[] = {Direction::IN, Direction::OUT};
  const uint16_t bounds[] = {0U,    1U,    7U,    8U,    10U,       16U,  40U,
                             63U,   64U,   200U,  256U,  511U,      512U, 700U,
                             1000U, 1023U, 1024U, 2048U, UINT16_MAX};
  const uint8_t counts[] = {1U, 0U, 2U, 3U, 200U};
  size_t configured = 0U;
  size_t refused = 0U;
  size_t several = 0U;  // 选出多于一个包的次数 / Times more than one packet was chosen
  for (Speed speed : speeds)
  {
    dev.SetBusSpeed(speed);
    for (PoolEndpoint* ep : eps)
    {
      for (Direction direction : directions)
      {
        for (Type type : types)
        {
          for (uint16_t bound : bounds)
          {
            for (uint8_t count : counts)
            {
              const Config cfg{direction, type, bound, count};
              const uint16_t packet = pool.PacketSizeFor(ep->GetNumber(), cfg, speed);
              const uint8_t packets =
                  pool.PacketsPerIntervalFor(ep->GetNumber(), cfg, speed);
              const ErrorCode result = dev.Configure(*ep, cfg);
              // 包数查询得 0：包长查询得 0，或包数上限为 0
              // The count query gives 0 when the size query does, or for a count bound
              // of 0
              TEST_ASSERT((packets == 0U) == (packet == 0U || count == 0U));
              if (packets == 0U)
              {
                TEST_ASSERT(result != ErrorCode::OK);
                TEST_ASSERT(ep->GetState() == State::DISABLED);
                ++refused;
                continue;
              }
              TEST_ASSERT(result == ErrorCode::OK);
              TEST_ASSERT(ep->MaxPacketSize() == packet);
              TEST_ASSERT(ep->PacketsPerInterval() == packets && packets <= count);
              // 多于一个包只在高速的等时和中断端点上，且后端允许
              // More than one packet only on high-speed isochronous and interrupt
              // endpoints, and where the backend allows it
              if (packets > 1U)
              {
                TEST_ASSERT(speed == Speed::HIGH);
                TEST_ASSERT(type == Type::ISOCHRONOUS || type == Type::INTR);
                TEST_ASSERT(packets <= ep->limits.max_packets_per_interval);
                TEST_ASSERT(packet >= (packets == 3U ? 683U : 513U));
                ++several;
              }
              if (type == Type::ISOCHRONOUS)
              {
                TEST_ASSERT(ep->MaxTransferSize() == size_t{packets} * packet);
              }
              // 已配置时查询照常，不受当前配置影响 / While configured the queries
              // answer as before, independent of the current configuration
              TEST_ASSERT(pool.PacketSizeFor(ep->GetNumber(), cfg, speed) == packet);
              TEST_ASSERT(pool.PacketsPerIntervalFor(ep->GetNumber(), cfg, speed) ==
                          packets);
              dev.Close(*ep);
              ++configured;
            }
          }
        }
      }
    }
  }
  // 两种结果都覆盖到 / Both outcomes are covered
  TEST_ASSERT(configured > 100U);
  TEST_ASSERT(refused > 100U);
  TEST_ASSERT(several > 20U);
  // 没有登记端点的号和方向得 0 / 0 for a number and direction without an endpoint
  TEST_ASSERT(pool.PacketsPerIntervalFor(EPNumber::EP6,
                                         {Direction::IN, Type::ISOCHRONOUS, 1024U, 3U},
                                         Speed::HIGH) == 0U);
  TEST_ASSERT(pool.PacketsPerIntervalFor(EPNumber::EP5,
                                         {Direction::OUT, Type::ISOCHRONOUS, 1024U, 3U},
                                         Speed::HIGH) == 0U);
  TEST_ASSERT(pool.PacketsPerIntervalFor(EPNumber::EP5,
                                         {Direction::IN, Type::ISOCHRONOUS, 1024U, 3U},
                                         Speed::HIGH) == 3U);

  // 端点已配置为另一种用法时查询不变，也不改变它 / With the endpoint configured for
  // another use the query is unchanged and leaves it alone
  dev.SetBusSpeed(Speed::FULL);
  TEST_ASSERT(dev.Configure(both4, {Direction::OUT, Type::BULK, 64U}) == ErrorCode::OK);
  const size_t configures = both4.configures;
  TEST_ASSERT(pool.PacketSizeFor(EPNumber::EP4, {Direction::IN, Type::ISOCHRONOUS, 1023U},
                                 Speed::HIGH) == 1023U);
  TEST_ASSERT(both4.configures == configures);
  TEST_ASSERT(both4.GetDirection() == Direction::OUT);
  TEST_ASSERT(both4.GetType() == Type::BULK);
  TEST_ASSERT(both4.MaxPacketSize() == 64U);
}

}  // namespace

void test_usb_ep_pool()
{
  CheckPut();
  CheckGetRelease();
  CheckEndpoint0();
  CheckFindEndpoint();
}

void test_usb_ep_pool_query()
{
  CheckQueryValues();
  CheckQueryMatchesConfigure();
}
