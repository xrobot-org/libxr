#include <algorithm>
#include <array>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

#include "cdc_uart.hpp"
#include "dev_core.hpp"
#include "ep_pool.hpp"
#include "test_assert.hpp"

using namespace LibXR;
using namespace LibXR::USB;

extern "C" void libxr_fatal_error(const char* file, uint32_t line, bool)
{
  std::fprintf(stderr, "%s:%u: USB product assertion failed\n", file, line);
  std::abort();
}

namespace
{
using Direction = Endpoint::Direction;
using Number = Endpoint::EPNumber;
using Type = Endpoint::Type;
using State = Endpoint::State;
using Completion = Endpoint::Completion;
using Slot = Endpoint::Slot;

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
  std::mutex lock;  // Only the threaded stress test plays hardware from another thread.

  const Start& Last() const { return starts.back(); }

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
    TEST_ASSERT(active && bytes.size() <= starts.back().capacity);
    active = false;
    if (!bytes.empty()) std::memcpy(starts.back().address, bytes.data(), bytes.size());
    OnHardwareComplete(true, starts.back().tag, bytes.size());
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

std::vector<uint8_t> Bytes(size_t size, uint8_t seed)
{
  std::vector<uint8_t> data(size);
  for (size_t i = 0; i != size; ++i) data[i] = static_cast<uint8_t>(seed + i);
  return data;
}

std::vector<uint8_t> Wire(const MockEndpoint& ep)
{
  std::vector<uint8_t> data;
  for (const auto& packet : ep.sent)
    data.insert(data.end(), packet.begin(), packet.end());
  return data;
}

struct Fixture
{
  // Two slots need two cache lines; the host build's line is 64 B.
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

void Fill(const Slot& slot, uint8_t seed, size_t length)
{
  for (size_t i = 0; i < length; ++i)
    static_cast<uint8_t*>(slot.data.addr_)[i] = static_cast<uint8_t>(seed + i);
}

// Slots an idle IN endpoint lends at once; they are returned before it returns.
size_t FreeSlots(Endpoint& ep)
{
  Slot slots[2];
  size_t count = 0;
  while (count < 2U && ep.AcquireSlot(slots[count], false) == ErrorCode::OK) ++count;
  for (size_t i = 0; i < count; ++i) ep.ReleaseSlot(slots[i], false);
  return count;
}

// ---------------------------------------------------------------------------
// Endpoint
// ---------------------------------------------------------------------------

void TestGeometry()
{
  Fixture f;
  TEST_ASSERT(f.in.Configure({Direction::IN, Type::BULK, 64}, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.MaxPacketSize() == 64 && FreeSlots(f.in) == 2 &&
              f.in.GetGranted().slot_capacity == 64);
  f.in.Close(false);
  TEST_ASSERT(f.in.GetState() == State::DISABLED && f.in.closes == 1);
  // 128 B as two slots of 64 B: a 128 B high-speed packet is NO_BUFF; the count never
  // drops to one.
  f.pool.SetSpeed(Speed::HIGH);
  TEST_ASSERT(f.comm.Configure({Direction::IN, Type::INTERRUPT, 128}, false) ==
              ErrorCode::NO_BUFF);
  f.comm.Close(false);
  TEST_ASSERT(f.comm.Configure({Direction::IN, Type::INTERRUPT, 64}, false) ==
              ErrorCode::OK);
  TEST_ASSERT(FreeSlots(f.comm) == 2 && f.comm.GetGranted().slot_capacity == 64);
  f.comm.Close(false);
  f.pool.SetSpeed(Speed::FULL);
  // The same 128 B built as one slot holds the 128 B packet.
  Block<128> single_storage;
  EndpointPool single_pool{2};
  MockEndpoint single{Number::EP4, Direction::IN, single_storage.Raw(), 1};
  TEST_ASSERT(single_pool.Put(&single) == ErrorCode::OK);
  single_pool.SetSpeed(Speed::HIGH);
  TEST_ASSERT(single.Configure({Direction::IN, Type::INTERRUPT, 128}, false) ==
              ErrorCode::OK);
  TEST_ASSERT(FreeSlots(single) == 1 && single.GetGranted().slot_capacity == 128);
  // EP0 is one slot holding one packet.
  Block<64> ep0_storage;
  MockEndpoint ep0{Number::EP0, Direction::IN, ep0_storage.Raw()};
  TEST_ASSERT(ep0.Configure({Direction::IN, Type::CONTROL, 64}, false) == ErrorCode::OK);
  TEST_ASSERT(FreeSlots(ep0) == 1 && ep0.GetGranted().slot_capacity == 64);
  // Bulk FS packets are 8/16/32/64 only.
  TEST_ASSERT(f.in.Configure({Direction::IN, Type::BULK, 48}, false) ==
              ErrorCode::ARG_ERR);
  TEST_ASSERT(f.in.GetState() == State::ERROR);
  f.in.Close(false);
  // A slot that cannot hold one packet is NO_BUFF, not fatal.
  f.pool.SetSpeed(Speed::HIGH);
  TEST_ASSERT(f.in.Configure({Direction::IN, Type::BULK, 512}, false) ==
              ErrorCode::NO_BUFF);
  f.in.Close(false);
  // A packet size that is illegal at the current speed is ARG_ERR: high-speed bulk is 512
  // only.
  TEST_ASSERT(f.in.Configure({Direction::IN, Type::BULK, 64}, false) ==
              ErrorCode::ARG_ERR);
  f.in.Close(false);
  f.pool.SetSpeed(Speed::FULL);
  // The packet size is never chosen for the caller: 0 is invalid.
  TEST_ASSERT(f.in.Configure({Direction::IN, Type::BULK}, false) == ErrorCode::ARG_ERR);
  f.in.Close(false);
  // Direction must match the registered direction.
  TEST_ASSERT(f.in.Configure({Direction::OUT, Type::BULK, 64}, false) ==
              ErrorCode::ARG_ERR);
  f.in.Close(false);
  // The query names the largest legal size the slot and the backend hold.
  TEST_ASSERT(f.in.LargestPacketSize(Type::BULK) == 64);
  f.pool.SetSpeed(Speed::HIGH);
  TEST_ASSERT(f.in.LargestPacketSize(Type::BULK) == 0);  // 512 does not fit a 64 B slot.
  TEST_ASSERT(f.in.LargestPacketSize(Type::INTERRUPT) == 64);
  f.pool.SetSpeed(Speed::FULL);
  // A packet above the backend FIFO limit is ARG_ERR; a legal smaller one fits.
  f.in.limit = 32;
  TEST_ASSERT(f.in.LargestPacketSize(Type::BULK) == 32);
  f.in.limit = 40;  // Not a legal bulk size: the next smaller legal one.
  TEST_ASSERT(f.in.LargestPacketSize(Type::BULK) == 32 &&
              f.in.LargestPacketSize(Type::INTERRUPT) == 40);
  f.in.limit = 32;
  TEST_ASSERT(f.in.Configure({Direction::IN, Type::BULK, 64}, false) ==
              ErrorCode::ARG_ERR);
  f.in.Close(false);
  TEST_ASSERT(f.in.Configure({Direction::IN, Type::BULK, 32}, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.MaxPacketSize() == 32 && f.in.GetGranted().slot_capacity == 64);
  f.in.Close(false);
  f.in.limit = UINT16_MAX;

  Block<1024> big;
  EndpointPool pool{2};
  MockEndpoint ep{Number::EP1, Direction::IN, big.Raw()};
  TEST_ASSERT(pool.Put(&ep) == ErrorCode::OK);
  TEST_ASSERT(ep.Configure({Direction::IN, Type::BULK, 64}, false) == ErrorCode::OK);
  TEST_ASSERT(FreeSlots(ep) == 2 && ep.GetGranted().slot_capacity == 512);
  Slot a, b;
  TEST_ASSERT(ep.AcquireSlot(a, false) == ErrorCode::OK);
  TEST_ASSERT(ep.AcquireSlot(b, false) == ErrorCode::OK);
  TEST_ASSERT(static_cast<uint8_t*>(b.data.addr_) - static_cast<uint8_t*>(a.data.addr_) ==
              512);
  ep.ReleaseSlot(a, false);
  ep.ReleaseSlot(b, false);
  ep.Close(false);
  // The speed changes the packet size, never the slot count or the slot places.
  pool.SetSpeed(Speed::HIGH);
  TEST_ASSERT(ep.Configure({Direction::IN, Type::BULK, 512}, false) == ErrorCode::OK);
  TEST_ASSERT(ep.MaxPacketSize() == 512 && ep.GetGranted().slot_capacity == 512);
  Slot c, d;
  TEST_ASSERT(ep.AcquireSlot(c, false) == ErrorCode::OK && c.data.addr_ == a.data.addr_);
  TEST_ASSERT(ep.AcquireSlot(d, false) == ErrorCode::OK && d.data.addr_ == b.data.addr_);
}

void TestInBorrowOrder()
{
  Fixture f;
  Recorder rec;
  f.in.SetOnComplete(rec.Bind());
  TEST_ASSERT(f.in.Configure({Direction::IN, Type::BULK, 64}, false) == ErrorCode::OK);
  Slot a, b, c;
  TEST_ASSERT(f.in.AcquireSlot(a, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.AcquireSlot(b, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.AcquireSlot(c, false) == ErrorCode::BUSY);
  TEST_ASSERT(a.index != b.index && a.data.size_ == 64);
  Fill(a, 1, 10);
  Fill(b, 100, 20);
  TEST_ASSERT(f.in.Submit(b, 65, false, false) == ErrorCode::SIZE_ERR);
  // Submission order decides the start order, not the slot index.
  TEST_ASSERT(f.in.Submit(b, 20, false, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.Submit(b, 20, false, false) == ErrorCode::STATE_ERR);
  TEST_ASSERT(f.in.Submit(a, 10, false, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.starts.size() == 1 && f.in.Last().length == 20);
  f.in.Complete();
  TEST_ASSERT(rec.list.size() == 1 && rec.list[0].result == ErrorCode::OK &&
              rec.list[0].actual == 20 && !rec.list[0].slot.Valid());
  TEST_ASSERT(f.in.starts.size() == 2 && f.in.Last().length == 10);
  f.in.Complete();
  TEST_ASSERT(rec.list.size() == 2 && rec.list[1].actual == 10);
  auto expected = Bytes(20, 100);
  auto tail = Bytes(10, 1);
  expected.insert(expected.end(), tail.begin(), tail.end());
  TEST_ASSERT(Wire(f.in) == expected);
  // Released slots are free again.
  TEST_ASSERT(f.in.AcquireSlot(c, false) == ErrorCode::OK);
  f.in.ReleaseSlot(c, false);
  // OUT-only calls on an IN endpoint.
  TEST_ASSERT(f.in.Receive(64, false) == ErrorCode::STATE_ERR);
}

void TestZeroLengthTailAndShort()
{
  Fixture f;
  Recorder rec;
  f.in.SetOnComplete(rec.Bind());
  TEST_ASSERT(f.in.Configure({Direction::IN, Type::BULK, 64}, false) == ErrorCode::OK);
  Slot slot;
  TEST_ASSERT(f.in.AcquireSlot(slot, false) == ErrorCode::OK);
  Fill(slot, 3, 64);
  TEST_ASSERT(f.in.Submit(slot, 64, true, false) == ErrorCode::OK);
  f.in.Complete();
  // One request: the data packet, then its ZLP, then one completion.
  TEST_ASSERT(rec.list.empty() && f.in.starts.size() == 2 && f.in.Last().length == 0);
  f.in.Complete();
  TEST_ASSERT(rec.list.size() == 1 && rec.list[0].result == ErrorCode::OK &&
              rec.list[0].actual == 64);
  // A short request needs no tail.
  TEST_ASSERT(f.in.AcquireSlot(slot, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.Submit(slot, 63, true, false) == ErrorCode::OK);
  f.in.Complete();
  TEST_ASSERT(rec.list.size() == 2 && f.in.starts.size() == 3);
  // The host taking fewer bytes than offered is a failed IN request.
  TEST_ASSERT(f.in.AcquireSlot(slot, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.Submit(slot, 40, false, false) == ErrorCode::OK);
  f.in.CompleteWith(30);
  TEST_ASSERT(rec.list.size() == 3 && rec.list[2].result == ErrorCode::FAILED);
  // Slotless zero-length request.
  TEST_ASSERT(f.in.AcquireSlot(slot, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.Submit(slot, 1, false, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.SubmitZeroLength(false) == ErrorCode::BUSY);
  f.in.Complete();
  TEST_ASSERT(f.in.SubmitZeroLength(false) == ErrorCode::OK);
  TEST_ASSERT(f.in.SubmitZeroLength(false) == ErrorCode::BUSY);
  TEST_ASSERT(f.in.Last().length == 0);
  f.in.Complete();
  TEST_ASSERT(rec.list.size() == 5 && rec.list[4].result == ErrorCode::OK &&
              rec.list[4].actual == 0);
}

void TestOutLease()
{
  Fixture f;
  Recorder rec;
  f.out.SetOnComplete(rec.Bind());
  TEST_ASSERT(f.out.Configure({Direction::OUT, Type::BULK, 64}, false) == ErrorCode::OK);
  TEST_ASSERT(f.out.Receive(63, false) == ErrorCode::SIZE_ERR);
  TEST_ASSERT(f.out.Receive(0, false) == ErrorCode::SIZE_ERR);
  TEST_ASSERT(f.out.Receive(64, false) == ErrorCode::OK);
  TEST_ASSERT(f.out.Receive(64, false) == ErrorCode::OK);
  TEST_ASSERT(f.out.Receive(64, false) == ErrorCode::BUSY);
  TEST_ASSERT(f.out.starts.size() == 1);
  const auto packet = Bytes(10, 50);
  f.out.HostSends(packet);
  TEST_ASSERT(rec.list.size() == 1 && rec.list[0].result == ErrorCode::OK &&
              rec.list[0].actual == 10 && rec.list[0].slot.Valid());
  const Slot lease = rec.list[0].slot;
  TEST_ASSERT(lease.data.size_ == 10 &&
              std::equal(packet.begin(), packet.end(),
                         static_cast<const uint8_t*>(lease.data.addr_)));
  // The second slot was armed in parallel; the lease is not touched by hardware.
  TEST_ASSERT(f.out.starts.size() == 2 && f.out.Last().address != lease.data.addr_);
  f.out.HostSends(Bytes(64, 1));
  TEST_ASSERT(rec.list.size() == 2 && rec.list[1].actual == 64);
  // Nothing armed while both leases are held: backpressure.
  TEST_ASSERT(!f.out.active);
  TEST_ASSERT(f.out.ReturnSlot(lease, false) == ErrorCode::OK);
  TEST_ASSERT(f.out.active && f.out.Last().address == lease.data.addr_ &&
              f.out.Last().length == 64);
  TEST_ASSERT(f.out.ReturnSlot(lease, false) == ErrorCode::STATE_ERR);
  f.out.ReleaseSlot(rec.list[1].slot, false);
  TEST_ASSERT(f.out.Receive(64, false) == ErrorCode::OK);
  Slot none;
  TEST_ASSERT(f.out.AcquireSlot(none, false) == ErrorCode::STATE_ERR);
}

/// A request queued from the endpoint's own completion callback is started by the
/// StartNext that follows the callback; it posts no notification of its own.
void TestQueueFromOwnCallbackPostsNothing()
{
  struct Context
  {
    Fixture* f;
    unsigned calls = 0;
    bool posted = true;
  };
  Fixture f;
  Context context{&f};
  f.out.SetOnComplete(Callback<const Completion&>::Create(
      [](bool in_isr, Context* self, const Completion& completion)
      {
        ++self->calls;
        TEST_ASSERT(self->f->out.ReturnSlot(completion.slot, in_isr) == ErrorCode::OK);
        self->posted = self->f->pool.HasPendingControl(EndpointPool::ENDPOINT_EVENT);
      },
      &context));
  TEST_ASSERT(f.out.Configure({Direction::OUT, Type::BULK, 64}, false) == ErrorCode::OK);
  // One request only: the early start finds nothing queued, so the returned slot is
  // started after the callback.
  TEST_ASSERT(f.out.Receive(64, false) == ErrorCode::OK);
  TEST_ASSERT(f.out.starts.size() == 1);
  f.out.HostSends(Bytes(10, 1));
  TEST_ASSERT(context.calls == 1 && !context.posted);
  TEST_ASSERT(f.out.active && f.out.starts.size() == 2);
  f.out.HostSends(Bytes(64, 2));
  TEST_ASSERT(context.calls == 2 && !context.posted);
  TEST_ASSERT(f.out.active && f.out.starts.size() == 3);
}

void TestCloseMakesLentSlotsStale()
{
  Fixture f;
  Recorder rec;
  f.in.SetOnComplete(rec.Bind());
  TEST_ASSERT(f.in.Configure({Direction::IN, Type::BULK, 64}, false) == ErrorCode::OK);
  Slot held, spare;
  TEST_ASSERT(f.in.AcquireSlot(held, false) == ErrorCode::OK);
  f.in.Close(false);
  TEST_ASSERT(f.in.AcquireSlot(spare, false) == ErrorCode::STATE_ERR);
  TEST_ASSERT(f.in.Configure({Direction::IN, Type::BULK, 64}, false) == ErrorCode::OK);
  // The new binding only gets the free slot.
  TEST_ASSERT(f.in.AcquireSlot(spare, false) == ErrorCode::OK);
  TEST_ASSERT(spare.index != held.index);
  Slot none;
  TEST_ASSERT(f.in.AcquireSlot(none, false) == ErrorCode::BUSY);
  // The old holder's submit is refused synchronously and frees the slot.
  TEST_ASSERT(f.in.Submit(held, 4, false, false) == ErrorCode::ABORTED);
  TEST_ASSERT(rec.list.empty() && f.in.starts.empty());
  TEST_ASSERT(f.in.AcquireSlot(none, false) == ErrorCode::OK && none.index == held.index);
  // A stale OUT lease returns ABORTED too.
  TEST_ASSERT(f.out.Configure({Direction::OUT, Type::BULK, 64}, false) == ErrorCode::OK);
  Recorder out_rec;
  f.out.SetOnComplete(out_rec.Bind());
  TEST_ASSERT(f.out.Receive(64, false) == ErrorCode::OK);
  f.out.HostSends(Bytes(5, 0));
  const Slot lease = out_rec.list.at(0).slot;
  f.out.Close(false);
  TEST_ASSERT(f.out.ReturnSlot(lease, false) == ErrorCode::ABORTED);
  TEST_ASSERT(f.out.ReturnSlot(lease, false) == ErrorCode::STATE_ERR);
}

void TestCloseWithPendingStop()
{
  Fixture f;
  Recorder rec;
  f.in.SetOnComplete(rec.Bind());
  f.in.pending_stop = true;
  f.in.reap_actual = 0;
  TEST_ASSERT(f.in.Configure({Direction::IN, Type::BULK, 64}, false) == ErrorCode::OK);
  Slot a, b;
  TEST_ASSERT(f.in.AcquireSlot(a, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.AcquireSlot(b, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.Submit(a, 8, false, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.Submit(b, 9, false, false) == ErrorCode::OK);
  f.in.Close(false);
  // The queued request ends only after the in-flight one: completions keep the order.
  TEST_ASSERT(f.in.IsStopping() && f.pool.AnyStopping() && rec.list.empty());
  TEST_ASSERT(f.in.closes == 0);
  f.in.ConfirmStop();
  TEST_ASSERT(!f.in.IsStopping() && !f.pool.AnyStopping());
  TEST_ASSERT(rec.list.size() == 2 && rec.list[0].result == ErrorCode::ABORTED &&
              rec.list[1].result == ErrorCode::ABORTED && f.in.closes == 1);
  // The slots are free for the next binding.
  TEST_ASSERT(f.in.Configure({Direction::IN, Type::BULK, 64}, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.AcquireSlot(a, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.AcquireSlot(b, false) == ErrorCode::OK);
  f.in.ReleaseSlot(a, false);
  f.in.ReleaseSlot(b, false);

  // A hardware completion that arrives before the stop confirmation wins.
  rec.list.clear();
  TEST_ASSERT(f.in.AcquireSlot(a, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.Submit(a, 8, false, false) == ErrorCode::OK);
  f.in.Close(false);
  TEST_ASSERT(f.in.IsStopping());
  f.in.Complete();
  TEST_ASSERT(rec.list.size() == 1 && rec.list[0].result == ErrorCode::OK);
  f.in.stop_requested = true;
  f.in.ConfirmStop();
  TEST_ASSERT(rec.list.size() == 1 && !f.pool.AnyStopping() && f.in.closes == 2);

  // A retired IN request reaped after the stop is reported OK with its bytes.
  rec.list.clear();
  TEST_ASSERT(f.in.Configure({Direction::IN, Type::BULK, 64}, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.AcquireSlot(a, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.Submit(a, 8, false, false) == ErrorCode::OK);
  f.in.reap_retired = true;
  f.in.reap_actual = 8;
  f.in.Close(false);
  f.in.ConfirmStop();
  TEST_ASSERT(rec.list.size() == 1 && rec.list[0].result == ErrorCode::OK &&
              rec.list[0].actual == 8);

  // OUT data that the stop overtakes is not handed to anybody.
  Recorder out_rec;
  f.out.SetOnComplete(out_rec.Bind());
  f.out.pending_stop = true;
  TEST_ASSERT(f.out.Configure({Direction::OUT, Type::BULK, 64}, false) == ErrorCode::OK);
  TEST_ASSERT(f.out.Receive(64, false) == ErrorCode::OK);
  f.out.Close(false);
  f.out.HostSends(Bytes(7, 1));
  TEST_ASSERT(out_rec.list.size() == 1 && out_rec.list[0].result == ErrorCode::ABORTED &&
              out_rec.list[0].actual == 7 && !out_rec.list[0].slot.Valid());
  f.out.stop_requested = true;
  f.out.ConfirmStop();
}

void TestLateCompletionDropped()
{
  Fixture f;
  Recorder rec;
  f.in.SetOnComplete(rec.Bind());
  TEST_ASSERT(f.in.Configure({Direction::IN, Type::BULK, 64}, false) == ErrorCode::OK);
  Slot slot;
  TEST_ASSERT(f.in.AcquireSlot(slot, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.Submit(slot, 8, false, false) == ErrorCode::OK);
  const uint32_t old_tag = f.in.Last().tag;
  f.in.Flush(false);
  TEST_ASSERT(rec.list.size() == 1 && rec.list[0].result == ErrorCode::ABORTED);
  // The old request's completion after its stop must not end the next request.
  TEST_ASSERT(f.in.AcquireSlot(slot, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.Submit(slot, 5, false, false) == ErrorCode::OK);
  f.in.Late(old_tag, 8);
  TEST_ASSERT(rec.list.size() == 1 && f.in.active);
  f.in.Complete();
  TEST_ASSERT(rec.list.size() == 2 && rec.list[1].result == ErrorCode::OK &&
              rec.list[1].actual == 5);
}

void TestStall()
{
  Fixture f;
  Recorder rec;
  f.in.SetOnComplete(rec.Bind());
  f.in.pending_stop = true;
  TEST_ASSERT(f.in.Configure({Direction::IN, Type::BULK, 64}, false) == ErrorCode::OK);
  Slot a, b;
  TEST_ASSERT(f.in.AcquireSlot(a, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.Submit(a, 8, false, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.Stall(false) == ErrorCode::OK);
  TEST_ASSERT(f.in.IsStalled() && f.in.stalls == 0);
  f.in.ConfirmStop();
  TEST_ASSERT(f.in.stalls == 1 && rec.list.size() == 1 &&
              rec.list[0].result == ErrorCode::ABORTED);
  // Requests submitted while halted wait for the clear.
  TEST_ASSERT(f.in.AcquireSlot(b, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.Submit(b, 3, false, false) == ErrorCode::OK);
  TEST_ASSERT(!f.in.active);
  TEST_ASSERT(f.in.ClearStall(false) == ErrorCode::OK);
  TEST_ASSERT(f.in.clear_stalls == 1 && f.in.active && f.in.Last().length == 3);
  f.in.Complete();
  TEST_ASSERT(rec.list.size() == 2 && rec.list[1].result == ErrorCode::OK);
}

void TestStaleSlotBlocksOnlyItself()
{
  // A slot lent across a rebinding stays its holder's; the other slot serves the new
  // binding at once, and the stale one comes back when its holder returns it.
  Block<256> storage;
  EndpointPool pool{2};
  MockEndpoint ep{Number::EP1, Direction::IN, storage.Raw()};
  TEST_ASSERT(pool.Put(&ep) == ErrorCode::OK);
  TEST_ASSERT(ep.Configure({Direction::IN, Type::BULK, 64}, false) == ErrorCode::OK);
  Slot a, b;
  TEST_ASSERT(ep.AcquireSlot(a, false) == ErrorCode::OK && a.index == 0);
  TEST_ASSERT(ep.AcquireSlot(b, false) == ErrorCode::OK && b.index == 1);
  ep.ReleaseSlot(a, false);
  ep.Close(false);
  // A different packet size in the new binding does not move the slots.
  TEST_ASSERT(ep.Configure({Direction::IN, Type::ISOCHRONOUS, 100}, false) ==
              ErrorCode::OK);
  TEST_ASSERT(ep.GetGranted().slot_capacity == 100);
  Slot c, d;
  TEST_ASSERT(ep.AcquireSlot(c, false) == ErrorCode::OK && c.index == 0);
  TEST_ASSERT(c.data.addr_ == a.data.addr_);
  TEST_ASSERT(ep.AcquireSlot(d, false) == ErrorCode::BUSY);
  TEST_ASSERT(ep.Submit(b, 1, false, false) == ErrorCode::ABORTED);
  TEST_ASSERT(ep.AcquireSlot(d, false) == ErrorCode::OK && d.index == 1);
  TEST_ASSERT(d.data.addr_ == b.data.addr_);
}

// Random interleavings of holders, owner and hardware on one endpoint. The oracle is
// the model in verify/usb_ep_model.py: every accepted request completes once, in
// submission order within a binding; hardware only uses in-flight slots; no slot leaks.
void TestRandomModel(Direction direction, uint32_t seed)
{
  Block<128> storage;
  EndpointPool pool{2};
  MockEndpoint ep{Number::EP1, direction, storage.Raw()};
  TEST_ASSERT(pool.Put(&ep) == ErrorCode::OK);
  const bool in = direction == Direction::IN;
  struct Track
  {
    std::deque<uint32_t> expected;  // Accepted request ids in submission order.
    std::vector<Slot> held;
    std::vector<Slot> stale;
    uint32_t next_id = 1;
    uint32_t completed = 0;
    std::vector<uint32_t> slot_id = std::vector<uint32_t>(2, 0);
    bool in = true;
  } track;
  track.in = in;
  struct Context
  {
    Track* track;
    MockEndpoint* ep;
  } ctx{&track, &ep};
  ep.SetOnComplete(Callback<const Completion&>::Create(
      [](bool, Context* ctx, const Completion& c)
      {
        TEST_ASSERT(!ctx->track->expected.empty());
        ctx->track->expected.pop_front();
        ++ctx->track->completed;
        if (c.slot.Valid())
        {
          TEST_ASSERT(!ctx->track->in && c.result == ErrorCode::OK);
          ctx->track->held.push_back(c.slot);
        }
      },
      &ctx));
  std::mt19937 rng(seed);
  bool enabled = false;
  auto configure = [&]
  {
    TEST_ASSERT(ep.Configure({direction, Type::BULK, 64}, false) == ErrorCode::OK);
    enabled = true;
  };
  configure();
  for (unsigned step = 0; step < 200000; ++step)
  {
    const unsigned op = rng() % 100;
    if (op < 25)
    {
      if (in)
      {
        Slot slot;
        const ErrorCode r = ep.AcquireSlot(slot, false);
        TEST_ASSERT(r == ErrorCode::OK || r == ErrorCode::BUSY ||
                    (r == ErrorCode::STATE_ERR && !enabled));
        if (r == ErrorCode::OK) track.held.push_back(slot);
      }
      else
      {
        const ErrorCode r = ep.Receive(64, false);
        TEST_ASSERT(r == ErrorCode::OK || r == ErrorCode::BUSY ||
                    (r == ErrorCode::STATE_ERR && !enabled));
        if (r == ErrorCode::OK) track.expected.push_back(track.next_id++);
      }
    }
    else if (op < 50 && !track.held.empty())
    {
      const size_t pick = rng() % track.held.size();
      const Slot slot = track.held[pick];
      track.held.erase(track.held.begin() + static_cast<long>(pick));
      const bool give_back = rng() % 4 == 0;
      if (give_back)
      {
        ep.ReleaseSlot(slot, false);
        continue;
      }
      // The id is recorded before the call: the owner may complete it inside.
      track.expected.push_back(track.next_id);
      const ErrorCode r = in ? ep.Submit(slot, 1 + rng() % 64, rng() % 2, false)
                             : ep.ReturnSlot(slot, false);
      if (r == ErrorCode::OK)
        ++track.next_id;
      else
      {
        TEST_ASSERT(r == ErrorCode::ABORTED);
        track.expected.pop_back();
      }
    }
    else if (op < 75)
    {
      if (!ep.active) continue;
      if (ep.stop_requested)
      {
        if (rng() % 2) ep.ConfirmStop();
        continue;
      }
      // The hardware only ever touches an in-flight slot that no holder has.
      for (const Slot& slot : track.held)
        TEST_ASSERT(slot.data.addr_ != ep.Last().address || ep.Last().length == 0);
      if (in)
        ep.Complete();
      else
        ep.HostSends(Bytes(rng() % 65, static_cast<uint8_t>(step)));
    }
    else if (op < 80 && ep.stop_requested)
    {
      ep.ConfirmStop();
    }
    else if (op < 84)
    {
      ep.pending_stop = rng() % 2;
      if (enabled)
      {
        ep.Close(false);
        enabled = false;
      }
    }
    else if (op < 90)
    {
      if (!enabled && !ep.IsStopping()) configure();
    }
    else if (op < 92 && enabled && !ep.IsStalled())
    {
      ep.pending_stop = rng() % 2;
      TEST_ASSERT(ep.Stall(false) == ErrorCode::OK);
    }
    else if (op < 96 && enabled && ep.IsStalled() && !ep.IsStopping())
    {
      TEST_ASSERT(ep.ClearStall(false) == ErrorCode::OK);
    }
  }
  // Drain: stop pending, everything returned, and nothing is lost.
  if (ep.stop_requested) ep.ConfirmStop();
  if (enabled && ep.IsStalled()) TEST_ASSERT(ep.ClearStall(false) == ErrorCode::OK);
  for (unsigned i = 0; i < 16 && ep.active; ++i)
  {
    if (in)
      ep.Complete();
    else
      ep.HostSends({});
  }
  for (const Slot& slot : track.held) ep.ReleaseSlot(slot, false);
  track.held.clear();
  if (enabled) ep.Close(false);
  if (ep.stop_requested) ep.ConfirmStop();
  TEST_ASSERT(track.expected.empty());
  TEST_ASSERT(ep.Configure({direction, Type::BULK, 64}, false) == ErrorCode::OK);
  if (in)
  {
    Slot a, b;
    TEST_ASSERT(ep.AcquireSlot(a, false) == ErrorCode::OK);
    TEST_ASSERT(ep.AcquireSlot(b, false) == ErrorCode::OK);
  }
  else
  {
    TEST_ASSERT(ep.Receive(64, false) == ErrorCode::OK);
    TEST_ASSERT(ep.Receive(64, false) == ErrorCode::OK);
  }
  TEST_ASSERT(track.completed + 0U == track.next_id - 1U);
  std::printf("random %s seed %u: %u requests, %zu hardware starts\n", in ? "IN" : "OUT",
              seed, track.completed, ep.starts.size());
}

// Producers in two threads borrow, fill and submit while a third thread plays the
// controller IRQ. The owner runs in whichever context wins the service.
void TestThreadedProducers()
{
  Block<256> storage;
  EndpointPool pool{2};
  MockEndpoint ep{Number::EP1, Direction::IN, storage.Raw()};
  TEST_ASSERT(pool.Put(&ep) == ErrorCode::OK);
  std::mutex guard;
  pool.SetHardwareGuard(
      &guard,
      [](void* context) -> uintptr_t
      {
        static_cast<std::mutex*>(context)->lock();
        return 0U;
      },
      [](void* context, uintptr_t) { static_cast<std::mutex*>(context)->unlock(); });
  std::atomic<uint32_t> completed{0};
  ep.SetOnComplete(Callback<const Completion&>::Create(
      [](bool, std::atomic<uint32_t>* count, const Completion& c)
      {
        TEST_ASSERT(c.result == ErrorCode::OK);
        count->fetch_add(1U, std::memory_order_relaxed);
      },
      &completed));
  TEST_ASSERT(ep.Configure({Direction::IN, Type::BULK, 64}, false) == ErrorCode::OK);
  constexpr uint32_t PER_THREAD = 20000;
  std::atomic<uint32_t> submitted{0};
  auto producer = [&](uint8_t seed)
  {
    for (uint32_t i = 0; i < PER_THREAD;)
    {
      Slot slot;
      if (ep.AcquireSlot(slot, false) != ErrorCode::OK)
      {
        std::this_thread::yield();
        continue;
      }
      std::memset(slot.data.addr_, seed, 32);
      TEST_ASSERT(ep.Submit(slot, 32, false, false) == ErrorCode::OK);
      submitted.fetch_add(1U, std::memory_order_relaxed);
      ++i;
    }
  };
  std::atomic<bool> stop{false};
  std::thread irq(
      [&]
      {
        while (!stop.load(std::memory_order_acquire) ||
               completed.load(std::memory_order_relaxed) <
                   submitted.load(std::memory_order_relaxed))
        {
          bool active;
          {
            std::lock_guard<std::mutex> lock(ep.lock);
            active = ep.active;
          }
          if (!active)
          {
            std::this_thread::yield();
            continue;
          }
          EndpointPool::InterruptScope scope(pool);
          ep.Complete(true);
        }
      });
  std::thread a(producer, 0x11), b(producer, 0x22);
  a.join();
  b.join();
  stop.store(true, std::memory_order_release);
  irq.join();
  TEST_ASSERT(completed.load() == 2U * PER_THREAD);
}

// ---------------------------------------------------------------------------
// CDC
// ---------------------------------------------------------------------------

static constexpr auto TEST_LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "LibXR", "USB tests", "0");

class TestCDC : public CDCUart
{
 public:
  explicit TestCDC(size_t receive = 128)
      : CDCUart(Number::EP1, Number::EP2, Number::EP3, receive, 1024, 32)
  {
  }
  using CDCUart::BindEndpoints;
  using CDCUart::UnbindEndpoints;
};

struct CDCFixture : Fixture
{
  TestCDC cdc;
  DeviceComposition composition;  // Connects the class to the pool's service.
  explicit CDCFixture(size_t receive = 128)
      : cdc(receive), composition(pool, {&TEST_LANGUAGE}, {{&cdc}})
  {
    // Stand-in for DeviceCore's class-service dispatch.
    pool.SetControlHandler(Callback<uint32_t>::Create(
        [](bool in_isr, TestCDC* cdc, uint32_t events)
        {
          if (events & EndpointPool::CLASS_EVENT) cdc->OnService(in_isr);
        },
        &cdc));
    pool.SetDataEnabled(true, false);
  }
};

void TestCDCWrite()
{
  CDCFixture f;
  f.cdc.BindEndpoints(f.pool, 0, false);
  unsigned finished = 0;
  auto complete = Callback<ErrorCode>::Create(
      [](bool, unsigned* count, ErrorCode result)
      {
        TEST_ASSERT(result == ErrorCode::OK);
        ++*count;
      },
      &finished);
  WriteOperation op(complete);
  const auto bytes = Bytes(150, 7);
  TEST_ASSERT(f.cdc.Write({bytes.data(), bytes.size()}, op) == ErrorCode::OK);
  // Both slots are filled at once; the second waits behind the first.
  TEST_ASSERT(f.in.starts.size() == 1 && f.in.Last().length == 64 && finished == 0);
  f.in.Complete();
  TEST_ASSERT(f.in.Last().length == 64);
  f.in.Complete();
  TEST_ASSERT(f.in.Last().length == 22 && finished == 1);
  f.in.Complete();
  TEST_ASSERT(!f.in.active && Wire(f.in) == bytes);

  // A full packet with nothing behind it carries its ZLP.
  f.in.sent.clear();
  const auto full = Bytes(64, 31);
  TEST_ASSERT(f.cdc.Write({full.data(), full.size()}, op) == ErrorCode::OK);
  f.in.Complete();
  TEST_ASSERT(f.in.active && f.in.Last().length == 0);
  // Data written during the ZLP follows it.
  const auto later = Bytes(17, 84);
  TEST_ASSERT(f.cdc.Write({later.data(), later.size()}, op) == ErrorCode::OK);
  TEST_ASSERT(f.in.Last().length == 0);
  f.in.Complete();
  TEST_ASSERT(f.in.Last().length == 17);
  f.in.Complete();
  TEST_ASSERT(!f.in.active && finished == 3);

  // A write issued from its own completion callback is accepted and sent.
  struct Context
  {
    TestCDC* cdc;
    const std::vector<uint8_t>* next;
    unsigned called;
  };
  Context ctx{&f.cdc, &later, 0};
  auto recursive_callback = Callback<ErrorCode>::Create(
      [](bool in_isr, Context* ctx, ErrorCode result)
      {
        TEST_ASSERT(result == ErrorCode::OK);
        ++ctx->called;
        WriteOperation plain;
        TEST_ASSERT(ctx->cdc->Write({ctx->next->data(), ctx->next->size()}, plain,
                                    in_isr) == ErrorCode::OK);
      },
      &ctx);
  WriteOperation recursive(recursive_callback);
  f.in.sent.clear();
  TEST_ASSERT(f.cdc.Write({full.data(), full.size()}, recursive) == ErrorCode::OK);
  TEST_ASSERT(ctx.called == 1);
  while (f.in.active) f.in.Complete();
  auto expected = full;
  expected.insert(expected.end(), later.begin(), later.end());
  TEST_ASSERT(Wire(f.in) == expected);
}

void TestCDCReceiveBackpressure()
{
  CDCFixture f(16);
  f.cdc.BindEndpoints(f.pool, 0, false);
  TEST_ASSERT(f.out.active && f.out.Last().length == 64);
  const auto packet = Bytes(64, 19);
  f.out.HostSends(packet);
  // The second slot is armed; the lease stays with CDC until the reader drains it.
  TEST_ASSERT(f.out.active && f.out.starts.size() == 2);
  std::vector<uint8_t> received;
  ReadOperation op;
  for (unsigned i = 0; i < 4; ++i)
  {
    std::array<uint8_t, 16> data{};
    TEST_ASSERT(f.cdc.Read({data.data(), data.size()}, op) == ErrorCode::OK);
    received.insert(received.end(), data.begin(), data.end());
  }
  TEST_ASSERT(received == packet);
  // Drained lease returned and rearmed behind the armed slot.
  f.out.HostSends(Bytes(3, 1));
  TEST_ASSERT(f.out.active && f.out.starts.size() == 3);
}

void TestCDCReceivesOnePacketPerRequest()
{
  // Slots of several packets: a full-speed 64 B packet ends no request longer than one
  // packet, so CDC must not wait for a slot to fill before handing bytes to the reader.
  Block<128> in_storage;
  Block<1024> out_storage;
  Block<128> comm_storage;
  EndpointPool pool{4};
  MockEndpoint in{Number::EP1, Direction::IN, in_storage.Raw()};
  MockEndpoint out{Number::EP2, Direction::OUT, out_storage.Raw()};
  MockEndpoint comm{Number::EP3, Direction::IN, comm_storage.Raw()};
  TEST_ASSERT(pool.Put(&in) == ErrorCode::OK && pool.Put(&out) == ErrorCode::OK &&
              pool.Put(&comm) == ErrorCode::OK);
  TestCDC cdc;
  DeviceComposition composition(pool, {&TEST_LANGUAGE}, {{&cdc}});
  pool.SetControlHandler(Callback<uint32_t>::Create(
      [](bool in_isr, TestCDC* cdc, uint32_t events)
      {
        if (events & EndpointPool::CLASS_EVENT) cdc->OnService(in_isr);
      },
      &cdc));
  pool.SetDataEnabled(true, false);
  cdc.BindEndpoints(pool, 0, false);
  TEST_ASSERT(out.GetGranted().slot_capacity == 512);
  TEST_ASSERT(out.active && out.Last().length == 64);
  const auto packet = Bytes(64, 3);
  out.HostSends(packet);
  std::array<uint8_t, 64> data{};
  ReadOperation op;
  TEST_ASSERT(cdc.Read({data.data(), data.size()}, op) == ErrorCode::OK);
  TEST_ASSERT(std::vector<uint8_t>(data.begin(), data.end()) == packet);
  TEST_ASSERT(out.active && out.Last().length == 64);
}

void TestCDCTakesTheEndpointPacketSize()
{
  // The controller only holds 32 B packets on these endpoints: CDC binds with 32 B and
  // still moves data in both directions.
  CDCFixture f;
  f.in.limit = 32;
  f.out.limit = 32;
  f.comm.limit = 8;
  f.cdc.BindEndpoints(f.pool, 0, false);
  TEST_ASSERT(f.in.MaxPacketSize() == 32 && f.out.MaxPacketSize() == 32 &&
              f.comm.MaxPacketSize() == 8);
  TEST_ASSERT(f.out.active && f.out.Last().length == 32);
  const auto bytes = Bytes(40, 9);
  WriteOperation op;
  TEST_ASSERT(f.cdc.Write({bytes.data(), bytes.size()}, op) == ErrorCode::OK);
  while (f.in.active) f.in.Complete();
  TEST_ASSERT(Wire(f.in) == bytes);
  const auto packet = Bytes(32, 4);
  f.out.HostSends(packet);
  std::array<uint8_t, 32> data{};
  ReadOperation read;
  TEST_ASSERT(f.cdc.Read({data.data(), data.size()}, read) == ErrorCode::OK);
  TEST_ASSERT(std::vector<uint8_t>(data.begin(), data.end()) == packet);
  f.cdc.UnbindEndpoints(f.pool, false);
  // Nothing legal fits (512 B high-speed bulk in 64 B slots): Configure rejects the size.
  f.pool.SetSpeed(Speed::HIGH);
  f.in.limit = f.out.limit = f.comm.limit = UINT16_MAX;
  f.cdc.BindEndpoints(f.pool, 0, false);
  TEST_ASSERT(f.in.GetState() == State::ERROR && f.out.GetState() == State::ERROR);
}

void TestCDCRebind()
{
  CDCFixture f;
  // Written before configuration: sent after binding.
  const auto early = Bytes(20, 5);
  WriteOperation plain;
  TEST_ASSERT(f.cdc.Write({early.data(), early.size()}, plain) == ErrorCode::OK);
  TEST_ASSERT(f.in.starts.empty());
  f.cdc.BindEndpoints(f.pool, 0, false);
  TEST_ASSERT(f.in.active && f.in.Last().length == 20);
  // A lease held across an unbind is freed; in-flight data is dropped.
  f.out.HostSends(Bytes(64, 1));
  f.cdc.UnbindEndpoints(f.pool, false);
  TEST_ASSERT(!f.in.active && !f.out.active);
  f.cdc.BindEndpoints(f.pool, 0, false);
  // Both slots are armed again: one on the wire, one queued behind it.
  TEST_ASSERT(f.out.starts.size() == 3 && f.out.active);
  f.out.HostSends(Bytes(1, 0));
  TEST_ASSERT(f.out.starts.size() == 4 && f.out.active);
  TEST_ASSERT(f.in.starts.size() == 1);
}

// ---------------------------------------------------------------------------
// Device core and EP0
// ---------------------------------------------------------------------------

struct BasicCapabilities : FullSpeedCapabilities
{
  static constexpr bool BOS = false;
  static constexpr bool BUS_TIME = false;
};

class ControlClass : public DeviceClass
{
 public:
  std::array<uint8_t, 256> input{}, output{};
  size_t response_size = 0, data_count = 0, complete_count = 0, abort_count = 0;
  size_t last_actual = 0;
  bool reject_data = false;
  std::vector<DeviceEvent> events;
  size_t binds = 0;  ///< 非规划模式的绑定次数 / Binds outside planning
  InterfaceDescriptor interface{};
  size_t GetControlReceiveCapacity() const override { return output.size(); }
  size_t GetInterfaceCount() override { return 1; }
  size_t GetMaxConfigSize() override { return sizeof(interface); }
  bool HasIAD() override { return false; }
  void BindEndpoints(EndpointPool& pool, uint8_t number, bool) override
  {
    if (!pool.IsPlanning()) ++binds;
    interface = {9, 4, number, 0, 0, 0xff, 0, 0, 0};
    SetData({&interface, sizeof(interface)});
  }
  void UnbindEndpoints(EndpointPool&, bool) override {}
  void OnDeviceEvent(bool, DeviceEvent event, uint8_t) override
  {
    events.push_back(event);
  }

 protected:
  ErrorCode OnControlRequest(bool, const SetupPacket& setup,
                             ControlTransferResult& result) override
  {
    if (setup.bRequest == 0x70 && (setup.bmRequestType & 0x80U))
      result.InData() = {input.data(), response_size};
    else if (setup.bRequest == 0x71 && !(setup.bmRequestType & 0x80U))
      return ErrorCode::OK;
    else if (setup.bRequest != 0x72 || setup.wLength != 0U)
      return ErrorCode::NOT_SUPPORT;
    return ErrorCode::OK;
  }
  ErrorCode OnControlData(bool, const SetupPacket& setup, ConstRawData& data) override
  {
    if (!(setup.bmRequestType & 0x80U))
    {
      ++data_count;
      last_actual = data.size_;
      std::memcpy(output.data(), data.addr_, data.size_);
    }
    return reject_data ? ErrorCode::ARG_ERR : ErrorCode::OK;
  }
  void OnControlComplete(bool, const SetupPacket&) override { ++complete_count; }
  void OnControlAbort(bool, const SetupPacket&) override { ++abort_count; }
};

class TestDevice : public DeviceCore<BasicCapabilities>
{
 public:
  TestDevice(EndpointPool& pool, DeviceClass& klass)
      : DeviceCore(pool, USBSpec::USB_2_0, Speed::FULL,
                   DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x4321, 0x0100,
                   {&TEST_LANGUAGE}, {{&klass}})
  {
  }
  void Start(bool) override {}
  void Stop(bool) override {}
  std::vector<ControlContext> address_phases;
  unsigned stop_timeouts = 0;

 protected:
  ErrorCode SetAddress(uint8_t, ControlContext context) override
  {
    address_phases.push_back(context);
    return ErrorCode::OK;
  }
  void OnStopTimeout(bool) override { ++stop_timeouts; }
};

struct ControlFixture
{
  Block<128> in_storage, out_storage, comm_storage;
  Block<64> ep0_in_storage, ep0_out_storage;
  EndpointPool pool{4};
  MockEndpoint in0{Number::EP0, Direction::IN, ep0_in_storage.Raw()};
  MockEndpoint out0{Number::EP0, Direction::OUT, ep0_out_storage.Raw()};
  void Setup(TestDevice& device, uint8_t kind, uint8_t request, uint16_t value,
             uint16_t length, uint16_t index = 0)
  {
    // A SETUP cancels the old hardware control transfer independently from software.
    in0.active = false;
    out0.active = false;
    SetupPacket packet{kind, request, value, index, length};
    device.OnSetupPacket(true, &packet);
  }
};

void TestControlTransactions()
{
  ControlFixture f;
  f.pool.SetEndpoint0(&f.in0, &f.out0);
  ControlClass klass;
  TestDevice device(f.pool, klass);
  device.Init(false);
  TEST_ASSERT(device.IsInited() && klass.events.empty());
  f.Setup(device, 0x80, 8, 0, 1);  // GET_CONFIGURATION
  TEST_ASSERT(f.in0.Last().data == std::vector<uint8_t>{0});
  f.in0.Complete();
  f.out0.HostSends({});
  f.Setup(device, 0, 9, 1, 0);  // SET_CONFIGURATION 1
  TEST_ASSERT(klass.events.empty() && klass.binds == 1);
  TEST_ASSERT(f.in0.active && f.in0.Last().length == 0);
  f.in0.Complete();

  // Empty IN is data, not "no action" or a STALL.
  klass.response_size = 0;
  f.Setup(device, 0xa1, 0x70, 0, 8);
  TEST_ASSERT(f.in0.active && f.in0.Last().length == 0 && klass.complete_count == 0);
  f.in0.Complete();
  TEST_ASSERT(klass.complete_count == 0);
  f.out0.HostSends({});
  TEST_ASSERT(klass.complete_count == 1);

  // Full-packet short response needs one real zero packet, then true status.
  klass.response_size = 64;
  for (unsigned i = 0; i < 64; ++i) klass.input[i] = static_cast<uint8_t>(i);
  f.Setup(device, 0xa1, 0x70, 0, 128);
  f.in0.Complete();
  TEST_ASSERT(f.in0.active && f.in0.Last().length == 0);
  f.out0.HostSends({});
  TEST_ASSERT(klass.complete_count == 1);
  f.in0.Complete();
  TEST_ASSERT(klass.complete_count == 2);

  // OUT data stage across packets; the class gets the whole stage once.
  f.Setup(device, 0x21, 0x71, 0, 150);
  auto a = Bytes(64, 1), b = Bytes(64, 65), c = Bytes(22, 129);
  f.out0.HostSends(a);
  f.out0.HostSends(b);
  TEST_ASSERT(klass.data_count == 0 && klass.complete_count == 2);
  f.out0.HostSends(c);
  TEST_ASSERT(klass.data_count == 1 && klass.last_actual == 150);
  TEST_ASSERT(std::equal(a.begin(), a.end(), klass.output.begin()) &&
              std::equal(c.begin(), c.end(), klass.output.begin() + 128));
  TEST_ASSERT(klass.complete_count == 2 && f.in0.active && f.in0.Last().length == 0);
  f.in0.Complete();
  TEST_ASSERT(klass.complete_count == 3);

  // A handler rejection is not acknowledged as a successful status stage.
  klass.reject_data = true;
  f.Setup(device, 0x21, 0x71, 0, 7);
  f.out0.HostSends(Bytes(7, 17));
  TEST_ASSERT(f.in0.IsStalled() && f.out0.IsStalled() && klass.complete_count == 3);
  TEST_ASSERT(klass.abort_count == 1);
  klass.reject_data = false;

  f.Setup(device, 0x21, 0x71, 0, 150);
  f.out0.HostSends(a);
  f.Setup(device, 0x21, 0x72, 0, 0);  // New SETUP cancels unfinished data.
  TEST_ASSERT(klass.abort_count == 2 && klass.data_count == 2);
  f.in0.Complete();
  TEST_ASSERT(klass.complete_count == 4);

  // SET_ADDRESS: status completion applies the address even when the next SETUP
  // arrives in the same IRQ batch.
  device.address_phases.clear();
  f.Setup(device, 0, 9, 0, 0);  // Deconfigure first; SET_ADDRESS needs config 0.
  f.in0.Complete();
  f.Setup(device, 0, 5, 7, 0);
  TEST_ASSERT(f.in0.active && f.in0.Last().length == 0);
  {
    EndpointPool::InterruptScope batch(f.pool);
    f.in0.Complete(true);
    SetupPacket next{0x80, 8, 0, 0, 1};
    device.OnSetupPacket(true, &next);
  }
  TEST_ASSERT(device.address_phases.size() == 3 &&
              device.address_phases.back() == ControlContext::STATUS_IN_COMPLETE);
  TEST_ASSERT(f.in0.active && f.in0.Last().data == std::vector<uint8_t>{0});

  device.OnSuspend(true);
  TEST_ASSERT(device.IsInited());
  device.OnResume(true);
  device.OnBusReset(true);
  TEST_ASSERT(device.IsInited() && klass.events.back() == DeviceEvent::RESET);
}

// A data endpoint whose stop needs a hardware event holds back the reset and the
// SET_CONFIGURATION status stage until the stop is confirmed.
void TestStopHoldsLifecycle()
{
  ControlFixture f;
  MockEndpoint in{Number::EP1, Direction::IN, f.in_storage.Raw()};
  MockEndpoint out{Number::EP2, Direction::OUT, f.out_storage.Raw()};
  MockEndpoint comm{Number::EP3, Direction::IN, f.comm_storage.Raw()};
  TEST_ASSERT(f.pool.Put(&in) == ErrorCode::OK);
  TEST_ASSERT(f.pool.Put(&out) == ErrorCode::OK);
  TEST_ASSERT(f.pool.Put(&comm) == ErrorCode::OK);
  f.pool.SetEndpoint0(&f.in0, &f.out0);
  TestCDC cdc;
  TestDevice device(f.pool, cdc);
  device.Init(false);
  f.Setup(device, 0, 9, 1, 0);
  f.in0.Complete();
  TEST_ASSERT(out.active && out.Last().length == 64);

  // SET_LINE_CODING with its data stage, then GET_LINE_CODING.
  const uint8_t coding[7] = {0x00, 0xC2, 0x01, 0x00, 0, 0, 8};  // 115200 8N1
  f.Setup(device, 0x21, 0x20, 0, 7, 0);
  f.out0.HostSends(std::vector<uint8_t>(coding, coding + 7));
  TEST_ASSERT(f.in0.active && f.in0.Last().length == 0 && !f.in0.IsStalled());
  f.in0.Complete();
  f.Setup(device, 0xA1, 0x21, 0, 7, 0);
  TEST_ASSERT(f.in0.Last().data == std::vector<uint8_t>(coding, coding + 7));
  f.in0.Complete();
  f.out0.HostSends({});

  // Bulk IN in flight with a slow stop; the host resets the bus.
  const auto bytes = Bytes(40, 3);
  WriteOperation plain;
  TEST_ASSERT(cdc.Write({bytes.data(), bytes.size()}, plain) == ErrorCode::OK);
  TEST_ASSERT(in.active);
  in.pending_stop = true;
  device.OnBusReset(true);
  TEST_ASSERT(in.stop_requested && !device.IsInited());
  // SETUP arriving meanwhile waits.
  SetupPacket get_config{0x80, 8, 0, 0, 1};
  device.OnSetupPacket(true, &get_config);
  TEST_ASSERT(!device.IsInited() && !f.in0.active);
  in.ConfirmStop();
  TEST_ASSERT(device.IsInited() && in.closes >= 1);
  in.pending_stop = false;

  // Configure again; in flight again; SET_CONFIGURATION 0 with a slow stop answers
  // only after the stop is confirmed.
  f.Setup(device, 0, 9, 1, 0);
  f.in0.Complete();
  TEST_ASSERT(cdc.Write({bytes.data(), bytes.size()}, plain) == ErrorCode::OK);
  TEST_ASSERT(in.active);
  in.pending_stop = true;
  f.Setup(device, 0, 9, 0, 0);
  TEST_ASSERT(in.stop_requested && !f.in0.active);
  in.ConfirmStop();
  TEST_ASSERT(f.in0.active && f.in0.Last().length == 0);
  f.in0.Complete();
  in.pending_stop = false;

  // A stop still pending when the host resets the bus: the reset waits, no endpoint
  // starts meanwhile, and after STOP_FRAME_LIMIT frames the backend is asked to reset.
  f.Setup(device, 0, 9, 1, 0);
  f.in0.Complete();
  TEST_ASSERT(out.active);
  TEST_ASSERT(cdc.Write({bytes.data(), bytes.size()}, plain) == ErrorCode::OK);
  TEST_ASSERT(in.active);
  in.pending_stop = true;
  f.Setup(device, 0x02, 3, 0, 0, 0x81);  // SET_FEATURE(ENDPOINT_HALT) on EP1 IN
  TEST_ASSERT(in.stop_requested && f.in0.active);
  f.in0.Complete();
  device.OnBusReset(true);
  const size_t out_starts = out.starts.size();
  out.HostSends(Bytes(64, 9));
  TEST_ASSERT(!out.active && out.starts.size() == out_starts);
  // The limit counts 1 ms frames: the eight microframes of one high-speed frame are one.
  for (uint8_t microframe = 0; microframe < 8U; ++microframe)
    device.OnSof(true, 100, microframe);
  TEST_ASSERT(device.stop_timeouts == 0);
  for (uint16_t frame = 101; frame < 100U + EndpointPool::STOP_FRAME_LIMIT; ++frame)
    device.OnSof(true, frame);
  TEST_ASSERT(device.stop_timeouts == 1);
  device.OnSof(true, 200);
  TEST_ASSERT(device.stop_timeouts == 1);
  in.ConfirmStop();
  TEST_ASSERT(device.IsInited());
}

// A queued request whose early start fails completes after the request that just
// finished.
void TestStartFailureOrder()
{
  Fixture f;
  Recorder rec;
  f.in.SetOnComplete(rec.Bind());
  TEST_ASSERT(f.in.Configure({Direction::IN, Type::BULK, 64}, false) == ErrorCode::OK);
  Slot a, b;
  TEST_ASSERT(f.in.AcquireSlot(a, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.AcquireSlot(b, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.Submit(a, 5, false, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.Submit(b, 6, false, false) == ErrorCode::OK);
  f.in.fail_next_start = true;
  f.in.Complete();
  TEST_ASSERT(rec.list.size() == 2 && rec.list[0].result == ErrorCode::OK &&
              rec.list[0].actual == 5 && rec.list[1].result == ErrorCode::FAILED);
  TEST_ASSERT(!f.in.active);
  TEST_ASSERT(f.in.AcquireSlot(a, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.AcquireSlot(b, false) == ErrorCode::OK);
}
}  // namespace

int main()
{
  TestGeometry();
  TestInBorrowOrder();
  TestZeroLengthTailAndShort();
  TestOutLease();
  TestQueueFromOwnCallbackPostsNothing();
  TestCloseMakesLentSlotsStale();
  TestCloseWithPendingStop();
  TestLateCompletionDropped();
  TestStall();
  TestStaleSlotBlocksOnlyItself();
  for (uint32_t seed = 1; seed <= 4; ++seed)
  {
    TestRandomModel(Direction::IN, seed);
    TestRandomModel(Direction::OUT, seed);
  }
  TestThreadedProducers();
  TestCDCWrite();
  TestCDCReceiveBackpressure();
  TestCDCReceivesOnePacketPerRequest();
  TestCDCTakesTheEndpointPacketSize();
  TestCDCRebind();
  TestControlTransactions();
  TestStopHoldsLifecycle();
  TestStartFailureOrder();
  std::puts("USB endpoint slots, stop, CDC and EP0 lifecycle passed");
}
