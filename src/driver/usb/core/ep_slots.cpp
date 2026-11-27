#include "ep_slots.hpp"

using namespace LibXR::USB;

// 内存序 / Memory ordering
//
// 成功的比较交换一律用 acq_rel：交回（Queue、Release、FinishFree）以 release 发布调用者
// 之前对槽和每槽记录的读写，取得（借出、认领、TakeNext、FinishLent）以 acquire 看到
// 上一方的读写。失败时与循环前的读取用 relaxed：读到的值只用来计算下一次比较交换，或者
// 返回错误码而不使用任何槽数据。TakeNext 例外：它在比较交换之前就要读排队次序，所以读取
// 和失败都用 acquire。
// Every successful compare-and-swap is acq_rel: a hand-back (Queue, Release,
// FinishFree) publishes with release what the caller did to the slot and to its record,
// and a take (lend, claim, TakeNext, FinishLent) sees with acquire what the previous
// party did. The load before the loop and the failure order are relaxed: the value only
// feeds the next compare-and-swap or an error return that uses no slot data. TakeNext is
// the exception: it reads the queueing order before its compare-and-swap, so its load
// and its failure order are acquire.
//
// 戳不在状态字里。它只在一次借出结束时推进，由结束这次借出的一方（持有者在 Queue 或
// Release 里，owner 在 FinishFree 里不需要，因为在途的槽没有句柄）在改变槽状态之前写入：
// 所以槽处于 LENT 或 STALE 时，它的戳就是当前这次借出的戳，别人不会改。戳的读写用
// acquire 和 release：过期句柄在任何上下文里读到的都不会是它自己的旧戳。
// The stamps are not in the state word. A stamp advances only when a lend ends, written
// by the party that ends it (the holder in Queue or Release) before the slot state
// changes: so while a slot is LENT or STALE its stamp is that of the current lend and
// nobody else changes it. Stamps are read with acquire and written with release, so an
// expired handle never reads its own old stamp in any context.

EndpointSlots::EndpointSlots(uint8_t slot_count)
    : slot_count_(slot_count), records_(new Record[slot_count])
{
  ASSERT(slot_count >= 1U && slot_count <= MAX_SLOTS);
}

EndpointSlots::~EndpointSlots() { delete[] records_; }

void EndpointSlots::Enable(bool sending)
{
  uint32_t old = word_.load(std::memory_order_relaxed);
  uint32_t next;
  do
  {
    ASSERT((old & ENABLED_BIT) == 0U);
    // 作废的槽、上一次绑定被停的在途槽、排队序号和所有戳原样保留。
    // Stale slots, the stopped in-flight slots of the previous binding, the queueing
    // sequence and all stamps are kept as they are.
    next = (old & ~(ENABLED_BIT | SENDING_BIT)) | ENABLED_BIT;
    if (sending)
    {
      next |= SENDING_BIT;
    }
  } while (!word_.compare_exchange_weak(old, next, std::memory_order_acq_rel,
                                        std::memory_order_relaxed));
}

uint8_t EndpointSlots::Disable()
{
  uint32_t old = word_.load(std::memory_order_relaxed);
  uint32_t next;
  uint8_t freed;
  do
  {
    next = old & ~ENABLED_BIT;
    freed = 0U;
    for (uint8_t i = 0U; i < slot_count_; ++i)
    {
      const uint32_t state = StateOf(old, i);
      if (state == QUEUED)
      {
        next = WithState(next, i, FREE);
        ++freed;
      }
      else if (state == LENT)
      {
        next = WithState(next, i, STALE);
      }
    }
  } while (!word_.compare_exchange_weak(old, next, std::memory_order_acq_rel,
                                        std::memory_order_relaxed));
  return freed;
}

uint8_t EndpointSlots::DropQueued()
{
  uint32_t old = word_.load(std::memory_order_relaxed);
  uint32_t next;
  uint8_t freed;
  do
  {
    next = old;
    freed = 0U;
    for (uint8_t i = 0U; i < slot_count_; ++i)
    {
      if (StateOf(old, i) == QUEUED)
      {
        next = WithState(next, i, FREE);
        ++freed;
      }
    }
  } while (!word_.compare_exchange_weak(old, next, std::memory_order_acq_rel,
                                        std::memory_order_relaxed));
  return freed;
}

LibXR::ErrorCode EndpointSlots::LendForSending(Lend& lend)
{
  return LendFree(true, lend);
}

LibXR::ErrorCode EndpointSlots::ClaimForReceiving(Lend& lend)
{
  return LendFree(false, lend);
}

LibXR::ErrorCode EndpointSlots::Queue(const Lend& lend, const Request& request,
                                      bool sending, size_t max_length)
{
  if (!Holds(lend))
  {
    return LibXR::ErrorCode::STATE_ERR;
  }
  const uint8_t index = lend.index;
  Record& record = records_[index];
  bool ended = false;
  uint32_t old = word_.load(std::memory_order_relaxed);
  while (true)
  {
    const uint32_t state = StateOf(old, index);
    if (state != STALE && (state != LENT || ((old & SENDING_BIT) != 0U) != sending))
    {
      // 句柄相符而槽不是 LENT 或 STALE，或方向不符：这次借出没有结束，戳不动。
      // The handle matches but the slot is neither LENT nor STALE, or the direction
      // differs: the lend has not ended and the stamp is left alone.
      ASSERT(!ended);
      return LibXR::ErrorCode::STATE_ERR;
    }
    if (state == STALE)
    {
      if (!ended)
      {
        record.stamp.store(lend.stamp + 1U, std::memory_order_release);
        ended = true;
      }
      if (word_.compare_exchange_weak(old, WithState(old, index, FREE),
                                      std::memory_order_acq_rel,
                                      std::memory_order_relaxed))
      {
        return LibXR::ErrorCode::ABORTED;
      }
      continue;
    }
    if (!ended && request.length > max_length)
    {
      // 非作废槽长度越界：保持 LENT 且不推进戳，调用者可继续释放或按短长度重提。
      // Oversized length on a valid slot: keep it LENT and leave the stamp intact.
      return LibXR::ErrorCode::SIZE_ERR;
    }
    if (!ended)
    {
      // 这次借出到此结束：先推进戳，再改变槽状态。之后只会在 LENT 与 STALE
      // 之间变（停用），两种情况都由下面的比较交换收尾。
      // The lend ends here: the stamp advances before the slot state changes. From now on
      // the slot can only move from LENT to STALE (a disable), and either case is
      // finished by the compare-and-swap below.
      record.stamp.store(lend.stamp + 1U, std::memory_order_release);
      ended = true;
    }
    // 槽是 LENT 且戳相符，只有持有者能改变它；release 使这些写先于 owner 在 TakeNext
    // 中的读取。排队序号取自这次比较交换所基于的状态字，所以先后与比较交换的先后一致。
    // The slot is LENT under this stamp, so only the holder can change it; the release
    // below orders these writes before the owner's reads in TakeNext. The queueing
    // sequence comes from the very word this compare-and-swap is based on, so the order
    // equals the order of the compare-and-swaps.
    record.request = request;
    record.order = static_cast<uint8_t>((old >> SEQUENCE_SHIFT) & SEQUENCE_MASK);
    if (word_.compare_exchange_weak(old, MarkQueued(old, index),
                                    std::memory_order_acq_rel, std::memory_order_relaxed))
    {
      return LibXR::ErrorCode::OK;
    }
  }
}

LibXR::ErrorCode EndpointSlots::Release(const Lend& lend)
{
  if (!Holds(lend))
  {
    return LibXR::ErrorCode::STATE_ERR;
  }
  const uint8_t index = lend.index;
  uint32_t old = word_.load(std::memory_order_relaxed);
  uint32_t state = StateOf(old, index);
  if (state != LENT && state != STALE)
  {
    return LibXR::ErrorCode::STATE_ERR;
  }
  // 这次借出到此结束：先推进戳，再把槽变空闲。持有期间槽只会在 LENT 与 STALE 之间变。
  // The lend ends here: the stamp advances, then the slot becomes free. While held the
  // slot can only move from LENT to STALE.
  records_[index].stamp.store(lend.stamp + 1U, std::memory_order_release);
  while (!word_.compare_exchange_weak(old, WithState(old, index, FREE),
                                      std::memory_order_acq_rel,
                                      std::memory_order_relaxed))
  {
    state = StateOf(old, index);
    ASSERT(state == LENT || state == STALE);
  }
  return LibXR::ErrorCode::OK;
}

EndpointSlots::Queued EndpointSlots::TakeNext()
{
  uint32_t old = word_.load(std::memory_order_acquire);
  uint8_t pick;
  do
  {
    // 排队最久的那个：它的序号离当前序号最远。同时排队的槽不超过 MAX_SLOTS 个，一个槽
    // 排队期间别的槽各自最多再排一次，所以真实的距离小于序号的周期；序号回绕过的槽
    // （序号比当前的大）一定更早，无符号相减正好让它的差更大，不需要再取模。
    // The one queued longest ago: its sequence is the farthest behind the current one.
    // At most MAX_SLOTS slots are queued at once and while one waits each other slot is
    // queued once more at most, so the real distance stays below the period of the
    // sequence; a slot queued before the sequence wrapped (its sequence is above the
    // current one) is always the earlier, and the unsigned subtraction gives it the
    // larger difference without a modulo.
    const uint32_t now = (old >> SEQUENCE_SHIFT) & SEQUENCE_MASK;
    uint32_t oldest = 0U;
    pick = NO_SLOT;
    for (uint8_t i = 0U; i < slot_count_; ++i)
    {
      if (StateOf(old, i) != QUEUED)
      {
        continue;
      }
      const uint32_t age = now - records_[i].order;
      if (pick == NO_SLOT || age > oldest)
      {
        pick = i;
        oldest = age;
      }
    }
    if (pick == NO_SLOT)
    {
      return {};
    }
  } while (!word_.compare_exchange_weak(old, WithState(old, pick, IN_FLIGHT),
                                        std::memory_order_acq_rel,
                                        std::memory_order_acquire));
  // acquire 读到了持有者以 release 写下的 QUEUED，它在 Queue 中对每槽记录的写入可见。
  // The acquire saw the QUEUED state the holder released, so its writes to the slot's
  // record in Queue are visible.
  return {pick, records_[pick].request};
}

void EndpointSlots::FinishFree(uint8_t index)
{
  uint32_t old = word_.load(std::memory_order_relaxed);
  do
  {
    ASSERT(index < slot_count_ && StateOf(old, index) == IN_FLIGHT);
  } while (!word_.compare_exchange_weak(old, WithState(old, index, FREE),
                                        std::memory_order_acq_rel,
                                        std::memory_order_relaxed));
}

uint32_t EndpointSlots::FinishLent(uint8_t index)
{
  uint32_t old = word_.load(std::memory_order_relaxed);
  do
  {
    ASSERT(index < slot_count_ && StateOf(old, index) == IN_FLIGHT);
    ASSERT((old & ENABLED_BIT) != 0U);
  } while (!word_.compare_exchange_weak(old, WithState(old, index, LENT),
                                        std::memory_order_acq_rel,
                                        std::memory_order_relaxed));
  // 上一次借出结束时戳已经推进过，所以现在的戳只属于这一次借出。
  // The stamp advanced when the previous lend ended, so the current one belongs to this
  // lend alone.
  return records_[index].stamp.load(std::memory_order_acquire);
}

EndpointSlots::Request EndpointSlots::RequestOf(uint8_t index) const
{
  // 在途的槽只有 owner 能改变，它的记录在下一次借出并排队之前不变。
  // Only the owner changes an in-flight slot, and its record stays as it is until the
  // slot is lent and queued again.
  ASSERT(index < slot_count_ &&
         StateOf(word_.load(std::memory_order_relaxed), index) == IN_FLIGHT);
  return records_[index].request;
}

uint32_t EndpointSlots::StateOf(uint32_t word, uint8_t index)
{
  return (word >> (index * SLOT_BITS)) & SLOT_MASK;
}

uint32_t EndpointSlots::WithState(uint32_t word, uint8_t index, uint32_t state)
{
  const uint32_t shift = index * SLOT_BITS;
  return (word & ~(SLOT_MASK << shift)) | (state << shift);
}

uint32_t EndpointSlots::MarkQueued(uint32_t word, uint8_t index)
{
  const uint32_t sequence = ((word >> SEQUENCE_SHIFT) + 1U) & SEQUENCE_MASK;
  const uint32_t next = WithState(word, index, QUEUED);
  return (next & ~(SEQUENCE_MASK << SEQUENCE_SHIFT)) | (sequence << SEQUENCE_SHIFT);
}

bool EndpointSlots::Holds(const Lend& lend) const
{
  return lend.index < slot_count_ &&
         records_[lend.index].stamp.load(std::memory_order_acquire) == lend.stamp;
}

LibXR::ErrorCode EndpointSlots::LendFree(bool sending, Lend& lend)
{
  uint32_t old = word_.load(std::memory_order_relaxed);
  uint8_t pick;
  do
  {
    if ((old & ENABLED_BIT) == 0U || ((old & SENDING_BIT) != 0U) != sending)
    {
      return LibXR::ErrorCode::STATE_ERR;
    }
    pick = NO_SLOT;
    for (uint8_t i = 0U; i < slot_count_; ++i)
    {
      if (StateOf(old, i) == FREE)
      {
        pick = i;
        break;
      }
    }
    if (pick == NO_SLOT)
    {
      return LibXR::ErrorCode::BUSY;
    }
  } while (!word_.compare_exchange_weak(old, WithState(old, pick, LENT),
                                        std::memory_order_acq_rel,
                                        std::memory_order_relaxed));
  lend.index = pick;
  lend.stamp = records_[pick].stamp.load(std::memory_order_acquire);
  return LibXR::ErrorCode::OK;
}
