/**
 * @file test_usb_ep_slots.cpp
 * @brief 端点槽记账的单线程与多线程检查 / Single- and multi-threaded EndpointSlots checks
 *
 * 单线程用例按头文件的文档逐个检查操作、返回码和前置条件路径；多线程用例让几个持有者
 * 并发借出、排队、交回，同时 owner 取出、结束并反复停用和启用，检查槽不会被两人同时持有、
 * 每个排队请求按原参数恰好被取出一次、计数守恒。
 * The single-threaded case walks every operation, return code and precondition path the
 * header documents. The multi-threaded cases let several holders lend, queue and hand
 * back while the owner takes, finishes and keeps disabling and enabling, and check that
 * no slot has two holders, every queued request is taken once with its own parameters,
 * and the counts add up.
 */

#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <vector>

#include "ep_slots.hpp"
#include "libxr.hpp"
#include "test_assert.hpp"

namespace
{
using LibXR::ErrorCode;
using LibXR::USB::EndpointSlots;
using Lend = EndpointSlots::Lend;
using Request = EndpointSlots::Request;

constexpr uint8_t NO_SLOT = EndpointSlots::NO_SLOT;

/// 借出一个发送槽并检查槽号 / Lend a sending slot and check its index
Lend LendIn(EndpointSlots& slots, uint8_t expected)
{
  Lend lend;
  TEST_ASSERT(slots.LendForSending(lend) == ErrorCode::OK);
  TEST_ASSERT(lend.index == expected);
  return lend;
}

/// 取出下一个请求并检查槽号与参数 / Take the next request and check index and request
void ExpectTake(EndpointSlots& slots, uint8_t index, size_t length, bool zlp)
{
  const EndpointSlots::Queued queued = slots.TakeNext();
  TEST_ASSERT(queued.index == index);
  TEST_ASSERT(queued.request.length == length);
  TEST_ASSERT(queued.request.zlp == zlp);
}

void CheckDisabledAndDirection()
{
  EndpointSlots slots(2U);
  Lend lend;
  // 构造后未启用 / Disabled after construction
  TEST_ASSERT(slots.LendForSending(lend) == ErrorCode::STATE_ERR);
  TEST_ASSERT(slots.ClaimForReceiving(lend) == ErrorCode::STATE_ERR);
  TEST_ASSERT(lend.index == NO_SLOT);
  TEST_ASSERT(slots.TakeNext().index == NO_SLOT);
  TEST_ASSERT(slots.DropQueued() == 0U);
  TEST_ASSERT(slots.Disable() == 0U);

  // IN：认领是错误方向 / IN: a claim is the wrong direction
  slots.Enable(true);
  TEST_ASSERT(slots.ClaimForReceiving(lend) == ErrorCode::STATE_ERR);
  const Lend first = LendIn(slots, 0U);
  const Lend second = LendIn(slots, 1U);
  TEST_ASSERT(slots.LendForSending(lend) == ErrorCode::BUSY);
  TEST_ASSERT(slots.Release(first) == ErrorCode::OK);
  TEST_ASSERT(slots.Release(second) == ErrorCode::OK);
  TEST_ASSERT(slots.Disable() == 0U);
  TEST_ASSERT(slots.LendForSending(lend) == ErrorCode::STATE_ERR);

  // OUT：借出发送槽是错误方向 / OUT: lending for sending is the wrong direction
  slots.Enable(false);
  TEST_ASSERT(slots.LendForSending(lend) == ErrorCode::STATE_ERR);
  Lend claim;
  TEST_ASSERT(slots.ClaimForReceiving(claim) == ErrorCode::OK);
  TEST_ASSERT(claim.index == 0U);
  // 方向不符时槽不受影响 / A wrong direction leaves the slot alone
  TEST_ASSERT(slots.Queue(claim, {64U, false}, true) == ErrorCode::STATE_ERR);
  TEST_ASSERT(slots.TakeNext().index == NO_SLOT);
  TEST_ASSERT(slots.Queue(claim, {64U, false}, false) == ErrorCode::OK);
  ExpectTake(slots, 0U, 64U, false);
  slots.FinishFree(0U);
  TEST_ASSERT(slots.Disable() == 0U);
  TEST_ASSERT(slots.ClaimForReceiving(lend) == ErrorCode::STATE_ERR);
}

void CheckQueueOrder()
{
  EndpointSlots slots(2U);
  slots.Enable(true);

  // 槽 1 先排队则先取槽 1 / Slot 1 queued first is taken first
  Lend first = LendIn(slots, 0U);
  Lend second = LendIn(slots, 1U);
  TEST_ASSERT(slots.Queue(second, {11U, true}, true) == ErrorCode::OK);
  TEST_ASSERT(slots.Queue(first, {10U, false}, true) == ErrorCode::OK);
  ExpectTake(slots, 1U, 11U, true);
  slots.FinishFree(1U);
  ExpectTake(slots, 0U, 10U, false);
  slots.FinishFree(0U);
  TEST_ASSERT(slots.TakeNext().index == NO_SLOT);

  // 槽 0 先排队则先取槽 0 / Slot 0 queued first is taken first
  first = LendIn(slots, 0U);
  second = LendIn(slots, 1U);
  TEST_ASSERT(slots.Queue(first, {20U, true}, true) == ErrorCode::OK);
  TEST_ASSERT(slots.Queue(second, {21U, false}, true) == ErrorCode::OK);
  ExpectTake(slots, 0U, 20U, true);
  // 槽 0 在途时槽 0 不能借，槽 1 仍在排队 / Slot 0 in flight, slot 1 still queued
  Lend lend;
  TEST_ASSERT(slots.LendForSending(lend) == ErrorCode::BUSY);
  slots.FinishFree(0U);
  // 槽 1 仍在排队时再排槽 0，槽 1 在前 / Slot 0 queued behind the waiting slot 1
  first = LendIn(slots, 0U);
  TEST_ASSERT(slots.Queue(first, {22U, false}, true) == ErrorCode::OK);
  ExpectTake(slots, 1U, 21U, false);
  slots.FinishFree(1U);
  ExpectTake(slots, 0U, 22U, false);
  slots.FinishFree(0U);
  TEST_ASSERT(slots.TakeNext().index == NO_SLOT);
}

void CheckHandles()
{
  EndpointSlots slots(2U);
  slots.Enable(true);

  // 不指向任何槽的句柄 / Handles that refer to no slot
  const Lend none;
  TEST_ASSERT(slots.Queue(none, {1U, false}, true) == ErrorCode::STATE_ERR);
  TEST_ASSERT(slots.Release(none) == ErrorCode::STATE_ERR);

  // 交回之后再交回被拒绝 / A second hand-back is refused
  const Lend queued = LendIn(slots, 0U);
  TEST_ASSERT(slots.Queue(queued, {5U, false}, true) == ErrorCode::OK);
  TEST_ASSERT(slots.Queue(queued, {6U, true}, true) == ErrorCode::STATE_ERR);
  TEST_ASSERT(slots.Release(queued) == ErrorCode::STATE_ERR);
  ExpectTake(slots, 0U, 5U, false);
  TEST_ASSERT(slots.Release(queued) == ErrorCode::STATE_ERR);
  slots.FinishFree(0U);
  TEST_ASSERT(slots.Release(queued) == ErrorCode::STATE_ERR);

  const Lend released = LendIn(slots, 0U);
  TEST_ASSERT(released.stamp != queued.stamp);
  TEST_ASSERT(slots.Release(released) == ErrorCode::OK);
  TEST_ASSERT(slots.Release(released) == ErrorCode::STATE_ERR);
  TEST_ASSERT(slots.Queue(released, {7U, false}, true) == ErrorCode::STATE_ERR);

  // 重新借出之后旧句柄被戳拒绝，新句柄可用 / After a re-lend the stamp refuses the old
  // handle and the new one works
  const Lend current = LendIn(slots, 0U);
  TEST_ASSERT(current.stamp != released.stamp);
  TEST_ASSERT(slots.Queue(released, {8U, false}, true) == ErrorCode::STATE_ERR);
  TEST_ASSERT(slots.Release(released) == ErrorCode::STATE_ERR);
  TEST_ASSERT(slots.Queue(queued, {8U, false}, true) == ErrorCode::STATE_ERR);
  TEST_ASSERT(slots.TakeNext().index == NO_SLOT);
  TEST_ASSERT(slots.Queue(current, {9U, true}, true) == ErrorCode::OK);
  ExpectTake(slots, 0U, 9U, true);
  slots.FinishFree(0U);

  // 槽号对，戳是别的槽的 / Right index, another slot's stamp
  const Lend a = LendIn(slots, 0U);
  const Lend b = LendIn(slots, 1U);
  const Lend mixed{1U, a.stamp};
  if (a.stamp != b.stamp)
  {
    TEST_ASSERT(slots.Release(mixed) == ErrorCode::STATE_ERR);
  }
  TEST_ASSERT(slots.Release(a) == ErrorCode::OK);
  TEST_ASSERT(slots.Release(b) == ErrorCode::OK);
}

void CheckDropAndDisable()
{
  EndpointSlots slots(2U);
  slots.Enable(true);

  // DropQueued 释放排队的槽，保持启用 / DropQueued frees queued slots, stays enabled
  Lend first = LendIn(slots, 0U);
  Lend second = LendIn(slots, 1U);
  TEST_ASSERT(slots.Queue(first, {1U, false}, true) == ErrorCode::OK);
  TEST_ASSERT(slots.Queue(second, {2U, false}, true) == ErrorCode::OK);
  TEST_ASSERT(slots.DropQueued() == 2U);
  TEST_ASSERT(slots.DropQueued() == 0U);
  TEST_ASSERT(slots.TakeNext().index == NO_SLOT);
  first = LendIn(slots, 0U);
  second = LendIn(slots, 1U);

  // 借出的槽不受 DropQueued 影响 / A lent slot is untouched by DropQueued
  TEST_ASSERT(slots.Queue(second, {3U, false}, true) == ErrorCode::OK);
  TEST_ASSERT(slots.DropQueued() == 1U);
  TEST_ASSERT(slots.Queue(first, {4U, true}, true) == ErrorCode::OK);
  ExpectTake(slots, 0U, 4U, true);
  slots.FinishFree(0U);

  // Disable：排队的变空闲并计数，借出的作废 / Disable: queued freed and counted, lent
  // marked stale
  first = LendIn(slots, 0U);
  second = LendIn(slots, 1U);
  TEST_ASSERT(slots.Queue(first, {5U, false}, true) == ErrorCode::OK);
  TEST_ASSERT(slots.Disable() == 1U);
  TEST_ASSERT(slots.TakeNext().index == NO_SLOT);
  Lend lend;
  TEST_ASSERT(slots.LendForSending(lend) == ErrorCode::STATE_ERR);
  TEST_ASSERT(slots.Queue(second, {6U, false}, true) == ErrorCode::ABORTED);
  TEST_ASSERT(slots.Queue(second, {6U, false}, true) == ErrorCode::STATE_ERR);
  TEST_ASSERT(slots.Release(second) == ErrorCode::STATE_ERR);
  slots.Enable(true);
  first = LendIn(slots, 0U);
  second = LendIn(slots, 1U);

  // 作废的槽 Release 返回 OK / Release of a stale slot returns OK
  TEST_ASSERT(slots.Disable() == 0U);
  TEST_ASSERT(slots.Release(first) == ErrorCode::OK);
  TEST_ASSERT(slots.Release(first) == ErrorCode::STATE_ERR);
  TEST_ASSERT(slots.Release(second) == ErrorCode::OK);

  // 在途的槽不受 Disable 影响，由 FinishFree 结束 / An in-flight slot survives Disable
  // and ends with FinishFree
  slots.Enable(true);
  first = LendIn(slots, 0U);
  second = LendIn(slots, 1U);
  TEST_ASSERT(slots.Queue(first, {7U, false}, true) == ErrorCode::OK);
  TEST_ASSERT(slots.Queue(second, {8U, true}, true) == ErrorCode::OK);
  ExpectTake(slots, 0U, 7U, false);
  TEST_ASSERT(slots.Disable() == 1U);
  slots.FinishFree(0U);
  slots.Enable(true);
  first = LendIn(slots, 0U);
  second = LendIn(slots, 1U);
  TEST_ASSERT(slots.Release(first) == ErrorCode::OK);
  TEST_ASSERT(slots.Release(second) == ErrorCode::OK);
}

void CheckStaleAcrossEnable()
{
  EndpointSlots slots(2U);
  slots.Enable(true);

  // 作废的槽跨过 Enable，只挡住自己 / A stale slot survives Enable and blocks only
  // itself
  const Lend stale = LendIn(slots, 0U);
  TEST_ASSERT(slots.Disable() == 0U);
  slots.Enable(true);
  Lend lend;
  const Lend other = LendIn(slots, 1U);
  TEST_ASSERT(slots.LendForSending(lend) == ErrorCode::BUSY);
  TEST_ASSERT(slots.Queue(other, {3U, false}, true) == ErrorCode::OK);
  ExpectTake(slots, 1U, 3U, false);
  slots.FinishFree(1U);

  // 交回作废的槽：ABORTED，之后空闲 / Handing back the stale slot: ABORTED, then free
  TEST_ASSERT(slots.Queue(stale, {4U, false}, true) == ErrorCode::ABORTED);
  TEST_ASSERT(slots.TakeNext().index == NO_SLOT);
  const Lend fresh = LendIn(slots, 0U);
  TEST_ASSERT(fresh.stamp != stale.stamp);
  TEST_ASSERT(slots.Release(fresh) == ErrorCode::OK);

  // 换方向之后作废的槽同样只挡住自己 / Same after a direction change
  const Lend held = LendIn(slots, 0U);
  TEST_ASSERT(slots.Disable() == 0U);
  slots.Enable(false);
  Lend claim;
  TEST_ASSERT(slots.ClaimForReceiving(claim) == ErrorCode::OK);
  TEST_ASSERT(claim.index == 1U);
  TEST_ASSERT(slots.ClaimForReceiving(lend) == ErrorCode::BUSY);
  TEST_ASSERT(slots.Release(held) == ErrorCode::OK);
  TEST_ASSERT(slots.ClaimForReceiving(lend) == ErrorCode::OK);
  TEST_ASSERT(lend.index == 0U);
  TEST_ASSERT(slots.Release(lend) == ErrorCode::OK);
  TEST_ASSERT(slots.Release(claim) == ErrorCode::OK);

  // 作废先于方向检查：方向不符也返回 ABORTED，槽变空闲 / Staleness comes before the
  // direction: a wrong direction also gets ABORTED and the slot becomes free
  TEST_ASSERT(slots.Disable() == 0U);
  slots.Enable(true);
  const Lend stale0 = LendIn(slots, 0U);
  const Lend stale1 = LendIn(slots, 1U);
  TEST_ASSERT(slots.Disable() == 0U);
  // 停用中，调用者的方向 OUT 与 IN 不符 / Disabled; the caller's OUT differs from IN
  TEST_ASSERT(slots.Queue(stale0, {5U, false}, false) == ErrorCode::ABORTED);
  slots.Enable(false);
  // 重新启用为 OUT，调用者的方向 IN 不符 / Enabled as OUT; the caller's IN differs
  TEST_ASSERT(slots.Queue(stale1, {6U, false}, true) == ErrorCode::ABORTED);
  TEST_ASSERT(slots.Queue(stale1, {6U, false}, true) == ErrorCode::STATE_ERR);
  TEST_ASSERT(slots.TakeNext().index == NO_SLOT);
  Lend a;
  Lend b;
  TEST_ASSERT(slots.ClaimForReceiving(a) == ErrorCode::OK);
  TEST_ASSERT(slots.ClaimForReceiving(b) == ErrorCode::OK);
  TEST_ASSERT(a.index == 0U && b.index == 1U);
  TEST_ASSERT(slots.Release(a) == ErrorCode::OK);
  TEST_ASSERT(slots.Release(b) == ErrorCode::OK);
}

/// 交回、再借出，然后使用旧句柄：被拒绝，新持有者不受影响 / Hand back, lend again,
/// then use the old handle: refused, and the new holder is unaffected
void CheckOldHandleAfterRelend()
{
  EndpointSlots slots(1U);
  slots.Enable(true);
  Lend lend;

  // 以 Release 交回 / Handed back with Release
  const Lend released = LendIn(slots, 0U);
  TEST_ASSERT(slots.Release(released) == ErrorCode::OK);
  const Lend current = LendIn(slots, 0U);
  TEST_ASSERT(current.stamp != released.stamp);
  TEST_ASSERT(slots.Queue(released, {1U, true}, true) == ErrorCode::STATE_ERR);
  TEST_ASSERT(slots.Release(released) == ErrorCode::STATE_ERR);
  TEST_ASSERT(slots.LendForSending(lend) == ErrorCode::BUSY);
  TEST_ASSERT(slots.TakeNext().index == NO_SLOT);
  TEST_ASSERT(slots.Queue(current, {2U, false}, true) == ErrorCode::OK);
  ExpectTake(slots, 0U, 2U, false);
  slots.FinishFree(0U);

  // 以 Queue 交回，取出并结束之后 / Handed back with Queue, then taken and finished
  const Lend queued = LendIn(slots, 0U);
  TEST_ASSERT(slots.Queue(queued, {3U, false}, true) == ErrorCode::OK);
  ExpectTake(slots, 0U, 3U, false);
  slots.FinishFree(0U);
  const Lend next = LendIn(slots, 0U);
  TEST_ASSERT(slots.Queue(queued, {4U, true}, true) == ErrorCode::STATE_ERR);
  TEST_ASSERT(slots.Release(queued) == ErrorCode::STATE_ERR);
  TEST_ASSERT(slots.Queue(next, {5U, true}, true) == ErrorCode::OK);
  ExpectTake(slots, 0U, 5U, true);
  slots.FinishFree(0U);

  // 作废后以 ABORTED 交回 / Handed back as stale with ABORTED
  const Lend stale = LendIn(slots, 0U);
  TEST_ASSERT(slots.Disable() == 0U);
  slots.Enable(true);
  TEST_ASSERT(slots.Queue(stale, {6U, false}, true) == ErrorCode::ABORTED);
  const Lend fresh = LendIn(slots, 0U);
  TEST_ASSERT(slots.Queue(stale, {7U, false}, true) == ErrorCode::STATE_ERR);
  TEST_ASSERT(slots.Release(stale) == ErrorCode::STATE_ERR);
  TEST_ASSERT(slots.Release(fresh) == ErrorCode::OK);
}

/// 戳是每个槽的借出计数：每结束一次借出加一，旧句柄一直被拒绝 / The stamp is a
/// per-slot lend count: it advances with every lend that ends and an old handle stays
/// refused
void CheckStampCounts()
{
  constexpr uint32_t ROUNDS = 5000U;
  EndpointSlots slots(2U);
  slots.Enable(true);
  const Lend old = LendIn(slots, 0U);
  TEST_ASSERT(slots.Release(old) == ErrorCode::OK);

  // 之后的每次借出：戳逐次加一，旧句柄始终被拒绝；槽 1 的借出不推进槽 0 的戳。交回的
  // 三种方式（Release、Queue 后被取走、Queue 后被清掉）都结束这次借出
  // Every further lend: the stamp counts up by one and the old handle is refused every
  // time; lends of slot 1 do not advance the stamp of slot 0. All three ways of handing
  // back (Release, Queue then taken, Queue then dropped) end the lend
  for (uint32_t i = 1U; i < ROUNDS; ++i)
  {
    const Lend lend = LendIn(slots, 0U);
    TEST_ASSERT(lend.stamp == old.stamp + i);
    const Lend other = LendIn(slots, 1U);
    TEST_ASSERT(slots.Queue(old, {1U, false}, true) == ErrorCode::STATE_ERR);
    TEST_ASSERT(slots.Release(old) == ErrorCode::STATE_ERR);
    TEST_ASSERT(slots.Release(other) == ErrorCode::OK);
    if (i % 3U == 0U)
    {
      TEST_ASSERT(slots.Release(lend) == ErrorCode::OK);
    }
    else
    {
      TEST_ASSERT(slots.Queue(lend, {i, false}, true) == ErrorCode::OK);
      if (i % 3U == 1U)
      {
        ExpectTake(slots, 0U, i, false);
        slots.FinishFree(0U);
      }
      else
      {
        TEST_ASSERT(slots.DropQueued() == 1U);
      }
    }
    TEST_ASSERT(slots.Release(lend) == ErrorCode::STATE_ERR);
  }
}

/// 多于两个槽：借出次序、排队次序、清空与停用 / More than two slots: lending order,
/// queueing order, dropping and disabling
void CheckManySlots(uint8_t count)
{
  EndpointSlots slots(count);
  slots.Enable(true);
  Lend lends[EndpointSlots::MAX_SLOTS];
  Lend lend;

  // 借槽取编号最小的空闲槽，借完为止 / A lend takes the lowest free slot until none is
  // left
  for (uint8_t i = 0U; i < count; ++i)
  {
    lends[i] = LendIn(slots, i);
  }
  TEST_ASSERT(slots.LendForSending(lend) == ErrorCode::BUSY);
  // 槽号超出槽数的句柄被拒绝 / A handle beyond the slot count is refused
  const Lend beyond{count, 0U};
  TEST_ASSERT(slots.Queue(beyond, {1U, false}, true) == ErrorCode::STATE_ERR);
  TEST_ASSERT(slots.Release(beyond) == ErrorCode::STATE_ERR);

  // 按打乱的次序排队，取出的次序与排队相同，与槽号无关
  // Queued in a shuffled order, taken in the order of queueing whatever the indices
  uint8_t order[EndpointSlots::MAX_SLOTS];
  for (uint8_t i = 0U; i < count; ++i)
  {
    order[i] = static_cast<uint8_t>((i * 5U + 3U) % count);
  }
  if (count % 5U == 0U)
  {
    // 5 与槽数不互质时上面不是排列，改用倒序 / Not a permutation when 5 divides the
    // count; use the reverse order instead
    for (uint8_t i = 0U; i < count; ++i)
    {
      order[i] = static_cast<uint8_t>(count - 1U - i);
    }
  }
  for (uint8_t i = 0U; i < count; ++i)
  {
    TEST_ASSERT(slots.Queue(lends[order[i]], {100U + i, (i & 1U) != 0U}, true) ==
                ErrorCode::OK);
  }
  for (uint8_t i = 0U; i < count; ++i)
  {
    ExpectTake(slots, order[i], 100U + i, (i & 1U) != 0U);
  }
  TEST_ASSERT(slots.TakeNext().index == NO_SLOT);
  // 全部在途：没有槽可借；逐个结束后再借到的是刚结束的那个
  // All in flight: nothing to lend; after each end the lend gets the slot just ended
  TEST_ASSERT(slots.LendForSending(lend) == ErrorCode::BUSY);
  for (uint8_t i = 0U; i < count; ++i)
  {
    slots.FinishFree(order[i]);
    lends[order[i]] = LendIn(slots, order[i]);
  }

  // 长时间轮转：每一步排进一个、取出最早的一个，队列里始终有 count - 1 个在等。排队序号
  // 回绕许多次，先后不乱
  // A long rotation: each step queues one and takes the earliest, with count - 1 always
  // waiting. The queueing sequence wraps many times and the order holds
  uint32_t queued = 0U;
  uint32_t taken = 0U;
  for (uint8_t i = 0U; i + 1U < count; ++i)
  {
    TEST_ASSERT(slots.Queue(lends[i], {1000U + queued, false}, true) == ErrorCode::OK);
    ++queued;
  }
  uint8_t free_slot = static_cast<uint8_t>(count - 1U);
  for (uint32_t step = 0U; step < 1000U; ++step)
  {
    TEST_ASSERT(slots.Queue(lends[free_slot], {1000U + queued, false}, true) ==
                ErrorCode::OK);
    ++queued;
    const EndpointSlots::Queued next = slots.TakeNext();
    TEST_ASSERT(next.index != NO_SLOT && next.request.length == 1000U + taken);
    ++taken;
    slots.FinishFree(next.index);
    free_slot = next.index;
    lends[free_slot] = LendIn(slots, free_slot);
  }

  // 清空排队：count - 1 个变空闲，借出的那个不受影响
  // Dropping the queue frees count - 1 slots and leaves the lent one alone
  TEST_ASSERT(slots.DropQueued() == count - 1U);
  TEST_ASSERT(slots.TakeNext().index == NO_SLOT);
  TEST_ASSERT(slots.Queue(lends[free_slot], {7U, true}, true) == ErrorCode::OK);
  ExpectTake(slots, free_slot, 7U, true);

  // 停用：一个在途、一个借出、其余排队。排队的变空闲，借出的作废，在途的不动
  // Disable with one in flight, one lent and the rest queued: the queued become free,
  // the lent one stale, the one in flight stays
  uint8_t held = NO_SLOT;
  uint8_t waiting = 0U;
  for (uint8_t i = 0U; i < count; ++i)
  {
    if (i == free_slot)
    {
      continue;
    }
    lends[i] = LendIn(slots, i);
    if (held == NO_SLOT)
    {
      held = i;
      continue;
    }
    TEST_ASSERT(slots.Queue(lends[i], {i, false}, true) == ErrorCode::OK);
    ++waiting;
  }
  TEST_ASSERT(slots.Disable() == waiting);
  TEST_ASSERT(slots.LendForSending(lend) == ErrorCode::STATE_ERR);
  slots.Enable(true);
  // 重新启用后：作废的和在途的不能借，其余都能借
  // Enabled again: the stale and the in-flight slot cannot be lent, the others can
  for (uint8_t i = 0U; i < waiting; ++i)
  {
    TEST_ASSERT(slots.LendForSending(lend) == ErrorCode::OK);
    TEST_ASSERT(lend.index != held && lend.index != free_slot);
  }
  TEST_ASSERT(slots.LendForSending(lend) == ErrorCode::BUSY);
  TEST_ASSERT(slots.Queue(lends[held], {1U, false}, true) == ErrorCode::ABORTED);
  TEST_ASSERT(slots.LendForSending(lend) == ErrorCode::OK && lend.index == held);
  slots.FinishFree(free_slot);
  TEST_ASSERT(slots.LendForSending(lend) == ErrorCode::OK && lend.index == free_slot);
}

#ifdef LIBXR_DEBUG_BUILD
/// 致命错误回调在子进程中的退出码 / Exit code of the fatal callback in the child
constexpr int SLOTS_FATAL_EXIT = 43;

/// 在子进程中运行 func，返回是否以致命错误退出 / Run func in a child; return whether
/// it ended through the fatal callback
template <typename Func>
bool EndsFatally(Func&& func)
{
  std::fflush(nullptr);
  const pid_t child = fork();
  TEST_ASSERT(child >= 0);
  if (child == 0)
  {
    auto cb = LibXR::Assert::FatalCallback::Create(
        [](bool, int code, const char*, uint32_t) { _exit(code); }, SLOTS_FATAL_EXIT);
    LibXR::Assert::RegisterFatalErrorCallback(cb);
    func();
    _exit(0);
  }
  int status = 0;
  TEST_ASSERT(waitpid(child, &status, 0) == child);
  TEST_ASSERT(WIFEXITED(status));
  TEST_ASSERT(WEXITSTATUS(status) == 0 || WEXITSTATUS(status) == SLOTS_FATAL_EXIT);
  return WEXITSTATUS(status) == SLOTS_FATAL_EXIT;
}

/**
 * @brief 停用后有槽在途时重新启用 / Enable again while a slot is in flight
 * @note 在途的槽属于上一次绑定被停的请求：保持在途、不借出，FinishFree 之后才借出；
 *       同一个槽 FinishFree 两次违反前置条件。
 *       The in-flight slot belongs to the stopped request of the previous binding: it
 *       stays in flight and is not lent until FinishFree; a second FinishFree of the
 *       same slot breaks the precondition.
 */
void CheckEnableWithSlotInFlight(uint8_t count)
{
  EndpointSlots slots(count);
  slots.Enable(true);
  Lend lend;
  TEST_ASSERT(slots.LendForSending(lend) == ErrorCode::OK);
  TEST_ASSERT(slots.Queue(lend, {1U, false}, true) == ErrorCode::OK);
  ExpectTake(slots, 0U, 1U, false);
  TEST_ASSERT(slots.Disable() == 0U);
  slots.Enable(false);
  Lend claim;
  if (count == 2U)
  {
    TEST_ASSERT(slots.ClaimForReceiving(claim) == ErrorCode::OK);
    TEST_ASSERT(claim.index == 1U);
  }
  TEST_ASSERT(slots.ClaimForReceiving(claim) == ErrorCode::BUSY);
  slots.FinishFree(0U);
  TEST_ASSERT(slots.ClaimForReceiving(claim) == ErrorCode::OK);
  TEST_ASSERT(claim.index == 0U);
  TEST_ASSERT(EndsFatally([&slots]() { slots.FinishFree(0U); }));
}
#endif

void CheckEnablePrecondition()
{
#ifdef LIBXR_DEBUG_BUILD
  CheckEnableWithSlotInFlight(1U);
  CheckEnableWithSlotInFlight(2U);
  // 槽数只能是 1 到 MAX_SLOTS / The slot count is 1 to MAX_SLOTS only
  TEST_ASSERT(EndsFatally([]() { const EndpointSlots none(0U); }));
  TEST_ASSERT(
      EndsFatally([]() { const EndpointSlots many(EndpointSlots::MAX_SLOTS + 1U); }));
  TEST_ASSERT(!EndsFatally([]() { const EndpointSlots most(EndpointSlots::MAX_SLOTS); }));
#endif
}

void CheckFinishLent()
{
  EndpointSlots slots(2U);
  slots.Enable(false);

  Lend claim;
  TEST_ASSERT(slots.ClaimForReceiving(claim) == ErrorCode::OK);
  TEST_ASSERT(slots.Queue(claim, {512U, false}, false) == ErrorCode::OK);
  ExpectTake(slots, 0U, 512U, false);
  const uint32_t stamp = slots.FinishLent(0U);
  TEST_ASSERT(stamp == claim.stamp + 1U);
  const Lend received{0U, stamp};

  // 认领的旧句柄失效，FinishLent 的戳是有效句柄 / The claim's handle is refused, the
  // FinishLent stamp works as a handle
  TEST_ASSERT(slots.Release(claim) == ErrorCode::STATE_ERR);
  TEST_ASSERT(slots.Queue(claim, {64U, false}, false) == ErrorCode::STATE_ERR);
  Lend lend;
  TEST_ASSERT(slots.ClaimForReceiving(lend) == ErrorCode::OK);
  TEST_ASSERT(lend.index == 1U);
  TEST_ASSERT(slots.ClaimForReceiving(lend) == ErrorCode::BUSY);
  TEST_ASSERT(slots.Release(lend) == ErrorCode::OK);
  TEST_ASSERT(slots.Release(received) == ErrorCode::OK);
  TEST_ASSERT(slots.Release(received) == ErrorCode::STATE_ERR);

  // FinishLent 的句柄也可以排队，Disable 后作废 / It can be queued too, and goes stale
  // on Disable
  TEST_ASSERT(slots.ClaimForReceiving(claim) == ErrorCode::OK);
  TEST_ASSERT(slots.Queue(claim, {128U, true}, false) == ErrorCode::OK);
  ExpectTake(slots, 0U, 128U, true);
  const Lend again{0U, slots.FinishLent(0U)};
  TEST_ASSERT(slots.Queue(again, {256U, false}, false) == ErrorCode::OK);
  ExpectTake(slots, 0U, 256U, false);
  const Lend last{0U, slots.FinishLent(0U)};
  TEST_ASSERT(slots.Disable() == 0U);
  TEST_ASSERT(slots.Queue(last, {64U, false}, false) == ErrorCode::ABORTED);
}

void CheckSingleSlot()
{
  EndpointSlots slots(1U);
  slots.Enable(true);
  const Lend only = LendIn(slots, 0U);
  Lend lend;
  TEST_ASSERT(slots.LendForSending(lend) == ErrorCode::BUSY);
  // 槽 1 不存在 / Slot 1 does not exist
  const Lend beyond{1U, 0U};
  TEST_ASSERT(slots.Queue(beyond, {1U, false}, true) == ErrorCode::STATE_ERR);
  TEST_ASSERT(slots.Release(beyond) == ErrorCode::STATE_ERR);
  TEST_ASSERT(slots.Queue(only, {2U, false}, true) == ErrorCode::OK);
  ExpectTake(slots, 0U, 2U, false);
  TEST_ASSERT(slots.LendForSending(lend) == ErrorCode::BUSY);
  slots.FinishFree(0U);
  const Lend next = LendIn(slots, 0U);
  TEST_ASSERT(slots.Queue(next, {3U, true}, true) == ErrorCode::OK);
  TEST_ASSERT(slots.Disable() == 1U);
}

// ---------------------------------------------------------------------------
// 多线程 / Multi-threaded
// ---------------------------------------------------------------------------

/// 持有者线程数 / Number of holder threads
constexpr unsigned HOLDER_COUNT = 3U;
/// 每个持有者的借出尝试次数 / Lend attempts per holder
constexpr unsigned HOLDER_ROUNDS = 200000U;
/// owner 每隔多少轮停用并重新启用一次 / Owner rounds between two disable cycles
constexpr unsigned OWNER_DISABLE_PERIOD = 257U;
/// 整个压测的时限 / Time limit of the whole stress run
constexpr auto STRESS_LIMIT = std::chrono::seconds(120);
/// owner 在槽标记里的编号 / The owner's id in a slot marker
constexpr uint32_t OWNER_MARK = 0xFFFFU;

/// 简单的伪随机数 / Small pseudo-random generator
uint32_t NextRandom(uint32_t& state)
{
  state ^= state << 13U;
  state ^= state >> 17U;
  state ^= state << 5U;
  return state;
}

/// 请求编号编码持有者与序号，零长度包标志由编号得出 / The request id encodes holder
/// and serial; the zero-length flag derives from the id
Request MakeRequest(unsigned holder, unsigned serial)
{
  const size_t id = (static_cast<size_t>(holder + 1U) << 20U) | serial;
  return {id, ((id * 2654435761U) >> 7U & 1U) != 0U};
}

struct StressShared
{
  explicit StressShared(uint8_t slot_count) : slots(slot_count) {}

  EndpointSlots slots;
  /// 每个槽当前的持有者，0 表示无人 / Current holder of each slot, 0: nobody
  std::array<std::atomic<uint32_t>, EndpointSlots::MAX_SLOTS> marker{};
  std::atomic<unsigned> holders_done{0U};
};

/// 持有者借到槽后立即标记，交回之前清除 / A holder marks the slot right after the lend
/// and clears the mark before handing it back
void MarkHeld(StressShared& shared, uint8_t index, uint32_t id)
{
  TEST_ASSERT(index < EndpointSlots::MAX_SLOTS);
  uint32_t expected = 0U;
  TEST_ASSERT(shared.marker[index].compare_exchange_strong(expected, id));
}

void ClearHeld(StressShared& shared, uint8_t index, uint32_t id)
{
  uint32_t expected = id;
  TEST_ASSERT(shared.marker[index].compare_exchange_strong(expected, 0U));
}

struct HolderLog
{
  std::vector<size_t> queued;  ///< Queue 返回 OK 的请求 / Requests Queue accepted
  unsigned aborted = 0U;       ///< Queue 返回 ABORTED 的次数 / ABORTED from Queue
  unsigned released = 0U;      ///< Release 的次数 / Releases
  unsigned lent = 0U;          ///< 成功借出或认领的次数 / Successful lends and claims
};

void RunHolder(StressShared& shared, unsigned holder, HolderLog& log)
{
  uint32_t random = 0x9E3779B9U * (holder + 1U);
  const uint32_t id = holder + 1U;
  for (unsigned serial = 0U; serial < HOLDER_ROUNDS; ++serial)
  {
    const bool sending = (NextRandom(random) & 1U) != 0U;
    Lend lend;
    const ErrorCode lent = sending ? shared.slots.LendForSending(lend)
                                   : shared.slots.ClaimForReceiving(lend);
    if (lent != ErrorCode::OK)
    {
      TEST_ASSERT(lent == ErrorCode::BUSY || lent == ErrorCode::STATE_ERR);
      std::this_thread::yield();
      continue;
    }
    ++log.lent;
    MarkHeld(shared, lend.index, id);
    if ((NextRandom(random) & 7U) == 0U)
    {
      std::this_thread::yield();
    }
    if ((NextRandom(random) & 3U) == 0U)
    {
      ClearHeld(shared, lend.index, id);
      TEST_ASSERT(shared.slots.Release(lend) == ErrorCode::OK);
      ++log.released;
      continue;
    }
    const Request request = MakeRequest(holder, serial);
    ClearHeld(shared, lend.index, id);
    const ErrorCode queued = shared.slots.Queue(lend, request, sending);
    if (queued == ErrorCode::OK)
    {
      log.queued.push_back(request.length);
    }
    else
    {
      // 自己的句柄只可能因停用而作废 / An own handle can only have gone stale
      TEST_ASSERT(queued == ErrorCode::ABORTED);
      ++log.aborted;
    }
  }
  shared.holders_done.fetch_add(1U, std::memory_order_release);
}

struct OwnerLog
{
  std::vector<size_t> taken;  ///< 取出的请求 / Requests taken
  /// 每个持有者上一个被取出的请求的序号加一 / Serial plus one of each holder's request
  /// taken last
  std::array<unsigned, 8> next_serial{};
  unsigned dropped = 0U;    ///< Disable 与 DropQueued 的计数之和 / Freed by both
  unsigned disables = 0U;   ///< 停用次数 / Disable cycles
  unsigned lent_back = 0U;  ///< FinishLent 次数 / FinishLent calls
};

/// 取出一个请求，检查无人持有、参数原样 / Take one request and check that nobody holds
/// it and that its parameters came through
uint8_t OwnerTake(StressShared& shared, OwnerLog& log)
{
  const EndpointSlots::Queued queued = shared.slots.TakeNext();
  if (queued.index == NO_SLOT)
  {
    return NO_SLOT;
  }
  TEST_ASSERT(queued.index < EndpointSlots::MAX_SLOTS);
  TEST_ASSERT(shared.marker[queued.index].load() == 0U);
  const size_t id = queued.request.length;
  const unsigned holder = static_cast<unsigned>(id >> 20U) - 1U;
  TEST_ASSERT(holder < HOLDER_COUNT);
  const unsigned serial = static_cast<unsigned>(id & 0xFFFFFU);
  const Request expected = MakeRequest(holder, serial);
  TEST_ASSERT(queued.request.zlp == expected.zlp);
  // 一个持有者一次只排一个，所以它的请求按排队的先后被取出
  // A holder queues one at a time, so its requests are taken in the order it queued them
  TEST_ASSERT(serial >= log.next_serial[holder]);
  log.next_serial[holder] = serial + 1U;
  log.taken.push_back(id);
  return queued.index;
}

/// 结束在途的槽；启用时有时借给接收者（这里是 owner 自己）再交回 / End the in-flight
/// slot; while enabled, sometimes lend it to the receiver (the owner here) who releases
/// it
void OwnerFinish(StressShared& shared, OwnerLog& log, uint8_t index, bool lend_back)
{
  TEST_ASSERT(shared.marker[index].load() == 0U);
  if (!lend_back)
  {
    shared.slots.FinishFree(index);
    return;
  }
  const Lend lend{index, shared.slots.FinishLent(index)};
  MarkHeld(shared, index, OWNER_MARK);
  ClearHeld(shared, index, OWNER_MARK);
  TEST_ASSERT(shared.slots.Release(lend) == ErrorCode::OK);
  ++log.lent_back;
}

void RunOwner(StressShared& shared, OwnerLog& log, size_t limit)
{
  const auto start = std::chrono::steady_clock::now();
  uint32_t random = 0x2545F491U;
  bool enabled = true;
  // 在途的槽，按取出的先后；最多 limit 个 / Slots in flight in the order taken, at
  // most limit
  std::vector<uint8_t> inflight;
  unsigned round = 0U;
  while (shared.holders_done.load(std::memory_order_acquire) < HOLDER_COUNT)
  {
    TEST_ASSERT(std::chrono::steady_clock::now() - start < STRESS_LIMIT);
    ++round;
    const uint32_t pick = NextRandom(random);
    if (!inflight.empty() && (pick & 1U) != 0U)
    {
      // FinishLent 要求已启用 / FinishLent requires the endpoint enabled
      OwnerFinish(shared, log, inflight.front(), enabled && (pick & 6U) == 0U);
      inflight.erase(inflight.begin());
      continue;
    }
    if (!enabled)
    {
      // Enable 只在没有在途槽时调用 / Enable only with nothing in flight
      if (inflight.empty())
      {
        shared.slots.Enable((pick & 8U) != 0U);
        enabled = true;
      }
      std::this_thread::yield();
      continue;
    }
    if (round % OWNER_DISABLE_PERIOD == 0U)
    {
      // 在途的槽可能跨过 Disable / The in-flight slot may stay across Disable
      log.dropped += shared.slots.Disable();
      ++log.disables;
      enabled = false;
      continue;
    }
    if ((pick & 126U) == 2U)
    {
      log.dropped += shared.slots.DropQueued();
      continue;
    }
    uint8_t took = NO_SLOT;
    if (inflight.size() < limit)
    {
      took = OwnerTake(shared, log);
    }
    if (took != NO_SLOT)
    {
      inflight.push_back(took);
    }
    else
    {
      std::this_thread::yield();
    }
  }
  // 持有者都已结束：取完剩下的请求 / All holders are done: drain what is left
  for (const uint8_t index : inflight)
  {
    OwnerFinish(shared, log, index, false);
  }
  if (!enabled)
  {
    shared.slots.Enable(true);
  }
  for (uint8_t index = OwnerTake(shared, log); index != NO_SLOT;
       index = OwnerTake(shared, log))
  {
    OwnerFinish(shared, log, index, false);
  }
}

/// limit：owner 同时保持在途的槽数上限 / limit: the most slots the owner keeps in
/// flight at once
void CheckStress(uint8_t slot_count, size_t limit)
{
  StressShared shared(slot_count);
  shared.slots.Enable(true);
  std::array<HolderLog, HOLDER_COUNT> holder_logs{};
  OwnerLog owner_log;

  std::vector<std::thread> holders;
  for (unsigned i = 0U; i < HOLDER_COUNT; ++i)
  {
    holders.emplace_back([&shared, &holder_logs, i]()
                         { RunHolder(shared, i, holder_logs[i]); });
  }
  std::thread owner([&shared, &owner_log, limit]()
                    { RunOwner(shared, owner_log, limit); });
  for (auto& holder : holders)
  {
    holder.join();
  }
  owner.join();

  // 每个被接受的请求恰好被取出一次或被清掉 / Every accepted request is taken once or
  // dropped
  std::vector<size_t> queued;
  unsigned aborted = 0U;
  unsigned released = 0U;
  unsigned lent = 0U;
  for (const auto& log : holder_logs)
  {
    queued.insert(queued.end(), log.queued.begin(), log.queued.end());
    aborted += log.aborted;
    released += log.released;
    lent += log.lent;
  }
  std::sort(queued.begin(), queued.end());
  TEST_ASSERT(std::adjacent_find(queued.begin(), queued.end()) == queued.end());
  std::vector<size_t> taken = owner_log.taken;
  std::sort(taken.begin(), taken.end());
  TEST_ASSERT(std::adjacent_find(taken.begin(), taken.end()) == taken.end());
  TEST_ASSERT(std::includes(queued.begin(), queued.end(), taken.begin(), taken.end()));
  TEST_ASSERT(queued.size() == taken.size() + owner_log.dropped);
  TEST_ASSERT(lent == queued.size() + aborted + released);

  // 结束后所有槽空闲 / Every slot is free at the end
  TEST_ASSERT(shared.slots.Disable() == 0U);
  shared.slots.Enable(true);
  for (uint8_t i = 0U; i < slot_count; ++i)
  {
    LendIn(shared.slots, i);
  }
  Lend lend;
  TEST_ASSERT(shared.slots.LendForSending(lend) == ErrorCode::BUSY);

  std::fprintf(stderr,
               "\t  ep_slots stress (%u slot, %zu in flight): lent %u, queued %zu, "
               "taken %zu, dropped %u, aborted %u, released %u, disables %u, lent back "
               "%u\n",
               static_cast<unsigned>(slot_count), limit, lent, queued.size(),
               taken.size(), owner_log.dropped, aborted, released, owner_log.disables,
               owner_log.lent_back);
}

/// 借出之后端点被停用又启用，持有者再排队：ABORTED，槽变空闲 / A holder lends, the
/// endpoint is disabled and enabled again, then the holder queues: ABORTED, slot free
void CheckDisableEnableBetweenLendAndQueue(bool same_direction)
{
  EndpointSlots slots(2U);
  slots.Enable(true);
  std::atomic<int> step{0};
  ErrorCode result = ErrorCode::OK;
  uint8_t index = NO_SLOT;

  std::thread holder(
      [&]()
      {
        Lend lend;
        TEST_ASSERT(slots.LendForSending(lend) == ErrorCode::OK);
        index = lend.index;
        step.store(1, std::memory_order_release);
        while (step.load(std::memory_order_acquire) != 2)
        {
          std::this_thread::yield();
        }
        result = slots.Queue(lend, {32U, false}, true);
        step.store(3, std::memory_order_release);
      });

  while (step.load(std::memory_order_acquire) != 1)
  {
    std::this_thread::yield();
  }
  TEST_ASSERT(slots.Disable() == 0U);
  slots.Enable(same_direction);
  step.store(2, std::memory_order_release);
  while (step.load(std::memory_order_acquire) != 3)
  {
    std::this_thread::yield();
  }
  holder.join();

  TEST_ASSERT(index == 0U);
  TEST_ASSERT(result == ErrorCode::ABORTED);
  TEST_ASSERT(slots.TakeNext().index == NO_SLOT);
  Lend a;
  Lend b;
  Lend c;
  if (same_direction)
  {
    TEST_ASSERT(slots.LendForSending(a) == ErrorCode::OK);
    TEST_ASSERT(slots.LendForSending(b) == ErrorCode::OK);
    TEST_ASSERT(slots.LendForSending(c) == ErrorCode::BUSY);
  }
  else
  {
    TEST_ASSERT(slots.ClaimForReceiving(a) == ErrorCode::OK);
    TEST_ASSERT(slots.ClaimForReceiving(b) == ErrorCode::OK);
    TEST_ASSERT(slots.ClaimForReceiving(c) == ErrorCode::BUSY);
  }
  TEST_ASSERT(a.index == 0U && b.index == 1U);
}

}  // namespace

void test_usb_ep_slots()
{
  CheckDisabledAndDirection();
  CheckQueueOrder();
  CheckHandles();
  CheckDropAndDisable();
  CheckStaleAcrossEnable();
  CheckOldHandleAfterRelend();
  CheckStampCounts();
  for (uint8_t count = 2U; count <= EndpointSlots::MAX_SLOTS; ++count)
  {
    CheckManySlots(count);
  }
  CheckFinishLent();
  CheckSingleSlot();
  CheckEnablePrecondition();
  CheckDisableEnableBetweenLendAndQueue(true);
  CheckDisableEnableBetweenLendAndQueue(false);
  CheckStress(2U, 1U);
  CheckStress(1U, 1U);
  CheckStress(2U, 2U);
  CheckStress(3U, 2U);
  CheckStress(EndpointSlots::MAX_SLOTS, EndpointSlots::MAX_SLOTS);
}
