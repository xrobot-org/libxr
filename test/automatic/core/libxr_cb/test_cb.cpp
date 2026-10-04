/**
 * @file test_cb.cpp
 * @brief 检查空回调、参数传递和递归调用。 /
 * Tests empty callbacks, argument forwarding and recursive calls.
 *
 * 分别检查直接回调和 Guarded 回调的调用顺序、递归深度及 ISR 标记。
 * Checks call order, nesting depth and ISR flags for direct and guarded callbacks.
 */

#include <array>

#include "libxr.hpp"
#include "test.hpp"
#include "test_assert.hpp"

namespace
{
struct DirectCallbackProbe
{
  LibXR::Callback<int> cb;
  bool runtime_in_isr = true;
  bool trigger_reentry = true;
  std::array<int, 4> seen = {};
  std::array<bool, 4> seen_in_isr = {};
  int seen_count = 0;
  int depth = 0;
  int max_depth = 0;

  DirectCallbackProbe() : cb(LibXR::Callback<int>::Create(OnCallback, this)) {}

  static void OnCallback(bool in_isr, DirectCallbackProbe* self, int value)
  {
    if (self->seen_count < static_cast<int>(self->seen.size()))
    {
      self->seen[static_cast<size_t>(self->seen_count++)] = value;
      self->seen_in_isr[static_cast<size_t>(self->seen_count - 1)] = in_isr;
    }

    self->depth++;
    if (self->depth > self->max_depth)
    {
      self->max_depth = self->depth;
    }

    if (self->trigger_reentry && value == 1)
    {
      self->trigger_reentry = false;
      self->cb.Run(self->runtime_in_isr, 2);
    }

    self->depth--;
  }
};

struct GuardedCreationProbe
{
  LibXR::Callback<int> cb;
  bool trigger_reentry = true;
  std::array<int, 4> seen = {};
  std::array<bool, 4> seen_in_isr = {};
  int seen_count = 0;
  int depth = 0;
  int max_depth = 0;

  GuardedCreationProbe() : cb(LibXR::Callback<int>::CreateGuarded(OnCallback, this)) {}

  static void OnCallback(bool in_isr, GuardedCreationProbe* self, int value)
  {
    if (self->seen_count < static_cast<int>(self->seen.size()))
    {
      self->seen[static_cast<size_t>(self->seen_count++)] = value;
      self->seen_in_isr[static_cast<size_t>(self->seen_count - 1)] = in_isr;
    }

    self->depth++;
    if (self->depth > self->max_depth)
    {
      self->max_depth = self->depth;
    }

    if (self->trigger_reentry && value == 1)
    {
      self->trigger_reentry = false;
      self->cb.Run(in_isr, 2);
    }

    self->depth--;
  }
};

struct LambdaCreationProbe
{
  LibXR::Callback<int> cb;
  int seen_value = 0;
  bool seen_in_isr = false;

  LambdaCreationProbe()
      : cb(LibXR::Callback<int>::Create(
            [](bool in_isr, LambdaCreationProbe* self, int value)
            {
              self->seen_value = value;
              self->seen_in_isr = in_isr;
            },
            this))
  {
  }
};

void TestEmptyAndDirectCallbacks()
{
  // 空回调不执行；直接递归调用的最大深度为二，并检查两次回调收到的 ISR 标记。
  // An empty callback does nothing; direct recursion reaches depth two. Check both
  // callback ISR flags.
  {
    LibXR::Callback<int> empty_cb;
    TEST_ASSERT(empty_cb.Empty());
    empty_cb.Run(false, 1);
  }

  {
    DirectCallbackProbe probe;
    probe.runtime_in_isr = true;
    probe.cb.Run(true, 1);
    TEST_ASSERT(probe.seen_count == 2);
    TEST_ASSERT(probe.seen[0] == 1);
    TEST_ASSERT(probe.seen[1] == 2);
    TEST_ASSERT(probe.seen_in_isr[0] == true);
    TEST_ASSERT(probe.seen_in_isr[1] == true);
    TEST_ASSERT(probe.max_depth == 2);
  }

  {
    DirectCallbackProbe probe;
    probe.runtime_in_isr = false;
    probe.cb.Run(false, 1);
    TEST_ASSERT(probe.seen_count == 2);
    TEST_ASSERT(probe.seen[0] == 1);
    TEST_ASSERT(probe.seen[1] == 2);
    TEST_ASSERT(probe.seen_in_isr[0] == false);
    TEST_ASSERT(probe.seen_in_isr[1] == false);
    TEST_ASSERT(probe.max_depth == 2);
  }
}

void TestGuardedAndLambdaCallbacks()
{
  // Guarded 回调把递归请求留到当前回调返回后处理，最大深度保持为一；再检查 lambda
  // 的参数传递。
  // Guarded callbacks defer recursive requests until return, keeping depth at one; also
  // check lambda arguments.
  {
    GuardedCreationProbe probe;
    probe.cb.Run(false, 1);
    TEST_ASSERT(probe.seen_count == 2);
    TEST_ASSERT(probe.seen[0] == 1);
    TEST_ASSERT(probe.seen[1] == 2);
    TEST_ASSERT(probe.seen_in_isr[0] == false);
    TEST_ASSERT(probe.seen_in_isr[1] == false);
    TEST_ASSERT(probe.max_depth == 1);
  }

  {
    LambdaCreationProbe probe;
    probe.cb.Run(true, 7);
    TEST_ASSERT(probe.seen_value == 7);
    TEST_ASSERT(probe.seen_in_isr == true);
  }
}

}  // namespace

void test_cb()
{
  TestEmptyAndDirectCallbacks();
  TestGuardedAndLambdaCallbacks();
}
