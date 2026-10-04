/**
 * @file test_stack.cpp
 * @brief 检查栈的后进先出、满空状态及指定位置的插入和删除。 /
 * Tests stack order, full and empty results, insertion and deletion.
 *
 * 不可平凡复制的元素类型只有 Insert 被拒绝（见 stack_insert_nontrivial.cpp），
 * 其余操作照常可用。栈不可复制，析构时释放存储数组。
 * For an element type that is not trivially copyable only Insert is rejected (see
 * stack_insert_nontrivial.cpp); the other operations remain available. A stack cannot
 * be copied and frees its storage array when destroyed.
 */

#include <string>
#include <type_traits>

#include "libxr.hpp"
#include "libxr_def.hpp"
#include "test.hpp"
#include "test_assert.hpp"

static_assert(!std::is_copy_constructible_v<LibXR::Stack<int>>);
static_assert(!std::is_copy_assignable_v<LibXR::Stack<int>>);

namespace
{
// 统计析构次数，用来确认栈析构时释放了存储数组中的每个元素。
// Counts destructor calls to confirm that destroying the stack frees every element of
// its storage array.
struct DestructionCounter
{
  static inline int destroyed = 0;
  ~DestructionCounter() { ++destroyed; }
};
}  // namespace

void test_stack()
{
  DestructionCounter::destroyed = 0;
  {
    LibXR::Stack<DestructionCounter> counters(3);
    TEST_ASSERT(DestructionCounter::destroyed == 0);
  }
  TEST_ASSERT(DestructionCounter::destroyed == 3);

  LibXR::Stack<int> stack(10);
  for (int i = 0; i < 10; i++)
  {
    stack.Push(i);
  }

  TEST_ASSERT(stack.Push(1) == LibXR::ErrorCode::FULL);

  for (int i = 0; i <= 9; i++)
  {
    int tmp = -1;
    stack.Pop(tmp);
    TEST_ASSERT(tmp == 9 - i);
  }

  TEST_ASSERT(stack.Pop() == LibXR::ErrorCode::EMPTY);

  for (int i = 0; i <= 5; i++)
  {
    stack.Push(i);
  }

  stack.Insert(10, 2);
  TEST_ASSERT(stack[2] == 10);
  TEST_ASSERT(stack[3] == 2);
  TEST_ASSERT(stack.Size() == 7);
  stack.Delete(2);
  TEST_ASSERT(stack[2] == 2);
  TEST_ASSERT(stack[3] == 3);
  TEST_ASSERT(stack.Size() == 6);

  LibXR::Stack<std::string> names(2);
  TEST_ASSERT(names.Push(std::string("first")) == LibXR::ErrorCode::OK);
  TEST_ASSERT(names.Push(std::string("second")) == LibXR::ErrorCode::OK);
  names.Delete(0);
  std::string top;
  TEST_ASSERT(names.Pop(top) == LibXR::ErrorCode::OK);
  TEST_ASSERT(top == "second");
}
