/**
 * @file test_double_buffer.cpp
 * @brief 检查双缓冲区初始化、状态标记和切换。 /
 * Tests double-buffer initialization, staging flags and switching.
 *
 * 检查空缓冲区、内存对齐、待发长度记录和双半区切换。
 * Checks empty buffers, alignment, pending/active length tracking and half switching.
 */

#include <cstring>

#include "double_buffer.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "test.hpp"
#include "test_assert.hpp"

void test_double_buffer()
{
  alignas(size_t) uint8_t buff[128] = {};  // 预分配大缓冲区
  LibXR::RawData raw(buff, sizeof(buff));

  LibXR::DoubleBuffer init_later;
  init_later.Init(raw);
  TEST_ASSERT(init_later.Size() == 64);
  TEST_ASSERT(
      (reinterpret_cast<uintptr_t>(init_later.ActiveBuffer()) % alignof(size_t)) == 0U);
  TEST_ASSERT(
      (reinterpret_cast<uintptr_t>(init_later.PendingBuffer()) % alignof(size_t)) == 0U);

  LibXR::DoubleBuffer empty_buffer;
  LibXR::RawData empty_raw(nullptr, 0);
  empty_buffer.Init(empty_raw);
  TEST_ASSERT(empty_buffer.Size() == 0);
  TEST_ASSERT(empty_buffer.ActiveBuffer() == nullptr);
  TEST_ASSERT(empty_buffer.PendingBuffer() == nullptr);
  TEST_ASSERT(empty_buffer.HasPending() == false);
  empty_buffer.SetPendingLength(0);
  empty_buffer.EnablePending();
  TEST_ASSERT(empty_buffer.HasPending() == true);
  TEST_ASSERT(empty_buffer.GetPendingLength() == 0);
  empty_buffer.Switch();
  TEST_ASSERT(empty_buffer.HasPending() == false);
  TEST_ASSERT(empty_buffer.ActiveBuffer() == nullptr);

  LibXR::DoubleBuffer buffer(raw);

  TEST_ASSERT(buffer.Size() == 64);  // 被平分成两块
  TEST_ASSERT((reinterpret_cast<uintptr_t>(buffer.ActiveBuffer()) % alignof(size_t)) ==
              0U);
  TEST_ASSERT((reinterpret_cast<uintptr_t>(buffer.PendingBuffer()) % alignof(size_t)) ==
              0U);
  TEST_ASSERT(buffer.Buffer(0) == buff);
  TEST_ASSERT(buffer.Buffer(1) == buff + 64);

  // 1. 检查初始状态
  TEST_ASSERT(buffer.HasPending() == false);
  TEST_ASSERT(buffer.GetPendingLength() == 0);
  TEST_ASSERT(buffer.GetActiveLength() == 0);

  // 2. 直接写入 pending buffer 并启用 pending 标记
  uint8_t test_data[16];
  for (int i = 0; i < 16; ++i)
  {
    test_data[i] = static_cast<uint8_t>(i);
  }

  std::memcpy(buffer.PendingBuffer(), test_data, 16);
  buffer.SetPendingLength(16);
  TEST_ASSERT(buffer.GetPendingLength() == 0);  // 未 EnablePending 前不可见
  buffer.EnablePending();
  TEST_ASSERT(buffer.HasPending() == true);
  TEST_ASSERT(buffer.GetPendingLength() == 16);
  TEST_ASSERT(std::memcmp(buffer.PendingBuffer(), test_data, 16) == 0);

  // 3. 执行 Switch() 切换并记录 active length
  const size_t pending_len = buffer.GetPendingLength();
  buffer.Switch();
  buffer.SetActiveLength(pending_len);
  TEST_ASSERT(buffer.HasPending() == false);
  TEST_ASSERT(buffer.GetActiveLength() == 16);
  TEST_ASSERT(buffer.ActiveBuffer() == buff + 64);  // 被切换为另一半
  TEST_ASSERT(std::memcmp(buffer.ActiveBuffer(), test_data, 16) == 0);

  // 4. 再次写入另一半并切换回 A 区
  for (int i = 0; i < 10; ++i)
  {
    test_data[i] = static_cast<uint8_t>(i + 100);
  }
  std::memcpy(buffer.PendingBuffer(), test_data, 10);
  buffer.SetPendingLength(10);
  buffer.EnablePending();
  TEST_ASSERT(std::memcmp(buffer.PendingBuffer(), test_data, 10) == 0);

  buffer.Switch();
  TEST_ASSERT(buffer.ActiveBuffer() == buff);  // 又回到了原来的 A 区

  buffer.FlipActiveBlock();
  TEST_ASSERT(buffer.ActiveBuffer() == buff + 64);

  buffer.Reset();
  TEST_ASSERT(buffer.ActiveBuffer() == buff);
  TEST_ASSERT(buffer.HasPending() == false);
  TEST_ASSERT(buffer.GetActiveLength() == 0);
  TEST_ASSERT(buffer.GetPendingLength() == 0);
}
