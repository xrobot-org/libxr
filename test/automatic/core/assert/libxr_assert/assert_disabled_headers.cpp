/**
 * @file assert_disabled_headers.cpp
 * @brief 公共头文件在产品断言关闭时的编译检查。 /
 * Compile check for public headers with product assertions disabled.
 *
 * 实例化对象池和共享 Topic，供严格告警构建检查未使用变量等问题。
 * Instantiates pools and shared topics for strict warning checks.
 */

#include "cdc_to_uart.hpp"
#include "linux_shared_topic.hpp"
#include "object_pool.hpp"

#if defined(LIBXR_DEV_ASSERT_BUILD) || defined(LIBXR_DEBUG_BUILD)
#error "This target checks public headers with product assertions disabled."
#endif

template <typename Pool>
void ExercisePool()
{
  Pool pool(4);
  typename Pool::Handle handle;
  (void)pool.Acquire(handle);
  typename Pool::Handle copy = handle;
  typename Pool::ConstHandle reader = copy;
  reader = std::move(handle);
  typename Pool::ConstHandle reader_copy(reader);
  copy.Reset();
  reader.Reset();
  reader_copy.Reset();
  handle.Reset();
}

void TestAssertDisabledHeaders()
{
  ExercisePool<LibXR::ObjectPool<uint32_t>>();
  ExercisePool<LibXR::SPSCObjectPool<uint32_t>>();
  ExercisePool<LibXR::MPMCObjectPool<uint32_t>>();
}

template class LibXR::LinuxSharedTopic<uint32_t>;
