#include "timer.hpp"

#include "lockfree_list.hpp"

using namespace LibXR;

void Timer::Start(TimerHandle handle) { handle->data_.enable_ = true; }

void Timer::Stop(TimerHandle handle) { handle->data_.enable_ = false; }

void Timer::SetCycle(TimerHandle handle, uint32_t cycle)
{
  ASSERT(cycle > 0);
  handle->data_.cycle_ = cycle;
}

void Timer::RefreshThreadFunction(void*)
{
  MillisecondTimestamp time = Thread::GetTime();
  while (true)
  {
    Timer::Refresh();
    Thread::SleepUntil(time, 1);
  }
}

void Timer::Add(TimerHandle handle)
{
  ASSERT(!handle->next_);

  if (!LibXR::Timer::list_)
  {
    LibXR::Timer::list_ = new LibXR::LockFreeList();
#ifdef LIBXR_NOT_SUPPORT_MUTI_THREAD
#else
    thread_handle_.Create<void*>(nullptr, RefreshThreadFunction, "libxr_timer_task",
                                 stack_depth_, priority_);
#endif
  }
  list_->Add(*handle);
}

void Timer::Refresh()
{
  // 第一次 Add() 才创建任务列表和管理线程；在此之前没有任务可刷新。
  // The first Add() creates the task list and the management thread; before that there
  // is no task to refresh.
  if (!LibXR::Timer::list_)
  {
    return;
  }

  auto fun = [](ControlBlock& block)
  {
    if (!block.enable_)
    {
      return ErrorCode::OK;
    }

    block.count_++;

    if (block.count_ >= block.cycle_)
    {
      block.count_ = 0;
      block.Run();
    }

    return ErrorCode::OK;
  };

  list_->Foreach<ControlBlock>(fun);
}
