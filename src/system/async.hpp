#pragma once

#include <atomic>

#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "semaphore.hpp"
#include "thread.hpp"

namespace LibXR
{

/**
 * @brief 单任务异步执行器 / Single-job asynchronous executor.
 *
 * 有线程时由工作线程执行；无多线程支持时，由软件 Timer 推进。
 * 任务完成后保持 DONE，GetStatus 取走完成状态后恢复 READY。
 *
 * Threaded systems use a worker; cooperative systems use a software Timer.
 * Completion stays DONE until GetStatus acknowledges it and restores READY.
 *
 * 对象保留到工作线程退出或系统复位；回调数据保留到任务完成，任务不得抛异常。
 *
 * Keep the object until its worker exits or the system resets. Keep callback data
 * until completion. Jobs must not throw.
 *
 * 裸机从普通上下文推进 Timer；长任务会拖延其他回调，也不能等待同一 Timer 的其他任务。
 *
 * Drive a bare-metal Timer in normal context. Long jobs delay other callbacks
 * and cannot wait for work on the same Timer.
 */
class ASync
{
 public:
  /**
   * @brief 异步任务的状态枚举。
   *        Enumeration of asynchronous task statuses.
   */
  enum class Status : uint32_t
  {
    READY = 0,         ///< 任务已准备就绪 / Task is ready.
    BUSY = 1,          ///< 任务已接收或正在执行 / Job accepted or running.
    DONE = UINT32_MAX  ///< 任务已完成，尚未取走状态 / Completed, awaiting acknowledgment.
  };

  /**
   * @brief 构造 `ASync` 并初始化线程或软件 Timer。
   *        Constructs an `ASync` and initializes its worker or software Timer.
   *
   * @param stack_depth 线程栈深度，无线程后端忽略。
   *                    Worker stack depth; ignored by cooperative backends.
   * @param priority 线程优先级，无线程后端忽略。
   *                 Worker priority; ignored by cooperative backends.
   */
  ASync(size_t stack_depth, Thread::Priority priority);

  /**
   * @brief 任务线程函数，等待信号量并执行任务。
   *        Task thread function that waits for a semaphore and executes the assigned job.
   *
   * 该函数作为 `ASync` 任务的主线程，持续等待 `sem_` 释放后执行任务，
   * 执行完成后更新 `status_` 状态。
   * This function serves as the main thread for `ASync`, continuously
   * waiting for `sem_` to be released before executing a job,
   * and updating `status_` upon completion.
   *
   * @param async 指向 `ASync` 实例的指针。
   *              Pointer to the `ASync` instance.
   */
  static void ThreadFun(ASync* async);

  std::atomic<Status> status_ = Status::READY;  ///< 当前异步任务状态

  /**
   * @brief 获取当前异步任务的状态。
   *        Retrieves the current status of the asynchronous task.
   *
   * 如果任务尚未完成，则返回当前 `status_`。
   * 如果任务已完成，则返回 `DONE` 并重置状态为 `READY`。
   * If the task is not yet completed, it returns the current `status_`.
   * If the task is completed, it returns `DONE` and resets the status to `READY`.
   *
   * @return 返回当前任务状态。
   *         Returns the current task status.
   */
  [[nodiscard]] Status GetStatus()
  {
    Status cur = status_.load(std::memory_order_acquire);
    if (cur != Status::DONE)
    {
      return cur;
    }
    // 只允许一个观察者取走 DONE；迟到的观察者不能覆盖下一次提交的 BUSY。
    // Consume DONE once; a late observer must not overwrite the next submission's BUSY.
    if (status_.compare_exchange_strong(cur, Status::READY, std::memory_order_acq_rel,
                                        std::memory_order_acquire))
    {
      return Status::DONE;
    }
    return cur;
  }

  using Job = LibXR::Callback<ASync*>;

  /**
   * @brief 提交任务 / Submit a job.
   * @param job 预先创建的任务回调 / Previously created job callback.
   * @return READY 时接收并返回 OK，否则返回 BUSY，保留原任务和完成状态。
   *         OK if READY; otherwise BUSY, preserving the existing job and completion.
   * @note 成功表示已接收，任务可能在本调用返回前完成。提交不会同步调用任务。
   *       Success means admission; execution may finish before this call returns.
   *       Submission does not invoke the job inline.
   */
  ErrorCode AssignJob(Job job);

  /**
   * @brief 从回调上下文提交任务 / Submit a job from callback context.
   * @param job 预先创建的任务回调 / Previously created job callback.
   * @param in_isr 当前是否处于 ISR；线程后端据此选择通知方式。
   *               Whether the caller is in an ISR; selects the worker notification path.
   * @return 与 AssignJob 相同：OK 表示接收，BUSY 表示原任务尚未释放。
   *         Same as AssignJob: OK admits the job; BUSY leaves the existing job intact.
   * @note 任务由工作线程或软件 Timer 执行，任务回调收到的 in_isr 为 false。
   *       The worker or software Timer executes the job with in_isr=false.
   */
  ErrorCode AssignJobFromCallback(Job job, bool in_isr);

  Job job_;  ///< 存储分配的异步任务回调 / Stores the assigned asynchronous job callback.
  Semaphore sem_;  ///< 控制任务执行的信号量 / Semaphore controlling task execution.

  Thread thread_handle_;  ///< 处理异步任务的线程 / Thread handling asynchronous tasks.

 private:
  void RunJob();

#ifdef LIBXR_NOT_SUPPORT_MUTI_THREAD
  // BUSY 在写入 job_ 前置位；独立标记只在任务完整写入后发布。
  // BUSY precedes the job_ write; publish this flag only after the complete job is
  // stored.
  std::atomic<uint32_t> pending_{0U};
#endif
};

}  // namespace LibXR
