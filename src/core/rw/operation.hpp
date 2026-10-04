#pragma once

#include <atomic>
#include <cstdint>
#include <type_traits>
#include <utility>

#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "semaphore.hpp"

namespace LibXR
{

/**
 * @brief 操作的完成通知描述符 / Completion notification descriptor for an operation.
 *
 * @tparam Args 回调参数类型 / Callback argument type.
 * @note 复制描述符仍引用相同通知对象；调用者负责其生命周期。
 *       Copies refer to the same notification targets; callers manage their lifetime.
 */
template <typename Args>
class Operation
{
 public:
  using Callback = LibXR::Callback<Args>;

  /// 完成通知方式 / Completion notification mode.
  enum class OperationType : uint8_t
  {
    CALLBACK,  ///< 执行回调 / Invoke a callback.
    BLOCK,     ///< 信号量等待 / Wait with a semaphore.
    POLLING,   ///< 查询原子状态 / Poll an atomic status.
    NONE       ///< 无完成通知 / No completion notification.
  };

  /// 轮询操作状态；保持字宽，避免 M0 引入 byte-atomic runtime ABI /
  /// Polling status; keep it word-sized to avoid byte-atomic runtime ABI on M0.
  enum class OperationPollingStatus : uint32_t
  {
    READY,    ///< 尚未开始 / Not started.
    RUNNING,  ///< 正在执行 / In progress.
    DONE,     ///< 成功完成 / Completed successfully.
    ERROR     ///< 错误结束 / Completed with an error.
  };

  /// @brief 构造无完成通知的操作 / Construct an operation without notification.
  Operation() : data{nullptr}, type(OperationType::NONE) {}

  /**
   * @brief 构造阻塞操作 / Construct a blocking operation.
   * @param sem 等待完成的信号量 / Semaphore for completion waiting.
   * @param timeout 传给 Semaphore::Wait 的超时值，默认 UINT32_MAX。
   *        Timeout passed to Semaphore::Wait; defaults to UINT32_MAX.
   * @pre 信号量同时只服务一个活动 BLOCK 调用，调用返回后才可复用。
   *      Use the semaphore for one live BLOCK call; reuse only after it returns.
   */
  Operation(Semaphore& sem, uint32_t timeout = UINT32_MAX)
      : data{.sem_info = {&sem, timeout}}, type(OperationType::BLOCK)
  {
  }

  /**
   * @brief 构造回调操作 / Construct a callback operation.
   * @param callback 借用的完成回调 / Borrowed completion callback.
   */
  Operation(Callback& callback)
      : data{.callback = &callback}, type(OperationType::CALLBACK)
  {
  }

  /**
   * @brief 构造轮询操作 / Construct a polling operation.
   * @param status 借用的原子状态，以 acquire 读取完成结果。
   *        Borrowed atomic status; load completion with acquire ordering.
   */
  Operation(std::atomic<OperationPollingStatus>& status)
      : data{.status = &status}, type(OperationType::POLLING)
  {
  }

  /// 复制通知描述符，共用通知对象 / Copy the descriptor, sharing notification targets.
  Operation(const Operation&) = default;
  /// 移动描述符，源描述符保持不变 / Move the descriptor without clearing the source.
  Operation(Operation&&) noexcept = default;

  /**
   * @brief 复制通知描述符 / Copy the notification descriptor.
   * @return 当前描述符 / This descriptor.
   */
  Operation& operator=(const Operation&) = default;

  /**
   * @brief 移动通知描述符 / Move the notification descriptor.
   * @return 当前描述符，源描述符保持不变 / This descriptor; the source remains unchanged.
   */
  Operation& operator=(Operation&&) noexcept = default;

  /**
   * @brief 按通知方式报告完成结果 / Report completion using the selected mode.
   * @tparam Status 完成结果类型 / Completion result type.
   * @param in_isr 当前是否在中断中 / Whether called in an ISR.
   * @param status 传给回调或用于判断轮询成功状态的结果。
   *        Result passed to the callback or used to set polling success or error.
   * @note 回调同步执行；轮询状态以 release 发布。BLOCK 仅释放信号量，NONE 无操作。
   *       Callbacks run inline; polling uses a release store. BLOCK posts; NONE is a
   * no-op.
   */
  template <typename Status>
  void UpdateStatus(bool in_isr, Status&& status)
  {
    switch (type)
    {
      case OperationType::CALLBACK:
        data.callback->Run(in_isr, std::forward<Status>(status));
        break;
      case OperationType::BLOCK:
        // 信号量只负责唤醒，结果由调用方另行保存。
        // The semaphore only wakes the caller; the result is stored separately.
        data.sem_info.sem->PostFromCallback(in_isr);
        break;
      case OperationType::POLLING:
        data.status->store((status == ErrorCode::OK) ? OperationPollingStatus::DONE
                                                     : OperationPollingStatus::ERROR,
                           std::memory_order_release);
        break;
      case OperationType::NONE:
        break;
    }
  }

  /**
   * @brief 标记轮询操作正在执行 / Mark a polling operation as running.
   * @note 以 release 写入 RUNNING，其他模式不执行操作。
   *       Stores RUNNING with release ordering; other modes are unaffected.
   */
  void MarkAsRunning()
  {
    if (type == OperationType::POLLING)
    {
      data.status->store(OperationPollingStatus::RUNNING, std::memory_order_release);
    }
  }

  /// 完成通知对象，由 type 选择有效成员 / Notification target selected by type.
  union
  {
    Callback* callback;  ///< 借用的回调对象 / Borrowed callback.
    struct
    {
      Semaphore* sem;    ///< 借用的信号量 / Borrowed semaphore.
      uint32_t timeout;  ///< 等待超时值 / Wait timeout.
    } sem_info;
    /// 借用的轮询状态 / Borrowed polling status.
    std::atomic<OperationPollingStatus>* status;
  } data;

  /// 当前通知方式 / Current notification mode.
  OperationType type;
};

/**
 * @brief 同步驱动操作的阻塞等待器 / Blocking waiter for synchronous driver operations.
 *
 * 超时解除调用者的等待，迟到的完成只清退状态，不再唤醒已超时的调用者。
 * Timeout detaches the caller. A late completion clears the state without waking it.
 */
class AsyncBlockWait
{
 public:
  /// 32 位等待状态，兼容 STM32 原子适配 / 32-bit wait state for STM32 atomic support.
  enum class State : uint32_t
  {
    IDLE = 0,      ///< 无等待 / No wait.
    PENDING = 1,   ///< 等待完成 / Awaiting completion.
    CLAIMED = 2,   ///< 完成方已认领通知 / Completion notification claimed.
    DETACHED = 3,  ///< 调用者已超时 / Caller timed out.
  };

  /**
   * @brief 绑定信号量并开始等待 / Bind a semaphore and start waiting.
   * @param sem 本次等待使用的信号量 / Semaphore for this wait.
   */
  void Start(Semaphore& sem)
  {
    sem_ = &sem;
    result_ = ErrorCode::OK;
    state_.store(State::PENDING, std::memory_order_release);
  }

  /// @brief 清除等待状态，不发送通知 / Clear the wait state without posting.
  void Cancel() { state_.store(State::IDLE, std::memory_order_release); }

  /**
   * @brief 等待完成或超时交接 / Wait for completion or a timeout handoff.
   * @param timeout 传给信号量的超时值 / Timeout passed to the semaphore.
   * @return 完成结果，或解除等待后的 TIMEOUT / Completion result or detached TIMEOUT.
   * @note 完成方已认领时，须等对应通知后才返回，可能超过指定时间。
   *       A claimed completion must post before return, possibly exceeding the timeout.
   */
  ErrorCode Wait(uint32_t timeout)
  {
    DEV_ASSERT(sem_ != nullptr);
    auto wait_ans = sem_->Wait(timeout);
    if (wait_ans == ErrorCode::OK)
    {
      DEV_ASSERT(state_.load(std::memory_order_acquire) == State::CLAIMED);
      state_.store(State::IDLE, std::memory_order_release);
      return result_;
    }

    State expected = State::PENDING;
    if (state_.compare_exchange_strong(expected, State::DETACHED,
                                       std::memory_order_acq_rel,
                                       std::memory_order_acquire))
    {
      return ErrorCode::TIMEOUT;
    }

    DEV_ASSERT(expected == State::CLAIMED || expected == State::DETACHED ||
               expected == State::IDLE);
    if (expected == State::DETACHED)
    {
      state_.store(State::IDLE, std::memory_order_release);
      return ErrorCode::TIMEOUT;
    }
    if (expected == State::IDLE)
    {
      return ErrorCode::TIMEOUT;
    }

    ErrorCode finish_wait_ans;
    do
    {
      finish_wait_ans = sem_->Wait(UINT32_MAX);
    } while (finish_wait_ans == ErrorCode::TIMEOUT);
    REQUIRE(finish_wait_ans == ErrorCode::OK);
    state_.store(State::IDLE, std::memory_order_release);
    return result_;
  }

  /**
   * @brief 尝试认领完成并唤醒调用者 / Try to claim completion and wake the caller.
   * @param in_isr 当前是否在中断中 / Whether called in an ISR.
   * @param ec 完成结果 / Completion result.
   * @return 成功发送通知为 true，调用者已解除等待则为 false。
   *         True when posted, false when the caller is no longer waiting.
   */
  bool TryPost(bool in_isr, ErrorCode ec)
  {
    DEV_ASSERT_FROM_CALLBACK(sem_ != nullptr, in_isr);

    State expected = State::PENDING;
    if (!state_.compare_exchange_strong(expected, State::CLAIMED,
                                        std::memory_order_acq_rel,
                                        std::memory_order_acquire))
    {
      DEV_ASSERT_FROM_CALLBACK(expected == State::DETACHED || expected == State::IDLE,
                               in_isr);
      if (expected == State::DETACHED)
      {
        expected = State::DETACHED;
        (void)state_.compare_exchange_strong(
            expected, State::IDLE, std::memory_order_acq_rel, std::memory_order_acquire);
      }
      return false;
    }

    result_ = ec;
    sem_->PostFromCallback(in_isr);
    return true;
  }

 private:
  Semaphore* sem_ = nullptr;  ///< 本次等待的信号量 / Semaphore for this wait.
  std::atomic<State> state_{State::IDLE};  ///< 完成与超时的交接状态 / Handoff state.
  ErrorCode result_ = ErrorCode::OK;       ///< 完成方写入的结果 / Result from completion.
};

class ReadPort;
class WritePort;

/// @brief 读操作的完成通知描述符 / Read completion descriptor.
typedef Operation<ErrorCode> ReadOperation;

/// @brief 写操作的完成通知描述符 / Write completion descriptor.
typedef Operation<ErrorCode> WriteOperation;

// RW 元数据存放在字节型 SPSC 环中，因此其指针/枚举操作描述必须保持可平凡复制。
// Byte-backed SPSC storage requires trivially copyable operation descriptors.
static_assert(std::is_trivially_copyable_v<ReadOperation>);
static_assert(std::is_trivially_copyable_v<WriteOperation>);
static_assert(std::is_trivially_destructible_v<ReadOperation>);
static_assert(std::is_trivially_destructible_v<WriteOperation>);

/// @brief 写入进度通知函数类型 / Write progress notification function type.
///
/// 数据已复制入队。普通后端通过 WriteQueue 消费并在析构时结算；Pipe 通知共享队列读端。
/// 本函数只通知进展，不返回完成结果。
/// Payload is already queued. Queued backends consume via WriteQueue and settle on
/// destruction; Pipe notifies its reader. This function returns no completion result.
typedef void (*WriteFun)(WritePort& port, bool in_isr);

}  // namespace LibXR
