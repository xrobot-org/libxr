#pragma once

#include "libxr_def.hpp"
#include "libxr_system.hpp"

namespace LibXR
{

/**
 * @brief 互斥锁类，提供线程同步机制
 *        Mutex class providing thread synchronization mechanisms.
 *
 * 该类实现了一个互斥锁，用于确保多线程环境下的线程安全，支持加锁、解锁，并对中断服务程序（ISR）进行特殊处理。
 * This class implements a mutex for thread-safe operations, supporting
 * locking, unlocking, and special handling for interrupt service routines (ISR).
 */
class Mutex
{
 public:
  /**
   * @brief 构造函数，初始化互斥锁
   *        Constructor to initialize the mutex.
   */
  Mutex();

  /**
   * @brief 析构函数，销毁互斥锁
   *        Destructor to destroy the mutex.
   */
  ~Mutex();

  /**
   * @brief 加锁，如果锁已被占用，则阻塞等待
   *        Lock the mutex, blocking if it is already locked.
   * @return 操作结果 / Operation result:
   *         - `ErrorCode::OK` 表示成功 / `ErrorCode::OK` on success.
   */
  ErrorCode Lock();

  /**
   * @brief 尝试加锁，如果锁已被占用，则立即返回失败
   *        Attempt to lock the mutex, returning immediately if already locked.
   * @return 操作结果 / Operation result:
   *         - `ErrorCode::OK` 表示成功 / `ErrorCode::OK` on success.
   *         - `ErrorCode::BUSY` 表示锁已被占用 / `ErrorCode::BUSY` if the mutex is
   *           already locked.
   */
  [[nodiscard]] ErrorCode TryLock();

  /**
   * @brief 解锁互斥锁
   *        Unlock the mutex.
   */
  void Unlock();

  /**
   * @brief 互斥锁的 RAII 机制封装
   *        RAII-style mechanism for automatic mutex management.
   *
   * 该类确保在构造时自动加锁，并在析构时自动释放锁。
   * This class ensures that a mutex is locked upon construction and
   * automatically released upon destruction.
   */
  class LockGuard
  {
   public:
    /**
     * @brief 构造函数，自动加锁
     *        Constructor automatically locking the mutex.
     * @param mutex 需要管理的互斥锁 / Reference to the mutex to manage.
     */
    LockGuard(Mutex& mutex) : mutex_(mutex) { mutex_.Lock(); }

    /**
     * @brief 析构函数，自动解锁
     *        Destructor automatically unlocking the mutex.
     */
    ~LockGuard() { mutex_.Unlock(); }

   private:
    Mutex& mutex_;  ///< 被管理的互斥锁 / Reference to the managed mutex.
  };

 private:
  libxr_mutex_handle mutex_handle_;  ///< 互斥锁句柄 / Handle for the mutex.
};

}  // namespace LibXR
