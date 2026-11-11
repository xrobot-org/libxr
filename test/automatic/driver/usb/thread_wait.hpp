/**
 * @file thread_wait.hpp
 * @brief 多线程设备类测试里的有界等待 / Bounded waits for the multi-threaded device-class
 *        tests.
 *
 * 测试线程要等另一个线程到达某个状态时，等那个状态本身（有上限），不用固定的睡眠：睡眠
 * 在负载高的主机上不够长，测试会悄悄跳过它想覆盖的路径，或者偶尔失败。
 * When a test thread has to wait for another thread to reach a state, it waits for the
 * state itself, with an upper bound, and not for a fixed sleep: on a loaded host the
 * sleep is too short, and the test either skips the path it wants to cover without saying
 * so or fails now and then.
 */

#pragma once

#include <sys/syscall.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

namespace LibXR::USB::Test
{

/// 轮询到条件成立，最多 timeout_ms；返回条件的值 / Poll until the condition holds, at
/// most timeout_ms; returns the condition
template <typename Condition>
bool WaitUntil(Condition condition, unsigned timeout_ms = 5000)
{
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (!condition())
  {
    if (std::chrono::steady_clock::now() >= deadline) return false;
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  }
  return true;
}

/// 调用线程的内核线程号 / The kernel thread id of the calling thread
inline long CurrentTid() { return syscall(SYS_gettid); }

/// 等到内核报告该线程在睡眠（阻塞在信号量或锁上），最多 timeout_ms。阻塞的线程状态是 S，
/// 运行或被抢占的是 R。读不到 /proc 时（不是 Linux）无从判断，睡 50 ms 当作到了。
/// Wait until the kernel reports the thread as sleeping (blocked on a semaphore or a
/// lock), at most timeout_ms. A blocked thread is in state S, a running or preempted one
/// in R. Where /proc cannot be read (not Linux) nothing can be observed, and it sleeps 50
/// ms and takes the thread to be there.
inline bool WaitUntilBlocked(long tid, unsigned timeout_ms = 5000)
{
  const std::string path = "/proc/self/task/" + std::to_string(tid) + "/stat";
  return WaitUntil(
      [&]()
      {
        std::FILE* file = std::fopen(path.c_str(), "r");
        if (file == nullptr)
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          return true;
        }
        char line[512] = {};
        const size_t count = std::fread(line, 1, sizeof(line) - 1, file);
        std::fclose(file);
        const char* close = nullptr;
        for (size_t i = 0; i < count; ++i)
          if (line[i] == ')') close = &line[i];
        // "pid (comm) S ..." : the state follows the last ')'.
        return close != nullptr && close[1] == ' ' && close[2] == 'S';
      },
      timeout_ms);
}

}  // namespace LibXR::USB::Test
