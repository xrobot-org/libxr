/**
 * @file linux_shared_topic_bench_common.hpp
 * @brief 共享 Topic 基准辅助 / Helpers for shared Topic benchmarks.
 *
 * 提供消息数据、计时统计和进程间通信辅助，由各基准文件调用。
 * Provide frames, timing statistics and process-communication helpers for the benchmark
 * files.
 */

#pragma once

#include <sched.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <type_traits>
#include <utility>
#include <vector>

#include "libxr.hpp"

namespace LinuxSharedTopicBench
{

using Clock = std::chrono::steady_clock;

inline uint64_t BenchScaleDivisor()
{
  const char* text = std::getenv("LIBXR_BENCH_SCALE_DIV");
  if (text == nullptr || text[0] == '\0')
  {
    return 1;
  }

  char* end = nullptr;
  const unsigned long long parsed = std::strtoull(text, &end, 10);
  if (end == text || (end != nullptr && *end != '\0') || parsed == 0)
  {
    return 1;
  }
  return static_cast<uint64_t>(parsed);
}

inline uint64_t ScaleBenchCount(uint64_t base_count, uint64_t minimum_count)
{
  const uint64_t divisor = BenchScaleDivisor();
  if (divisor <= 1)
  {
    return base_count;
  }

  const uint64_t scaled = base_count / divisor;
  return (scaled >= minimum_count) ? scaled : minimum_count;
}

inline uint64_t NowNs()
{
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   Clock::now().time_since_epoch())
                                   .count());
}

struct BenchStats
{
  uint64_t count = 0;
  uint64_t sequence_errors = 0;
  uint64_t timeout_errors = 0;
  double min_us = 0.0;
  double avg_us = 0.0;
  double p50_us = 0.0;
  double p95_us = 0.0;
  double p99_us = 0.0;
  double max_us = 0.0;
};

struct OverloadStats
{
  BenchStats latency = {};
  uint64_t subscriber_drop_num = 0;
  uint64_t sequence_gap = 0;
};

struct ModeSubConfig
{
  LibXR::LinuxSharedSubscriberMode mode =
      LibXR::LinuxSharedSubscriberMode::BROADCAST_FULL;
  uint32_t delay_us = 0;
  const char* label = "";
};

struct ModeSubResult
{
  BenchStats latency = {};
  uint64_t recv_count = 0;
  uint64_t drop_count = 0;
  uint64_t first_seq = 0;
  uint64_t last_seq = 0;
  uint64_t timeout_errors = 0;
};

template <size_t PayloadBytes>
struct BenchFrame
{
  uint64_t seq = 0;
  uint64_t pub_ns = 0;
  uint32_t checksum = 0;
  uint32_t reserved = 0;
  std::array<uint8_t, PayloadBytes> payload = {};
};

template <size_t PayloadBytes>
uint32_t ComputeChecksum(const BenchFrame<PayloadBytes>& frame)
{
  return static_cast<uint32_t>((frame.seq * 1315423911ULL) ^
                               (frame.pub_ns * 2654435761ULL) ^ PayloadBytes);
}

template <size_t PayloadBytes>
uint64_t CountForPayload()
{
  if constexpr (PayloadBytes <= 64)
  {
    return ScaleBenchCount(100000, 64);
  }
  else if constexpr (PayloadBytes <= 4096)
  {
    return ScaleBenchCount(50000, 64);
  }
  else if constexpr (PayloadBytes <= 65536)
  {
    return ScaleBenchCount(5000, 32);
  }
  else
  {
    return ScaleBenchCount(256, 8);
  }
}

template <size_t PayloadBytes>
LibXR::LinuxSharedTopicConfig ConfigForPayload()
{
  LibXR::LinuxSharedTopicConfig config;
  config.subscriber_num = 1;
  if constexpr (PayloadBytes <= 64)
  {
    config.slot_num = 4096;
    config.queue_num = 4096;
  }
  else if constexpr (PayloadBytes <= 4096)
  {
    config.slot_num = 2048;
    config.queue_num = 2048;
  }
  else if constexpr (PayloadBytes <= 65536)
  {
    config.slot_num = 256;
    config.queue_num = 256;
  }
  else
  {
    config.slot_num = 64;
    config.queue_num = 64;
  }
  return config;
}

template <size_t PayloadBytes>
uint64_t LatencyCountForPayload()
{
  if constexpr (PayloadBytes <= 64)
  {
    return ScaleBenchCount(20000, 64);
  }
  else if constexpr (PayloadBytes <= 4096)
  {
    return ScaleBenchCount(10000, 64);
  }
  else if constexpr (PayloadBytes <= 65536)
  {
    return ScaleBenchCount(2000, 32);
  }
  else
  {
    return ScaleBenchCount(128, 8);
  }
}

template <size_t PayloadBytes>
BenchStats BuildStats(const std::vector<double>& lat_us, uint64_t sequence_errors,
                      uint64_t timeout_errors)
{
  BenchStats stats = {};
  stats.sequence_errors = sequence_errors;
  stats.timeout_errors = timeout_errors;
  if (lat_us.empty())
  {
    return stats;
  }

  stats.count = lat_us.size();
  stats.min_us = lat_us.front();
  stats.max_us = lat_us.front();
  double sum_us = 0.0;
  for (double value : lat_us)
  {
    stats.min_us = std::min(stats.min_us, value);
    stats.max_us = std::max(stats.max_us, value);
    sum_us += value;
  }
  stats.avg_us = sum_us / static_cast<double>(lat_us.size());

  std::vector<double> sorted = lat_us;
  std::sort(sorted.begin(), sorted.end());
  auto percentile = [&](double p)
  {
    const size_t index = static_cast<size_t>(
        std::floor((static_cast<double>(sorted.size() - 1U)) * p + 0.5));
    return sorted[index];
  };
  stats.p50_us = percentile(0.50);
  stats.p95_us = percentile(0.95);
  stats.p99_us = percentile(0.99);
  return stats;
}

template <typename Fn>
class ScopeExit
{
 public:
  explicit ScopeExit(Fn fn) : fn_(std::move(fn)) {}

  ScopeExit(const ScopeExit&) = delete;
  ScopeExit& operator=(const ScopeExit&) = delete;

  ScopeExit(ScopeExit&& other) noexcept
      : fn_(std::move(other.fn_)), active_(other.active_)
  {
    other.active_ = false;
  }

  ~ScopeExit()
  {
    if (active_)
    {
      fn_();
    }
  }

 private:
  Fn fn_;
  bool active_ = true;
};

template <typename Fn>
auto MakeScopeExit(Fn&& fn)
{
  return ScopeExit<std::decay_t<Fn>>(std::forward<Fn>(fn));
}

inline void CloseFd(int& fd)
{
  if (fd >= 0)
  {
    close(fd);
    fd = -1;
  }
}

inline void KillAndReapChild(pid_t& child)
{
  if (child <= 0)
  {
    return;
  }

  (void)::kill(child, SIGTERM);
  int child_status = 0;
  while (waitpid(child, &child_status, 0) == -1 && errno == EINTR)
  {
  }
  child = -1;
}

inline bool WriteAll(int fd, const void* buffer, size_t size)
{
  const auto* bytes = static_cast<const uint8_t*>(buffer);
  size_t written_total = 0;
  while (written_total < size)
  {
    const ssize_t written = write(fd, bytes + written_total, size - written_total);
    if (written > 0)
    {
      written_total += static_cast<size_t>(written);
      continue;
    }
    if (written < 0 && errno == EINTR)
    {
      continue;
    }
    return false;
  }
  return true;
}

inline bool ReadAll(int fd, void* buffer, size_t size)
{
  auto* bytes = static_cast<uint8_t*>(buffer);
  size_t read_total = 0;
  while (read_total < size)
  {
    const ssize_t read_size = read(fd, bytes + read_total, size - read_total);
    if (read_size > 0)
    {
      read_total += static_cast<size_t>(read_size);
      continue;
    }
    if (read_size < 0 && errno == EINTR)
    {
      continue;
    }
    return false;
  }
  return true;
}

template <typename TopicType>
bool WaitForSubscriberAttach(TopicType& topic, uint32_t expected_num,
                             const char* case_label)
{
  for (int retry = 0; retry < 500 && topic.GetSubscriberNum() < expected_num; ++retry)
  {
    usleep(1000);
  }
  if (topic.GetSubscriberNum() < expected_num)
  {
    std::fprintf(stderr, "%s subscriber attach timeout: expected=%u actual=%u\n",
                 case_label, expected_num, topic.GetSubscriberNum());
    return false;
  }
  return true;
}

/// 一条消息的重试期限，与订阅子进程的等待超时相同 / Retry deadline for one message, the
/// same as the wait timeout of the subscriber child.
inline constexpr uint64_t PUBLISH_RETRY_TIMEOUT_NS = 5000ULL * 1000ULL * 1000ULL;

/**
 * @brief 申请槽位、填写并发布一条消息 / Acquire a slot, fill it and publish one message.
 *
 * Publish() 返回 FULL 时已经释放句柄，所以每次重试都重新调用 CreateData() 并重新填写。
 * CreateData() 和 Publish() 只在返回 FULL 时重试；遇到其他错误码，或超过
 * PUBLISH_RETRY_TIMEOUT_NS 仍未发布时，输出原因并返回 false。
 * Publish() has released the handle when it returns FULL, so every retry calls
 * CreateData() again and refills the payload. CreateData() and Publish() are retried only
 * on FULL; on any other error code, or when the message is still unpublished after
 * PUBLISH_RETRY_TIMEOUT_NS, the reason is printed and false is returned.
 *
 * @param fill 填写 payload 的函数，每次申请到槽位后调用 /
 *        Function that fills the payload, called after every successful acquisition.
 */
template <typename TopicType, typename FillFn>
bool PublishWithRetry(TopicType& publisher, FillFn&& fill, uint64_t& create_retries,
                      uint64_t& publish_retries, const char* case_label)
{
  const uint64_t deadline_ns = NowNs() + PUBLISH_RETRY_TIMEOUT_NS;
  while (true)
  {
    typename TopicType::Data data;
    const LibXR::ErrorCode create_ans = publisher.CreateData(data);
    if (create_ans == LibXR::ErrorCode::OK)
    {
      fill(*data.GetData());
      const LibXR::ErrorCode publish_ans = publisher.Publish(data);
      if (publish_ans == LibXR::ErrorCode::OK)
      {
        return true;
      }
      if (publish_ans != LibXR::ErrorCode::FULL)
      {
        std::fprintf(stderr, "%s Publish failed: error %d\n", case_label,
                     static_cast<int>(publish_ans));
        return false;
      }
      ++publish_retries;
    }
    else if (create_ans == LibXR::ErrorCode::FULL)
    {
      ++create_retries;
    }
    else
    {
      std::fprintf(stderr, "%s CreateData failed: error %d\n", case_label,
                   static_cast<int>(create_ans));
      return false;
    }

    if (NowNs() > deadline_ns)
    {
      std::fprintf(stderr,
                   "%s publish retry timeout: create_retry=%" PRIu64
                   " publish_retry=%" PRIu64 "\n",
                   case_label, create_retries, publish_retries);
      return false;
    }
    sched_yield();
  }
}

int RunStandardBenchmarksSmoke();
int RunStandardBenchmarks();
int RunLatencyBenchmarksSmoke();
int RunLatencyBenchmarks();
int RunOverloadBenchmarksSmoke();
int RunOverloadBenchmarks();
int RunModeBenchmarksSmoke();
int RunModeBenchmarks();
}  // namespace LinuxSharedTopicBench
