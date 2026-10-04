/**
 * @file bench_latency.cpp
 * @brief 共享 Topic 单条消息延迟基准 / Shared Topic per-message latency benchmark.
 *
 * 每条消息等待接收确认，检查序号和校验值，并统计发布到接收的延迟。
 * Acknowledge each message, check sequence/checksum, and measure publication-to-receipt
 * latency.
 */

#include <cerrno>
#include <cstring>

#include "linux_shared_topic_bench_common.hpp"

namespace LinuxSharedTopicBench
{

template <size_t PayloadBytes, bool TouchPayload>
int RunLatencyCase(uint64_t count_override = 0)
{
  // 每发一条消息就等子进程确认；延迟从发布前计时到接收完成，不包含确认管道的返回时间。
  // Wait for an acknowledgement per message; measure publication-to-receipt, excluding
  // the pipe reply.
  using Topic = LibXR::LinuxSharedTopic<BenchFrame<PayloadBytes>>;
  using Data = typename Topic::Data;
  using Subscriber = typename Topic::SyncSubscriber;

  const uint64_t count =
      (count_override == 0) ? LatencyCountForPayload<PayloadBytes>() : count_override;

  LibXR::LinuxSharedTopicConfig config = {};
  config.slot_num = 4;
  config.subscriber_num = 1;
  config.queue_num = 4;

  char topic_name[96] = {};
  std::snprintf(topic_name, sizeof(topic_name), "linux_shared_latency_%zu_%d",
                PayloadBytes, static_cast<int>(getpid()));
  (void)Topic::Remove(topic_name);

  int ready_pipe[2] = {-1, -1};
  int ack_pipe[2] = {-1, -1};
  pid_t child = -1;
  auto cleanup = MakeScopeExit(
      [&]()
      {
        CloseFd(ready_pipe[0]);
        CloseFd(ready_pipe[1]);
        CloseFd(ack_pipe[0]);
        CloseFd(ack_pipe[1]);
        KillAndReapChild(child);
        (void)Topic::Remove(topic_name);
      });

  if (pipe(ready_pipe) != 0 || pipe(ack_pipe) != 0)
  {
    std::fprintf(stderr, "latency pipe failed: %s\n", std::strerror(errno));
    return 1;
  }

  Topic publisher(topic_name, config);
  if (!publisher.Valid())
  {
    std::fprintf(stderr, "latency publisher open failed for payload=%zu\n", PayloadBytes);
    return 1;
  }

  child = fork();
  if (child < 0)
  {
    std::fprintf(stderr, "latency fork failed: %s\n", std::strerror(errno));
    return 1;
  }

  if (child == 0)
  {
    CloseFd(ready_pipe[0]);
    CloseFd(ack_pipe[0]);

    Subscriber subscriber(topic_name);
    if (!subscriber.Valid())
    {
      _exit(70);
    }

    const uint8_t ready = 1;
    if (!WriteAll(ready_pipe[1], &ready, sizeof(ready)))
    {
      _exit(71);
    }
    CloseFd(ready_pipe[1]);

    uint64_t expected_seq = 1;
    for (uint64_t i = 0; i < count; ++i)
    {
      Data data;
      if (subscriber.Wait(data, 5000) != LibXR::ErrorCode::OK)
      {
        _exit(72);
      }

      const auto* frame = data.GetData();
      if (frame == nullptr || frame->seq != expected_seq ||
          frame->checksum != ComputeChecksum(*frame))
      {
        _exit(73);
      }
      expected_seq = frame->seq + 1U;

      const uint64_t latency_ns = NowNs() - frame->pub_ns;
      if (!WriteAll(ack_pipe[1], &latency_ns, sizeof(latency_ns)))
      {
        _exit(74);
      }
    }

    CloseFd(ack_pipe[1]);
    _exit(0);
  }

  CloseFd(ready_pipe[1]);
  CloseFd(ack_pipe[1]);

  uint8_t ready = 0;
  if (!ReadAll(ready_pipe[0], &ready, sizeof(ready)) ||
      !WaitForSubscriberAttach(publisher, 1, "latency"))
  {
    return 1;
  }
  CloseFd(ready_pipe[0]);

  std::vector<double> lat_us;
  lat_us.reserve(static_cast<size_t>(count));
  uint64_t create_retries = 0;
  uint64_t publish_retries = 0;

  const uint64_t start_ns = NowNs();
  for (uint64_t seq = 1; seq <= count; ++seq)
  {
    auto fill = [seq](BenchFrame<PayloadBytes>& frame)
    {
      frame.seq = seq;
      frame.pub_ns = NowNs();
      if constexpr (TouchPayload)
      {
        std::memset(frame.payload.data(), static_cast<int>(seq & 0xFFU),
                    frame.payload.size());
      }
      else if constexpr (PayloadBytes > 0)
      {
        frame.payload[0] = static_cast<uint8_t>(seq & 0xFFU);
        frame.payload[PayloadBytes - 1U] = static_cast<uint8_t>((seq >> 8U) & 0xFFU);
      }
      frame.checksum = ComputeChecksum(frame);
    };
    if (!PublishWithRetry(publisher, fill, create_retries, publish_retries, "latency"))
    {
      std::fprintf(stderr, "latency publish failed for payload=%zu seq=%" PRIu64 "\n",
                   PayloadBytes, seq);
      return 1;
    }

    uint64_t latency_ns = 0;
    if (!ReadAll(ack_pipe[0], &latency_ns, sizeof(latency_ns)))
    {
      std::fprintf(stderr, "latency child ack failed for payload=%zu\n", PayloadBytes);
      return 1;
    }
    lat_us.push_back(static_cast<double>(latency_ns) / 1000.0);
  }
  const uint64_t end_ns = NowNs();

  CloseFd(ack_pipe[0]);

  int status = 0;
  while (waitpid(child, &status, 0) == -1 && errno == EINTR)
  {
  }
  child = -1;
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
  {
    std::fprintf(stderr, "latency child failed for payload=%zu\n", PayloadBytes);
    return 1;
  }

  const BenchStats stats = BuildStats<PayloadBytes>(lat_us, 0, 0);
  const double total_s = static_cast<double>(end_ns - start_ns) / 1e9;
  const double exchange_rate = static_cast<double>(count) / total_s;
  std::printf("[BENCH] shared_latency mode=%s payload=%zuB count=%" PRIu64
              " exchange_rate=%.0f msg/s create_retry=%" PRIu64 " publish_retry=%" PRIu64
              " one_way_avg=%.3f us p50=%.3f us p95=%.3f us p99=%.3f us max=%.3f us\n",
              TouchPayload ? "full-touch" : "transport", sizeof(BenchFrame<PayloadBytes>),
              count, exchange_rate, create_retries, publish_retries, stats.avg_us,
              stats.p50_us, stats.p95_us, stats.p99_us, stats.max_us);
  std::fflush(stdout);

  return 0;
}

int RunLatencyBenchmarksSmoke() { return RunLatencyCase<64, false>(128); }

int RunLatencyBenchmarks()
{
  int status = 0;
  status |= RunLatencyCase<64, false>();
  status |= RunLatencyCase<64, true>();
  status |= RunLatencyCase<4096, false>();
  status |= RunLatencyCase<4096, true>();
  status |= RunLatencyCase<65536, false>();
  status |= RunLatencyCase<65536, true>();
  status |= RunLatencyCase<1048576, false>();
  status |= RunLatencyCase<1048576, true>();
  return status;
}
}  // namespace LinuxSharedTopicBench
