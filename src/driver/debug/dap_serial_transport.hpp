#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "crc.hpp"
#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "libxr_time.hpp"
#include "libxr_type.hpp"
#include "packet/packet.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "uart.hpp"

namespace LibXR
{

/**
 * @class DapSerialTransport
 * @brief 把 CMSIS-DAP 命令跑在 libxr Topic 打包帧上的通用串口传输 /
 *        Generic serial transport that carries CMSIS-DAP commands in libxr Topic
 *        packet frames.
 *
 * @tparam DapT 提供 `ExecuteExternal` 的 DAP 实现 / DAP implementation exposing
 *         `ExecuteExternal`.
 * @tparam MaxPacket DAP 包长上限（`DAP_Info` 报告的包长）/ Maximum DAP packet size.
 *
 * 线上帧与 `Topic` 完全一致：`prefix(1) + data_len(3, LE) + topic_crc32(4) +
 * timestamp_us(6) + version(1) + header_crc8(1) + payload + crc8(1)`；请求与响应的
 * 载荷都是 `len(u16 LE) + DAP 包`。
 * The wire frame is exactly the `Topic` packet: `prefix(1) + data_len(3, LE) +
 * topic_crc32(4) + timestamp_us(6) + version(1) + header_crc8(1) + payload + crc8(1)`;
 * the request and response payloads are both `len(u16 LE) + DAP packet`.
 *
 * @note 本头文件只依赖 libxr 公共接口（`UART`/`Thread`/`Semaphore`/`CRC8`/`Timebase`
 *       与 `Topic` 的帧布局常量），不含芯片、RTOS 或厂商采样代码；host 上用 mock UART
 *       即可编译和验证。
 *       This header depends only on public libxr interfaces (`UART`, `Thread`,
 *       `Semaphore`, `CRC8`, `Timebase`, and the `Topic` frame-layout constants). It
 *       carries no chip, RTOS, or vendor sampling code, so a mock UART builds and
 *       exercises it on a host.
 *
 * @note 帧拼接、CRC 校验与重新同步都在本模块内完成，因此 `Stats` 能分别报告头 CRC、
 *       尾 CRC 与头字段错误；语义与 `Topic::Server` 一致：出错时丢弃当前帧并跳到下一个
 *       `prefix` 字节继续。
 *       Frame assembly, CRC validation, and resynchronization live here, so `Stats`
 *       separates header-CRC, trailing-CRC, and header-field errors. The semantics
 *       match `Topic::Server`: on error the frame is dropped and parsing resumes at the
 *       next `prefix` byte.
 */
template <typename DapT, uint16_t MaxPacket>
class DapSerialTransport
{
 public:
  static_assert(MaxPacket > 0U, "DapSerialTransport requires a positive packet size");
  static_assert(MaxPacket <= 0xF000U,
                "DapSerialTransport packet size must fit the u16 length prefix");

  /// 最大 DAP 包长 / Maximum DAP packet size.
  static constexpr uint16_t kMaxPacket = MaxPacket;
  /// 打包帧固定头长度（16 字节）/ Fixed packet header size (16 bytes).
  static constexpr size_t kHeaderSize = sizeof(Topic::PackedDataHeader);
  /// 载荷容量：`len(u16) + DAP 包` / Payload capacity: `len(u16) + DAP packet`.
  static constexpr size_t kPayloadCapacity = 2U + MaxPacket;
  /// 单次从 UART 取走的字节上限 / Maximum bytes taken from the UART per read.
  static constexpr size_t kChunkSize = 512U;
  /// 一帧的最大字节数：头 + 载荷 + 尾 CRC / Largest frame: header + payload + tail CRC.
  static constexpr size_t kFrameCapacity = kHeaderSize + kPayloadCapacity + 1U;

  /// 工作线程参数 / Worker thread parameters.
  struct ThreadConfig
  {
    const char* name = "dap_serial";                     ///< 线程名 / Thread name.
    size_t stack_depth = 8192U;                          ///< 栈大小（字节）/ Stack bytes.
    Thread::Priority priority = Thread::Priority::HIGH;  ///< 优先级 / Priority.
    uint8_t core = 0xFFU;  ///< 平台核绑定提示，0xFF 表示不关心 / Core hint; 0xFF = any.
  };

  /// 平台线程启动器 / Platform thread starter.
  ///
  /// 需要把执行 DAP 的线程固定到某个核时（例如只在该核有效的专用 GPIO 指令）由平台提供；
  /// 模块本身不包含任何 RTOS 调用。
  /// Platforms that must pin the DAP-executing thread to one core (for example
  /// dedicated-GPIO instructions valid only on the creating core) supply a starter. The
  /// module itself contains no RTOS calls.
  using ThreadStarter = void (*)(void* ctx, const ThreadConfig& config,
                                 void (*entry)(void*), void* arg);

  /// 统计快照 / Statistics snapshot.
  struct Stats
  {
    uint32_t frames = 0;            ///< 已执行的请求帧数 / Request frames executed.
    uint32_t rx_bytes = 0;          ///< 收到的字节数 / Bytes received.
    uint32_t tx_bytes = 0;          ///< 发出的字节数 / Bytes transmitted.
    uint32_t crc_errors = 0;        ///< 头/尾 CRC8 错误 / Header or trailing CRC8 errors.
    uint32_t header_errors = 0;     ///< 版本/topic/长度非法 / Bad version, topic, length.
    uint32_t dropped = 0;           ///< 丢弃的帧或字节 / Dropped frames or bytes.
    uint32_t rx_errors = 0;         ///< 串口读失败 / UART read failures.
    uint32_t tx_errors = 0;         ///< 串口写失败或超时 / UART write failures/timeouts.
    uint32_t last_activity_ms = 0;  ///< 最近一次请求的时间 / Last request time (ms).
  };

  /**
   * @brief 构造传输 / Construct the transport.
   * @param dap DAP 实现 / DAP implementation.
   * @param uart 已初始化的串口 / Initialized UART.
   * @note 只保存引用，不做任何分配 / Stores references only; allocates nothing.
   */
  DapSerialTransport(DapT& dap, UART& uart) : dap_(dap), uart_(uart) {}

  DapSerialTransport(const DapSerialTransport&) = delete;
  DapSerialTransport& operator=(const DapSerialTransport&) = delete;

  /**
   * @brief 注册平台线程启动器 / Register a platform thread starter.
   * @param starter 启动器；为空时用 `LibXR::Thread` / Starter; null uses `LibXR::Thread`.
   * @param ctx 启动器上下文 / Starter context.
   */
  void SetThreadStarter(ThreadStarter starter, void* ctx = nullptr)
  {
    starter_ = starter;
    starter_ctx_ = ctx;
  }

  /**
   * @brief 启动工作线程 / Start the worker thread.
   * @param config 线程参数 / Thread parameters.
   * @return 成功返回 OK，重复启动返回 BUSY，串口不可读写返回 NOT_SUPPORT /
   *         OK on success, BUSY when already started, NOT_SUPPORT when the UART cannot
   *         read or write.
   */
  ErrorCode Start(const ThreadConfig& config = ThreadConfig())
  {
    if (started_)
    {
      return ErrorCode::BUSY;
    }
    if (uart_.read_port_ == nullptr || !uart_.read_port_->Readable() ||
        uart_.write_port_ == nullptr)
    {
      return ErrorCode::NOT_SUPPORT;
    }

    stop_.store(false, std::memory_order_release);
    running_.store(true, std::memory_order_release);
    started_ = true;
    if (starter_ != nullptr)
    {
      starter_(starter_ctx_, config, &DapSerialTransport::Entry, this);
    }
    else
    {
      thread_.Create(static_cast<void*>(this), &DapSerialTransport::Entry, config.name,
                     config.stack_depth, config.priority);
    }
    return ErrorCode::OK;
  }

  /**
   * @brief 停止工作线程 / Stop the worker thread.
   * @note 幂等；阻塞等待工作线程退出（最长 1 s），等待中的读取最长 `kWaitTimeoutMs`
   * 返回。 Idempotent. Blocks until the worker exits (at most 1 s); a pending read
   * returns within `kWaitTimeoutMs`.
   */
  void Stop()
  {
    if (!started_)
    {
      return;
    }
    stop_.store(true, std::memory_order_release);
    (void)exited_.Wait(kStopTimeoutMs);
    started_ = false;
  }

  /// 工作线程是否在运行 / Whether the worker thread is running.
  [[nodiscard]] bool Running() const { return running_.load(std::memory_order_acquire); }

  /// 读取统计快照 / Read a statistics snapshot.
  [[nodiscard]] Stats GetStats() const
  {
    Stats out;
    out.frames = frames_.load(std::memory_order_relaxed);
    out.rx_bytes = rx_bytes_.load(std::memory_order_relaxed);
    out.tx_bytes = tx_bytes_.load(std::memory_order_relaxed);
    out.crc_errors = crc_errors_.load(std::memory_order_relaxed);
    out.header_errors = header_errors_.load(std::memory_order_relaxed);
    out.dropped = dropped_.load(std::memory_order_relaxed);
    out.rx_errors = rx_errors_.load(std::memory_order_relaxed);
    out.tx_errors = tx_errors_.load(std::memory_order_relaxed);
    out.last_activity_ms = last_activity_ms_.load(std::memory_order_relaxed);
    return out;
  }

  /// 清零统计 / Clear the counters.
  void ResetStats()
  {
    frames_.store(0U, std::memory_order_relaxed);
    rx_bytes_.store(0U, std::memory_order_relaxed);
    tx_bytes_.store(0U, std::memory_order_relaxed);
    crc_errors_.store(0U, std::memory_order_relaxed);
    header_errors_.store(0U, std::memory_order_relaxed);
    dropped_.store(0U, std::memory_order_relaxed);
    rx_errors_.store(0U, std::memory_order_relaxed);
    tx_errors_.store(0U, std::memory_order_relaxed);
    last_activity_ms_.store(0U, std::memory_order_relaxed);
  }

  /**
   * @brief 直接送入一段已收到的字节 / Feed one received byte run directly.
   * @param data 数据 / Data.
   * @param len 长度 / Length.
   * @note `Start` 的工作线程用它消费串口；裸机或协作调度平台（以及 host 测试）也可以
   *       在非中断上下文自行调用，不必启动线程。整帧在这里同步执行并回复。
   *       The `Start` worker uses this to consume the UART. Bare-metal or cooperatively
   *       scheduled platforms (and host tests) may also call it outside an ISR without
   *       starting a thread. Whole frames execute and reply synchronously here.
   */
  void Feed(const uint8_t* data, size_t len)
  {
    if (data == nullptr || len == 0U)
    {
      return;
    }
    rx_bytes_.fetch_add(static_cast<uint32_t>(len), std::memory_order_relaxed);

    size_t index = 0U;
    while (index < len)
    {
      if (!have_header_ && rx_fill_ == 0U)
      {
        // 还没有候选帧：直接在输入流里找前缀，跳过的字节全部算丢弃 /
        // No candidate frame yet: hunt for the prefix in the input stream and count the
        // skipped bytes as dropped.
        const auto* base = data + index;
        const size_t left = len - index;
        const void* hit = std::memchr(base, Topic::PACKET_PREFIX, left);
        if (hit == nullptr)
        {
          dropped_.fetch_add(static_cast<uint32_t>(left), std::memory_order_relaxed);
          return;
        }
        const size_t skip = static_cast<size_t>(static_cast<const uint8_t*>(hit) - base);
        if (skip != 0U)
        {
          dropped_.fetch_add(static_cast<uint32_t>(skip), std::memory_order_relaxed);
          index += skip;
        }
        rx_frame_[rx_fill_++] = Topic::PACKET_PREFIX;
        ++index;
        continue;
      }

      const size_t target = have_header_ ? rx_total_ : kHeaderSize;
      const size_t take = LibXR::min(target - rx_fill_, len - index);
      if (take != 0U)
      {
        std::memcpy(rx_frame_ + rx_fill_, data + index, take);
        rx_fill_ += take;
        index += take;
      }

      // 用缓冲区里已有的字节把解析推进到不能再推进为止 /
      // Advance the parser as far as the buffered bytes allow.
      for (;;)
      {
        if (!have_header_)
        {
          if (rx_fill_ < kHeaderSize)
          {
            break;
          }
          if (AcceptHeader())
          {
            continue;
          }
          Resync();
          if (rx_fill_ == 0U)
          {
            break;
          }
          continue;
        }

        if (rx_fill_ < rx_total_)
        {
          break;
        }
        if (!AcceptTail())
        {
          crc_errors_.fetch_add(1U, std::memory_order_relaxed);
          Resync();
          continue;
        }
        OnFrame(rx_frame_ + kHeaderSize, rx_payload_len_);
        ResetFrame();
      }
    }
  }

  /**
   * @brief 执行一次“等待 -> 取数 -> 解析”/ Run one wait, read, and parse cycle.
   * @return 继续运行时返回 true，收到停止请求时返回 false /
   *         True to keep running, false once a stop was requested.
   * @note 这是 `Start` 工作线程的循环体，也是协作调度与 host 测试的单步入口。
   *       This is the loop body of the `Start` worker and the single-step entry point for
   *       cooperative schedulers and host tests.
   *
   * @note 先用零长度 BLOCK 读等“队列非空”（不消费字节），再按 `read_port_->Size()`
   *       取 `min(size, kChunkSize)` 做一次正长度读。直接发定长读会等满，重新引入
   *       20 ms 级批处理延迟。
   *       Wait for a nonempty queue with a zero-length BLOCK read (consumes nothing),
   * then take `min(Size(), kChunkSize)` with one positive-length read. A fixed-length
   *       read would wait for the whole length and reintroduce 20 ms-scale batching.
   */
  bool Poll()
  {
    if (stop_.load(std::memory_order_acquire))
    {
      return false;
    }
    if (uart_.read_port_ == nullptr || !uart_.read_port_->Readable())
    {
      rx_errors_.fetch_add(1U, std::memory_order_relaxed);
      Thread::Sleep(1U);
      return true;
    }

    ReadOperation wait_op(wait_sem_, kWaitTimeoutMs);
    const ErrorCode wait_ec =
        uart_.Read(RawData(static_cast<void*>(nullptr), 0U), wait_op);
    if (wait_ec == ErrorCode::TIMEOUT)
    {
      return true;
    }
    if (wait_ec != ErrorCode::OK)
    {
      rx_errors_.fetch_add(1U, std::memory_order_relaxed);
      Thread::Sleep(1U);
      return true;
    }

    const size_t queued = uart_.read_port_->Size();
    if (queued == 0U)
    {
      return true;
    }
    const size_t take = LibXR::min(queued, kChunkSize);

    ReadOperation read_op(read_sem_);
    const ErrorCode read_ec = uart_.Read(RawData(rx_chunk_, take), read_op);
    if (read_ec != ErrorCode::OK)
    {
      rx_errors_.fetch_add(1U, std::memory_order_relaxed);
      return true;
    }
    Feed(rx_chunk_, take);
    return true;
  }

 private:
  /// 等待接收队列非空的最长时间（ms）/ Longest wait for a nonempty queue (ms).
  static constexpr uint32_t kWaitTimeoutMs = 50U;
  /// 单帧发送的最长等待时间（ms）/ Longest wait for one frame to be accepted (ms).
  static constexpr uint32_t kTxTimeoutMs = 200U;
  /// `Stop` 等待工作线程退出的最长时间（ms）/ Longest `Stop` wait for the worker (ms).
  static constexpr uint32_t kStopTimeoutMs = 1000U;

  static void Entry(void* arg) { static_cast<DapSerialTransport*>(arg)->Run(); }

  void Run()
  {
    while (Poll())
    {
    }
    running_.store(false, std::memory_order_release);
    exited_.Post();
  }

  /// 校验帧头；成功时确定整帧长度 / Validate the header and fix the frame length.
  bool AcceptHeader()
  {
    if (!CRC8::Verify(rx_frame_, kHeaderSize))
    {
      crc_errors_.fetch_add(1U, std::memory_order_relaxed);
      return false;
    }
    auto* header = reinterpret_cast<Topic::PackedDataHeader*>(rx_frame_);
    if (header->version != Topic::PACKET_VERSION ||
        header->topic_name_crc32 != req_crc32_)
    {
      header_errors_.fetch_add(1U, std::memory_order_relaxed);
      return false;
    }
    const uint32_t payload_len = header->GetDataLen();
    if (payload_len < 2U || payload_len > kPayloadCapacity)
    {
      header_errors_.fetch_add(1U, std::memory_order_relaxed);
      return false;
    }
    rx_payload_len_ = payload_len;
    rx_total_ = kHeaderSize + payload_len + 1U;
    have_header_ = true;
    return true;
  }

  /// 校验尾 CRC8 / Validate the trailing CRC8.
  [[nodiscard]] bool AcceptTail() const
  {
    return CRC8::Verify(rx_frame_, kHeaderSize + rx_payload_len_ + 1U);
  }

  void ResetFrame()
  {
    rx_fill_ = 0U;
    rx_total_ = 0U;
    rx_payload_len_ = 0U;
    have_header_ = false;
  }

  /**
   * @brief 丢弃当前候选帧，从缓冲区里的下一个前缀字节重新同步 /
   *        Drop the current candidate and resynchronize on the next buffered prefix byte.
   * @note 与 `Topic::Server` 相同：只保留从下一个前缀开始的后缀，其余计为丢弃。
   *       Same as `Topic::Server`: keep only the suffix starting at the next prefix and
   *       count the rest as dropped.
   */
  void Resync()
  {
    const size_t have = rx_fill_;
    size_t next = 1U;
    while (next < have && rx_frame_[next] != Topic::PACKET_PREFIX)
    {
      ++next;
    }
    dropped_.fetch_add(static_cast<uint32_t>(next), std::memory_order_relaxed);
    if (next == have)
    {
      ResetFrame();
      return;
    }
    std::memmove(rx_frame_, rx_frame_ + next, have - next);
    rx_fill_ = have - next;
    rx_total_ = 0U;
    rx_payload_len_ = 0U;
    have_header_ = false;
  }

  /// 执行一条请求并回复响应 / Execute one request and write the response.
  void OnFrame(const uint8_t* payload, uint32_t payload_len)
  {
    const size_t request_len =
        static_cast<size_t>(payload[0]) | (static_cast<size_t>(payload[1]) << 8U);
    if (request_len > MaxPacket || request_len + 2U > payload_len)
    {
      dropped_.fetch_add(1U, std::memory_order_relaxed);
      return;
    }

    last_activity_ms_.store(static_cast<uint32_t>(Timebase::GetMilliseconds()),
                            std::memory_order_relaxed);

    uint16_t out_len = 0U;
    (void)dap_.ExecuteExternal(payload + 2U, static_cast<uint16_t>(request_len), resp_,
                               MaxPacket, out_len);
    if (out_len > MaxPacket)
    {
      out_len = MaxPacket;
    }

    WriteFrame(PackResponse(out_len));

    frames_.fetch_add(1U, std::memory_order_relaxed);
  }

  /// 打包一条响应帧，返回整帧长度（含尾 CRC）/ Pack one response frame and return its
  /// total length including the trailing CRC.
  size_t PackResponse(uint16_t out_len)
  {
    const uint32_t payload_len = 2U + out_len;
    auto* header = reinterpret_cast<Topic::PackedDataHeader*>(tx_frame_);
    header->prefix = Topic::PACKET_PREFIX;
    header->version = Topic::PACKET_VERSION;
    header->topic_name_crc32 = resp_crc32_;
    header->SetDataLen(payload_len);
    header->SetTimestamp(Timebase::GetMicroseconds());
    header->pack_header_crc8 = CRC8::Calculate(tx_frame_, kHeaderSize - 1U);

    uint8_t* out = tx_frame_ + kHeaderSize;
    out[0] = static_cast<uint8_t>(out_len);
    out[1] = static_cast<uint8_t>(out_len >> 8U);
    if (out_len != 0U)
    {
      std::memcpy(out + 2U, resp_, out_len);
    }

    const size_t body = kHeaderSize + payload_len;
    tx_frame_[body] = CRC8::Calculate(tx_frame_, body);
    return body + 1U;
  }

  void WriteFrame(size_t len)
  {
    // 写端口在队列放不下时立即返回 FULL（BLOCK 不排队等待），所以这里自己重试到超时。
    // The write port returns FULL immediately when the queue cannot take the frame
    // (BLOCK does not wait there), so the retry loop keeps the bound here.
    for (uint32_t waited = 0U;; ++waited)
    {
      WriteOperation op(tx_sem_, kTxTimeoutMs);
      const ErrorCode ec = uart_.Write(ConstRawData(tx_frame_, len), op);
      if (ec == ErrorCode::OK)
      {
        tx_bytes_.fetch_add(static_cast<uint32_t>(len), std::memory_order_relaxed);
        return;
      }
      if (ec != ErrorCode::FULL || waited >= kTxTimeoutMs)
      {
        tx_errors_.fetch_add(1U, std::memory_order_relaxed);
        return;
      }
      Thread::Sleep(1U);
    }
  }

  DapT& dap_;
  UART& uart_;

  ThreadStarter starter_ = nullptr;
  void* starter_ctx_ = nullptr;

  const uint32_t req_crc32_ = CRC32::Calculate("dap_req", 7U);
  const uint32_t resp_crc32_ = CRC32::Calculate("dap_resp", 8U);

  Thread thread_{};
  bool started_ = false;
  std::atomic<bool> stop_{false};
  std::atomic<bool> running_{false};
  Semaphore wait_sem_{0U};
  Semaphore read_sem_{0U};
  Semaphore tx_sem_{0U};
  Semaphore exited_{0U};

  uint8_t rx_chunk_[kChunkSize] = {};
  // 8 字节对齐：帧头含 32 位字段，未对齐访问在 xtensa 上会异常 /
  // 8-byte alignment: the header holds 32-bit fields, and unaligned access faults on
  // xtensa.
  alignas(8) uint8_t rx_frame_[kFrameCapacity] = {};
  size_t rx_fill_ = 0U;
  size_t rx_total_ = 0U;
  uint32_t rx_payload_len_ = 0U;
  bool have_header_ = false;

  alignas(8) uint8_t resp_[MaxPacket] = {};
  alignas(8) uint8_t tx_frame_[kFrameCapacity] = {};

  std::atomic<uint32_t> frames_{0U};
  std::atomic<uint32_t> rx_bytes_{0U};
  std::atomic<uint32_t> tx_bytes_{0U};
  std::atomic<uint32_t> crc_errors_{0U};
  std::atomic<uint32_t> header_errors_{0U};
  std::atomic<uint32_t> dropped_{0U};
  std::atomic<uint32_t> rx_errors_{0U};
  std::atomic<uint32_t> tx_errors_{0U};
  std::atomic<uint32_t> last_activity_ms_{0U};
};

}  // namespace LibXR
