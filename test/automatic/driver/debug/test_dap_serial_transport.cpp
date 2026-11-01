/**
 * @file test_dap_serial_transport.cpp
 * @brief DapSerialTransport 主机测试 / Host tests for DapSerialTransport.
 *
 * 用一个只有 ReadPort/Pipe 的 mock UART 和一个 mock DAP 验证解析、CRC、重同步、
 * max packet 边界、发送背压、生命周期，以及与独立编写的参考打包的字节级一致。
 * A mock UART built from ReadPort/Pipe and a mock DAP verify parsing, CRC handling,
 * resynchronization, the max-packet boundary, TX backpressure, lifecycle, and byte-level
 * agreement with an independently written reference packer.
 *
 * oracle：请求 `len(u16 LE) + 载荷` 的响应固定为载荷逐字节异或 0x5A，长度相同；帧布局按
 * `Topic` 打包帧定义逐字段断言。
 * Oracle: for a request `len(u16 LE) + payload` the mock DAP responds with the payload
 * XOR 0x5A byte by byte and the same length; the frame layout is asserted field by field
 * against the `Topic` packed-frame definition.
 *
 * 这个测试不依赖 ESP-IDF/FreeRTOS：模块头文件只用到 libxr 的公共接口。
 * This test needs no ESP-IDF/FreeRTOS: the module header uses only public libxr
 * interfaces.
 */

#include <array>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

#include "crc.hpp"
#include "debug/dap_serial_transport.hpp"
#include "libxr_pipe.hpp"
#include "libxr_rw.hpp"
#include "packet/packet.hpp"
#include "test_assert.hpp"
#include "timebase.hpp"

namespace
{
using namespace LibXR;

constexpr uint16_t kMaxPacket = 64U;
constexpr size_t kHeaderSize = sizeof(Topic::PackedDataHeader);
constexpr size_t kPayloadCapacity = 2U + kMaxPacket;
constexpr size_t kFrameCapacity = kHeaderSize + kPayloadCapacity + 1U;

/// 只包含 ReadPort 和 Pipe 的最小 UART：接收由测试推入，发送由测试抽走。
/// Minimal UART with just a ReadPort and a Pipe: the test pushes RX and drains TX.
class MockUart : public UART
{
 public:
  explicit MockUart(size_t rx_capacity = 4096U, size_t tx_capacity = 1U << 16U)
      : UART(static_cast<ReadPort*>(nullptr), static_cast<WritePort*>(nullptr)),
        rx_(rx_capacity),
        tx_pipe_(tx_capacity)
  {
    read_port_ = &rx_;
    write_port_ = &tx_pipe_.GetWritePort();
  }

  ErrorCode SetConfig(Configuration, bool = false) override { return ErrorCode::OK; }

  /// 把字节放进接收队列，模拟 UART ISR 入队 /
  /// Queue bytes on the receive side, modelling the UART ISR producer.
  void PushRx(const uint8_t* data, size_t len)
  {
    auto queue = rx_.GetReadQueue();
    TEST_ASSERT(queue.PushBatch(data, len) == ErrorCode::OK);
    queue.Publish();
  }

  /// 取走已发出的字节 / Take the bytes that were transmitted.
  std::vector<uint8_t> DrainTx()
  {
    std::vector<uint8_t> out;
    auto& port = tx_pipe_.GetReadPort();
    for (;;)
    {
      const size_t queued = port.Size();
      if (queued == 0U)
      {
        break;
      }
      const size_t take = LibXR::min(queued, static_cast<size_t>(256U));
      std::array<uint8_t, 256U> chunk{};
      ReadOperation op(sem_);
      TEST_ASSERT(port(RawData(chunk.data(), take), op) == ErrorCode::OK);
      out.insert(out.end(), chunk.begin(), chunk.begin() + static_cast<long>(take));
    }
    return out;
  }

  /// 取走刚好 `len` 个字节；队列为空时立即返回已取到的部分 /
  /// Take exactly `len` bytes; return what is available when the queue runs dry.
  std::vector<uint8_t> DrainTx(size_t len)
  {
    std::vector<uint8_t> out;
    auto& port = tx_pipe_.GetReadPort();
    out.resize(len);
    size_t offset = 0U;
    while (offset < len)
    {
      const size_t queued = port.Size();
      if (queued == 0U)
      {
        break;
      }
      const size_t take = LibXR::min(queued, len - offset);
      ReadOperation op(sem_);
      TEST_ASSERT(port(RawData(out.data() + offset, take), op) == ErrorCode::OK);
      offset += take;
    }
    out.resize(offset);
    return out;
  }

  /// 发送队列容量，供背压用例断言 / TX queue capacity, asserted by the backpressure case.
  static constexpr size_t kTxCapacity = kFrameCapacity;

  ReadPort& ReadSide() { return rx_; }

 private:
  ReadPort rx_;
  Pipe tx_pipe_;
  Semaphore sem_{0U};
};

/// 响应用例：把请求逐字节异或 0x5A 返回，并记录调用次数。
/// Response rule: return the request XOR 0x5A byte by byte and count the calls.
class MockDap
{
 public:
  ErrorCode ExecuteExternal(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                            uint16_t resp_cap, uint16_t& out_len)
  {
    ++calls;
    if (req_len > resp_cap)
    {
      return ErrorCode::SIZE_ERR;
    }
    for (uint16_t i = 0U; i < req_len; ++i)
    {
      resp[i] = static_cast<uint8_t>(req[i] ^ 0x5AU);
    }
    out_len = req_len;
    last_request.assign(req, req + req_len);
    return ErrorCode::OK;
  }

  std::atomic<uint32_t> calls{0U};
  std::vector<uint8_t> last_request;
};

using Transport = DapSerialTransport<MockDap, kMaxPacket>;

/// 请求 topic `dap_req` 的帧头 CRC32 / Frame-header CRC32 of the `dap_req` topic.
uint32_t RequestCrc32() { return CRC32::Calculate("dap_req", std::strlen("dap_req")); }

/// 响应 topic `dap_resp` 的帧头 CRC32 / Frame-header CRC32 of the `dap_resp` topic.
uint32_t ResponseCrc32() { return CRC32::Calculate("dap_resp", std::strlen("dap_resp")); }

/// 组装一条请求帧（不填时间戳，由调用方决定）/ Build one request frame; the caller
/// decides the timestamp.
std::vector<uint8_t> MakeFrame(uint32_t topic_crc32, const std::vector<uint8_t>& payload,
                               MicrosecondTimestamp timestamp)
{
  std::vector<uint8_t> frame(kHeaderSize + payload.size() + 1U, 0U);
  auto* header = reinterpret_cast<Topic::PackedDataHeader*>(frame.data());
  header->prefix = Topic::PACKET_PREFIX;
  header->version = Topic::PACKET_VERSION;
  header->topic_name_crc32 = topic_crc32;
  header->SetDataLen(static_cast<uint32_t>(payload.size()));
  header->SetTimestamp(timestamp);
  header->pack_header_crc8 = CRC8::Calculate(frame.data(), kHeaderSize - 1U);
  std::memcpy(frame.data() + kHeaderSize, payload.data(), payload.size());
  frame[kHeaderSize + payload.size()] =
      CRC8::Calculate(frame.data(), kHeaderSize + payload.size());
  return frame;
}

/// 请求载荷：`len(u16 LE) + DAP 包` / Request payload: `len(u16 LE) + DAP packet`.
std::vector<uint8_t> MakeRequestPayload(const std::vector<uint8_t>& packet)
{
  std::vector<uint8_t> payload(2U + packet.size(), 0U);
  payload[0] = static_cast<uint8_t>(packet.size());
  payload[1] = static_cast<uint8_t>(packet.size() >> 8U);
  std::memcpy(payload.data() + 2U, packet.data(), packet.size());
  return payload;
}

/// 按 Topic 打包帧定义独立编写的响应打包，用作字节级 oracle。
/// 时间戳沿用模块实际写入的值，因此除该字段外的每个字节都必须完全一致。
/// Response packer written independently from the Topic frame definition, used as the
/// byte-level oracle. The timestamp is
/// taken from the frame the module produced, so every other byte must match exactly.
std::vector<uint8_t> ReferenceFrame(uint32_t resp_crc32, const uint8_t* resp,
                                    uint16_t out_len, MicrosecondTimestamp timestamp)
{
  std::vector<uint8_t> reply(2U + out_len, 0U);
  reply[0] = static_cast<uint8_t>(out_len);
  reply[1] = static_cast<uint8_t>(out_len >> 8U);
  if (out_len != 0U)
  {
    std::memcpy(reply.data() + 2U, resp, out_len);
  }

  std::vector<uint8_t> frame(kHeaderSize + reply.size() + 1U, 0U);
  Topic::PackedDataHeader header{};
  header.prefix = Topic::PACKET_PREFIX;
  header.version = Topic::PACKET_VERSION;
  header.topic_name_crc32 = resp_crc32;
  header.SetDataLen(static_cast<uint32_t>(reply.size()));
  header.SetTimestamp(timestamp);
  header.pack_header_crc8 = CRC8::Calculate(&header, sizeof(header) - sizeof(uint8_t));
  std::memcpy(frame.data(), &header, sizeof(header));
  std::memcpy(frame.data() + kHeaderSize, reply.data(), reply.size());
  frame[kHeaderSize + reply.size()] =
      CRC8::Calculate(frame.data(), kHeaderSize + reply.size());
  return frame;
}

/// 校验一条响应帧的布局并返回其中的 DAP 响应 / Validate a response frame's layout and
/// return the DAP response it carries.
std::vector<uint8_t> CheckResponse(const std::vector<uint8_t>& frame,
                                   uint32_t expected_topic_crc32)
{
  TEST_ASSERT(frame.size() >= kHeaderSize + 3U);
  TEST_ASSERT(frame[0] == Topic::PACKET_PREFIX);
  const auto* header = reinterpret_cast<const Topic::PackedDataHeader*>(frame.data());
  TEST_ASSERT(header->version == Topic::PACKET_VERSION);
  TEST_ASSERT(header->topic_name_crc32 == expected_topic_crc32);
  const uint32_t payload_len = header->GetDataLen();
  TEST_ASSERT(frame.size() == kHeaderSize + payload_len + 1U);
  TEST_ASSERT(CRC8::Verify(frame.data(), kHeaderSize));
  TEST_ASSERT(CRC8::Verify(frame.data(), static_cast<size_t>(frame.size())));

  const size_t out_len = static_cast<size_t>(frame[kHeaderSize]) |
                         (static_cast<size_t>(frame[kHeaderSize + 1U]) << 8U);
  TEST_ASSERT(payload_len == 2U + out_len);
  TEST_ASSERT(out_len + 2U <= payload_len);
  return std::vector<uint8_t>(
      frame.begin() + static_cast<long>(kHeaderSize) + 2,
      frame.begin() + static_cast<long>(kHeaderSize) + 2 + static_cast<long>(out_len));
}

std::vector<uint8_t> Xor(const std::vector<uint8_t>& data)
{
  std::vector<uint8_t> out(data.size());
  for (size_t i = 0U; i < data.size(); ++i)
  {
    out[i] = static_cast<uint8_t>(data[i] ^ 0x5AU);
  }
  return out;
}

const std::vector<uint8_t> kPacketA{0x02U, 0x01U};
const std::vector<uint8_t> kPacketB{0x05U, 0x00U, 0x11U, 0x22U, 0x33U};

/// 单帧：一条请求得到一条响应 / One frame in, one response out.
void TestSingleFrame()
{
  MockUart uart;
  MockDap dap;
  Transport link(dap, uart);

  const auto frame = MakeFrame(RequestCrc32(), MakeRequestPayload(kPacketA),
                               MicrosecondTimestamp(1000U));
  link.Feed(frame.data(), frame.size());

  TEST_ASSERT(dap.calls.load() == 1U);
  TEST_ASSERT(dap.last_request == kPacketA);
  const auto response = CheckResponse(uart.DrainTx(), ResponseCrc32());
  TEST_ASSERT(response == Xor(kPacketA));
  TEST_ASSERT(uart.DrainTx().empty());

  const auto stats = link.GetStats();
  TEST_ASSERT(stats.frames == 1U);
  TEST_ASSERT(stats.rx_bytes == frame.size());
  TEST_ASSERT(stats.crc_errors == 0U);
  TEST_ASSERT(stats.header_errors == 0U);
  TEST_ASSERT(stats.dropped == 0U);
  // 时间戳用 Timebase 的毫秒；进程刚启动时它可以是 0，只能是“不晚于现在”。
  // The timestamp is Timebase milliseconds; it may legitimately be 0 right after process
  // start, so the only invariant is that it is not in the future.
  TEST_ASSERT(stats.last_activity_ms <=
              static_cast<uint32_t>(Timebase::GetMilliseconds()));
}

/// 粘包：一次 Feed 里的多条帧按序全部执行 / Coalesced frames all execute in order.
void TestCoalescedFrames()
{
  MockUart uart;
  MockDap dap;
  Transport link(dap, uart);

  std::vector<uint8_t> stream;
  for (int i = 0; i < 3; ++i)
  {
    const auto& packet = (i % 2 == 0) ? kPacketA : kPacketB;
    const auto frame = MakeFrame(RequestCrc32(), MakeRequestPayload(packet),
                                 MicrosecondTimestamp(2000U));
    stream.insert(stream.end(), frame.begin(), frame.end());
  }
  link.Feed(stream.data(), stream.size());

  TEST_ASSERT(dap.calls.load() == 3U);
  const auto tx = uart.DrainTx();
  // 三个响应依次对应 A、B、A / The three responses correspond to A, B, A in order.
  const size_t expected_size = 2U * (kHeaderSize + 2U + kPacketA.size() + 1U) +
                               (kHeaderSize + 2U + kPacketB.size() + 1U);
  TEST_ASSERT(tx.size() == expected_size);
  TEST_ASSERT(link.GetStats().frames == 3U);
  TEST_ASSERT(link.GetStats().dropped == 0U);
  TEST_ASSERT(link.GetStats().tx_bytes == expected_size);
  // 三个响应按顺序排在同一段字节里，逐个校验布局与内容 /
  // The three responses share one byte run; validate layout and payload in order.
  const std::vector<std::vector<uint8_t>> expected_payloads{Xor(kPacketA), Xor(kPacketB),
                                                            Xor(kPacketA)};
  size_t offset = 0U;
  for (const auto& expected : expected_payloads)
  {
    const size_t frame_len = kHeaderSize + 2U + expected.size() + 1U;
    const std::vector<uint8_t> frame(tx.begin() + static_cast<long>(offset),
                                     tx.begin() + static_cast<long>(offset + frame_len));
    TEST_ASSERT(CheckResponse(frame, ResponseCrc32()) == expected);
    offset += frame_len;
  }
}

/// 半帧：逐字节分多次送入，帧只在补全后执行一次 /
/// Half frames: byte-at-a-time feeds execute the frame exactly once, after completion.
void TestSplitFrame()
{
  MockUart uart;
  MockDap dap;
  Transport link(dap, uart);

  const auto frame = MakeFrame(RequestCrc32(), MakeRequestPayload(kPacketB),
                               MicrosecondTimestamp(3000U));
  for (const uint8_t byte : frame)
  {
    link.Feed(&byte, 1U);
  }

  TEST_ASSERT(dap.calls.load() == 1U);
  TEST_ASSERT(CheckResponse(uart.DrainTx(), ResponseCrc32()) == Xor(kPacketB));
  TEST_ASSERT(link.GetStats().dropped == 0U);
  TEST_ASSERT(link.GetStats().crc_errors == 0U);
}

/// 尾 CRC 错误：当前帧丢弃，同一批里的下一帧仍然执行 /
/// Trailing CRC error: the frame is dropped while the next frame in the same batch runs.
void TestTailCrcError()
{
  MockUart uart;
  MockDap dap;
  Transport link(dap, uart);

  auto bad = MakeFrame(RequestCrc32(), MakeRequestPayload(kPacketA),
                       MicrosecondTimestamp(4000U));
  bad.back() ^= 0xFFU;
  const auto good = MakeFrame(RequestCrc32(), MakeRequestPayload(kPacketB),
                              MicrosecondTimestamp(4001U));
  std::vector<uint8_t> stream = bad;
  stream.insert(stream.end(), good.begin(), good.end());
  link.Feed(stream.data(), stream.size());

  TEST_ASSERT(dap.calls.load() == 1U);
  TEST_ASSERT(dap.last_request == kPacketB);
  TEST_ASSERT(link.GetStats().crc_errors == 1U);
  TEST_ASSERT(link.GetStats().frames == 1U);
  TEST_ASSERT(CheckResponse(uart.DrainTx(), ResponseCrc32()) == Xor(kPacketB));
}

/// 头 CRC 错误：解析器跳到下一个前缀字节重新同步 /
/// Header CRC error: the parser resynchronizes on the next prefix byte.
void TestHeaderCrcError()
{
  MockUart uart;
  MockDap dap;
  Transport link(dap, uart);

  auto bad = MakeFrame(RequestCrc32(), MakeRequestPayload(kPacketA),
                       MicrosecondTimestamp(5000U));
  bad[kHeaderSize - 1U] ^= 0xFFU;
  const auto good = MakeFrame(RequestCrc32(), MakeRequestPayload(kPacketB),
                              MicrosecondTimestamp(5001U));
  std::vector<uint8_t> stream = bad;
  stream.insert(stream.end(), good.begin(), good.end());
  link.Feed(stream.data(), stream.size());

  TEST_ASSERT(dap.calls.load() == 1U);
  TEST_ASSERT(dap.last_request == kPacketB);
  TEST_ASSERT(link.GetStats().crc_errors == 1U);
}

/// 别的 topic：头 CRC 正确但名字对不上，计为头字段错误 /
/// Another topic: the header CRC is valid but the name differs, counted as a header
/// error.
void TestWrongTopic()
{
  MockUart uart;
  MockDap dap;
  Transport link(dap, uart);

  const auto frame = MakeFrame(CRC32::Calculate("dap_other", std::strlen("dap_other")),
                               MakeRequestPayload(kPacketA), MicrosecondTimestamp(6000U));
  link.Feed(frame.data(), frame.size());

  TEST_ASSERT(dap.calls.load() == 0U);
  TEST_ASSERT(link.GetStats().header_errors == 1U);
  TEST_ASSERT(uart.DrainTx().empty());
}

/// 超长帧：载荷长度超过容量，拒绝且不执行 /
/// Oversize frame: the payload length exceeds capacity, so it is rejected, not executed.
void TestOversizeFrame()
{
  MockUart uart;
  MockDap dap;
  Transport link(dap, uart);

  // 头声称的载荷长度刚好超过容量：头 CRC 合法，字段检查必须拦住它 /
  // The header claims a payload one byte over capacity: the header CRC is valid, so the
  // field check has to reject it.
  std::vector<uint8_t> frame(kHeaderSize + 4U, 0U);
  auto* header = reinterpret_cast<Topic::PackedDataHeader*>(frame.data());
  header->prefix = Topic::PACKET_PREFIX;
  header->version = Topic::PACKET_VERSION;
  header->topic_name_crc32 = RequestCrc32();
  header->SetDataLen(static_cast<uint32_t>(kPayloadCapacity + 1U));
  header->SetTimestamp(MicrosecondTimestamp(7000U));
  header->pack_header_crc8 = CRC8::Calculate(frame.data(), kHeaderSize - 1U);
  frame[kHeaderSize - 1U] = header->pack_header_crc8;
  link.Feed(frame.data(), frame.size());

  TEST_ASSERT(dap.calls.load() == 0U);
  TEST_ASSERT(link.GetStats().header_errors == 1U);
  TEST_ASSERT(uart.DrainTx().empty());
}

/// 长度前缀与载荷不符：请求长度大于实际载荷，丢弃且不执行 /
/// Length prefix mismatch: the request length exceeds the payload, so nothing executes.
void TestLengthPrefixMismatch()
{
  MockUart uart;
  MockDap dap;
  Transport link(dap, uart);

  std::vector<uint8_t> payload(2U + 2U, 0U);
  payload[0] = 8U;  // 声称 8 字节，实际只有 2 字节 / claims 8 bytes, only 2 present
  payload[1] = 0U;
  payload[2] = 0xAAU;
  payload[3] = 0xBBU;
  const auto frame = MakeFrame(RequestCrc32(), payload, MicrosecondTimestamp(8000U));
  link.Feed(frame.data(), frame.size());

  TEST_ASSERT(dap.calls.load() == 0U);
  TEST_ASSERT(link.GetStats().dropped == 1U);
  TEST_ASSERT(uart.DrainTx().empty());
}

/// max packet 边界：请求和响应都正好等于 MaxPacket /
/// Max-packet boundary: both request and response are exactly MaxPacket bytes.
void TestMaxPacketBoundary()
{
  MockUart uart(8192U);
  MockDap dap;
  Transport link(dap, uart);

  std::vector<uint8_t> packet(kMaxPacket);
  for (size_t i = 0U; i < packet.size(); ++i)
  {
    packet[i] = static_cast<uint8_t>(i * 7U + 1U);
  }
  const auto frame =
      MakeFrame(RequestCrc32(), MakeRequestPayload(packet), MicrosecondTimestamp(9000U));
  TEST_ASSERT(frame.size() == kFrameCapacity);
  link.Feed(frame.data(), frame.size());

  TEST_ASSERT(dap.calls.load() == 1U);
  const auto tx = uart.DrainTx(kFrameCapacity);
  TEST_ASSERT(tx.size() == kFrameCapacity);
  TEST_ASSERT(CheckResponse(tx, ResponseCrc32()) == Xor(packet));
  TEST_ASSERT(link.GetStats().dropped == 0U);
}

/// 发送背压：发送队列刚好只放得下一帧，先被占满，模块必须重试到队列腾空才发完 /
/// TX backpressure: the TX queue holds exactly one frame and is pre-filled, so the module
/// must retry until the drainer frees space and the whole frame lands.
std::vector<uint8_t> drained;

void TestTxBackpressure()
{
  MockUart uart(4096U, MockUart::kTxCapacity);
  MockDap dap;
  Transport link(dap, uart);

  std::vector<uint8_t> packet(kMaxPacket, 0x3CU);
  const auto frame =
      MakeFrame(RequestCrc32(), MakeRequestPayload(packet), MicrosecondTimestamp(10000U));

  // 后台持续抽走发送队列，让背压重试最终成功 /
  // Keep draining TX in the background so the backpressure retry eventually wins.
  std::thread drainer(
      [&]
      {
        for (uint32_t i = 0U; i < 20000U && drained.size() < kFrameCapacity; ++i)
        {
          const auto chunk = uart.DrainTx();
          drained.insert(drained.end(), chunk.begin(), chunk.end());
          if (chunk.empty())
          {
            std::this_thread::sleep_for(std::chrono::microseconds(200));
          }
        }
      });

  link.Feed(frame.data(), frame.size());
  drainer.join();

  TEST_ASSERT(drained.size() >= kFrameCapacity);
  const auto response = CheckResponse(drained, ResponseCrc32());
  TEST_ASSERT(response == Xor(packet));
  TEST_ASSERT(link.GetStats().tx_errors == 0U);
}

/// 平台线程启动器：模块不自己调 RTOS，由启动器决定线程怎么起 /
/// Platform thread starter: the module does not call an RTOS, the starter decides how the
/// thread is created.
struct StarterLog
{
  bool called = false;
  void (*entry)(void*) = nullptr;
  void* arg = nullptr;
  std::thread worker;
};

void RecordingStarter(void* ctx, const Transport::ThreadConfig&, void (*entry)(void*),
                      void* arg)
{
  auto* log = static_cast<StarterLog*>(ctx);
  log->called = true;
  log->entry = entry;
  log->arg = arg;
  log->worker = std::thread(entry, arg);
}

/// Start/Stop：重复启动返回 BUSY，Stop 之后工作线程退出 /
/// Start/Stop: a second start reports BUSY and the worker exits after Stop.
void TestStartStop()
{
  MockUart uart;
  MockDap dap;
  Transport link(dap, uart);

  StarterLog log;
  link.SetThreadStarter(&RecordingStarter, &log);
  TEST_ASSERT(link.Start() == ErrorCode::OK);
  TEST_ASSERT(log.called);
  TEST_ASSERT(link.Running());
  TEST_ASSERT(link.Start() == ErrorCode::BUSY);

  // 工作线程在跑：从另一个线程推入一帧，检查它被处理 /
  // The worker is live: push a frame from this thread and check it is handled.
  const auto frame = MakeFrame(RequestCrc32(), MakeRequestPayload(kPacketB),
                               MicrosecondTimestamp(14000U));
  uart.PushRx(frame.data(), frame.size());
  for (uint32_t i = 0U; i < 2000U && dap.calls.load() == 0U; ++i)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  TEST_ASSERT(dap.calls.load() == 1U);

  link.Stop();
  TEST_ASSERT(!link.Running());
  log.worker.join();
}

/// 串口不可读：未绑定接收队列时 Start 拒绝 /
/// Unusable UART: Start refuses when the receive queue is unbound.
void TestStartUnbound()
{
  MockUart uart(0U);
  MockDap dap;
  Transport link(dap, uart);
  TEST_ASSERT(link.Start() == ErrorCode::NOT_SUPPORT);
  TEST_ASSERT(!link.Running());
}

/// 与旧内联实现的字节级兼容：同一请求得到逐字节相同的响应帧 /
/// Byte-level compatibility with the previous inline implementation: the same request
/// yields a byte-identical response frame.
void TestByteCompatibility()
{
  MockUart uart;
  MockDap dap;
  Transport link(dap, uart);

  const auto frame = MakeFrame(RequestCrc32(), MakeRequestPayload(kPacketB),
                               MicrosecondTimestamp(15000U));
  link.Feed(frame.data(), frame.size());
  const auto produced = uart.DrainTx();

  const auto* produced_header =
      reinterpret_cast<const Topic::PackedDataHeader*>(produced.data());
  const auto expected = ReferenceFrame(ResponseCrc32(), Xor(kPacketB).data(),
                                       static_cast<uint16_t>(kPacketB.size()),
                                       produced_header->GetTimestamp());
  TEST_ASSERT(produced.size() == expected.size());
  TEST_ASSERT(std::memcmp(produced.data(), expected.data(), produced.size()) == 0);
  TEST_ASSERT(produced_header->GetTimestamp() != MicrosecondTimestamp(0U));
}
}  // namespace

/// 每个用例入口向 stderr 打印一行，便于定位挂起位置 /
/// Print one line to stderr per case so a hang can be located.
#define RUN_CASE(fn)                         \
  do                                         \
  {                                          \
    std::fprintf(stderr, "case: %s\n", #fn); \
    std::fflush(stderr);                     \
    fn();                                    \
  } while (0)

int main()
{
  if (!LibXR::Timebase::IsReady())
  {
    std::fprintf(stderr, "timebase not ready: the host backend must provide Timebase\n");
    return 1;
  }

  RUN_CASE(TestSingleFrame);
  RUN_CASE(TestCoalescedFrames);
  RUN_CASE(TestSplitFrame);
  RUN_CASE(TestTailCrcError);
  RUN_CASE(TestHeaderCrcError);
  RUN_CASE(TestWrongTopic);
  RUN_CASE(TestOversizeFrame);
  RUN_CASE(TestLengthPrefixMismatch);
  RUN_CASE(TestMaxPacketBoundary);
  RUN_CASE(TestTxBackpressure);
  RUN_CASE(TestStartStop);
  RUN_CASE(TestStartUnbound);
  RUN_CASE(TestByteCompatibility);

  std::puts(
      "dap_serial_transport: framing, CRC, resync, max packet, backpressure, "
      "lifecycle and byte compatibility passed");
  return 0;
}
