/**
 * @file test_cdc_uart.cpp
 * @brief CDCUart / CDCToUart 线路编码主机测试 / Host tests for the CDCUart and
 *        CDCToUart line coding.
 *
 * 判据来自 CDC 1.2 PSTN 6.3.10/6.3.11 与用户决定：SET_LINE_CODING 的 7 字节数据阶段
 * 总是应答（只查长度），GET_LINE_CODING 返回当前生效的线路编码。纯虚拟串口没有实际
 * 端口，主机设的值就是当前值；CDCToUart 在 UART 不支持时取最接近的可用配置并应答，
 * GET_LINE_CODING 返回实际生效值。
 * Oracles come from CDC 1.2 PSTN 6.3.10/6.3.11 and the user's decision: the 7-byte
 * SET_LINE_CODING data stage is always acknowledged (only its length is checked), and
 * GET_LINE_CODING returns the line coding currently in effect. The plain virtual port
 * has no physical port, so the host's value is the current one; CDCToUart applies the
 * nearest configuration its UART supports, acknowledges, and GET_LINE_CODING returns
 * what was applied.
 */

#include <cstdio>
#include <cstring>
#include <vector>

#include "../../mock_endpoint.hpp"
#include "cdc_to_uart.hpp"
#include "cdc_uart.hpp"
#include "libxr_pipe.hpp"

using namespace LibXR;
using namespace LibXR::USB;
using namespace LibXR::USB::Test;

namespace
{
constexpr uint8_t SET_LINE_CODING = 0x20;
constexpr uint8_t GET_LINE_CODING = 0x21;

/// Exposes the protected request entry points of a CDC class.
template <typename Base>
class Exposed : public Base
{
 public:
  using Base::Base;
  using Base::BindEndpoints;
  using Base::OnClassData;
  using Base::OnClassRequest;
  using ControlTransferResult = DeviceClass::ControlTransferResult;
};

/// The 7-byte line coding of CDC PSTN 6.3.11, little-endian dwDTERate first.
std::vector<uint8_t> Coding(uint32_t baud, uint8_t char_format, uint8_t parity,
                            uint8_t data_bits)
{
  std::vector<uint8_t> bytes(7);
  std::memcpy(bytes.data(), &baud, sizeof(baud));
  bytes[4] = char_format;
  bytes[5] = parity;
  bytes[6] = data_bits;
  return bytes;
}

/// SET_LINE_CODING as the core runs it: the SETUP, then the received data stage.
template <typename Class>
ErrorCode SetLineCoding(Class& cdc, const std::vector<uint8_t>& bytes)
{
  typename Class::ControlTransferResult result;
  const ErrorCode setup = cdc.OnClassRequest(false, SET_LINE_CODING, 0, 7, 0, result);
  if (setup != ErrorCode::OK) return setup;
  ConstRawData data{bytes.data(), bytes.size()};
  return cdc.OnClassData(false, SET_LINE_CODING, data);
}

template <typename Class>
std::vector<uint8_t> GetLineCoding(Class& cdc)
{
  typename Class::ControlTransferResult result;
  TEST_ASSERT(cdc.OnClassRequest(false, GET_LINE_CODING, 0, 7, 0, result) ==
              ErrorCode::OK);
  TEST_ASSERT(result.write_data.size_ == 7U);
  const auto* p = static_cast<const uint8_t*>(result.write_data.addr_);
  return std::vector<uint8_t>(p, p + 7);
}

/// A UART that supports 8 data bits with 1 stop bit at 9600..115200 baud, any parity.
class LimitedUart : public UART
{
 public:
  LimitedUart()
      : UART(static_cast<ReadPort*>(nullptr), static_cast<WritePort*>(nullptr)),
        rx_(256),
        tx_pipe_(1024)
  {
    read_port_ = &rx_;
    write_port_ = &tx_pipe_.GetWritePort();
  }

  ErrorCode SetConfig(Configuration config, bool = false) override
  {
    ++calls;
    if (config.baudrate < 9600U || config.baudrate > 115200U || config.data_bits != 8U ||
        config.stop_bits != 1U)
      return ErrorCode::NOT_SUPPORT;
    applied = config;
    return ErrorCode::OK;
  }

  Configuration applied{115200, Parity::NO_PARITY, 8, 1};
  unsigned calls = 0;

 private:
  ReadPort rx_;
  Pipe tx_pipe_;
};

void TestCdcUartAcceptsEveryLineCoding()
{
  Fixture f;
  Exposed<CDCUart> cdc(Number::EP1, Number::EP2, Number::EP3);
  cdc.BindEndpoints(f.pool, 0, false);

  UART::Configuration seen{};
  unsigned seen_count = 0;
  struct Seen
  {
    UART::Configuration* cfg;
    unsigned* count;
  } ctx{&seen, &seen_count};
  cdc.SetOnSetLineCodingCallback(Callback<UART::Configuration>::Create(
      [](bool, Seen* s, UART::Configuration cfg)
      {
        *s->cfg = cfg;
        ++*s->count;
      },
      &ctx));

  // 1.5 stop bits, Mark parity and 7 data bits: all valid PSTN values, acknowledged.
  const auto one_and_half = Coding(9600, 1, 3, 7);
  TEST_ASSERT(SetLineCoding(cdc, one_and_half) == ErrorCode::OK);
  // With no physical port the host's values are the current ones.
  TEST_ASSERT(GetLineCoding(cdc) == one_and_half);
  TEST_ASSERT(seen_count == 1 && seen.baudrate == 9600U && seen.data_bits == 7U);

  // Space parity and 16 data bits likewise.
  const auto space = Coding(921600, 2, 4, 16);
  TEST_ASSERT(SetLineCoding(cdc, space) == ErrorCode::OK);
  TEST_ASSERT(GetLineCoding(cdc) == space);

  // Only the 7-byte length is checked.
  const std::vector<uint8_t> short_stage(6, 0);
  ConstRawData data{short_stage.data(), short_stage.size()};
  TEST_ASSERT(cdc.OnClassData(false, SET_LINE_CODING, data) == ErrorCode::ARG_ERR);
  TEST_ASSERT(GetLineCoding(cdc) == space);
}

void TestCdcToUartAppliesTheNearestSupportedCoding()
{
  Fixture f;
  LimitedUart uart;
  Exposed<CDCToUart> cdc(Number::EP1, Number::EP2, Number::EP3, uart);
  cdc.BindEndpoints(f.pool, 0, false);

  // A supported coding is applied as requested.
  TEST_ASSERT(SetLineCoding(cdc, Coding(57600, 0, 2, 8)) == ErrorCode::OK);
  TEST_ASSERT(GetLineCoding(cdc) == Coding(57600, 0, 2, 8));
  TEST_ASSERT(uart.applied.baudrate == 57600U &&
              uart.applied.parity == UART::Parity::EVEN && uart.applied.stop_bits == 1U);

  // 2 stop bits are not supported: acknowledged, the UART runs with 1 stop bit and
  // GET_LINE_CODING reports that.
  TEST_ASSERT(SetLineCoding(cdc, Coding(19200, 2, 1, 8)) == ErrorCode::OK);
  TEST_ASSERT(GetLineCoding(cdc) == Coding(19200, 0, 1, 8));
  TEST_ASSERT(uart.applied.baudrate == 19200U &&
              uart.applied.parity == UART::Parity::ODD && uart.applied.stop_bits == 1U &&
              uart.applied.data_bits == 8U);

  // 7 data bits are not supported either: the result is 8N1 at the requested rate.
  TEST_ASSERT(SetLineCoding(cdc, Coding(38400, 1, 2, 7)) == ErrorCode::OK);
  TEST_ASSERT(GetLineCoding(cdc) == Coding(38400, 0, 0, 8));
  TEST_ASSERT(uart.applied.baudrate == 38400U && uart.applied.data_bits == 8U);

  // An unsupported rate: acknowledged, the previous coding stays in effect.
  TEST_ASSERT(SetLineCoding(cdc, Coding(3000000, 0, 0, 8)) == ErrorCode::OK);
  TEST_ASSERT(GetLineCoding(cdc) == Coding(38400, 0, 0, 8));
  TEST_ASSERT(uart.applied.baudrate == 38400U);
}
}  // namespace

void test_usb_cdc_uart()
{
  TestCdcUartAcceptsEveryLineCoding();
  TestCdcToUartAppliesTheNearestSupportedCoding();
}
