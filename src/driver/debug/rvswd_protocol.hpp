#pragma once

#include <cstdint>

namespace LibXR::Debug::RvSwdProtocol
{
enum class Op : uint8_t
{
  NOP = 0x00,
  READ = 0x01,
  WRITE = 0x02,
};

enum class Ack : uint8_t
{
  OK = 0x00,
  RESERVED = 0x01,
  FAILED = 0x02,
  BUSY = 0x03,
  PROTOCOL = 0xFF,
};

struct Request
{
  uint8_t addr = 0u;
  uint32_t data = 0u;
  Op op = Op::NOP;
};

struct Response
{
  uint8_t addr = 0u;
  uint32_t data = 0u;
  Ack ack = Ack::PROTOCOL;
};

inline constexpr uint8_t ONLINE_CAPR_STA = static_cast<uint8_t>(0x7Cu << 1u);
inline constexpr uint8_t ONLINE_CFGR_MASK = static_cast<uint8_t>(0x7Du << 1u);
inline constexpr uint8_t ONLINE_CFGR_SHAD = static_cast<uint8_t>(0x7Eu << 1u);
inline constexpr uint32_t ONLINE_ENABLE_OUTPUT = 0x5AA50400u;

constexpr uint8_t EncodeLegacyReadAddr(uint8_t raw_addr)
{
  // Legacy ONLINE_* register access does not encode direction in addr[0].
  // Read vs. write is selected by the frame shape itself:
  // - read:  9-bit header, then turnaround + 32-bit target data
  // - write: 41-bit header+payload from the host
  // Keep the captured wire address unchanged.
  return raw_addr;
}

constexpr uint8_t EncodeLegacyWriteAddr(uint8_t raw_addr)
{
  return raw_addr;
}

constexpr uint8_t EncodeReadAddr(uint8_t dmi_addr)
{
  return static_cast<uint8_t>(dmi_addr & 0x7Fu);
}

constexpr uint8_t EncodeWriteAddr(uint8_t dmi_addr)
{
  return static_cast<uint8_t>(dmi_addr & 0x7Fu);
}

constexpr Ack DecodeDmiStatus(uint8_t status)
{
  switch (status & 0x03u)
  {
    case 0x00u:
      return Ack::OK;
    case 0x01u:
      return Ack::RESERVED;
    case 0x02u:
      return Ack::FAILED;
    case 0x03u:
      return Ack::BUSY;
    default:
      return Ack::PROTOCOL;
  }
}

}  // namespace LibXR::Debug::RvSwdProtocol


