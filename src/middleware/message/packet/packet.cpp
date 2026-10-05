#include "packet.hpp"

#include "crc.hpp"
#include "libxr_mem.hpp"

using namespace LibXR;

void Topic::PackedDataHeader::SetDataLen(uint32_t len)
{
  data_len_raw[0] = static_cast<uint8_t>(len);
  data_len_raw[1] = static_cast<uint8_t>(len >> 8);
  data_len_raw[2] = static_cast<uint8_t>(len >> 16);
}

uint32_t Topic::PackedDataHeader::GetDataLen() const
{
  return static_cast<uint32_t>(data_len_raw[0]) |
         static_cast<uint32_t>(data_len_raw[1]) << 8 |
         static_cast<uint32_t>(data_len_raw[2]) << 16;
}

void Topic::PackedDataHeader::SetTimestamp(MicrosecondTimestamp timestamp)
{
  uint64_t value = static_cast<uint64_t>(timestamp);
  ASSERT((value >> 48U) == 0);
  for (size_t i = 0; i < sizeof(timestamp_us_raw); i++)
  {
    timestamp_us_raw[i] = static_cast<uint8_t>(value >> (i * 8U));
  }
}

MicrosecondTimestamp Topic::PackedDataHeader::GetTimestamp() const
{
  uint64_t value = 0;
  for (size_t i = 0; i < sizeof(timestamp_us_raw); i++)
  {
    value |= static_cast<uint64_t>(timestamp_us_raw[i]) << (i * 8U);
  }
  return MicrosecondTimestamp(value);
}

void Topic::PackBytes(uint32_t topic_name_crc32, RawData buffer,
                      MicrosecondTimestamp timestamp, ConstRawData data)
{
  DEV_ASSERT(buffer.addr_ != nullptr);
  DEV_ASSERT(buffer.size_ >= PACK_BASE_SIZE + data.size_);

  auto* pack = reinterpret_cast<PackedData<uint8_t>*>(buffer.addr_);

  LibXR::Memory::FastCopy(&pack->raw.data_, data.addr_, data.size_);

  pack->raw.header_.prefix = PACKET_PREFIX;
  pack->raw.header_.version = PACKET_VERSION;
  pack->raw.header_.topic_name_crc32 = topic_name_crc32;
  pack->raw.header_.SetDataLen(data.size_);
  pack->raw.header_.SetTimestamp(timestamp);
  pack->raw.header_.pack_header_crc8 =
      CRC8::Calculate(&pack->raw, sizeof(PackedDataHeader) - sizeof(uint8_t));

  uint8_t* crc8_pack = reinterpret_cast<uint8_t*>(
      reinterpret_cast<uint8_t*>(pack) + PACK_BASE_SIZE + data.size_ - sizeof(uint8_t));
  *crc8_pack = CRC8::Calculate(pack, PACK_BASE_SIZE - sizeof(uint8_t) + data.size_);
}
