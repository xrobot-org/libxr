#include "desc_str.hpp"

using namespace LibXR::USB;

size_t DescriptorStrings::CheckedLangCount(
    const std::initializer_list<const LanguagePack*>& lang_list)
{
  REQUIRE(lang_list.size() > 0U && lang_list.size() <= (MAX_DESCRIPTOR_BYTES - 2U) / 2U);
  return lang_list.size();
}

DescriptorStrings::DescriptorStrings(
    const std::initializer_list<const LanguagePack*>& lang_list, const uint8_t* uid,
    size_t uid_len)
    : LANG_NUM(CheckedLangCount(lang_list)),
      header_(new uint16_t[LANG_NUM + 1]),
      land_id_(header_ + 1),
      string_list_(new const LanguagePack*[LANG_NUM])
{
  REQUIRE(uid != nullptr || uid_len == 0U);

  auto lang = lang_list.begin();
  size_t max_str_len = 0U;

  // 每个字节 -> 2 个 HEX 字符 -> 每字符 2 字节 UTF-16LE => 每字节 4 字节 UTF-16LE。
  // Each UID byte expands to 2 hex digits (4 bytes in UTF-16LE).
  if (uid != nullptr)
  {
    REQUIRE(uid_len <= (MAX_DESCRIPTOR_BYTES - 2U) / 4U);
  }
  const size_t extra_serial_utf16 = uid != nullptr ? uid_len * 4U : 0U;

  for (size_t i = 0; i < LANG_NUM; i++)
  {
    const LanguagePack* pack = *lang;
    REQUIRE(pack != nullptr);
    REQUIRE(pack->max_string_length <= MAX_DESCRIPTOR_BYTES - 2U &&
            (pack->max_string_length & 1U) == 0U);
    for (size_t j = 0U; j < STRING_LIST_SIZE; ++j)
    {
      REQUIRE(pack->string_lens[j] <= pack->max_string_length &&
              (pack->string_lens[j] & 1U) == 0U);
    }

    land_id_[i] = static_cast<uint16_t>(pack->lang_id);
    string_list_[i] = pack;

    size_t lang_max = pack->max_string_length;

    if (extra_serial_utf16 != 0U)
    {
      constexpr size_t IDX_SERIAL = static_cast<size_t>(Index::SERIAL_NUMBER_STRING) - 1U;
      const size_t serial_prefix_len = pack->string_lens[IDX_SERIAL];
      REQUIRE(serial_prefix_len <= MAX_DESCRIPTOR_BYTES - 2U - extra_serial_utf16);
      const size_t serial_total_len = serial_prefix_len + extra_serial_utf16;
      if (serial_total_len > lang_max)
      {
        lang_max = serial_total_len;
      }
    }

    if (max_str_len < lang_max)
    {
      max_str_len = lang_max;
    }

    ++lang;
  }

  REQUIRE(max_str_len <= MAX_DESCRIPTOR_BYTES - 2U);
  *header_ = (static_cast<uint16_t>(LANG_NUM * 2 + 2)) | (0x03 << 8);
  buffer_.addr_ = new uint8_t[max_str_len + 2];
  buffer_.size_ = max_str_len + 2;
  static_cast<uint8_t*>(buffer_.addr_)[0] = 0U;

  serial_uid_ = uid;
  serial_uid_len_ = uid_len;
}

LibXR::ErrorCode DescriptorStrings::GenerateString(Index index, uint16_t lang)
{
  DEV_ASSERT(buffer_.addr_ != nullptr);

  if (index == Index::LANGUAGE_ID)
  {
    return LibXR::ErrorCode::NOT_SUPPORT;
  }

  int ans = -1;
  for (size_t i = 0; i < LANG_NUM; ++i)
  {
    if (land_id_[i] == lang)
    {
      ans = static_cast<int>(i);
      break;
    }
  }
  if (ans == -1)
  {
    return LibXR::ErrorCode::NOT_FOUND;
  }

  uint8_t* buffer = reinterpret_cast<uint8_t*>(buffer_.addr_);

  const auto index_num = static_cast<uint8_t>(index);
  if (index_num < static_cast<uint8_t>(Index::MANUFACTURER_STRING) ||
      index_num > static_cast<uint8_t>(Index::SERIAL_NUMBER_STRING))
  {
    return LibXR::ErrorCode::NOT_FOUND;
  }

  if (buffer_.size_ < 2U)
  {
    return LibXR::ErrorCode::FAILED;
  }
  buffer[0] = 0U;

  if (index == Index::SERIAL_NUMBER_STRING && serial_uid_ != nullptr)
  {
    // 序列号字符串 = 配置前缀 + UID 的 UTF-16LE 十六进制表示。
    // Serial string = configured prefix + UID rendered as UTF-16LE hex pairs.
    const LanguagePack* pack = string_list_[ans];
    constexpr size_t IDX = static_cast<size_t>(Index::SERIAL_NUMBER_STRING) - 1;

    const char* const serial_prefix = pack->strings[IDX];
    const size_t prefix_utf16_len = pack->string_lens[IDX];
    const size_t uid_utf16_len = serial_uid_len_ * 4U;
    if (uid_utf16_len > buffer_.size_ - 2U ||
        prefix_utf16_len > buffer_.size_ - 2U - uid_utf16_len)
    {
      return LibXR::ErrorCode::FAILED;
    }
    const size_t total_len = prefix_utf16_len + uid_utf16_len + 2U;
    if (total_len > MAX_DESCRIPTOR_BYTES)
    {
      return LibXR::ErrorCode::FAILED;
    }

    uint8_t* out = buffer + 2;
    size_t written = 0U;
    if (!EncodeUtf16Le(serial_prefix, out, buffer_.size_ - 2U - uid_utf16_len, written) ||
        written != prefix_utf16_len)
    {
      return LibXR::ErrorCode::FAILED;
    }
    out += written;

    static constexpr char HEX[16] = {'0', '1', '2', '3', '4', '5', '6', '7',
                                     '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

    for (size_t i = 0; i < serial_uid_len_; ++i)
    {
      uint8_t b = serial_uid_[i];
      char hi = HEX[(b >> 4) & 0x0F];
      char lo = HEX[b & 0x0F];

      *out++ = static_cast<uint8_t>(hi);
      *out++ = 0x00;
      *out++ = static_cast<uint8_t>(lo);
      *out++ = 0x00;
    }

    buffer[0] = static_cast<uint8_t>(total_len);
    buffer[1] = 0x03;
    return LibXR::ErrorCode::OK;
  }

  const char* str = string_list_[ans]->strings[index_num - 1];
  const size_t expected_len = string_list_[ans]->string_lens[index_num - 1];
  size_t written = 0U;
  if (!EncodeUtf16Le(str, buffer + 2, buffer_.size_ - 2U, written) ||
      written != expected_len || written > MAX_DESCRIPTOR_BYTES - 2U)
  {
    return LibXR::ErrorCode::FAILED;
  }
  buffer[0] = static_cast<uint8_t>(written + 2U);
  buffer[1] = 0x03;
  return LibXR::ErrorCode::OK;
}

LibXR::RawData DescriptorStrings::GetData()
{
  DEV_ASSERT(buffer_.addr_ != nullptr);
  uint8_t* buffer = reinterpret_cast<uint8_t*>(buffer_.addr_);
  return RawData{buffer_.addr_, buffer[0]};
}

LibXR::RawData DescriptorStrings::GetLangIDData()
{
  return RawData{header_, (LANG_NUM + 1) * sizeof(uint16_t)};
}

bool DescriptorStrings::HasLanguage(uint16_t lang) const
{
  // 接口字符串与内建字符串表共用同一套语言门控。
  // Interface strings follow the same language gating as the built-in string table.
  for (size_t i = 0; i < LANG_NUM; ++i)
  {
    if (land_id_[i] == lang)
    {
      return true;
    }
  }
  return false;
}

bool DescriptorStrings::EncodeUtf16Le(const char* str, uint8_t* buffer,
                                      size_t buffer_bytes, size_t& written)
{
  return ProcessUtf8(str, std::numeric_limits<size_t>::max(), buffer, buffer_bytes,
                     written);
}
