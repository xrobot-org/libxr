#include "desc_str.hpp"

using namespace LibXR::USB;

DescriptorStrings::DescriptorStrings(
    const std::initializer_list<const LanguagePack*>& lang_list, const uint8_t* uid,
    size_t uid_len)
    : LANG_NUM(lang_list.size()),
      header_(new uint16_t[LANG_NUM + 1]),
      land_id_(header_ + 1),
      string_list_(new const LanguagePack*[LANG_NUM])
{
  ASSERT(LANG_NUM > 0);
  ASSERT(uid != nullptr || uid_len == 0);

  auto lang = lang_list.begin();
  size_t max_str_len = 0;

  // 每个字节 -> 2个HEX字符 -> 每字符2字节UTF-16LE => 每字节4字节UTF-16LE
  const size_t EXTRA_SERIAL_UTF16 = uid ? uid_len * 4 : 0;

  for (size_t i = 0; i < LANG_NUM; i++)
  {
    land_id_[i] = static_cast<uint16_t>((*lang)->lang_id);
    string_list_[i] = *lang;

    size_t lang_max = (*lang)->max_string_length;

    if (EXTRA_SERIAL_UTF16 != 0)
    {
      const size_t IDX_SERIAL = static_cast<size_t>(Index::SERIAL_NUMBER_STRING) - 1;
      size_t serial_prefix_len = (*lang)->string_lens[IDX_SERIAL];
      size_t serial_total_len = serial_prefix_len + EXTRA_SERIAL_UTF16;
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

  *header_ = (static_cast<uint16_t>(LANG_NUM * 2 + 2)) | (0x03 << 8);
  buffer_.addr_ = new uint8_t[max_str_len + 2];
  buffer_.size_ = max_str_len + 2;

  ASSERT(max_str_len + 2 <= 255);

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

  const auto INDEX_NUM = static_cast<uint8_t>(index);
  if (INDEX_NUM < static_cast<uint8_t>(Index::MANUFACTURER_STRING) ||
      INDEX_NUM > static_cast<uint8_t>(Index::SERIAL_NUMBER_STRING))
  {
    return LibXR::ErrorCode::NOT_FOUND;
  }

  if (index == Index::SERIAL_NUMBER_STRING && serial_uid_ != nullptr)
  {
    // 序列号字符串 = 配置前缀 + UID 的 UTF-16LE 十六进制表示。
    // Serial string = configured prefix + UID rendered as UTF-16LE hex pairs.
    const LanguagePack* pack = string_list_[ans];
    constexpr size_t IDX = static_cast<size_t>(Index::SERIAL_NUMBER_STRING) - 1;

    const char* const SERIAL_PREFIX = pack->strings[IDX];
    const size_t PREFIX_UTF16_LEN = pack->string_lens[IDX];

    const uint8_t DATA_LEN =
        static_cast<uint8_t>(PREFIX_UTF16_LEN + serial_uid_len_ * 4 + 2);

    ASSERT(PREFIX_UTF16_LEN + serial_uid_len_ * 4 + 2 <= 255);

    buffer[1] = 0x03;
    buffer[0] = DATA_LEN;

    uint8_t* out = buffer + 2;

    ToUTF16LE(SERIAL_PREFIX, out);
    out += PREFIX_UTF16_LEN;

    static const char HEX[16] = {'0', '1', '2', '3', '4', '5', '6', '7',
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

    return LibXR::ErrorCode::OK;
  }

  auto data_len = string_list_[ans]->string_lens[INDEX_NUM - 1] + 2;
  buffer[1] = 0x03;
  buffer[0] = static_cast<uint8_t>(data_len);
  const char* str = string_list_[ans]->strings[INDEX_NUM - 1];
  ToUTF16LE(str, buffer + 2);
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

void DescriptorStrings::ToUTF16LE(const char* str, uint8_t* buffer)
{
  size_t len = 0;
  const unsigned char* s = reinterpret_cast<const unsigned char*>(str);

  while (*s)
  {
    uint32_t codepoint = 0;

    if (*s < 0x80)
    {
      codepoint = *s++;
    }
    else if ((*s & 0xE0) == 0xC0)
    {
      codepoint = (*s & 0x1F) << 6;
      s++;
      codepoint |= (*s & 0x3F);
      s++;
    }
    else if ((*s & 0xF0) == 0xE0)
    {
      codepoint = (*s & 0x0F) << 12;
      s++;
      codepoint |= (*s & 0x3F) << 6;
      s++;
      codepoint |= (*s & 0x3F);
      s++;
    }
    else if ((*s & 0xF8) == 0xF0)
    {
      // 忽略 BMP 之外的字符，保持描述符生成逻辑简单且有界。
      // Ignore code points outside BMP to keep descriptor generation simple and bounded.
      s++;
      if (*s)
      {
        s++;
      }
      if (*s)
      {
        s++;
      }
      if (*s)
      {
        s++;
      }
      continue;
    }
    else
    {
      s++;
      continue;
    }

    buffer[len++] = codepoint & 0xFF;
    buffer[len++] = (codepoint >> 8) & 0xFF;
  }
}
