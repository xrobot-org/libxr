#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

#include "core.hpp"
#include "libxr_type.hpp"

namespace LibXR::USB
{
class DeviceComposition;

/**
 * @brief 字符串描述符管理器 / USB string descriptor manager
 */
class DescriptorStrings
{
 public:
  /**
   * @brief 描述符字符串索引 / USB descriptor string index
   */
  enum class Index : uint8_t
  {
    LANGUAGE_ID = 0x00,          ///< 语言ID描述符 / LangID descriptor
    MANUFACTURER_STRING = 0x01,  ///< 厂商字符串索引 / Manufacturer string
    PRODUCT_STRING = 0x02,       ///< 产品字符串索引 / Product string
    SERIAL_NUMBER_STRING = 0x03  ///< 序列号字符串索引 / Serial number string
  };

  /**
   * @brief 语言 / Supported language
   */
  enum class Language : uint16_t
  {
    EN_US = 0x0409,  ///< 英语 / English (US)
    ZH_CN = 0x0804   ///< 简体中文 / Simplified Chinese
  };

  typedef const char* StringData;  ///< 静态字符串指针类型 / Static string pointer alias.
  static constexpr size_t STRING_LIST_SIZE =
      3;  ///< 单语言固定字符串条目数（厂商/产品/序列号） / Strings per language pack
          ///< (manufacturer/product/serial).

  /**
   * @brief 单语言描述符包（用于注册静态多语言字符串） / Single-language USB string
   *        descriptor pack for registration
   * @note `string_lens[i]` 须与 `strings[i]` 经 BMP 编码后的 UTF-16LE 字节数一致
   *       （由 `MakeLanguagePack()` 保证）。
   *       `string_lens[i]` must match the UTF-16LE byte length of `strings[i]`
   *       (guaranteed by `MakeLanguagePack()`).
   */
  struct LanguagePack
  {
    Language lang_id;  ///< 语言 / Language
    /// 指向以 '\\0' 结尾的静态 UTF-8 字符串，在 DescriptorStrings 生命周期内须保持不变 /
    /// NUL-terminated static UTF-8 strings kept immutable for DescriptorStrings lifetime
    StringData strings[STRING_LIST_SIZE];
    size_t string_lens[STRING_LIST_SIZE];  ///< UTF-16LE 字节数 / UTF-16LE byte counts
    size_t max_string_length;  ///< 最大 UTF-16LE 字节数 / Maximum UTF-16LE byte length
  };

  /**
   * @brief 编译期构造 LanguagePack / Compile-time LanguagePack constructor
   */
  template <size_t N1, size_t N2, size_t N3>
  static const constexpr LanguagePack MakeLanguagePack(Language lang,
                                                       const char (&manu)[N1],
                                                       const char (&prod)[N2],
                                                       const char (&serial)[N3])
  {
    static_assert(N1 > 0U && N2 > 0U && N3 > 0U, "Strings must be NUL-terminated.");
    static_assert(N1 < 128 && N2 < 128 && N3 < 128,
                  "String length must be less than 128.");

    if (!HasNul(manu, N1) || !HasNul(prod, N2) || !HasNul(serial, N3))
    {
      REQUIRE(false);
    }

    size_t len_manu = 0U;
    size_t len_prod = 0U;
    size_t len_serial = 0U;
    (void)ProcessUtf8(manu, N1, nullptr, 0U, len_manu);
    (void)ProcessUtf8(prod, N2, nullptr, 0U, len_prod);
    (void)ProcessUtf8(serial, N3, nullptr, 0U, len_serial);

    size_t maxlen = len_manu;
    if (len_prod > maxlen)
    {
      maxlen = len_prod;
    }
    if (len_serial > maxlen)
    {
      maxlen = len_serial;
    }

    return LanguagePack{
        lang, {manu, prod, serial}, {len_manu, len_prod, len_serial}, maxlen};
  }

  /**
   * @brief USB 描述符字符串管理器构造函数
   *        USB descriptor string manager constructor
   *
   * @param lang_list 全局/静态 LanguagePack 对象指针表 / Pointer table of LanguagePack
   * @param uid       UID 字节数组（可选） / UID byte array (optional)
   * @param uid_len   UID 长度（单位字节，可选） / UID length (bytes, optional)
   *
   * @note Serial = SERIAL_NUMBER_STRING + UID
   */
  DescriptorStrings(const std::initializer_list<const LanguagePack*>& lang_list,
                    const uint8_t* uid = nullptr, size_t uid_len = 0);

  DescriptorStrings(const DescriptorStrings&) = delete;
  DescriptorStrings& operator=(const DescriptorStrings&) = delete;

  /**
   * @brief 生成指定语言和索引的字符串描述符
   *        Generate USB string descriptor for given language and string index
   *
   * @param index 字符串类型索引 / String index
   * @param lang  语言ID / Language ID
   * @return 错误码 / Error code
   */
  ErrorCode GenerateString(Index index, uint16_t lang);

  /**
   * @brief 获取当前构建好的字符串描述符数据 / Get current string descriptor buffer
   * @return 描述符视图；首次成功前或编码失败后长度为 0 / Descriptor view (size 0 before
   *         first success or after FAILED)
   * @note 返回最近一次 `GenerateString()` 成功生成的结果；下一次调用前有效。
   *       Returns the last successfully generated descriptor; valid until next call.
   */
  RawData GetData();

  /**
   * @brief 获取语言ID描述符内容
   *        Get LangID descriptor data
   * @return RawData 数据结构 / RawData
   */
  RawData GetLangIDData();

  /**
   * @brief 检查是否注册了指定语言 / Check whether the given language is registered
   * @param lang 语言ID / Language ID
   * @return true：已注册；false：未注册 / true: registered; false: not registered
   */
  [[nodiscard]] bool HasLanguage(uint16_t lang) const;

 private:
  friend class DeviceComposition;

  /// USB 字符串描述符最大总字节数（2 字节头 + 偶数 UTF-16LE 负载） / Maximum byte length
  /// of a USB string descriptor (2-byte header + even UTF-16LE payload)
  static constexpr size_t MAX_DESCRIPTOR_BYTES = 254U;

  /// 分配前校验语言包总数在 1..126 之内 / Validate language pack count in 1..126
  static size_t CheckedLangCount(
      const std::initializer_list<const LanguagePack*>& lang_list);

  /// 检查前 limit 字节内是否存在 '\\0' 终止符 / Check whether '\\0' appears within limit
  static constexpr bool HasNul(const char* text, size_t limit)
  {
    for (size_t i = 0U; i < limit; ++i)
    {
      if (text[i] == '\0')
      {
        return true;
      }
    }
    return false;
  }

  /**
   * @brief 计算 UTF-8 串的 UTF-16LE 字节数 / Measure UTF-8 string as UTF-16LE bytes
   * @note 仅保留 BMP，拒绝 overlong 与代理项并跳过截断或孤立续字节。
   *       BMP only; rejects overlongs and surrogates and skips truncated/orphan bytes.
   */
  static size_t CalcUTF16LELen(const char* input)
  {
    size_t len = 0U;
    (void)EncodeUtf16Le(input, nullptr, 0U, len);
    return len;
  }

  /// 将 UTF-8 编码为 UTF-16LE / Encode UTF-8 to UTF-16LE (false on capacity overflow)
  static bool EncodeUtf16Le(const char* str, uint8_t* buffer, size_t buffer_bytes,
                            size_t& written);

  /**
   * @brief UTF-8 计长与 UTF-16LE 编码核心 / Core UTF-8 sizing and UTF-16LE encoder
   * @note 按 limit 与 '\\0' 双边界截断安全，out 为 nullptr 时仅计算字节数。
   *       Bounded by both limit and NUL truncation; measures bytes only when out is
   *       nullptr.
   */
  static constexpr bool ProcessUtf8(const char* text, size_t limit, uint8_t* out,
                                    size_t out_capacity, size_t& written)
  {
    written = 0U;
    if (text == nullptr)
    {
      return true;
    }
    for (size_t i = 0U; i < limit && text[i] != '\0';)
    {
      const auto c0 = static_cast<unsigned char>(text[i]);
      uint32_t codepoint = 0U;
      bool emit = false;
      if (c0 < 0x80U)
      {
        codepoint = c0;
        emit = true;
        i += 1U;
      }
      else if (c0 >= 0xC2U && (c0 & 0xE0U) == 0xC0U)
      {
        if (limit - i > 1U && text[i + 1U] != '\0' &&
            (static_cast<unsigned char>(text[i + 1U]) & 0xC0U) == 0x80U)
        {
          codepoint =
              (static_cast<uint32_t>(c0 & 0x1FU) << 6U) |
              static_cast<uint32_t>(static_cast<unsigned char>(text[i + 1U]) & 0x3FU);
          emit = true;
          i += 2U;
        }
        else
        {
          i += 1U;
        }
      }
      else if ((c0 & 0xF0U) == 0xE0U)
      {
        if (limit - i > 2U && text[i + 1U] != '\0' && text[i + 2U] != '\0' &&
            (static_cast<unsigned char>(text[i + 1U]) & 0xC0U) == 0x80U &&
            (static_cast<unsigned char>(text[i + 2U]) & 0xC0U) == 0x80U)
        {
          const uint32_t decoded =
              (static_cast<uint32_t>(c0 & 0x0FU) << 12U) |
              (static_cast<uint32_t>(static_cast<unsigned char>(text[i + 1U]) & 0x3FU)
               << 6U) |
              static_cast<uint32_t>(static_cast<unsigned char>(text[i + 2U]) & 0x3FU);
          // 拒绝 overlong（< 0x0800）与 UTF-16 代理区（0xD800..0xDFFF）。
          // Reject overlong (< 0x0800) and UTF-16 surrogate (0xD800..0xDFFF) sequences.
          if (decoded >= 0x0800U && (decoded < 0xD800U || decoded > 0xDFFFU))
          {
            codepoint = decoded;
            emit = true;
            i += 3U;
          }
          else
          {
            i += 1U;
          }
        }
        else
        {
          i += 1U;
        }
      }
      else if ((c0 & 0xF8U) == 0xF0U)
      {
        // 忽略 BMP 之外的码点；遇 '\\0' 立即停止，不越界。
        // Ignore code points outside BMP; stop at NUL immediately without overreading.
        i += 1U;
        for (size_t k = 0U; k < 3U && i < limit && text[i] != '\0' &&
                            (static_cast<unsigned char>(text[i]) & 0xC0U) == 0x80U;
             ++k)
        {
          i += 1U;
        }
      }
      else
      {
        i += 1U;
      }

      if (emit)
      {
        if (out != nullptr)
        {
          if (out_capacity - written < 2U)
          {
            return false;
          }
          out[written] = static_cast<uint8_t>(codepoint & 0xFFU);
          out[written + 1U] = static_cast<uint8_t>((codepoint >> 8U) & 0xFFU);
        }
        written += 2U;
      }
    }
    return true;
  }

  const size_t LANG_NUM;              ///< 已注册语言数量 / Registered language count
  uint16_t* header_;                  ///< 语言ID描述符头部 / LangID descriptor header
  uint16_t* land_id_;                 ///< 语言ID数组 / LangID array
  const LanguagePack** string_list_;  ///< 多语言包指针表 / LanguagePack pointer table
  RawData buffer_;                    ///< 临时描述符缓冲区 / Temp descriptor buffer

  const uint8_t* serial_uid_;  ///< 序列号后缀 UID （原始字节） / Serial number suffix UID
                               ///< (original bytes)
  size_t serial_uid_len_;      ///< UID 字节数 / UID byte count
};

}  // namespace LibXR::USB
