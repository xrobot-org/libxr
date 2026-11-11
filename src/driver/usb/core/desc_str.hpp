#pragma once

#include <cstddef>
#include <cstdint>

#include "core.hpp"
#include "libxr_type.hpp"

namespace LibXR::USB
{
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

  /// 指向静态 UTF-16LE 字符串数据的指针 / Pointer to static UTF-16LE string data
  typedef const char* StringData;
  /// 每种语言登记的字符串数量（厂商、产品、序列号）/ Strings registered per language
  /// (manufacturer, product, serial number)
  static constexpr size_t STRING_LIST_SIZE = 3;

  /**
   * @brief 单语言描述符包（用于注册静态多语言字符串）
   *        Single language USB string descriptor pack for registration
   */
  struct LanguagePack
  {
    Language lang_id;                      ///< 语言 / Language
    StringData strings[STRING_LIST_SIZE];  ///< 指向 UTF-16LE 静态数组 / UTF-16LE strings
    size_t string_lens[STRING_LIST_SIZE];  ///< 每个字符串的字节数 / String byte lengths
    size_t max_string_length;              ///< 最大字符串长度 / Maximum string length
  };

  /**
   * @brief 编译期构造 LanguagePack
   *        Compile-time LanguagePack constructor
   */
  template <size_t N1, size_t N2, size_t N3>
  static const constexpr LanguagePack MakeLanguagePack(Language lang,
                                                       const char (&manu)[N1],
                                                       const char (&prod)[N2],
                                                       const char (&serial)[N3])
  {
    static_assert(N1 < 128 && N2 < 128 && N3 < 128,
                  "String length must be less than 128.");

    auto len_manu = CalcUTF16LELen(manu);
    auto len_prod = CalcUTF16LELen(prod);
    auto len_serial = CalcUTF16LELen(serial);

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
   * @brief 获取当前构建好的字符串描述符数据
   *        Get the descriptor buffer
   * @return RawData 数据结构 / RawData
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
  template <size_t N>
  static constexpr size_t CalcUTF16LELen(const char (&input)[N])
  {
    size_t len = 0;
    for (size_t i = 0; i < N && input[i];)
    {
      unsigned char c = static_cast<unsigned char>(input[i]);
      if (c < 0x80)
      {
        len += 2;
        i += 1;
      }
      else if ((c & 0xE0) == 0xC0)
      {
        len += 2;
        i += 2;
      }
      else if ((c & 0xF0) == 0xE0)
      {
        len += 2;
        i += 3;
      }
      else
      {
        i += 4;
      }
    }
    return len;
  }

  static void ToUTF16LE(const char* str, uint8_t* buffer);

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
