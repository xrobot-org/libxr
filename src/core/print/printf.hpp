#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "format_argument.hpp"

namespace LibXR::Print
{
/**
 * @brief 作为 printf 格式源的结构化字符串字面量包装 / Structural literal wrapper used as
 * the NTTP source for printf formats
 */
template <size_t N>
struct Text
{
  /**
   * @brief 含结尾零字节的字面量字节序列 / Literal bytes including the terminating zero
   * byte
   */
  char data[N]{};

  /**
   * @brief 将字符串字面量复制到结构化 NTTP 对象中 / Copy the string literal into the
   * structural NTTP object
   * @param text 原始字符串字面量 / Source string literal
   */
  constexpr Text(const char (&text)[N])
  {
    for (size_t i = 0; i < N; ++i)
    {
      data[i] = text[i];
    }
  }

  /**
   * @brief 返回不含结尾零字节的格式串长度 / Return the format length without the
   * terminating zero byte
   * @return 不含结尾零字节的字面量长度 / Returns the literal size without the trailing
   * zero byte
   */
  [[nodiscard]] constexpr size_t Size() const { return N - 1; }
  /**
   * @brief 返回含结尾零字节的字面量指针 / Return the literal bytes including the
   * terminating zero byte
   * @return 指向含结尾零字节的字面量字节序列 / Returns a pointer to the literal bytes
   * including the trailing zero byte
   */
  [[nodiscard]] constexpr const char* Data() const { return data; }
};

/**
 * @brief printf 风格前端，输出内部格式表示 / Printf-style frontend that emits the
 * internal format representation
 */
class Printf
{
 public:
  /**
   * @brief 归一化后的 printf 长度修饰符 / Supported printf length modifiers after
   * normalization
   */
  enum class Length : uint8_t
  {
    Default,     ///< 无长度修饰符 / no length modifier
    Char,        ///< char 长度修饰 / hh
    Short,       ///< short 长度修饰 / h
    Long,        ///< long 长度修饰 / l
    LongLong,    ///< long long 长度修饰 / ll
    IntMax,      ///< intmax_t 长度修饰 / j
    Size,        ///< size_t 长度修饰 / z
    PtrDiff,     ///< ptrdiff_t 长度修饰 / t
    LongDouble,  ///< long double 长度修饰 / L
  };

  /**
   * @brief 通过 static_assert 暴露的编译期解析或构建失败类别 / Compile-time parse or
   * build failure categories surfaced through static_assert
   */
  enum class Error : uint8_t
  {
    None,                         ///< 成功 / success
    NumberOverflow,               ///< 宽度或精度字面量超出字段范围
                                  ///< width / precision literal does not fit in its field
    FloatPrecisionLimitExceeded,  ///< 浮点精度超出当前配置的前端上限
                                  ///< float precision exceeds the configured frontend
                                  ///< limit
    UnexpectedEnd,                ///< 格式串在转换项中途结束
                                  ///< format string ended in the middle of one conversion
    EmbeddedNul,                  ///< 格式串字面量内部包含嵌入式 NUL 字节
                                  ///< format literal contains an embedded NUL byte
    MixedIndexing,                ///< 混用了位置参数与顺序参数
                                  ///< positional and sequential arguments were mixed
    PositionalArgumentDisabled,   ///< 位置参数 n$ 索引已被配置关闭
                                  ///< positional n$ indexing is disabled by configuration
    DynamicField,          ///< 不支持 * 宽度或精度 / * width / precision is not supported
    InvalidArgumentIndex,  ///< 位置参数索引非法 / positional argument index is invalid
    InvalidSpecifier,      ///< 转换说明符无效或被禁用
                           ///< unsupported or disabled conversion specifier
    InvalidLength,         ///< 长度修饰符与转换说明不兼容
                           ///< length modifier is incompatible with the conversion
    ConflictingArgument,   ///< 同一位置参数被不兼容的规则重复使用
                           ///< one positional argument was reused with incompatible rules
    TextOffsetOverflow,    ///< 文本池偏移超出 uint16_t
                           ///< referenced text offset no longer fits in uint16_t
    TextSizeOverflow,      ///< 文本长度超出 uint16_t
                           ///< referenced text size no longer fits in uint16_t
  };

  template <Text Source>
  class Compiler;

  template <Text Source>
  struct Compiled;

  /**
   * @brief 在编译期解析并校验 printf 格式串 / Parse and validate a printf format at
   * compile time
   * @tparam Source printf 风格格式串字面量 / Printf-style format literal
   * @return 编译后的 printf 格式对象 / Returns the compiled printf format object
   */
  template <Text Source>
  [[nodiscard]] static consteval Compiled<Source> Build()
  {
    return {};
  }

  /**
   * @brief 判断 `Args...` 是否与启用的转换项精确匹配 / Return whether `Args...` exactly
   * match the enabled conversions
   * @tparam Source printf 风格格式串字面量 / Printf-style format literal
   * @tparam Args 待检查的 C++ 实参类型列表 / C++ argument types to test
   * @return 完全匹配返回 `true`，否则返回 `false` / Returns `true` when the enabled
   * conversions match `Args...` exactly, otherwise `false`
   */
  template <Text Source, typename... Args>
  [[nodiscard]] static consteval bool Matches()
  {
    return Compiled<Source>::template Matches<Args...>();
  }
};
}  // namespace LibXR::Print

#include "printf/printf_frontend_detail.hpp"
namespace LibXR::Print
{
template <Text Source>
struct Printf::Compiled
{
 private:
  using Frontend = typename Printf::template Compiler<Source>;
  inline static constexpr auto source_analysis = Detail::PrintfCompile::Analyze<Source>();
  inline static constexpr auto result = FormatCompiler<Frontend>::Compile();

  static_assert(result.compile_error != Printf::Error::NumberOverflow,
                "LibXR::Print::Printf: numeric field is too large");
  static_assert(
      result.compile_error != Printf::Error::FloatPrecisionLimitExceeded,
      "LibXR::Print::Printf: float precision exceeds the configured print limit");
  static_assert(result.compile_error != Printf::Error::UnexpectedEnd,
                "LibXR::Print::Printf: unexpected end of format string");
  static_assert(
      result.compile_error != Printf::Error::EmbeddedNul,
      "LibXR::Print::Printf: embedded NUL bytes are not supported in the format literal");
  static_assert(
      result.compile_error != Printf::Error::MixedIndexing,
      "LibXR::Print::Printf: positional and sequential arguments cannot be mixed");
  static_assert(result.compile_error != Printf::Error::PositionalArgumentDisabled,
                "LibXR::Print::Printf: positional argument indexing is disabled in the "
                "current profile");
  static_assert(result.compile_error != Printf::Error::DynamicField,
                "LibXR::Print::Printf: dynamic width and precision are not supported");
  static_assert(result.compile_error != Printf::Error::InvalidArgumentIndex,
                "LibXR::Print::Printf: invalid positional argument index");
  static_assert(result.compile_error != Printf::Error::InvalidSpecifier,
                "LibXR::Print::Printf: invalid format specifier");
  static_assert(result.compile_error != Printf::Error::InvalidLength,
                "LibXR::Print::Printf: invalid length modifier");
  static_assert(result.compile_error != Printf::Error::ConflictingArgument,
                "LibXR::Print::Printf: one positional argument is reused with "
                "incompatible conversions");
  static_assert(result.compile_error != Printf::Error::TextOffsetOverflow,
                "LibXR::Print::Printf: text pool offset is too large");
  static_assert(result.compile_error != Printf::Error::TextSizeOverflow,
                "LibXR::Print::Printf: text span is too large");
  static_assert(source_analysis.error != Printf::Error::ConflictingArgument,
                "LibXR::Print::Printf: one positional argument is reused with "
                "incompatible conversions");

 public:
  /**
   * @brief 返回运行期 writer 最终会执行的字节流 / Returns the final byte stream that
   * the runtime writer will execute.
   */
  inline static constexpr auto codes = result.codes;
  /**
   * @brief 返回当前格式需要哪些 writer 分支的编译期摘要 / Returns the compile-time
   * summary of which writer branches this format needs.
   */
  inline static constexpr FormatProfile profile = result.profile;

  /**
   * @brief 返回运行期参数打包时要按字段顺序读取的参数列表 / Returns the field-ordered
   * argument list the runtime packer will follow.
   */
  [[nodiscard]] static constexpr auto ArgumentList() { return result.arg_info; }

  /**
   * @brief 返回每个字段对应的是第几个源参数 / Returns, for each field, which source
   * argument index it refers to.
   */
  [[nodiscard]] static constexpr auto ArgumentOrder() { return source_analysis.order; }

  /**
   * @brief 返回仅供编译期类型匹配使用的源参数列表 / Returns the source-argument list
   * used only for compile-time type matching.
   */
  [[nodiscard]] static constexpr auto SourceArgumentList()
  {
    return source_analysis.args;
  }

  /**
   * @brief 返回与 `codes` 相同的最终字节流 / Returns the same final byte stream as
   * `codes`.
   */
  [[nodiscard]] static constexpr const auto& Codes() { return codes; }

  /**
   * @brief 返回当前编译格式携带的 writer 分支摘要 / Returns the writer-branch summary
   * carried by this compiled format.
   */
  [[nodiscard]] static constexpr FormatProfile Profile() { return profile; }

  /**
   * @brief 判断 `Args...` 是否就是当前编译格式期望的那组 C++ 参数类型 / Returns whether
   * `Args...` are exactly the C++ argument types this compiled format expects.
   * @tparam Args 待比较的 C++ 实参类型列表 / C++ argument types to compare.
   * @return 完全匹配返回 `true`，否则返回 `false`。
   *         Returns `true` when the type list matches exactly, otherwise `false`.
   */
  template <typename... Args>
  [[nodiscard]] static consteval bool Matches()
  {
    return Detail::FormatArgument::template Matches<Args...>(SourceArgumentList());
  }
};
}  // namespace LibXR::Print
