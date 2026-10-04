#pragma once

#include <cstddef>
#include <tuple>
#include <type_traits>
#include <utility>

#include "format/format_frontend_detail.hpp"
#include "writer.hpp"

namespace LibXR
{
template <Print::Text Source>
class Format
{
 private:
  using SourceError = Print::Detail::FormatFrontend::Error;
  inline static constexpr auto source_analysis =
      Print::Detail::FormatFrontend::Analyze<Source>();

  static_assert(source_analysis.error != SourceError::NumberOverflow,
                "LibXR::Format: index, width, or precision is too large");
  static_assert(source_analysis.error != SourceError::UnexpectedEnd,
                "LibXR::Format: unexpected end of format string");
  static_assert(
      source_analysis.error != SourceError::EmbeddedNul,
      "LibXR::Format: embedded NUL bytes are not supported in the format literal");
  static_assert(source_analysis.error != SourceError::UnmatchedBrace,
                "LibXR::Format: unmatched brace in format string");
  static_assert(source_analysis.error != SourceError::MixedIndexing,
                "LibXR::Format: automatic and manual argument indexing cannot be mixed");
  static_assert(
      source_analysis.error != SourceError::ManualIndexingDisabled,
      "LibXR::Format: explicit argument indexing is disabled in the current profile");
  static_assert(source_analysis.error != SourceError::DynamicField,
                "LibXR::Format: dynamic width and precision are not supported");
  static_assert(source_analysis.error != SourceError::InvalidArgumentIndex,
                "LibXR::Format: invalid argument index");
  static_assert(source_analysis.error != SourceError::InvalidSpecifier,
                "LibXR::Format: invalid format specifier");
  static_assert(source_analysis.error != SourceError::InvalidPresentation,
                "LibXR::Format: invalid presentation type");

 public:
  /**
   * @brief 面向用户的 brace 风格编译期失败类别 / Public brace-style compile-time failure
   * categories
   */
  using Error = Print::Detail::FormatFrontend::Error;

  /**
   * @brief 绑定到一组具体 C++ 参数类型上的前端适配器 / Frontend adapter bound to one
   * concrete C++ argument list
   * @tparam Args 这次调用点对应的 C++ 实参类型列表 / Concrete C++ argument types at one
   * call site
   */
  template <typename... Args>
  using Compiler = Print::Detail::FormatFrontend::Compiler<Source, Args...>;

  /**
   * @brief 一条 brace 风格源串绑定到具体调用点参数后的编译结果 / One brace-style source
   * bound to a concrete call-site argument list
   * @tparam Args 这份编译结果对应的 C++ 实参类型列表 / Concrete C++ argument types bound
   * into this compiled result
   */
  template <typename... Args>
  struct Compiled
  {
   private:
    using Frontend = Compiler<Args...>;
    inline static constexpr auto result = Print::FormatCompiler<Frontend>::Compile();

    static_assert(source_analysis.error != SourceError::None ||
                      sizeof...(Args) == source_analysis.required_argument_count,
                  "LibXR::Format: call-site argument count does not match the "
                  "referenced replacement fields");
    static_assert(result.compile_error != Error::NumberOverflow,
                  "LibXR::Format: index, width, or precision is too large");
    static_assert(result.compile_error != Error::FloatPrecisionLimitExceeded,
                  "LibXR::Format: float precision exceeds the configured print limit");
    static_assert(result.compile_error != Error::UnexpectedEnd,
                  "LibXR::Format: unexpected end of format string");
    static_assert(
        result.compile_error != Error::EmbeddedNul,
        "LibXR::Format: embedded NUL bytes are not supported in the format literal");
    static_assert(result.compile_error != Error::UnmatchedBrace,
                  "LibXR::Format: unmatched brace in format string");
    static_assert(
        result.compile_error != Error::MixedIndexing,
        "LibXR::Format: automatic and manual argument indexing cannot be mixed");
    static_assert(
        result.compile_error != Error::ManualIndexingDisabled,
        "LibXR::Format: explicit argument indexing is disabled in the current profile");
    static_assert(result.compile_error != Error::DynamicField,
                  "LibXR::Format: dynamic width and precision are not supported");
    static_assert(result.compile_error != Error::InvalidArgumentIndex,
                  "LibXR::Format: invalid argument index");
    static_assert(result.compile_error != Error::InvalidSpecifier,
                  "LibXR::Format: invalid format specifier");
    static_assert(result.compile_error != Error::InvalidPresentation,
                  "LibXR::Format: invalid presentation type");
    static_assert(result.compile_error != Error::MissingArgument,
                  "LibXR::Format: referenced argument index is out of range");
    static_assert(
        result.compile_error != Error::ArgumentTypeMismatch,
        "LibXR::Format: format options are incompatible with the selected argument type");
    static_assert(result.compile_error != Error::UnsupportedArgumentType,
                  "LibXR::Format: unsupported C++ argument type");
    static_assert(result.compile_error != Error::TextOffsetOverflow,
                  "LibXR::Format: text pool offset is too large");
    static_assert(result.compile_error != Error::TextSizeOverflow,
                  "LibXR::Format: text span is too large");

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
    inline static constexpr Print::FormatProfile profile = result.profile;

    /**
     * @brief 返回运行期参数打包时要按字段顺序读取的参数列表 / Returns the field-ordered
     * argument list the runtime packer will follow.
     */
    [[nodiscard]] static constexpr auto ArgumentList() { return result.arg_info; }

    /**
     * @brief 返回每个字段对应的是第几个源参数 / Returns, for each field, which source
     * argument index it refers to.
     */
    [[nodiscard]] static constexpr auto ArgumentOrder()
    {
      return source_analysis.argument_order;
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
    [[nodiscard]] static constexpr Print::FormatProfile Profile() { return profile; }

    /**
     * @brief 判断另一组 C++ 参数类型是否与当前格式绑定时的参数列表完全一致 / Returns
     * whether another C++ argument list is exactly the one this format was built for.
     * @tparam Actual 待比较的另一组 C++ 实参类型。
     *         Another C++ argument-type list to compare against.
     * @return 完全一致返回 `true`，否则返回 `false`。
     *         Returns `true` when the type lists match exactly, otherwise `false`.
     */
    template <typename... Actual>
    [[nodiscard]] static consteval bool Matches()
    {
      return std::is_same_v<std::tuple<std::remove_cvref_t<Actual>...>,
                            std::tuple<Args...>>;
    }
  };

  /**
   * @brief 返回当前源串会寻址的调用点参数个数 / Returns the required call-site argument
   * count addressed by this source.
   * @return 当前源串实际引用到的调用点参数个数。
   *         Returns the number of call-site arguments actually referenced by this source.
   */
  [[nodiscard]] static constexpr size_t ArgumentCount()
  {
    return source_analysis.required_argument_count;
  }

  /**
   * @brief 判断这条源格式串能否和 `Args...` 一起通过编译 / Returns whether this source
   * string can be compiled with `Args...`.
   * @tparam Args 待检查的 C++ 实参类型列表 / C++ argument types to test.
   * @return 可编译返回 `true`，否则返回 `false`。
   *         Returns `true` when the source can compile with `Args...`, otherwise `false`.
   */
  template <typename... Args>
  [[nodiscard]] static consteval bool Matches()
  {
    if constexpr (sizeof...(Args) != source_analysis.required_argument_count)
    {
      return false;
    }
    else
    {
      using Frontend = Compiler<std::remove_cvref_t<Args>...>;
      return Print::FormatCompiler<Frontend>::Compile().compile_error == Error::None;
    }
  }

  /**
   * @brief 将当前格式写入一个输出端，并且只返回 sink 状态 / Write this format into one
   * sink and return only the sink status
   * @tparam Sink 输出端类型，需满足 `Print::OutputSink` / Sink type satisfying
   * `Print::OutputSink`
   * @tparam Args 调用点实参类型列表 / Call-site argument types
   * @param sink 输出端 / Destination sink
   * @param args 本次写出使用的实参 / Arguments used by this write
   * @return sink 写入状态 / Returns the sink write status
   */
  template <Print::OutputSink Sink, typename... Args>
  [[nodiscard]] ErrorCode WriteTo(Sink& sink, Args&&... args) const
  {
    using Built = Compiled<std::remove_cvref_t<Args>...>;
    return Print::Writer::template RunArgumentOrder<Sink, Built, Built::ArgumentOrder()>(
        sink, Built{}, std::forward<Args>(args)...);
  }
};
}  // namespace LibXR

namespace LibXR::Print
{
/**
 * @brief 写出一条 brace 格式：先确定它对应的具体参数类型，再交给共享 writer 执行 / Write
 * one brace format by first fixing its concrete argument types, then run the shared
 * writer
 * @tparam Source brace 风格格式串字面量 / Brace-style format literal
 * @tparam Sink 输出端类型，需满足 `OutputSink` / Sink type satisfying `OutputSink`
 * @tparam Args 调用点实参类型列表 / Call-site argument types
 * @param sink 输出端 / Destination sink
 * @param args 本次写出使用的实参 / Arguments used by this write
 * @return sink 写入状态 / Returns the sink write status
 */
template <Text Source, OutputSink Sink, typename... Args>
[[nodiscard]] inline ErrorCode Write(Sink& sink, const LibXR::Format<Source>&,
                                     Args&&... args)
{
  using Built =
      typename LibXR::Format<Source>::template Compiled<std::remove_cvref_t<Args>...>;
  return Writer::template RunArgumentOrder<Sink, Built, Built::ArgumentOrder()>(
      sink, Built{}, std::forward<Args>(args)...);
}
}  // namespace LibXR::Print
