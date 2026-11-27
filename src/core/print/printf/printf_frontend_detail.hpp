#pragma once

#include <array>
#include <limits>
#include <string_view>

#include "../format_compile.hpp"

namespace LibXR::Print::Detail::PrintfCompile
{
/**
 * @brief 把一条 printf 字面量转换成共享打印字段的编译期入口层 / Compile-time entry layer
 * that turns one printf literal into shared print fields
 */
using Error = Printf::Error;
using Length = Printf::Length;

// These implementation headers form an ordered dependency chain.
// clang-format off
#include "printf_frontend_spec.hpp"

#include "printf_frontend_source.hpp"
// clang-format on

namespace FieldSelection = SourceSyntax::FieldSelection;

/**
 * @brief 对单条 printf 字面量执行仅源串分析 / Run source-only analysis for one printf
 * literal
 * @tparam Source printf 风格格式串字面量 / Printf-style format literal
 * @return 源串分析结果，包含字段顺序、参数引用摘要与首个源级错误 / Returns the source
 * analysis result, including field order, argument reference summary, and the first
 * source-level error
 */
template <Text Source>
[[nodiscard]] consteval auto Analyze()
{
  return SourceSyntax::Analyze<Source>();
}

/**
 * @brief 遍历一条 printf 字面量，并为每个已解析转换产出共享 `FormatField` 记录 / Walk one
 * printf literal and emit shared `FormatField` records for each parsed conversion
 * @tparam Source printf 风格格式串字面量 / Printf-style format literal
 * @param visitor 接收文本片段与最终 `FormatField` 记录的 visitor / Visitor receiving text
 * spans and final `FormatField` records
 * @return 首个源级解析错误或字段选择错误；成功时返回 `Error::None` / Returns the first
 * source-level parse or field-selection error; returns `Error::None` on success
 */
template <Text Source>
[[nodiscard]] consteval Error WalkSourceAsFormatFields(auto& visitor)
{
  struct ResolvingVisitor
  {
    decltype(visitor)& inner;

    [[nodiscard]] consteval Error Text(size_t offset, size_t text_size)
    {
      return inner.Text(offset, text_size);
    }

    [[nodiscard]] consteval Error Field(const Conversion& conversion)
    {
      return inner.Field(FieldSelection::BuildFormatField(conversion));
    }
  };

  ResolvingVisitor resolving{visitor};
  return SourceSyntax::WalkSource(std::string_view(Source.Data(), Source.Size()),
                                  resolving);
}
}  // namespace LibXR::Print::Detail::PrintfCompile

namespace LibXR::Print
{
/**
 * @brief 单个 printf 源字符串的编译期前端，负责解析并降为共享格式语义 / Compile-time
 * printf frontend that parses and lowers one source string
 * @tparam Source printf 风格格式串字面量 / Printf-style format literal
 */
template <Text Source>
class Printf::Compiler
{
 public:
  using ErrorType = Printf::Error;  ///< 前端错误码类型 / Frontend error code type.

  /**
   * @brief 返回去掉结尾零字节后的源字符串数据 / Return the underlying source bytes
   * without the terminating zero byte
   * @return 指向当前格式字面量正文的指针 / Returns a pointer to the literal body
   */
  [[nodiscard]] static constexpr const char* SourceData() { return Source.Data(); }
  /**
   * @brief 返回去掉结尾零字节后的源字符串长度 / Return the source length without the
   * terminating zero byte
   * @return 当前格式字面量正文长度 / Returns the literal-body size
   */
  [[nodiscard]] static constexpr size_t SourceSize() { return Source.Size(); }

  /**
   * @brief 按源串顺序遍历这条字面量，并产出文本片段和最终字段记录 / Walk the literal in
   * source order and emit text spans plus final field records
   * @param visitor 接收文本片段与最终字段记录的 visitor / Visitor receiving text spans
   * and final field records
   * @return 首个源级解析错误或字段选择错误；成功时返回 `Error::None` / Returns the first
   * source-level parse or field-selection error; returns `Error::None` on success
   */
  [[nodiscard]] static consteval ErrorType Walk(auto& visitor)
  {
    return Detail::PrintfCompile::WalkSourceAsFormatFields<Source>(visitor);
  }
};
}  // namespace LibXR::Print
