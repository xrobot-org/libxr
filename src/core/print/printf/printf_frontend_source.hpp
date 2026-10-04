#pragma once

#include "printf_frontend_binding_base.hpp"
#include "printf_frontend_binding_field.hpp"

namespace SourceSyntax
{
/**
 * @brief 供位置参数分析使用的仅源串临时摘要 / Source-only scratch summary for positional
 * argument analysis
 * @tparam MaxFieldCount 已解析转换项数量的保守上界 / Conservative upper bound for parsed
 * conversion count
 */
template <size_t MaxFieldCount>
struct SourceAnalysisScratch
{
  std::array<size_t, MaxFieldCount>
      order{};  ///< 按字段顺序排列的源参数索引 / field-ordered source argument indexes
  size_t field_count = 0;  ///< 已解析的转换项数量 / parsed conversion count
  size_t argument_count =
      0;  ///< 最高引用到的参数个数 / highest referenced argument count
  Error error = Error::None;  ///< 首个解析或分析错误 / first parse/analysis error

  [[nodiscard]] consteval Error Text(size_t, size_t) const { return Error::None; }

  [[nodiscard]] consteval Error Field(const Conversion& conversion)
  {
    order[field_count++] = conversion.arg_index;
    size_t used_argument_count = conversion.arg_index + 1;
    if (used_argument_count > argument_count)
    {
      argument_count = used_argument_count;
    }
    return Error::None;
  }
};

/**
 * @brief 最终的源级位置参数或顺序参数摘要 / Final source-level positional or sequential
 * argument summary
 * @tparam FieldCount 最终解析出的转换项数量 / Final parsed conversion count
 * @tparam ArgCount 最终引用到的参数个数 / Final referenced argument count
 */
template <size_t FieldCount, size_t ArgCount>
struct SourceAnalysis
{
  std::array<size_t, FieldCount>
      order{};  ///< 按字段顺序排列的源参数索引 / field-ordered source argument indexes
  std::array<FormatArgumentInfo, ArgCount>
      args{};  ///< 按源参数顺序排列的参数元信息 / source-ordered argument metadata
  Error error = Error::None;  ///< 首个解析或分析错误 / first parse/analysis error
};

/**
 * @brief 将已解析转换解析成最终源顺序参数元信息摘要的 visitor / Visitor that resolves
 * parsed conversions into the final source-ordered argument metadata summary
 * @tparam FieldCount 最终解析出的转换项数量 / Final parsed conversion count
 * @tparam ArgCount 最终引用到的参数个数 / Final referenced argument count
 */
template <size_t FieldCount, size_t ArgCount>
struct ResolvedArgumentVisitor
{
  SourceAnalysis<FieldCount, ArgCount>& analysis;

  [[nodiscard]] consteval Error Text(size_t, size_t) const { return Error::None; }

  [[nodiscard]] consteval Error Field(const Conversion& conversion)
  {
    auto field = FieldSelection::BuildFormatField(conversion);
    auto info = FormatArgumentInfo{
        .pack = field.pack,
        .rule = field.rule,
    };
    auto& slot = analysis.args[conversion.arg_index];
    if (slot.rule != FormatArgumentRule::None &&
        (slot.rule != info.rule || slot.pack != info.pack))
    {
      return Error::ConflictingArgument;
    }

    slot = info;
    return Error::None;
  }
};

#include "printf_frontend_parser.hpp"

/**
 * @brief 遍历一个 printf 源字符串，并发射字面文本片段与已解析转换项 / Walk one printf
 * source string and emit literal-text spans plus parsed conversions
 * @param source 当前待解析的 printf 源字符串 / Printf source string to walk
 * @param visitor 接收文本片段与已解析转换项的 visitor / Visitor receiving text spans and
 * parsed conversions
 * @return 首个源级解析错误；成功时返回 `Error::None` / Returns the first source-level
 * parse error, or `Error::None` on success
 */
[[nodiscard]] consteval Error WalkSource(std::string_view source, auto& visitor)
{
  if (HasEmbeddedNul(source))
  {
    return Error::EmbeddedNul;
  }

  size_t pos = 0;
  size_t text_begin = 0;
  IndexingState indexing{};

  while (pos < source.size())
  {
    if (source[pos] != '%')
    {
      ++pos;
      continue;
    }

    auto error = visitor.Text(text_begin, pos - text_begin);
    if (error != Error::None)
    {
      return error;
    }

    if (pos + 1 < source.size() && source[pos + 1] == '%')
    {
      error = visitor.Text(pos, 1);
      if (error != Error::None)
      {
        return error;
      }

      pos += 2;
      text_begin = pos;
      continue;
    }

    auto parse_pos = pos;
    Conversion conversion{};
    error = Parse(source, parse_pos, indexing, conversion);
    if (error != Error::None)
    {
      return error;
    }

    error = visitor.Field(conversion);
    if (error != Error::None)
    {
      return error;
    }

    pos = parse_pos;
    text_begin = pos;
  }

  return visitor.Text(text_begin, source.size() - text_begin);
}

/**
 * @brief 对一个 printf 字面量执行仅源串分析，并返回按顺序整理的参数引用摘要 / Run
 * source-only analysis for one printf literal and return the ordered argument-reference
 * summary
 * @tparam Source printf 风格格式串字面量 / Printf-style format literal
 * @return 源串分析结果，包含字段顺序、参数引用摘要与首个源级错误 / Returns the source
 * analysis result, including field order, argument reference summary, and the first
 * source-level error
 */
template <Text Source>
[[nodiscard]] consteval auto Analyze()
{
  constexpr auto scratch = []() consteval
  {
    SourceAnalysisScratch<Source.Size()> visitor{};
    visitor.error = WalkSource(std::string_view(Source.Data(), Source.Size()), visitor);
    return visitor;
  }();

  SourceAnalysis<scratch.field_count, scratch.argument_count> result{};
  result.error = scratch.error;
  for (size_t i = 0; i < scratch.field_count; ++i)
  {
    result.order[i] = scratch.order[i];
  }

  if constexpr (scratch.error != Error::None)
  {
    return result;
  }
  else
  {
    ResolvedArgumentVisitor<scratch.field_count, scratch.argument_count> visitor{result};
    result.error = WalkSource(std::string_view(Source.Data(), Source.Size()), visitor);
    return result;
  }
}
}  // namespace SourceSyntax
