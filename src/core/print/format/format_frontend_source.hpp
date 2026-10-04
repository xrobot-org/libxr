#pragma once

namespace SourceSyntax
{
/**
 * @brief 单条 brace 风格格式字面量的仅源串分析数据 / Source-only analysis data for one
 * brace-style format literal
 * @tparam FieldCount 最终解析出的字段数量 / Final parsed field count
 */
template <size_t FieldCount>
struct SourceAnalysis
{
  std::array<size_t, FieldCount>
      argument_order{};  ///< 按源串顺序引用的参数索引
                         ///< source-ordered argument references
  size_t required_argument_count =
      0;  ///< 调用点至少需要的参数个数 / minimum call-site argument count
  Error error = Error::None;  ///< 首个仅源串解析错误 / first source-only parse error
};

/**
 * @brief 仅源串分析阶段使用的保守临时累加器 / Conservative temporary accumulator used
 * during source-only analysis
 * @tparam MaxFieldCount 已解析字段数量的保守上界 / Conservative upper bound for parsed
 * field count
 */
template <size_t MaxFieldCount>
struct SourceAnalysisScratch
{
  std::array<size_t, MaxFieldCount> order{};  ///< 按字段顺序记录参数索引的临时缓冲区
                                              ///< conservative field-order scratch buffer
  size_t field_count = 0;  ///< 已解析的替换字段数量 / parsed replacement-field count
  size_t required_argument_count =
      0;  ///< 调用点至少需要的参数个数 / minimum call-site argument count
  Error error = Error::None;  ///< 首个解析错误 / first parse error

  [[nodiscard]] consteval Error Text(size_t, size_t) const { return Error::None; }

  [[nodiscard]] consteval Error Field(const ParsedField& field)
  {
    order[field_count++] = field.arg_index;
    size_t used_argument_count = field.arg_index + 1;
    if (used_argument_count > required_argument_count)
    {
      required_argument_count = used_argument_count;
    }
    return Error::None;
  }
};

#include "format_frontend_parser.hpp"

/**
 * @brief 遍历一个 brace 源字符串，并发射字面文本片段与已解析字段 / Walk one brace source
 * string and emit literal-text spans plus parsed fields
 * @param source 当前待解析的 brace 源字符串 / Brace source string to walk
 * @param visitor 接收文本片段与已解析字段的 visitor / Visitor receiving text spans and
 * parsed fields
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
    if (source[pos] == '{')
    {
      if (pos + 1 < source.size() && source[pos + 1] == '{')
      {
        auto error = visitor.Text(text_begin, pos - text_begin);
        if (error != Error::None)
        {
          return error;
        }
        error = visitor.Text(pos, 1);
        if (error != Error::None)
        {
          return error;
        }
        pos += 2;
        text_begin = pos;
        continue;
      }

      auto error = visitor.Text(text_begin, pos - text_begin);
      if (error != Error::None)
      {
        return error;
      }

      ParsedField field{};
      error = ParseField(source, pos, indexing, field);
      if (error != Error::None)
      {
        return error;
      }

      error = visitor.Field(field);
      if (error != Error::None)
      {
        return error;
      }

      text_begin = pos;
      continue;
    }

    if (source[pos] == '}')
    {
      if (pos + 1 < source.size() && source[pos + 1] == '}')
      {
        auto error = visitor.Text(text_begin, pos - text_begin);
        if (error != Error::None)
        {
          return error;
        }
        error = visitor.Text(pos, 1);
        if (error != Error::None)
        {
          return error;
        }
        pos += 2;
        text_begin = pos;
        continue;
      }

      return Error::UnmatchedBrace;
    }

    ++pos;
  }

  return visitor.Text(text_begin, source.size() - text_begin);
}

/**
 * @brief 对一个 brace 字面量执行仅源串分析，并返回按顺序整理的参数索引摘要 / Run
 * source-only analysis for one brace literal and return the ordered argument-index
 * summary
 * @tparam Source brace 风格格式串字面量 / Brace-style format literal
 * @return 源串分析结果，包含字段顺序、所需参数个数与首个源级错误 / Returns the source
 * analysis result, including field order, required argument count, and the first
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

  SourceAnalysis<scratch.field_count> result{};
  result.required_argument_count = scratch.required_argument_count;
  result.error = scratch.error;
  for (size_t i = 0; i < scratch.field_count; ++i)
  {
    result.argument_order[i] = scratch.order[i];
  }
  return result;
}
}  // namespace SourceSyntax
