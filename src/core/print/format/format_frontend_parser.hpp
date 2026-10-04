#pragma once

/**
 * @brief brace 风格语法的源串解析辅助函数 / Source parser helpers for the brace-style
 * grammar
 */
/**
 * @brief 判断一个源字节是否为 ASCII 十进制数字 / Return whether one source byte is an
 * ASCII decimal digit
 * @param ch 待检查的源字节 / Source byte to inspect
 * @return 若是 ASCII 十进制数字则返回 `true`，否则返回 `false` / Returns `true` for an
 * ASCII decimal digit, otherwise `false`
 */
[[nodiscard]] constexpr bool IsDigit(char ch) { return ch >= '0' && ch <= '9'; }

/**
 * @brief 判断一个源字符是否为合法的 brace 格式对齐标记 / Return whether one source
 * character is a valid brace-format align token
 * @param ch 待检查的源字符 / Source character to inspect
 * @return 合法对齐字符返回 `true`，否则返回 `false` / Returns `true` for a valid align
 * token, otherwise `false`
 */
[[nodiscard]] constexpr bool IsAlign(char ch)
{
  return ch == '<' || ch == '>' || ch == '^';
}

/**
 * @brief 将一个 brace 格式对齐字符转换为前端枚举值 / Convert one brace-format align token
 * to the frontend enum value
 * @param ch 对齐字符 / Align token
 * @return 对应的 `Align` 枚举；未知字符返回 `Align::None` / Returns the corresponding
 * `Align` enum; unknown tokens map to `Align::None`
 */
[[nodiscard]] constexpr Align ParseAlign(char ch)
{
  switch (ch)
  {
    case '<':
      return Align::Left;
    case '>':
      return Align::Right;
    case '^':
      return Align::Center;
    default:
      return Align::None;
  }
}

/**
 * @brief 判断源字符串在终止前是否包含嵌入式 NUL / Return whether the source contains an
 * embedded NUL before the terminator
 * @param source 待检查的源字符串 / Source string to inspect
 * @return 若终止前包含 `\\0` 则返回 `true`，否则返回 `false` / Returns `true` when `\\0`
 * appears before the terminator, otherwise `false`
 */
[[nodiscard]] constexpr bool HasEmbeddedNul(std::string_view source)
{
  for (char ch : source)
  {
    if (ch == '\0')
    {
      return true;
    }
  }
  return false;
}

/**
 * @brief 判断一个展示字符是否被 brace 前端支持 / Return whether one presentation token is
 * supported by the brace frontend
 * @param ch 展示字符 / Presentation token
 * @return 支持返回 `true`，否则返回 `false` / Returns `true` when the token is supported,
 * otherwise `false`
 */
[[nodiscard]] constexpr bool IsSupportedPresentation(char ch)
{
  switch (ch)
  {
    case 'd':
    case 'b':
    case 'B':
    case 'o':
    case 'x':
    case 'X':
    case 'c':
    case 's':
    case 'p':
    case 'f':
    case 'F':
    case 'e':
    case 'E':
    case 'g':
    case 'G':
      return true;
    default:
      return false;
  }
}

/**
 * @brief 解析一个十进制整数字段片段，并检查是否溢出 / Parse one decimal integer fragment
 * with overflow checking
 * @tparam UInt 目标无符号整数类型 / Target unsigned integer type
 * @param source 源字符串 / Source string
 * @param pos 当前解析位置，成功时推进到片段之后 / Current parse position; advanced past
 * the fragment on success
 * @param limit 允许的最大值 / Inclusive upper bound accepted by this field
 * @param value 输出解析结果 / Parsed result output
 * @return 成功返回 `Error::None`；溢出或语法不合法时返回对应错误 / Returns `Error::None`
 * on success, or the first overflow or syntax error
 */
template <typename UInt>
[[nodiscard]] consteval Error ParseUnsigned(std::string_view source, size_t& pos,
                                            UInt limit, UInt& value)
{
  static_assert(std::is_unsigned_v<UInt>);

  value = 0;
  if (pos >= source.size() || !IsDigit(source[pos]))
  {
    return Error::InvalidSpecifier;
  }

  while (pos < source.size() && IsDigit(source[pos]))
  {
    auto digit = static_cast<UInt>(source[pos] - '0');
    if (value > static_cast<UInt>((limit - digit) / 10))
    {
      return Error::NumberOverflow;
    }
    value = static_cast<UInt>(value * 10 + digit);
    ++pos;
  }

  return Error::None;
}

/**
 * @brief brace 字段自动索引与手动索引的源级状态 / Source-level indexing mode for
 * automatic versus manual brace fields
 */
struct IndexingState
{
  bool uses_manual = false;    ///< 正在使用手动编号 / manual field numbering is in use
  bool uses_auto = false;      ///< 正在使用自动编号 / automatic field numbering is in use
  size_t next_auto_index = 0;  ///< 下一个自动参数索引 / next automatic argument index
};

/**
 * @brief 解析 `:` 或 `}` 之前的字段头，包括自动或手动参数索引选择 / Parse the field head
 * before `:` or `}`, including automatic or manual argument index selection
 * @param source 源字符串 / Source string
 * @param pos 当前解析位置，成功时推进到字段头之后 / Current parse position; advanced past
 * the field head on success
 * @param indexing 自动或手动索引状态 / Auto or manual indexing state
 * @param field 输出字段头结果 / Parsed field-head output
 * @return 成功返回 `Error::None`；语法或索引模式不合法时返回对应错误 / Returns
 * `Error::None` on success, or the first syntax or indexing error
 */
[[nodiscard]] consteval Error ParseFieldHead(std::string_view source, size_t& pos,
                                             IndexingState& indexing, ParsedField& field)
{
  if (pos >= source.size())
  {
    return Error::UnexpectedEnd;
  }

  if (source[pos] == ':' || source[pos] == '}')
  {
    if (indexing.uses_manual)
    {
      return Error::MixedIndexing;
    }

    indexing.uses_auto = true;
    field.arg_index = indexing.next_auto_index++;
    return Error::None;
  }

  if (!IsDigit(source[pos]))
  {
    return Error::InvalidArgumentIndex;
  }

  size_t index = 0;
  auto error = ParseUnsigned(source, pos, std::numeric_limits<size_t>::max(), index);
  if (error != Error::None)
  {
    return error;
  }

  if (indexing.uses_auto)
  {
    return Error::MixedIndexing;
  }

  if (!Config::enable_explicit_argument_indexing)
  {
    return Error::ManualIndexingDisabled;
  }

  indexing.uses_manual = true;
  field.arg_index = index;
  return Error::None;
}

/**
 * @brief 解析一个 brace 字段中 `:` 之后的可选 format-spec 部分 / Parse the optional
 * format-spec portion after `:` inside one brace field
 * @param source 源字符串 / Source string
 * @param pos 当前解析位置，成功时推进到 format-spec 之后 / Current parse position;
 * advanced past the format-spec on success
 * @param field 待填充的字段结果 / Parsed field output being filled
 * @return 成功返回 `Error::None`；format-spec 非法时返回对应错误 / Returns `Error::None`
 * on success, or the first format-spec error
 */
[[nodiscard]] consteval Error ParseFormatSpec(std::string_view source, size_t& pos,
                                              ParsedField& field)
{
  if (pos >= source.size())
  {
    return Error::UnexpectedEnd;
  }

  if (pos + 1 < source.size() && IsAlign(source[pos + 1]))
  {
    if (source[pos] == '{' || source[pos] == '}')
    {
      return Error::InvalidSpecifier;
    }
    field.fill = source[pos];
    field.align = ParseAlign(source[pos + 1]);
    pos += 2;
  }
  else if (IsAlign(source[pos]))
  {
    field.align = ParseAlign(source[pos]);
    ++pos;
  }

  if (pos < source.size() &&
      (source[pos] == '+' || source[pos] == '-' || source[pos] == ' '))
  {
    if (source[pos] == '+')
    {
      field.force_sign = true;
    }
    else if (source[pos] == ' ')
    {
      field.space_sign = true;
    }
    // '-' is the default sign mode (negative values only); accepted for
    // fmt/std::format compatibility and produces the same output as omitting
    // the sign option entirely.
    ++pos;
  }

  if (pos < source.size() && source[pos] == '#')
  {
    if (!Config::enable_alternate)
    {
      return Error::InvalidSpecifier;
    }
    field.alternate = true;
    ++pos;
  }

  if (pos < source.size() && source[pos] == '0')
  {
    field.zero_pad = true;
    ++pos;
  }

  if (pos < source.size() && source[pos] == '{')
  {
    return Error::DynamicField;
  }

  if (pos < source.size() && IsDigit(source[pos]))
  {
    if (!Config::enable_width)
    {
      return Error::InvalidSpecifier;
    }
    auto error =
        ParseUnsigned(source, pos, std::numeric_limits<uint8_t>::max(), field.width);
    if (error != Error::None)
    {
      return error;
    }
  }

  if (pos < source.size() && source[pos] == '.')
  {
    if (!Config::enable_precision)
    {
      return Error::InvalidSpecifier;
    }

    ++pos;
    if (pos < source.size() && source[pos] == '{')
    {
      return Error::DynamicField;
    }

    field.has_precision = true;
    auto error = ParseUnsigned(
        source, pos, static_cast<uint8_t>(std::numeric_limits<uint8_t>::max() - 1),
        field.precision);
    if (error != Error::None)
    {
      return error;
    }
  }

  if (pos < source.size() && source[pos] != '}')
  {
    field.presentation = source[pos++];
    if (!IsSupportedPresentation(field.presentation))
    {
      return Error::InvalidPresentation;
    }
  }

  return Error::None;
}

/**
 * @brief 解析一个完整 brace 字段，从 `{` 一直到匹配的 `}` / Parse one complete brace
 * field from `{` through the matching `}`
 * @param source 源字符串 / Source string
 * @param pos 当前解析位置，进入时应指向 `{`，成功时推进到 `}` 之后 / Current parse
 * position; must point at `{` on entry and lands after `}` on success
 * @param indexing 自动或手动索引状态 / Auto or manual indexing state
 * @param field 输出字段结果 / Parsed field output
 * @return 成功返回 `Error::None`；字段语法不合法时返回对应错误 / Returns `Error::None` on
 * success, or the first field-syntax error
 */
[[nodiscard]] consteval Error ParseField(std::string_view source, size_t& pos,
                                         IndexingState& indexing, ParsedField& field)
{
  ++pos;
  if (pos >= source.size())
  {
    return Error::UnexpectedEnd;
  }

  auto error = ParseFieldHead(source, pos, indexing, field);
  if (error != Error::None)
  {
    return error;
  }

  if (pos < source.size() && source[pos] != ':' && source[pos] != '}')
  {
    return Error::InvalidArgumentIndex;
  }

  if (pos < source.size() && source[pos] == ':')
  {
    ++pos;
    error = ParseFormatSpec(source, pos, field);
    if (error != Error::None)
    {
      return error;
    }
  }

  if (pos >= source.size())
  {
    return Error::UnexpectedEnd;
  }
  if (source[pos] != '}')
  {
    return Error::InvalidSpecifier;
  }

  ++pos;
  return Error::None;
}
