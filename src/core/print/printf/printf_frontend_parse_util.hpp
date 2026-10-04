#pragma once

/**
 * @brief 源级位置参数或顺序参数索引状态 / Source-level positional or sequential indexing
 * state
 */
struct IndexingState
{
  bool uses_positional =
      false;  ///< 至少有一个转换使用了 n$ 语法 / at least one conversion used n$ syntax
  bool uses_sequential = false;  ///< 至少有一个转换使用了隐式顺序参数
                                 ///< at least one conversion used implicit sequential
                                 ///< order
  size_t next_index = 0;         ///< 下一个顺序参数索引 / next sequential argument index
};

/**
 * @brief 判断一个源字节是否为 ASCII 十进制数字 / Return whether one source byte is an
 * ASCII decimal digit
 * @param ch 待检查的源字节 / Source byte to test
 * @return 若是 ASCII 十进制数字则返回 `true`，否则返回 `false` / Returns `true` for an
 * ASCII decimal digit, otherwise `false`
 */
[[nodiscard]] constexpr bool IsDigit(char ch) { return ch >= '0' && ch <= '9'; }

/**
 * @brief 判断源字符串在结尾终止字节之前是否包含嵌入式 NUL / Return whether the source
 * contains an embedded NUL byte before the terminator
 * @param source 待检查的源字符串 / Source string to inspect
 * @return 若终止前包含嵌入式 NUL 则返回 `true`，否则返回 `false` / Returns `true` when an
 * embedded NUL exists before the terminator, otherwise `false`
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
 * @brief 解析可选的前导 n$ 位置参数选择器 / Parse the optional leading n$ positional
 * argument selector
 *
 * 只有当数字后面紧跟 '$' 时，这个探测才会真正消费它们；像 %05d 这样的普通宽度
 * 数字会完整保留给后续宽度解析阶段。
 * This probe only consumes digits when they are immediately followed by '$'.
 * Plain width digits such as %05d stay untouched for the later width parser.
 * @param source 完整 printf 源字符串 / Full printf source string
 * @param pos 当前解析位置，成功时推进到 `n$` 之后 / Current parse position; advanced past
 * `n$` on success
 * @param indexing 源级索引模式跟踪状态 / Source-level indexing mode tracker
 * @param conversion 当前正在填充的转换项 / Conversion being filled
 * @return 解析成功或本段没有位置参数选择器时返回 `Error::None`；出错时返回首个解析错误 /
 * Returns the first parse error, or `Error::None` when no positional selector is present
 * or parsing succeeds
 */
[[nodiscard]] consteval Error ParseArgumentIndex(std::string_view source, size_t& pos,
                                                 IndexingState& indexing,
                                                 Conversion& conversion)
{
  if (pos >= source.size() || !IsDigit(source[pos]))
  {
    return Error::None;
  }

  size_t probe = pos;
  size_t index = 0;
  while (probe < source.size() && IsDigit(source[probe]))
  {
    auto digit = static_cast<size_t>(source[probe] - '0');
    if (index > (std::numeric_limits<size_t>::max() - digit) / 10)
    {
      return Error::NumberOverflow;
    }

    index = index * 10 + digit;
    ++probe;
  }

  if (probe >= source.size() || source[probe] != '$')
  {
    return Error::None;
  }

  if (!Config::enable_explicit_argument_indexing)
  {
    return Error::PositionalArgumentDisabled;
  }

  if (index == 0)
  {
    return Error::InvalidArgumentIndex;
  }
  if (indexing.uses_sequential)
  {
    return Error::MixedIndexing;
  }

  indexing.uses_positional = true;
  conversion.positional = true;
  conversion.arg_index = index - 1;
  pos = probe + 1;
  return Error::None;
}

/**
 * @brief 解析一个目标为字节宽度的十进制整数字段片段，并检查是否溢出 / Parse one decimal
 * byte-sized integer fragment with overflow checking
 * @param source 含该十进制片段的源字符串 / Source string holding the decimal fragment
 * @param pos 当前解析位置，成功时推进到片段之后 / Current parse position; advanced past
 * the fragment on success
 * @param limit 该字段允许的最大值 / Inclusive upper bound accepted by this field
 * @param value 输出解析结果 / Parsed result output
 * @return 成功返回 `Error::None`；溢出或语法非法时返回对应错误 / Returns `Error::None` on
 * success, or the first overflow or syntax failure
 */
[[nodiscard]] consteval Error ParseByte(std::string_view source, size_t& pos,
                                        uint8_t limit, uint8_t& value)
{
  value = 0;
  if (pos >= source.size() || !IsDigit(source[pos]))
  {
    return Error::None;
  }

  while (pos < source.size() && IsDigit(source[pos]))
  {
    auto digit = static_cast<uint8_t>(source[pos] - '0');
    if (value > static_cast<uint8_t>((limit - digit) / 10))
    {
      return Error::NumberOverflow;
    }

    value = static_cast<uint8_t>(value * 10 + digit);
    ++pos;
  }

  return Error::None;
}
