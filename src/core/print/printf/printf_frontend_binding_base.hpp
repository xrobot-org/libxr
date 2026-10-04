#pragma once

/**
 * @brief printf 降级阶段共享的描述表与策略辅助函数 / Shared descriptor-table and policy
 * helpers for printf lowering
 */
namespace SourceSyntax
{
namespace FieldSelection
{

/**
 * @brief 将一个归一化长度标签转换为描述表索引 / Converts one normalized length tag to its
 * descriptor-table index.
 * @param length 归一化后的 printf 长度标签 / Normalized printf length tag.
 * @return 返回该长度标签对应的描述表索引。
 *         Returns the descriptor-table index for that length.
 */
[[nodiscard]] constexpr size_t LengthIndex(Length length)
{
  return static_cast<size_t>(length);
}

/**
 * @brief 判断某个前端语义族是否应使用无符号存储与匹配规则 / Returns whether one frontend
 * semantic family should use unsigned storage/rules.
 * @param type 前端语义族 / Frontend semantic family.
 * @return 应使用无符号存储与匹配规则时返回 `true`，否则返回 `false`。
 *         Returns `true` when unsigned storage/rules should be used, otherwise `false`.
 */
[[nodiscard]] constexpr bool IsUnsignedType(ValueKind type)
{
  switch (type)
  {
    case ValueKind::Unsigned:
    case ValueKind::Binary:
    case ValueKind::Octal:
    case ValueKind::HexLower:
    case ValueKind::HexUpper:
      return true;
    default:
      return false;
  }
}

/**
 * @brief 在描述表中查找一个源级转换字符 / Looks up one source conversion character in the
 * descriptor table.
 * @param token 源级转换字符 / Source conversion character.
 * @return 返回匹配的描述项；找不到时返回空描述项。
 *         Returns the matching descriptor, or an empty one when not found.
 */
[[nodiscard]] consteval SpecifierDescriptor LookupSpecifier(char token)
{
  for (const auto& descriptor : specifiers)
  {
    if (descriptor.token == token)
    {
      return descriptor;
    }
  }

  return {};
}

/**
 * @brief 判断某个说明符家族在当前功能开关下是否启用 / Return whether one specifier family
 * is enabled under the current feature gates
 * @param gate 待检查的功能开关类别 / Feature-switch family to test
 * @param length 已解析的归一化长度修饰符 / Parsed normalized length modifier
 * @return 该说明符家族在当前配置下启用时返回 `true`，否则返回 `false` / Returns `true`
 * when the selected family is enabled, otherwise `false`
 */
[[nodiscard]] constexpr bool TypeEnabled(FeatureGate gate, Length length)
{
  switch (gate)
  {
    case FeatureGate::Integer:
      return Config::enable_integer;
    case FeatureGate::IntegerBase8_16:
      return Config::enable_integer_base8_16;
    case FeatureGate::Pointer:
      return Config::enable_pointer;
    case FeatureGate::Text:
      return Config::enable_text;
    case FeatureGate::FloatFixed:
      return Config::enable_float_fixed &&
             (length != Length::LongDouble || Config::enable_float_long_double);
    case FeatureGate::FloatScientific:
      return Config::enable_float_scientific &&
             (length != Length::LongDouble || Config::enable_float_long_double);
    case FeatureGate::FloatGeneral:
      return Config::enable_float_general &&
             (length != Length::LongDouble || Config::enable_float_long_double);
  }

  return false;
}

/**
 * @brief 判断某个长度修饰符对当前说明符家族是否合法 / Returns whether one parsed length
 * modifier is legal for the selected specifier family.
 * @param policy 描述项选中的长度策略族 / Length-policy family selected by the descriptor.
 * @param length 已解析的归一化长度修饰符 / Parsed normalized length modifier.
 * @return 当前长度对该策略合法时返回 `true`，否则返回 `false`。
 *         Returns `true` when this length is legal for the selected policy, otherwise
 *         `false`.
 */
[[nodiscard]] constexpr bool LengthAllowed(LengthPolicy policy, Length length)
{
  switch (policy)
  {
    case LengthPolicy::Integer:
      return length != Length::LongDouble;
    case LengthPolicy::NoneOnly:
      return length == Length::Default;
    case LengthPolicy::Float:
      return length == Length::Default || length == Length::LongDouble;
  }

  return false;
}

/**
 * @brief 返回某个归一化长度索引选中的参数匹配规则 / Return the argument rule selected by
 * one normalized length index
 * @param rules 按长度索引的规则表 / Length-indexed rule table
 * @param length 已解析的归一化长度修饰符 / Parsed normalized length modifier
 * @return 返回选中的编译期参数匹配规则 / Returns the selected compile-time argument rule
 */
[[nodiscard]] constexpr FormatArgumentRule RuleFromTable(
    const std::array<FormatArgumentRule, length_rule_count>& rules, Length length)
{
  return rules[LengthIndex(length)];
}

/**
 * @brief 判断某个参数规则在当前目标上是否需要 64 位打包存储 / Returns whether one
 * argument rule requires 64-bit packed storage on this target.
 * @param rule 编译期参数匹配规则 / Compile-time argument rule.
 * @return 该规则在当前目标上需要 64 位打包存储时返回 `true`，否则返回 `false`。
 *         Returns `true` when this rule requires 64-bit packed storage on the current
 *         target, otherwise `false`.
 */
[[nodiscard]] consteval bool RuleUses64BitStorage(FormatArgumentRule rule)
{
  switch (rule)
  {
    case FormatArgumentRule::SignedLong:
      return sizeof(long) > sizeof(int32_t);
    case FormatArgumentRule::SignedLongLong:
      return sizeof(long long) > sizeof(int32_t);
    case FormatArgumentRule::SignedIntMax:
      return sizeof(intmax_t) > sizeof(int32_t);
    case FormatArgumentRule::SignedSize:
      return sizeof(std::make_signed_t<size_t>) > sizeof(int32_t);
    case FormatArgumentRule::SignedPtrDiff:
      return sizeof(ptrdiff_t) > sizeof(int32_t);
    case FormatArgumentRule::UnsignedLong:
      return sizeof(unsigned long) > sizeof(uint32_t);
    case FormatArgumentRule::UnsignedLongLong:
      return sizeof(unsigned long long) > sizeof(uint32_t);
    case FormatArgumentRule::UnsignedIntMax:
      return sizeof(uintmax_t) > sizeof(uint32_t);
    case FormatArgumentRule::UnsignedSize:
      return sizeof(size_t) > sizeof(uint32_t);
    case FormatArgumentRule::UnsignedPtrDiff:
      return sizeof(std::make_unsigned_t<ptrdiff_t>) > sizeof(uint32_t);
    default:
      return false;
  }
}

/**
 * @brief 判断普通 printf 浮点转换是否降为 double 存储 / Returns whether ordinary printf
 * float conversions lower to double storage.
 * @return 普通 printf 浮点转换降为 double 存储时返回 `true`，否则返回 `false`。
 *         Returns `true` when ordinary printf float conversions lower to double-backed
 *         storage, otherwise `false`.
 */
[[nodiscard]] constexpr bool UsesDoubleFloatStorage()
{
  return Config::enable_float_double;
}
}  // namespace FieldSelection
}  // namespace SourceSyntax
