#pragma once

/**
 * @brief 为一个已解析 printf 转换选择运行期语义类型 / Choose the runtime semantic type
 * for one parsed printf conversion
 */
namespace SourceSyntax
{
namespace FieldSelection
{
/**
 * @brief 在已知参数匹配规则后，为一个已解析转换选择运行期语义类型 / Choose the runtime
 * semantic type for one parsed conversion after its argument rule is known
 * @param conversion 已解析的 printf 转换项 / Parsed printf conversion
 * @param rule 该转换项对应的编译期参数匹配规则 / Compile-time argument rule selected for
 * that conversion
 * @return 返回该转换项最终选中的运行期语义类型 / Returns the runtime semantic type
 * selected for this conversion
 */
[[nodiscard]] consteval FormatType SelectFormatType(const Conversion& conversion,
                                                    FormatArgumentRule rule)
{
  if (conversion.length == Length::LongDouble)
  {
    if (conversion.type == ValueKind::FloatFixed)
    {
      return FormatType::LongDoubleFixed;
    }
    if (conversion.type == ValueKind::FloatScientific)
    {
      return FormatType::LongDoubleScientific;
    }
    if (conversion.type == ValueKind::FloatGeneral)
    {
      return FormatType::LongDoubleGeneral;
    }
  }

  if (conversion.type == ValueKind::Signed)
  {
    return RuleUses64BitStorage(rule) ? FormatType::Signed64 : FormatType::Signed32;
  }
  if (conversion.type == ValueKind::Unsigned)
  {
    return RuleUses64BitStorage(rule) ? FormatType::Unsigned64 : FormatType::Unsigned32;
  }
  if (conversion.type == ValueKind::Binary)
  {
    return RuleUses64BitStorage(rule) ? FormatType::Binary64 : FormatType::Binary32;
  }
  if (conversion.type == ValueKind::Octal)
  {
    return RuleUses64BitStorage(rule) ? FormatType::Octal64 : FormatType::Octal32;
  }
  if (conversion.type == ValueKind::HexLower)
  {
    return RuleUses64BitStorage(rule) ? FormatType::HexLower64 : FormatType::HexLower32;
  }
  if (conversion.type == ValueKind::HexUpper)
  {
    return RuleUses64BitStorage(rule) ? FormatType::HexUpper64 : FormatType::HexUpper32;
  }

  switch (conversion.type)
  {
    case ValueKind::Pointer:
      return FormatType::Pointer;
    case ValueKind::Character:
      return FormatType::Character;
    case ValueKind::String:
      return FormatType::String;
    case ValueKind::FloatFixed:
      return UsesDoubleFloatStorage() ? FormatType::DoubleFixed : FormatType::FloatFixed;
    case ValueKind::FloatScientific:
      return UsesDoubleFloatStorage() ? FormatType::DoubleScientific
                                      : FormatType::FloatScientific;
    case ValueKind::FloatGeneral:
      return UsesDoubleFloatStorage() ? FormatType::DoubleGeneral
                                      : FormatType::FloatGeneral;
    case ValueKind::None:
    case ValueKind::Signed:
    case ValueKind::Unsigned:
    case ValueKind::Binary:
    case ValueKind::Octal:
    case ValueKind::HexLower:
    case ValueKind::HexUpper:
      return FormatType::End;
  }

  return FormatType::End;
}

/**
 * @brief 为一个运行期语义类型选择参数打包存储类别 / Chooses the packed storage kind for
 * one runtime semantic type.
 * @param type 运行期语义类型 / Runtime semantic type.
 * @return 返回该运行期类型使用的参数打包存储类别。
 *         Returns the packed storage kind used by that runtime type.
 */
[[nodiscard]] consteval FormatPackKind SelectPackKind(FormatType type)
{
  switch (type)
  {
    case FormatType::Signed32:
      return FormatPackKind::I32;
    case FormatType::Signed64:
      return FormatPackKind::I64;
    case FormatType::Unsigned32:
    case FormatType::Binary32:
    case FormatType::Octal32:
    case FormatType::HexLower32:
    case FormatType::HexUpper32:
      return FormatPackKind::U32;
    case FormatType::Unsigned64:
    case FormatType::Binary64:
    case FormatType::Octal64:
    case FormatType::HexLower64:
    case FormatType::HexUpper64:
      return FormatPackKind::U64;
    case FormatType::Pointer:
      return FormatPackKind::Pointer;
    case FormatType::Character:
      return FormatPackKind::Character;
    case FormatType::String:
      return FormatPackKind::StringView;
    case FormatType::FloatFixed:
    case FormatType::FloatScientific:
    case FormatType::FloatGeneral:
      return FormatPackKind::F32;
    case FormatType::DoubleFixed:
    case FormatType::DoubleScientific:
    case FormatType::DoubleGeneral:
      return FormatPackKind::F64;
    case FormatType::LongDoubleFixed:
    case FormatType::LongDoubleScientific:
    case FormatType::LongDoubleGeneral:
      return FormatPackKind::LongDouble;
    case FormatType::End:
    case FormatType::TextInline:
    case FormatType::TextRef:
    case FormatType::TextSpace:
      DEV_ASSERT(false);
      return FormatPackKind::U32;
  }

  DEV_ASSERT(false);
  return FormatPackKind::U32;
}

/**
 * @brief 为一个已解析转换选择它消耗的编译期参数匹配规则 / Chooses which compile-time
 * argument rule one parsed conversion consumes.
 * @param conversion 已解析的 printf 转换项 / Parsed printf conversion.
 * @return 返回该转换项消耗的编译期参数匹配规则。
 *         Returns the compile-time argument rule consumed by this conversion.
 */
[[nodiscard]] consteval FormatArgumentRule SelectArgumentRule(
    const Conversion& conversion)
{
  if (conversion.type == ValueKind::Signed)
  {
    return RuleFromTable(signed_rules, conversion.length);
  }

  if (IsUnsignedType(conversion.type))
  {
    return RuleFromTable(unsigned_rules, conversion.length);
  }

  switch (conversion.type)
  {
    case ValueKind::Pointer:
      return FormatArgumentRule::Pointer;
    case ValueKind::Character:
      return FormatArgumentRule::Character;
    case ValueKind::String:
      return FormatArgumentRule::String;
    case ValueKind::FloatFixed:
    case ValueKind::FloatScientific:
    case ValueKind::FloatGeneral:
      return (conversion.length == Length::LongDouble) ? FormatArgumentRule::LongDouble
                                                       : FormatArgumentRule::Float;
    case ValueKind::None:
    case ValueKind::Signed:
    case ValueKind::Unsigned:
    case ValueKind::Binary:
    case ValueKind::Octal:
    case ValueKind::HexLower:
    case ValueKind::HexUpper:
      break;
  }

  return FormatArgumentRule::None;
}

/**
 * @brief 在解析后校验与目标相关的格式选择约束 / Validates target-dependent
 * format-selection constraints after parsing.
 * @param conversion 已解析的 printf 转换项 / Parsed printf conversion.
 * @return 当前目标与配置允许该转换时返回 `Error::None`；否则返回首个目标相关错误。
 *         Returns `Error::None` when the conversion is legal on the current
 *         target/profile, otherwise the first target-dependent error.
 */
[[nodiscard]] consteval Error ValidateConversion(const Conversion& conversion)
{
  if ((conversion.type == ValueKind::FloatFixed ||
       conversion.type == ValueKind::FloatScientific ||
       conversion.type == ValueKind::FloatGeneral) &&
      conversion.has_precision && conversion.precision > Config::max_float_precision)
  {
    return Error::FloatPrecisionLimitExceeded;
  }

  if ((conversion.type == ValueKind::Signed || IsUnsignedType(conversion.type)) &&
      RuleUses64BitStorage(SelectArgumentRule(conversion)) &&
      !Config::enable_integer_64bit)
  {
    return Error::InvalidLength;
  }

  return Error::None;
}

/**
 * @brief 为一个已解析 printf 转换构造共享 FormatField 记录 / Builds the shared
 * FormatField record for one parsed printf conversion.
 * @param conversion 已解析的 printf 转换项 / Parsed printf conversion.
 * @return 返回共享编译后端要消费的 `FormatField` 记录。
 *         Returns the shared `FormatField` record consumed by the compile-time backend.
 */
[[nodiscard]] consteval FormatField BuildFormatField(const Conversion& conversion)
{
  auto rule = SelectArgumentRule(conversion);
  auto type = SelectFormatType(conversion, rule);
  return FormatField{
      .type = type,
      .pack = SelectPackKind(type),
      .rule = rule,
      .flags = conversion.FlagsByte(),
      .width = conversion.width,
      .precision = conversion.PrecisionByte(),
  };
}
}  // namespace FieldSelection
}  // namespace SourceSyntax
