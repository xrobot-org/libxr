#pragma once

/**
 * @brief 降为运行时类型之前，仅供前端使用的语义转换族 / Frontend-only semantic conversion
 * families before runtime lowering
 *
 * 这一层刻意比 FormatType 更窄，只描述源级 printf 说明符本身的语义，不提前决定
 * 目标相关的整数宽度、浮点存储宽度以及参数打包方式。
 * This layer is intentionally narrower than FormatType. It only describes what
 * the source-level printf token means before target-dependent width selection,
 * float storage selection, and argument packing are decided.
 */
enum class ValueKind : uint8_t
{
  None,             ///< 无效或尚未解析的说明符 / invalid or unresolved specifier
  Signed,           ///< %d / %i 族 / %d / %i family
  Unsigned,         ///< %u 族 / %u family
  Binary,           ///< %b / %B 族 / %b / %B family
  Octal,            ///< %o 族 / %o family
  HexLower,         ///< %x 族 / %x family
  HexUpper,         ///< %X 族 / %X family
  Pointer,          ///< %p
  Character,        ///< %c
  String,           ///< %s
  FloatFixed,       ///< %f / %F
  FloatScientific,  ///< %e / %E
  FloatGeneral,     ///< %g / %G
};

/**
 * @brief 单个 printf 转换在降为共享格式前的解析结果 / One parsed printf conversion before
 * lowering into the shared format
 */
struct Conversion
{
  size_t arg_index =
      0;  ///< 当前字段消耗的源参数索引 / source argument index consumed by this field
  ValueKind type =
      ValueKind::None;  ///< 转换项归一化后的语义类别 / semantic conversion category
  Length length = Length::Default;  ///< 已解析的长度修饰符 / parsed length modifier
  bool left_align = false;          ///< 已解析的 - 标志 / parsed - flag
  bool force_sign = false;          ///< 已解析的 + 标志 / parsed + flag
  bool space_sign = false;          ///< 已解析的空格正号标志 / parsed space-sign flag
  bool alternate = false;           ///< 已解析的 # 标志 / parsed # flag
  bool zero_pad = false;            ///< 已解析的 0 标志 / parsed 0 flag
  bool upper_case = false;          ///< 隐含的大写输出标志 / implied uppercase output
  bool positional =
      false;  ///< arg_index 是否来自 n$ 语法 / whether arg_index came from n$ syntax
  uint8_t width = 0;  ///< 已解析的字段宽度 / parsed field width
  bool has_precision =
      false;  ///< 是否显式提供了精度 / whether precision was explicitly provided
  uint8_t precision = 0;  ///< 已解析的精度值 / parsed precision value

  /**
   * @brief 将前端解析出的标志位打包成共享的 `FormatFlag` 字节 / Pack the parsed frontend
   * flags into the shared `FormatFlag` byte
   * @return 返回打包后的共享 `FormatFlag` 字节 / Returns the packed shared `FormatFlag`
   * byte
   */
  [[nodiscard]] constexpr uint8_t FlagsByte() const
  {
    uint8_t flags = 0;
    if (left_align)
    {
      flags |= static_cast<uint8_t>(FormatFlag::LeftAlign);
    }
    if (force_sign)
    {
      flags |= static_cast<uint8_t>(FormatFlag::ForceSign);
    }
    if (space_sign)
    {
      flags |= static_cast<uint8_t>(FormatFlag::SpaceSign);
    }
    if (alternate)
    {
      flags |= static_cast<uint8_t>(FormatFlag::Alternate);
    }
    if (zero_pad)
    {
      flags |= static_cast<uint8_t>(FormatFlag::ZeroPad);
    }
    if (upper_case)
    {
      flags |= static_cast<uint8_t>(FormatFlag::UpperCase);
    }
    return flags;
  }

  /**
   * @brief 返回共享精度字节；若未显式指定精度则为 `0xFF` / Return the shared precision
   * byte, or `0xFF` when precision is absent
   * @return 返回运行期协议期望的共享精度字节 / Returns the shared precision byte expected
   * by the runtime protocol
   */
  [[nodiscard]] constexpr uint8_t PrecisionByte() const
  {
    return has_precision ? precision : std::numeric_limits<uint8_t>::max();
  }
};

/**
 * @brief printf 转换说明符描述表使用的功能开关类别 / Feature-switch families referenced
 * by printf conversion descriptors
 */
enum class FeatureGate : uint8_t
{
  Integer,          ///< %d / %i / %u 整数族 / %d / %i / %u family
  IntegerBase8_16,  ///< %o / %x / %X
  Pointer,          ///< %p
  Text,             ///< %c / %s
  FloatFixed,       ///< %f / %F
  FloatScientific,  ///< %e / %E
  FloatGeneral,     ///< %g / %G
};

/**
 * @brief 单个 printf 转换说明符允许的长度修饰类别 / Length families accepted by one
 * printf conversion descriptor
 */
enum class LengthPolicy : uint8_t
{
  Integer,   ///< 接受整数长度修饰，但不接受 L / integer lengths except L
  NoneOnly,  ///< 只接受默认无长度修饰 / only the default no-length form
  Float,     ///< 只接受默认或 L / default or L
};

/**
 * @brief 单个源级 printf 说明符描述项 / One source-level printf specifier descriptor
 */
struct SpecifierDescriptor
{
  char token = 0;  ///< 源格式转换字符 / source conversion character
  ValueKind type =
      ValueKind::None;  ///< 归一化后的语义类别 / normalized semantic category
  FeatureGate gate = FeatureGate::Integer;  ///< 控制该说明符的功能开关
                                            ///< feature gate controlling this specifier
  LengthPolicy length_policy =
      LengthPolicy::NoneOnly;  ///< 允许的长度修饰类别 / accepted length family
  bool upper_case =
      false;  ///< 说明符是否隐含大写输出 / whether the specifier implies uppercase output
};

/**
 * @brief 归一化 printf 长度表的宽度 / Normalized printf length-table width
 */
inline constexpr size_t length_rule_count = static_cast<size_t>(Length::LongDouble) + 1;

/**
 * @brief 按长度索引的有符号参数规则表 / Per-length signed argument-rule table
 */
inline constexpr std::array<FormatArgumentRule, length_rule_count> signed_rules{
    FormatArgumentRule::SignedAny,      FormatArgumentRule::SignedChar,
    FormatArgumentRule::SignedShort,    FormatArgumentRule::SignedLong,
    FormatArgumentRule::SignedLongLong, FormatArgumentRule::SignedIntMax,
    FormatArgumentRule::SignedSize,     FormatArgumentRule::SignedPtrDiff,
    FormatArgumentRule::None,
};

/**
 * @brief 按归一化 printf 长度索引的无符号参数规则表 / Per-length unsigned argument-rule
 * table indexed by normalized printf length
 */
inline constexpr std::array<FormatArgumentRule, length_rule_count> unsigned_rules{
    FormatArgumentRule::UnsignedAny,
    FormatArgumentRule::UnsignedChar,
    FormatArgumentRule::UnsignedShort,
    FormatArgumentRule::UnsignedLong,
    FormatArgumentRule::UnsignedLongLong,
    FormatArgumentRule::UnsignedIntMax,
    FormatArgumentRule::UnsignedSize,
    FormatArgumentRule::UnsignedPtrDiff,
    FormatArgumentRule::None,
};

/**
 * @brief 源级 printf 说明符描述表 / Source-level printf specifier descriptor table
 */
inline constexpr std::array<SpecifierDescriptor, 17> specifiers{{
    {'d', ValueKind::Signed, FeatureGate::Integer, LengthPolicy::Integer, false},
    {'i', ValueKind::Signed, FeatureGate::Integer, LengthPolicy::Integer, false},
    {'u', ValueKind::Unsigned, FeatureGate::Integer, LengthPolicy::Integer, false},
    {'b', ValueKind::Binary, FeatureGate::IntegerBase8_16, LengthPolicy::Integer, false},
    {'B', ValueKind::Binary, FeatureGate::IntegerBase8_16, LengthPolicy::Integer, true},
    {'o', ValueKind::Octal, FeatureGate::IntegerBase8_16, LengthPolicy::Integer, false},
    {'x', ValueKind::HexLower, FeatureGate::IntegerBase8_16, LengthPolicy::Integer,
     false},
    {'X', ValueKind::HexUpper, FeatureGate::IntegerBase8_16, LengthPolicy::Integer,
     false},
    {'p', ValueKind::Pointer, FeatureGate::Pointer, LengthPolicy::NoneOnly, false},
    {'c', ValueKind::Character, FeatureGate::Text, LengthPolicy::NoneOnly, false},
    {'s', ValueKind::String, FeatureGate::Text, LengthPolicy::NoneOnly, false},
    {'f', ValueKind::FloatFixed, FeatureGate::FloatFixed, LengthPolicy::Float, false},
    {'F', ValueKind::FloatFixed, FeatureGate::FloatFixed, LengthPolicy::Float, true},
    {'e', ValueKind::FloatScientific, FeatureGate::FloatScientific, LengthPolicy::Float,
     false},
    {'E', ValueKind::FloatScientific, FeatureGate::FloatScientific, LengthPolicy::Float,
     true},
    {'g', ValueKind::FloatGeneral, FeatureGate::FloatGeneral, LengthPolicy::Float, false},
    {'G', ValueKind::FloatGeneral, FeatureGate::FloatGeneral, LengthPolicy::Float, true},
}};
