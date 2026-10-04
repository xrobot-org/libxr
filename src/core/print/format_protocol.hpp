#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string_view>
#include <type_traits>

/**
 * @brief 供直接包含头文件的用户使用的打印功能默认值 / Print feature defaults for direct
 * header consumers.
 *
 * CMake 会用同名 0 或 1 编译定义导出这些开关；不经过 CMake 直接使用头文件时，
 * 这里的回退值与常规嵌入式默认配置一致：保留十进制整数、文本、定点浮点、
 * 8/16 进制整数、宽度与精度，默认关闭备用格式（#）、指针、显式参数索引、
 * 64 位整数、double、long double、科学计数法与通用浮点格式。
 * CMake exports the same names as 0 or 1 target compile definitions. When the
 * headers are used without CMake, these fallback values match the ordinary
 * embedded default profile: decimal integers, text, fixed float, base 8/16
 * integers, width, and precision stay enabled, while alternate form (#),
 * pointers, explicit argument indexing, 64-bit integer, double, long double,
 * scientific, and general float formatting stay disabled.
 */
#ifndef LIBXR_PRINT_ENABLE_INTEGER
#define LIBXR_PRINT_ENABLE_INTEGER 1
#endif

#ifndef LIBXR_PRINT_ENABLE_TEXT
#define LIBXR_PRINT_ENABLE_TEXT 1
#endif

#ifndef LIBXR_PRINT_ENABLE_POINTER
#define LIBXR_PRINT_ENABLE_POINTER 0
#endif

#ifndef LIBXR_PRINT_ENABLE_FLOAT
#define LIBXR_PRINT_ENABLE_FLOAT 1
#endif

#ifndef LIBXR_PRINT_INTEGER_ENABLE_BASE8_16
#define LIBXR_PRINT_INTEGER_ENABLE_BASE8_16 1
#endif

#ifndef LIBXR_PRINT_INTEGER_ENABLE_64BIT
#define LIBXR_PRINT_INTEGER_ENABLE_64BIT 0
#endif

#ifndef LIBXR_PRINT_FLOAT_ENABLE_FIXED
#define LIBXR_PRINT_FLOAT_ENABLE_FIXED 1
#endif

#ifndef LIBXR_PRINT_FLOAT_ENABLE_DOUBLE
#define LIBXR_PRINT_FLOAT_ENABLE_DOUBLE 0
#endif

#ifndef LIBXR_PRINT_FLOAT_ENABLE_SCIENTIFIC
#define LIBXR_PRINT_FLOAT_ENABLE_SCIENTIFIC 0
#endif

#ifndef LIBXR_PRINT_FLOAT_ENABLE_GENERAL
#define LIBXR_PRINT_FLOAT_ENABLE_GENERAL 0
#endif

#ifndef LIBXR_PRINT_FLOAT_ENABLE_LONG_DOUBLE
#define LIBXR_PRINT_FLOAT_ENABLE_LONG_DOUBLE 0
#endif

#ifndef LIBXR_PRINT_FLOAT_MAX_PRECISION
#define LIBXR_PRINT_FLOAT_MAX_PRECISION 32
#endif

#ifndef LIBXR_PRINT_FLOAT_MAX_INTEGER_DIGITS
#define LIBXR_PRINT_FLOAT_MAX_INTEGER_DIGITS 32
#endif

#ifndef LIBXR_PRINT_ENABLE_WIDTH
#define LIBXR_PRINT_ENABLE_WIDTH 1
#endif

#ifndef LIBXR_PRINT_ENABLE_PRECISION
#define LIBXR_PRINT_ENABLE_PRECISION 1
#endif

#ifndef LIBXR_PRINT_ENABLE_ALTERNATE
#define LIBXR_PRINT_ENABLE_ALTERNATE 0
#endif

#ifndef LIBXR_PRINT_ENABLE_EXPLICIT_ARGUMENT_INDEXING
#define LIBXR_PRINT_ENABLE_EXPLICIT_ARGUMENT_INDEXING 0
#endif

namespace LibXR::Print::Config
{
/**
 * @brief 启用有符号与无符号十进制整数转换 / Enables signed and unsigned decimal integer
 * conversions.
 */
inline constexpr bool enable_integer = LIBXR_PRINT_ENABLE_INTEGER;
/**
 * @brief 启用字符与字符串转换 / Enables character and string conversions.
 */
inline constexpr bool enable_text = LIBXR_PRINT_ENABLE_TEXT;
/**
 * @brief 启用指针转换 / Enables pointer conversions.
 */
inline constexpr bool enable_pointer = LIBXR_PRINT_ENABLE_POINTER;
/**
 * @brief 所有浮点转换的总开关 / Master switch for all floating-point conversions.
 */
inline constexpr bool enable_float = LIBXR_PRINT_ENABLE_FLOAT;

/**
 * @brief 在整数功能开启时启用二进制、八进制和十六进制转换 / Enables binary, octal, and
 * hexadecimal integer conversions when integers are enabled.
 */
inline constexpr bool enable_integer_base8_16 =
    enable_integer && LIBXR_PRINT_INTEGER_ENABLE_BASE8_16;
/**
 * @brief 在整数功能开启时启用 64 位整数格式化族 / Enables 64-bit integer formatting
 * families when integers are enabled.
 */
inline constexpr bool enable_integer_64bit =
    enable_integer && LIBXR_PRINT_INTEGER_ENABLE_64BIT;

/**
 * @brief 在浮点功能开启时启用 %f / %F / Enables %f / %F when floating-point support is
 * enabled.
 */
inline constexpr bool enable_float_fixed = enable_float && LIBXR_PRINT_FLOAT_ENABLE_FIXED;
/**
 * @brief 在浮点功能开启时启用基于 double 的默认浮点格式化 / Enables double-backed default
 * float formatting when floating-point support is enabled.
 * @note 关闭时仍接受 double 输入，但会按 F32 降精度格式化 / When disabled, double input
 * remains accepted but is formatted at reduced F32 precision
 */
inline constexpr bool enable_float_double =
    enable_float && LIBXR_PRINT_FLOAT_ENABLE_DOUBLE;
/**
 * @brief 在浮点功能开启时启用 %e / %E / Enables %e / %E when floating-point support is
 * enabled.
 */
inline constexpr bool enable_float_scientific =
    enable_float && LIBXR_PRINT_FLOAT_ENABLE_SCIENTIFIC;
/**
 * @brief 在浮点功能开启时启用 %g / %G / Enables %g / %G when floating-point support is
 * enabled.
 */
inline constexpr bool enable_float_general =
    enable_float && LIBXR_PRINT_FLOAT_ENABLE_GENERAL;
/**
 * @brief 在浮点功能开启时启用 L 长度修饰 / Enables the L floating-point length modifier
 * when floating-point support is enabled.
 */
inline constexpr bool enable_float_long_double =
    enable_float_double && LIBXR_PRINT_FLOAT_ENABLE_LONG_DOUBLE;
static_assert(LIBXR_PRINT_FLOAT_MAX_PRECISION <=
                  static_cast<int>(std::numeric_limits<uint8_t>::max() - 1),
              "LibXR::Print: LIBXR_PRINT_FLOAT_MAX_PRECISION must fit in uint8_t "
              "and stay below the unspecified-precision sentinel");
static_assert(LIBXR_PRINT_FLOAT_MAX_INTEGER_DIGITS > 0,
              "LibXR::Print: LIBXR_PRINT_FLOAT_MAX_INTEGER_DIGITS must be positive");
/**
 * @brief 前端接受的浮点显式精度上限 / Maximum explicit float precision accepted by the
 * frontend.
 */
inline constexpr uint8_t max_float_precision =
    static_cast<uint8_t>(LIBXR_PRINT_FLOAT_MAX_PRECISION);
/**
 * @brief 定点浮点格式接受的整数位数上限 / Maximum integer-digit count accepted by fixed
 * float formatting.
 */
inline constexpr size_t max_float_integer_digits =
    static_cast<size_t>(LIBXR_PRINT_FLOAT_MAX_INTEGER_DIGITS);

/**
 * @brief 启用常量字段宽度解析 / Enables constant field width parsing.
 */
inline constexpr bool enable_width = LIBXR_PRINT_ENABLE_WIDTH;
/**
 * @brief 启用常量精度解析 / Enables constant precision parsing.
 */
inline constexpr bool enable_precision = LIBXR_PRINT_ENABLE_PRECISION;
/**
 * @brief 启用备用格式语法，例如用于整数前缀和浮点保留小数点的 # / Enables alternate-form
 * syntax such as # for integer prefixes and float decimal-point retention.
 */
inline constexpr bool enable_alternate = LIBXR_PRINT_ENABLE_ALTERNATE;
/**
 * @brief 启用源级显式参数索引，例如 printf 的 n$ 和 format 的 {1} / Enables source-level
 * explicit argument indexing such as printf n$ and format {1}.
 */
inline constexpr bool enable_explicit_argument_indexing =
    LIBXR_PRINT_ENABLE_EXPLICIT_ARGUMENT_INDEXING;
}  // namespace LibXR::Print::Config

namespace LibXR::Print
{
/**
 * @brief 编译期解析层与运行期写出层之间共用的打印格式协议 / Shared print-format
 * protocol used between compile-time parsing and runtime writing.
 *
 * 前端会先决定：
 * - 允许哪些 C++ 实参类型
 * - 每个实参在运行期该怎样打包
 * - 运行期最终该走哪条文本写出路径
 *
 * 运行期 writer 随后直接读取这套协议，不再重新解析原始格式串。
 * The frontends first decide:
 * - which C++ argument types are allowed
 * - how each accepted argument is packed for runtime use
 * - which text-writing path the runtime writer should take later
 *
 * The runtime writer then reads that protocol directly instead of reparsing the
 * original format string.
 */
/**
 * @brief 每个运行期参数附带的编译期匹配规则 / Compile-time argument matching rules
 * attached to each runtime argument.
 */
enum class FormatArgumentRule : uint8_t
{
  None,            ///< 不消耗运行期参数 / no runtime argument is consumed
  SignedAny,       ///< 默认宽度有符号整数族 / default-width signed integer family
  SignedChar,      ///< 精确匹配 signed char / exact signed char
  SignedShort,     ///< 精确匹配 short / exact short
  SignedLong,      ///< 精确匹配 long / exact long
  SignedLongLong,  ///< 精确匹配 long long / exact long long
  SignedIntMax,    ///< 精确匹配 intmax_t / exact intmax_t
  SignedSize,  ///< 精确匹配 size_t 的有符号对应类型 / exact signed counterpart of size_t
  SignedPtrDiff,     ///< 精确匹配 ptrdiff_t / exact ptrdiff_t
  UnsignedAny,       ///< 默认宽度无符号整数族 / default-width unsigned integer family
  UnsignedChar,      ///< 精确匹配 unsigned char / exact unsigned char
  UnsignedShort,     ///< 精确匹配 unsigned short / exact unsigned short
  UnsignedLong,      ///< 精确匹配 unsigned long / exact unsigned long
  UnsignedLongLong,  ///< 精确匹配 unsigned long long / exact unsigned long long
  UnsignedIntMax,    ///< 精确匹配 uintmax_t / exact uintmax_t
  UnsignedSize,      ///< 精确匹配 size_t / exact size_t
  UnsignedPtrDiff,   ///< 精确匹配 ptrdiff_t 的无符号对应类型
                     ///< exact unsigned counterpart of ptrdiff_t
  Pointer,           ///< 对象指针或 nullptr / object pointer or nullptr
  Character,         ///< 可按 %c 接受的整数或枚举 / any integral or enum accepted by %c
  String,      ///< C 字符串、string_view 或 string / C string, string_view, or string
  Float,       ///< float 或 double / float or double
  LongDouble,  ///< 精确匹配 long double / exact long double
};

/**
 * @brief 保存在值记录字段描述字节中的位标志 / Bit flags stored in a value record's
 * field-spec byte.
 *
 * 这些位不存放在记录操作码字节中，因此数值允许与 FormatOp 重叠。
 * These bits are not stored in the record-op byte, so their numeric values may
 * overlap with FormatOp values.
 */
enum class FormatFlag : uint8_t
{
  LeftAlign = 1U << 0,    ///< 左对齐输出字段 / - flag; left-align field output
  ForceSign = 1U << 1,    ///< 总是输出符号位 / + flag; always emit sign for signed values
  SpaceSign = 1U << 2,    ///< 正数前补空格 / leading space for positive signed values
  Alternate = 1U << 3,    ///< 备用格式，如前缀 / # flag; alternate form such as prefixes
  ZeroPad = 1U << 4,      ///< 允许时用 0 填充 / 0 flag; pad field with 0 when allowed
  UpperCase = 1U << 5,    ///< 使用大写十六进制或浮点格式 / uppercase hex / float output
  CenterAlign = 1U << 6,  ///< 居中对齐输出字段 / centered field output
};

/**
 * @brief Writer 消费的运行期字节码操作 / Runtime bytecode operations consumed by
 * Writer.
 *
 * 常见小场景会直接降为只携带必要立即数的窄操作码；其余情况全部回落到
 * GenericField，继续沿用共享的
 * “type + flags + fill + width + precision” 载荷形状。
 * Small common cases lower directly to narrow opcodes with only the immediates
 * they actually need. Everything else falls back to GenericField, which keeps
 * the shared "type + flags + fill + width + precision" payload shape.
 *
 * @note FormatOp 是线级操作码；下面的 FormatType 是 GenericField 分发用的语义
 *       类别。二者不是一一对应——一个 FormatType 可能由多个 FormatOp 编码
 *       （例如 Unsigned32 由 U32Dec、U32ZeroPadWidth、U32Binary、U32Octal、
 *       U32Hex* 承载），而窄操作码直接以其确切操作命名，完全绕过 FormatType。
 *       FormatOp is the wire-level opcode; FormatType (below) is the semantic
 *       category used for GenericField dispatch. The two are not 1:1 — one
 *       FormatType may be encoded by several FormatOps (e.g. Unsigned32 is
 *       carried by U32Dec, U32ZeroPadWidth, U32Binary, U32Octal, U32Hex*),
 *       and narrow opcodes bypass FormatType entirely by naming their exact
 *       operation.
 */
enum class FormatOp : uint8_t
{
  TextInline = 0x01,  ///< 直接内嵌在码流中的短字面文本 / short inline literal text
  TextRef = 0x02,  ///< 引用尾部文本池中的文本片段 / text span stored in the trailing pool
  TextSpace = 0x03,        ///< 单个字面空格 / one literal space
  U32Dec = 0x10,           ///< 直接输出 uint32_t 十进制 / raw uint32_t decimal output
  U32ZeroPadWidth = 0x11,  ///< 带零填充宽度字节的 uint32_t 十进制
                           ///< uint32_t decimal with zero-pad width byte
  I32Dec = 0x12,           ///< 直接输出 int32_t 十进制 / raw int32_t decimal output
  U32Binary = 0x13,        ///< 直接输出 uint32_t 二进制 / raw uint32_t binary output
  U32Octal = 0x14,         ///< 直接输出 uint32_t 八进制 / raw uint32_t octal output
  U32HexLower =
      0x15,  ///< 直接输出 uint32_t 小写十六进制 / raw uint32_t lowercase hex output
  U32HexUpper =
      0x16,  ///< 直接输出 uint32_t 大写十六进制 / raw uint32_t uppercase hex output
  StringRaw = 0x20,     ///< 直接输出 string_view / raw string_view output
  CharacterRaw = 0x21,  ///< 直接输出字符 / raw character output
  GenericField = 0xF0,  ///< 宽回退载荷：type、flags、fill、width、precision
                        ///< wide fallback payload: type, flags, fill, width, precision
  End = 0xFF,           ///< 结束整条编译记录流 / terminates the compiled record stream
};

/**
 * @brief 编码后的单个操作码后面跟随的载荷字节数 / Number of payload bytes that follow one
 * encoded opcode.
 */
[[nodiscard]] constexpr size_t FormatOpPayloadBytes(FormatOp op)
{
  switch (op)
  {
    case FormatOp::TextInline:
      return 0;
    case FormatOp::TextRef:
      return 2 * sizeof(uint16_t);
    case FormatOp::U32ZeroPadWidth:
      return 1;
    case FormatOp::GenericField:
      return 5;
    case FormatOp::TextSpace:
    case FormatOp::U32Dec:
    case FormatOp::I32Dec:
    case FormatOp::U32Binary:
    case FormatOp::U32Octal:
    case FormatOp::U32HexLower:
    case FormatOp::U32HexUpper:
    case FormatOp::StringRaw:
    case FormatOp::CharacterRaw:
    case FormatOp::End:
      return 0;
  }

  // Unreachable for valid streams: every FormatOp value is enumerated above.
  // A corrupt/unknown opcode falls through here and yields 0 payload bytes.
  return 0;
}

/**
 * @brief 编译期分析和运行期分发共用的语义处理类别 / Semantic handler categories used by
 * compile-time analysis and runtime dispatch.
 */
enum class FormatType : uint8_t
{
  End,         ///< 仅语义哨兵；不等于字节流结束符 FormatOp::End
               ///< semantic sentinel only; not the byte-stream terminator FormatOp::End
  TextInline,  ///< 直接内嵌在码流中的短文本 / short inline text stored in the code stream
  TextRef,     ///< 引用尾部文本池中的长文本 / long text stored in the trailing text pool
  TextSpace,   ///< 单个字面空格 / one literal space
  Signed32,    ///< 运行期按 int32_t 存储的有符号十进制
               ///< runtime signed decimal stored as int32_t
  Signed64,    ///< 运行期按 int64_t 存储的有符号十进制
               ///< runtime signed decimal stored as int64_t
  Unsigned32,  ///< 运行期按 uint32_t 存储的无符号十进制
               ///< runtime unsigned decimal stored as uint32_t
  Unsigned64,  ///< 运行期按 uint64_t 存储的无符号十进制
               ///< runtime unsigned decimal stored as uint64_t
  Binary32,    ///< 运行期按 uint32_t 存储的二进制 / runtime binary stored as uint32_t
  Binary64,    ///< 运行期按 uint64_t 存储的二进制 / runtime binary stored as uint64_t
  Octal32,     ///< 运行期按 uint32_t 存储的八进制 / runtime octal stored as uint32_t
  Octal64,     ///< 运行期按 uint64_t 存储的八进制 / runtime octal stored as uint64_t
  HexLower32,  ///< 运行期按 uint32_t 存储的小写十六进制
               ///< runtime lowercase hex stored as uint32_t
  HexLower64,  ///< 运行期按 uint64_t 存储的小写十六进制
               ///< runtime lowercase hex stored as uint64_t
  HexUpper32,  ///< 运行期按 uint32_t 存储的大写十六进制
               ///< runtime uppercase hex stored as uint32_t
  HexUpper64,  ///< 运行期按 uint64_t 存储的大写十六进制
               ///< runtime uppercase hex stored as uint64_t
  Pointer,     ///< 指针值 / pointer value
  Character,   ///< 单个字符 / character
  String,      ///< 字符串、字符串视图或 C 字符串 / string / string view / C string
  FloatFixed,  ///< 定点 float32 输出 / %f / %F fixed-point float32
  FloatScientific,       ///< 科学计数法 float32 输出 / %e / %E scientific float32
  FloatGeneral,          ///< 通用 float32 输出 / %g / %G general float32
  DoubleFixed,           ///< 定点 double 输出 / %f / %F fixed-point double
  DoubleScientific,      ///< 科学计数法 double 输出 / %e / %E scientific double
  DoubleGeneral,         ///< 通用 double 输出 / %g / %G general double
  LongDoubleFixed,       ///< long double 定点输出 / %Lf / %LF fixed-point long double
  LongDoubleScientific,  ///< long double 科学计数法输出
                         ///< %Le / %LE scientific long double
  LongDoubleGeneral,     ///< long double 通用输出 / %Lg / %LG general long double
};

/**
 * @brief 运行期参数的打包存储类别 / Packed storage categories for runtime arguments.
 *
 * 这个枚举只回答“单个参数在运行期参数字节块里如何存储”，不描述最终文本如何渲染。
 * This only answers "how is one argument stored in the packed argument blob".
 * It does not describe how the final text is rendered.
 */
enum class FormatPackKind : uint8_t
{
  U32,         ///< 按 uint32_t 存储 / stored as uint32_t
  U64,         ///< 按 uint64_t 存储 / stored as uint64_t
  I32,         ///< 按 int32_t 存储 / stored as int32_t
  I64,         ///< 按 int64_t 存储 / stored as int64_t
  Pointer,     ///< 按 uintptr_t 存储 / stored as uintptr_t
  Character,   ///< 按 char 存储 / stored as char
  StringView,  ///< 按 std::string_view 存储 / stored as std::string_view
  F32,         ///< 按 float 存储 / stored as float
  F64,         ///< 按 double 存储 / stored as double
  LongDouble,  ///< 按 long double 存储 / stored as long double
};

/**
 * @brief 编译期选出的精确运行期执行器配置 / Precise runtime executor profiles selected
 * at compile time.
 *
 * 低位描述当前字节码里出现了哪些窄快路径族；其余位精确标记通用字段使用的语义类型，
 * 使无关的整数、文本、指针和浮点后端仍可被裁剪。
 * The low bits describe which narrow fast-path families appear in the bytecode.
 * The remaining bits identify the exact semantic types used by generic fields,
 * so unrelated integer, text, pointer, and float backends remain prunable.
 */
enum class FormatProfile : uint32_t
{
  None = 0,             ///< 只有文本记录的流 / text-only stream
  NarrowInt = 1U << 0,  ///< 窄整数快路径族 / narrow integer fast-path family
  TextArg = 1U << 1,    ///< 原始文本参数快路径族 / raw text argument fast-path family
  GenericSigned32 = 1U << 2,  ///< 通用 32 位有符号十进制 / generic signed 32-bit decimal
  GenericSigned64 = 1U << 3,  ///< 通用 64 位有符号十进制 / generic signed 64-bit decimal
  GenericUnsigned32 =
      1U << 4,  ///< 通用 32 位无符号十进制 / generic unsigned 32-bit decimal
  GenericUnsigned64 =
      1U << 5,  ///< 通用 64 位无符号十进制 / generic unsigned 64-bit decimal
  GenericBinary32 = 1U << 6,  ///< 通用 32 位二进制 / generic 32-bit binary
  GenericBinary64 = 1U << 7,  ///< 通用 64 位二进制 / generic 64-bit binary
  GenericOctal32 = 1U << 8,   ///< 通用 32 位八进制 / generic 32-bit octal
  GenericOctal64 = 1U << 9,   ///< 通用 64 位八进制 / generic 64-bit octal
  GenericHexLower32 =
      1U << 10,  ///< 通用小写 32 位十六进制 / generic lowercase 32-bit hex
  GenericHexLower64 =
      1U << 11,  ///< 通用小写 64 位十六进制 / generic lowercase 64-bit hex
  GenericHexUpper32 =
      1U << 12,  ///< 通用大写 32 位十六进制 / generic uppercase 32-bit hex
  GenericHexUpper64 =
      1U << 13,                 ///< 通用大写 64 位十六进制 / generic uppercase 64-bit hex
  GenericPointer = 1U << 14,    ///< 通用指针字段 / generic pointer field
  GenericCharacter = 1U << 15,  ///< 通用字符字段 / generic character field
  GenericString = 1U << 16,     ///< 通用字符串字段 / generic string field
  GenericFloatFixed = 1U << 17,  ///< 通用 float 定点格式 / generic float fixed-point
  GenericFloatScientific =
      1U << 18,                    ///< 通用 float 科学计数法 / generic float scientific
  GenericFloatGeneral = 1U << 19,  ///< 通用 float 通用格式 / generic float general
  GenericDoubleFixed = 1U << 20,   ///< 通用 double 定点格式 / generic double fixed-point
  GenericDoubleScientific =
      1U << 21,  ///< 通用 double 科学计数法 / generic double scientific
  GenericDoubleGeneral = 1U << 22,  ///< 通用 double 通用格式 / generic double general
  GenericLongDoubleFixed =
      1U << 23,  ///< 通用 long double 定点格式 / generic long double fixed-point
  GenericLongDoubleScientific =
      1U << 24,  ///< 通用 long double 科学计数法 / generic long double scientific
  GenericLongDoubleGeneral =
      1U << 25,           ///< 通用 long double 通用格式 / generic long double general
  Generic = 0x03FFFFFCU,  ///< 所有通用字段的兼容掩码
                          ///< compatibility mask for every generic field
};

/**
 * @brief 合并两组 profile 位 / Combine two profile bit sets
 * @param left 左侧 profile 位集合 / Left profile bit set
 * @param right 右侧 profile 位集合 / Right profile bit set
 * @return 合并后的 profile 位集合 / Returns the merged profile bit set
 */
[[nodiscard]] constexpr FormatProfile operator|(FormatProfile left, FormatProfile right)
{
  return static_cast<FormatProfile>(static_cast<uint32_t>(left) |
                                    static_cast<uint32_t>(right));
}

/**
 * @brief 将一组 profile 位累加到另一组 / Accumulate one profile bit set into another
 * @param left 累加目标 / Accumulation target
 * @param right 待并入的 profile 位 / Profile bits to merge in
 * @return 并入 `right` 后的 `left` / Returns `left` after merging `right`
 */
constexpr FormatProfile& operator|=(FormatProfile& left, FormatProfile right)
{
  left = left | right;
  return left;
}

/**
 * @brief 判断某个 profile 位是否存在 / Test whether one profile bit is present
 * @param profile 待检查的 profile 位集合 / Profile bit set to inspect
 * @param bit 待测试的单个 profile 位 / Single profile bit to test
 * @return `profile` 中存在 `bit` 时返回 `true`，否则返回 `false` / Returns `true` when
 * `bit` is present in `profile`, otherwise `false`
 */
[[nodiscard]] constexpr bool HasProfile(FormatProfile profile, FormatProfile bit)
{
  return (static_cast<uint32_t>(profile) & static_cast<uint32_t>(bit)) != 0;
}

/**
 * @brief 返回一个通用字段语义类型对应的精确 profile 位 / Return the precise profile bit
 * for one generic-field semantic type
 * @param type 通用字段携带的语义类型 / Semantic type carried by the generic field
 * @return 对应的精确 profile 位；非通用值类型返回 `None` / The precise profile bit, or
 * `None` for non-value semantic types
 */
[[nodiscard]] constexpr FormatProfile GenericProfileFor(FormatType type)
{
  auto value = static_cast<uint8_t>(type);
  auto first = static_cast<uint8_t>(FormatType::Signed32);
  auto last = static_cast<uint8_t>(FormatType::LongDoubleGeneral);
  if (value < first || value > last)
  {
    return FormatProfile::None;
  }
  return static_cast<FormatProfile>(uint32_t{1} << (2U + value - first));
}

static_assert(GenericProfileFor(FormatType::Signed32) == FormatProfile::GenericSigned32);
static_assert(GenericProfileFor(FormatType::LongDoubleGeneral) ==
              FormatProfile::GenericLongDoubleGeneral);
static_assert(GenericProfileFor(FormatType::TextSpace) == FormatProfile::None);
static_assert(static_cast<uint32_t>(FormatProfile::Generic) ==
              ((uint32_t{1} << 26U) - (uint32_t{1} << 2U)));

/**
 * @brief Writer 消费的编译格式运行期协议 / Compiled-format runtime contract consumed by
 * Writer.
 *
 * 编译格式源总会提供三部分：Codes()，即一段在块内用 FormatOp::End 结束的连续
 * uint8_t 字节块；ArgumentList()，即按字段执行顺序排列、供运行期参数打包使用的
 * FormatArgumentInfo 数组；以及 Profile()，即一个编译期运行期执行器配置。若某个
 * 前端允许调用点参数重排，还可以额外提供独立的编译期 ArgumentOrder() 索引表或
 * 按源参数顺序排列的匹配元信息表。
 * A compiled format source always provides Codes(), one contiguous uint8_t byte
 * block terminated in-band by FormatOp::End; ArgumentList(), one field-ordered
 * FormatArgumentInfo array used for runtime argument packing; and Profile(),
 * one compile-time runtime executor profile. Frontends that allow reordered
 * call-site arguments may additionally expose a separate compile-time
 * ArgumentOrder() list or a source-ordered matching list.
 */

/**
 * @brief 每个参数对应的元信息，同时用于编译期类型检查和运行期打包 / Per-argument
 * metadata used both for compile-time type checking and runtime packing.
 *
 * `pack` 描述运行期参数字节块里如何存这个参数。
 * `rule` 描述编译期允许哪些 C++ 类型匹配这个参数。
 * `pack` says how the runtime argument blob stores this argument.
 * `rule` says which C++ types are accepted at compile time.
 */
struct FormatArgumentInfo
{
  FormatPackKind pack{};  ///< 运行期打包存储类别 / packed runtime storage kind
  FormatArgumentRule rule =
      FormatArgumentRule::None;  ///< 编译期实参匹配规则 / compile-time argument rule
};
static_assert(sizeof(FormatArgumentInfo) == 2,
              "LibXR::Print::FormatArgumentInfo must stay tightly packed");

/**
 * @brief 一条已经决定完毕、运行期 writer 知道如何打印的值字段 / One fully-decided value
 * field that the runtime writer knows how to print.
 *
 * 它描述的是：
 * - 该走哪条打印路径
 * - 参数当时是怎么打包的
 * - 编译期是按哪条规则接受这个参数的
 * - 以及运行期仍然需要的宽度/填充/精度信息
 * It says:
 * - which printing path to use
 * - how the argument was packed
 * - which compile-time rule accepted that argument
 * - which width/fill/precision flags still matter at runtime
 */
struct FormatField
{
  FormatType type = FormatType::End;  ///< 语义写出类别 / semantic render category
  FormatPackKind pack{};  ///< 运行期打包存储类别 / packed runtime storage kind
  FormatArgumentRule rule =
      FormatArgumentRule::None;  ///< 编译期实参匹配规则 / compile-time argument rule
  uint8_t flags = 0;             ///< 字段修饰位集合 / FormatFlag bitset
  char fill = ' ';               ///< 字段填充字符 / field fill character
  uint8_t width = 0;             ///< 已解析的字段宽度 / parsed field width
  uint8_t precision = 0xFF;  ///< 已解析精度，或未指定 / parsed precision, or unspecified
};

/**
 * @brief 返回一个运行期已打包参数会占多少字节 / Returns how many bytes one packed
 * runtime argument occupies.
 *
 * 这里回答的是“一个参数打包后有多大”，不是“某条操作码记录有多长”。
 * This answers "how big is one packed argument", not "how long is one opcode".
 * @param pack 待检查的打包存储类型 / Packed storage kind to inspect.
 * @return 返回该存储类型下单个已打包参数的字节数。
 *         Returns the byte size of one packed argument in this storage kind.
 */
[[nodiscard]] constexpr size_t FormatArgumentBytes(FormatPackKind pack)
{
  switch (pack)
  {
    case FormatPackKind::I32:
      return sizeof(int32_t);
    case FormatPackKind::I64:
      return sizeof(int64_t);
    case FormatPackKind::U32:
      return sizeof(uint32_t);
    case FormatPackKind::U64:
      return sizeof(uint64_t);
    case FormatPackKind::Pointer:
      return sizeof(uintptr_t);
    case FormatPackKind::Character:
      return sizeof(char);
    case FormatPackKind::StringView:
      return sizeof(std::string_view);
    case FormatPackKind::F32:
      return sizeof(float);
    case FormatPackKind::F64:
      return sizeof(double);
    case FormatPackKind::LongDouble:
      return sizeof(long double);
  }

  return 0;
}

}  // namespace LibXR::Print
