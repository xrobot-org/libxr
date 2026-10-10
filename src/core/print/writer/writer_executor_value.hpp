#pragma once

/**
 * @brief 执行器的具体运行期数值写出函数 / Concrete runtime value writers for the executor
 */

/**
 * @brief 为一个有符号整数载荷确定最终可见的符号字符 / Resolve the visible sign character
 * for one signed integer payload
 * @tparam Int 有符号整数类型 / Signed integer type
 * @param value 当前要输出的整数值 / Integer value being emitted
 * @param spec 解码后的字段规格 / Decoded field spec
 * @return 返回 `'-'`、`'+'`、`' '` 或 `'\0'` / Returns `'-'`, `'+'`, `' '`, or `'\0'`
 */
template <OutputSink Sink>
template <std::signed_integral Int>
char Writer::Executor<Sink>::ResolveSignChar(Int value, const Spec& spec)
{
  if (value < 0)
  {
    return '-';
  }
  if (spec.ForceSign())
  {
    return '+';
  }
  if (spec.SpaceSign())
  {
    return ' ';
  }
  return '\0';
}

#if LIBXR_PRINT_ENABLE_FLOAT
/**
 * @brief 为一个浮点载荷确定最终可见的符号字符 / Resolve the visible sign character for
 * one float payload
 * @tparam T 浮点类型 / Float type
 * @param value 当前要输出的浮点值 / Float value being emitted
 * @param spec 解码后的字段规格 / Decoded field spec
 * @return 返回 `'-'`、`'+'`、`' '` 或 `'\0'` / Returns `'-'`, `'+'`, `' '`, or `'\0'`
 */
template <OutputSink Sink>
template <typename T>
char Writer::Executor<Sink>::ResolveFloatSignChar(T value, const Spec& spec)
{
  if (std::isnan(value))
  {
    // NaN signbit is implementation-defined and platform-dependent.
    // Derive sign from the explicit format flag only so output is stable
    // across platforms and FPU implementations:
    //   {:f}  -> "nan"   {:+f} -> "+nan"   {: f} -> " nan"
    return spec.ForceSign() ? '+' : spec.SpaceSign() ? ' ' : '\0';
  }
  if (std::signbit(value))
  {
    return '-';
  }
  if (spec.ForceSign())
  {
    return '+';
  }
  if (spec.SpaceSign())
  {
    return ' ';
  }
  return '\0';
}
#endif

/**
 * @brief 通过共享整数字段路径写出一个有符号整数值 / Write one signed integer value
 * through the shared integer-field path
 * @tparam Int 有符号整数类型 / Signed integer type
 * @param spec 解码后的字段规格 / Decoded field spec
 * @param value 待写出的整数值 / Integer value to write
 * @return 返回共享整数字段路径的写出结果 / Returns the shared integer-field write result
 */
template <OutputSink Sink>
template <std::signed_integral Int>
ErrorCode Writer::Executor<Sink>::WriteSigned(const Spec& spec, Int value)
{
  using UInt = std::make_unsigned_t<Int>;
  char digit_buffer[UnsignedDigitCapacity<UInt, 10>()];
  UInt bits = static_cast<UInt>(value);
  UInt magnitude = (value < 0) ? (UInt{0} - bits) : bits;
  size_t digit_count = AppendUnsigned<10>(digit_buffer, magnitude);

  std::string_view digits(digit_buffer, digit_count);
  if (value == 0 && spec.precision == 0)
  {
    digits = {};
  }

  return WriteIntegerField(ResolveSignChar(value, spec), {}, digits, spec);
}

/**
 * @brief 通过共享整数字段路径写出一个无符号整数语义值 / Write one unsigned integer
 * semantic value through the shared integer-field path
 * @tparam Base 整数进制 / Integer radix
 * @tparam UpperCase 十六进制数字是否使用大写字符 / Whether hexadecimal digits should use
 * uppercase characters
 * @tparam PrependOctalZero 是否把 `%#o` 的前导 `0` 直接并入数字载荷 / Whether `%#o`
 * should inline its leading `0` into the digit payload
 * @tparam UInt 无符号整数类型 / Unsigned integer type
 * @param prefix 脱离数字载荷输出的前缀 / Prefix emitted outside the digit payload
 * @param spec 解码后的字段规格 / Decoded field spec
 * @param value 待写出的整数值 / Integer value to write
 * @return 返回共享整数字段路径的写出结果 / Returns the shared integer-field write result
 */
template <OutputSink Sink>
template <uint8_t Base, bool UpperCase, bool PrependOctalZero,
          std::unsigned_integral UInt>
ErrorCode Writer::Executor<Sink>::WriteUnsignedDigits(std::string_view prefix,
                                                      const Spec& spec, UInt value)
{
  char digit_buffer[UnsignedDigitCapacity<UInt, Base>() + (PrependOctalZero ? 1U : 0U)];
  size_t digit_count = AppendUnsigned<Base, UpperCase>(digit_buffer, value);

  if constexpr (PrependOctalZero)
  {
    digit_count = ApplyAlternateOctal(digit_buffer, digit_count, spec, value);
  }
  else if (value == 0 && spec.precision == 0)
  {
    digit_count = 0;
  }

  return WriteIntegerField('\0', prefix, std::string_view(digit_buffer, digit_count),
                           spec);
}

/**
 * @brief 通过共享整数字段路径写出一个无符号整数语义值 / Write one unsigned integer
 * semantic value through the shared integer-field path
 * @tparam Type 运行期整数语义类型 / Runtime integer semantic type
 * @tparam UInt 无符号整数类型 / Unsigned integer type
 * @param spec 解码后的字段规格 / Decoded field spec
 * @param value 待写出的整数值 / Integer value to write
 * @return 返回共享整数字段路径的写出结果 / Returns the shared integer-field write result
 *
 * 这个桥接函数本身不直接格式化数字；它只负责把运行期整数语义类型映射到上面的
 * 编译期进制/大小写共享辅助路径。
 * This bridge does not format digits itself; it only maps one runtime integer
 * semantic type onto the shared compile-time radix/case helper above.
 */
template <OutputSink Sink>
template <FormatType Type, std::unsigned_integral UInt>
ErrorCode Writer::Executor<Sink>::DispatchUnsigned(const Spec& spec, UInt value)
{
  auto prefix = IntegerPrefix(Type, spec, value);

  if constexpr (Type == FormatType::Unsigned32 || Type == FormatType::Unsigned64)
  {
    return WriteUnsignedDigits<10>(prefix, spec, value);
  }
  if constexpr (Type == FormatType::Binary32 || Type == FormatType::Binary64)
  {
    return WriteUnsignedDigits<2>(prefix, spec, value);
  }
  if constexpr (Type == FormatType::Octal32 || Type == FormatType::Octal64)
  {
    return WriteUnsignedDigits<8, false, true>(prefix, spec, value);
  }
  if constexpr (Type == FormatType::HexLower32 || Type == FormatType::HexLower64)
  {
    return WriteUnsignedDigits<16>(prefix, spec, value);
  }
  if constexpr (Type == FormatType::HexUpper32 || Type == FormatType::HexUpper64)
  {
    return WriteUnsignedDigits<16, true>(prefix, spec, value);
  }

  return ErrorCode::ARG_ERR;
}

/**
 * @brief 按规范指针字段策略写出一个指针值 / Write one pointer value using the canonical
 * pointer field policy
 * @param spec 解码后的字段规格 / Decoded field spec
 * @param value 以 `uintptr_t` 编码的指针值 / Pointer value encoded as `uintptr_t`
 * @return 返回指针字段写出结果 / Returns the pointer-field write result
 */
template <OutputSink Sink>
ErrorCode Writer::Executor<Sink>::WritePointer(const Spec& spec, uintptr_t value)
{
  char digit_buffer[UnsignedDigitCapacity<uintptr_t, 16>()];
  size_t digit_count = AppendUnsigned<16>(digit_buffer, value);
  Spec actual = spec;

  if (!actual.HasPrecision() || actual.precision == 0)
  {
    actual.precision = 1;
  }

  return WriteIntegerField('\0', "0x", std::string_view(digit_buffer, digit_count),
                           actual);
}

/**
 * @brief 写出一个字符字段值 / Write one character field value
 * @param spec 解码后的字段规格 / Decoded field spec
 * @param ch 待写出的字符值 / Character value to write
 * @return 返回字符字段写出结果 / Returns the character-field write result
 */
template <OutputSink Sink>
ErrorCode Writer::Executor<Sink>::WriteCharacter(const Spec& spec, char ch)
{
  return WriteTextField(std::string_view(&ch, 1), spec);
}

/**
 * @brief 写出一个字符串字段值，并在需要时应用精度截断 / Write one string field value,
 * including precision truncation when present
 * @param spec 解码后的字段规格 / Decoded field spec
 * @param text 待写出的字符串载荷 / String payload to write
 * @return 返回字符串字段写出结果 / Returns the string-field write result
 */
template <OutputSink Sink>
ErrorCode Writer::Executor<Sink>::WriteString(const Spec& spec, std::string_view text)
{
  auto view = text;
  if (spec.HasPrecision() && spec.precision < view.size())
  {
    // 不用 substr：它的越界分支会链接 libstdc++ 的异常支持。
    // Not substr: its out-of-range path links the libstdc++ exception support.
    view = std::string_view(view.data(), spec.precision);
  }

  return WriteTextField(view, spec);
}

/**
 * @brief 通过共享浮点文本后端写出一个浮点语义值 / Write one float semantic value through
 * the shared float-text backend
 * @tparam T 浮点类型 / Float type
 * @param type 运行期浮点语义类型 / Runtime float semantic type
 * @param spec 解码后的字段规格 / Decoded field spec
 * @param value 待写出的浮点值 / Float value to write
 * @return 返回浮点字段写出结果 / Returns the float-field write result
 */
#if LIBXR_PRINT_ENABLE_FLOAT
template <OutputSink Sink>
template <typename T>
ErrorCode Writer::Executor<Sink>::WriteFloat(FormatType type, const Spec& spec, T value)
{
  if (!UsesFloatTextBackend(type))
  {
    return ErrorCode::ARG_ERR;
  }

  Spec actual = spec;
  if (!std::isfinite(value))
  {
    actual.flags &= static_cast<uint8_t>(~static_cast<uint8_t>(FormatFlag::ZeroPad));
  }

  char sign_char = ResolveFloatSignChar(value, actual);
  T magnitude = std::signbit(value) ? -static_cast<T>(value) : static_cast<T>(value);
  uint8_t precision = actual.HasPrecision() ? actual.precision : DefaultFloatPrecision();
  if (type == FormatType::FloatFixed || type == FormatType::DoubleFixed ||
      type == FormatType::LongDoubleFixed)
  {
    if (ExceedsFixedIntegerDigits(magnitude, precision))
    {
      return ErrorCode::OUT_OF_RANGE;
    }
  }
  else if (type == FormatType::FloatGeneral || type == FormatType::DoubleGeneral ||
           type == FormatType::LongDoubleGeneral)
  {
    uint8_t significant = precision == 0 ? 1 : precision;
    int exponent =
        RoundScientificDigits(magnitude, static_cast<uint8_t>(significant - 1)).exponent;
    if (!(exponent < -4 || exponent >= significant))
    {
      int fractional_precision = static_cast<int>(significant) - (exponent + 1);
      if (fractional_precision < 0)
      {
        fractional_precision = 0;
      }
      if (ExceedsFixedIntegerDigits(magnitude,
                                    static_cast<uint8_t>(fractional_precision)))
      {
        return ErrorCode::OUT_OF_RANGE;
      }
    }
  }
  char output_buffer[float_buffer_capacity];
  size_t output_size = 0;
  if (!FormatFloatText(type, actual, magnitude, output_buffer, output_size))
  {
    return ErrorCode::NO_BUFF;
  }

  return WriteFloatField(sign_char, std::string_view(output_buffer, output_size), actual);
}
#endif

/**
 * @brief 写出一个原始 uint32 十进制快路径字段 / Writes one raw uint32 decimal fast-path
 * field.
 * @param value 待写出的无符号值 / Unsigned value to write.
 * @return 返回 sink 写出结果 / Returns the sink write result.
 */
template <OutputSink Sink>
ErrorCode Writer::Executor<Sink>::WriteU32Dec(uint32_t value)
{
  char digit_buffer[UnsignedDigitCapacity<uint32_t, 10>()];
  size_t digit_count = AppendUnsigned<10>(digit_buffer, value);
  return WriteRaw(std::string_view(digit_buffer, digit_count));
}

/**
 * @brief 写出一个原始 int32 十进制快路径字段 / Writes one raw int32 decimal fast-path
 * field.
 * @param value 待写出的有符号值 / Signed value to write.
 * @return 返回 sink 写出结果 / Returns the sink write result.
 */
template <OutputSink Sink>
ErrorCode Writer::Executor<Sink>::WriteI32Dec(int32_t value)
{
  using UInt = std::make_unsigned_t<int32_t>;
  char digit_buffer[UnsignedDigitCapacity<UInt, 10>()];
  UInt bits = static_cast<UInt>(value);
  UInt magnitude = (value < 0) ? (UInt{0} - bits) : bits;
  size_t digit_count = AppendUnsigned<10>(digit_buffer, magnitude);

  if (value < 0)
  {
    if (auto ec = WriteRaw("-"); ec != ErrorCode::OK)
    {
      return ec;
    }
  }

  return WriteRaw(std::string_view(digit_buffer, digit_count));
}

/**
 * @brief 写出一个原始 uint32 多进制快路径字段 / Writes one raw uint32 base-specific
 * fast-path field.
 * @tparam Base 整数进制 / Integer radix.
 * @tparam UpperCase 十六进制数字是否大写 / Whether hexadecimal digits are uppercase.
 * @param value 待写出的无符号值 / Unsigned value to write.
 * @return 返回 sink 写出结果 / Returns the sink write result.
 */
template <OutputSink Sink>
template <uint8_t Base, bool UpperCase>
ErrorCode Writer::Executor<Sink>::WriteU32Base(uint32_t value)
{
  char digit_buffer[UnsignedDigitCapacity<uint32_t, Base>()];
  size_t digit_count = AppendUnsigned<Base, UpperCase>(digit_buffer, value);
  return WriteRaw(std::string_view(digit_buffer, digit_count));
}

/**
 * @brief 写出一个带零填充宽度的 `uint32_t` 十进制快路径字段 / Write one zero-padded
 * `uint32_t` decimal fast-path field
 * @param width 目标零填充宽度 / Target zero-padded width
 * @param value 待写出的无符号值 / Unsigned value to write
 * @return 返回该快路径的写出结果 / Returns the fast-path write result
 */
template <OutputSink Sink>
ErrorCode Writer::Executor<Sink>::WriteU32ZeroPadWidth(uint8_t width, uint32_t value)
{
  char digit_buffer[UnsignedDigitCapacity<uint32_t, 10>()];
  size_t digit_count = AppendUnsigned<10>(digit_buffer, value);
  size_t zeros = FieldPadding(width, digit_count);
  if (auto ec = WritePadding('0', zeros); ec != ErrorCode::OK)
  {
    return ec;
  }
  return WriteRaw(std::string_view(digit_buffer, digit_count));
}

/**
 * @brief 写出一个原始字符串快路径字段 / Writes one raw string fast-path field.
 * @param text 待写出的字符串载荷 / String payload to write.
 * @return 返回 sink 写出结果 / Returns the sink write result.
 */
template <OutputSink Sink>
ErrorCode Writer::Executor<Sink>::WriteStringRaw(std::string_view text)
{
  return WriteRaw(text);
}

/**
 * @brief 写出一个原始字符快路径字段 / Writes one raw character fast-path field.
 * @param ch 待写出的字符载荷 / Character payload to write.
 * @return 返回 sink 写出结果 / Returns the sink write result.
 */
template <OutputSink Sink>
ErrorCode Writer::Executor<Sink>::WriteCharacterRaw(char ch)
{
  return WriteRaw(std::string_view(&ch, 1));
}
