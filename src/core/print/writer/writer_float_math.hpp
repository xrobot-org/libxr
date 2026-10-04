#pragma once

/**
 * @brief 通用浮点文本格式化器使用的数学辅助函数 / Math helpers used by the generic
 * float text formatter.
 */

/**
 * @brief 浮点文本输出归一化过程中使用的十进制缩放对 / Decimal-scale pair used while
 * normalizing one float for text output.
 * @tparam Float 浮点类型 / Float type.
 */
template <typename Float>
struct Writer::DecimalScale
{
  int exponent = 0;  ///< 十进制指数 / decimal exponent
  Float scale = 1;   ///< 10 的 exponent 次幂 / 10 ^ exponent
};

/**
 * @brief 科学计数法归一化后的尾数数字、缩放因子与十进制指数 / Rounded mantissa digits,
 * scale factor, and decimal exponent after scientific normalization.
 * @tparam Float 浮点类型 / Float type.
 */
template <typename Float>
struct Writer::ScientificDigits
{
  Float digits =
      0;  ///< 舍入后按整数位缩放的尾数 / rounded mantissa scaled to integer digits
  Float scale = 1;   ///< 施加到尾数上的 10 的 precision 次幂
                     ///< 10 ^ precision applied to the mantissa
  int exponent = 0;  ///< 十进制指数 / decimal exponent
};

/**
 * @brief 返回 10 的指定十进制指数次幂 / Returns 10 raised to the given decimal exponent.
 * @tparam Float 浮点类型 / Float type.
 * @param exponent 十进制指数 / Decimal exponent.
 * @return 返回对应的十进制幂 / Returns the corresponding power of ten.
 */
template <typename Float>
Float Writer::Power10(int exponent)
{
  Float result = 1;
  Float base = 10;
  unsigned int remaining = static_cast<unsigned int>(exponent < 0 ? -exponent : exponent);

  while (remaining != 0)
  {
    if ((remaining & 1U) != 0U)
    {
      if (exponent < 0)
      {
        result /= base;
      }
      else
      {
        result *= base;
      }
    }

    remaining >>= 1U;
    if (remaining != 0U)
    {
      base *= base;
    }
  }

  return result;
}

/**
 * @brief 把一个值舍入到指定小数位 / Round one value to the requested decimal precision.
 * @tparam Float 浮点类型 / Float type.
 * @param value 有限非负值 / Finite non-negative value.
 * @param precision 保留的小数位数 / Decimal places to retain.
 * @return 返回舍入后的值；若缩放溢出且输入为非负数，则返回 +infinity。
 *         Rounded value, or +infinity if scaling would overflow (input is
 *         expected to be a non-negative magnitude).
 */
template <typename Float>
Float Writer::RoundDecimal(Float value, uint8_t precision)
{
  Float scale = Power10<Float>(static_cast<int>(precision));
  Float scaled = value * scale;
  if (!std::isfinite(scaled))
  {
    // Scaling overflowed: propagate as infinity so callers' isfinite() guard
    // rejects the result cleanly instead of silently returning the original
    // value and later producing OUT_OF_RANGE. Input is expected to be a
    // non-negative magnitude, so plain infinity is sufficient.
    return std::numeric_limits<Float>::infinity();
  }

  return std::nearbyint(scaled) / scale;
}

/**
 * @brief 把一个值归一化为科学计数法的尾数数字与十进制指数 / Normalize one value into
 * scientific-notation mantissa digits and a decimal exponent.
 * @tparam Float 浮点类型 / Float type.
 * @param value 有限非负值 / Finite non-negative value.
 * @param precision 尾数中保留的有效小数位数。
 * Significant fractional digits to retain in the mantissa.
 * @return 返回舍入后的尾数、缩放因子与十进制指数；若舍入进位溢出一位，会相应调整指数。
 *         Rounded mantissa, its scale, and the decimal exponent (carry-out
 *         adjusts the exponent when rounding overflows one digit).
 */
template <typename Float>
Writer::ScientificDigits<Float> Writer::RoundScientificDigits(Float value,
                                                              uint8_t precision)
{
  ScientificDigits<Float> result{};
  auto normalized = NormalizeDecimal(value);
  result.exponent = (value == 0) ? 0 : normalized.exponent;
  Float decimal_scale = Power10<Float>(result.exponent);
  Float mantissa = (value == 0) ? 0 : value / decimal_scale;
  result.scale = Power10<Float>(static_cast<int>(precision));
  result.digits = std::nearbyint(mantissa * result.scale);
  if (result.digits >= static_cast<Float>(10) * result.scale)
  {
    result.digits /= 10;
    ++result.exponent;
  }

  return result;
}

/**
 * @brief 将一个浮点值归一化为十进制指数与缩放因子 / Normalizes one float into a decimal
 * exponent plus scale pair.
 * @tparam Float 浮点类型 / Float type.
 * @param value 有限正数绝对值 / Finite positive magnitude.
 * @return 返回规范化后的十进制缩放描述。
 *         Returns the normalized decimal scale description.
 */
template <typename Float>
Writer::DecimalScale<Float> Writer::NormalizeDecimal(Float value)
{
  DecimalScale<Float> normalized{};
  if (value == 0)
  {
    return normalized;
  }

  int binary_exponent = 0;
  std::frexp(value, &binary_exponent);
  constexpr Float log10_of_2 = static_cast<Float>(0.30102999566398119521373889472449L);
  normalized.exponent =
      static_cast<int>(static_cast<Float>(binary_exponent - 1) * log10_of_2);
  normalized.scale = Power10<Float>(normalized.exponent);

  Float scaled = value / normalized.scale;
  while (scaled < 1)
  {
    normalized.scale /= 10;
    --normalized.exponent;
    scaled *= 10;
  }
  while (scaled >= 10)
  {
    normalized.scale *= 10;
    ++normalized.exponent;
    scaled /= 10;
  }

  return normalized;
}

/**
 * @brief 在当前缩放位提取一个十进制数字并推进剩余值 / Extracts one decimal digit at the
 * current scale and advances the remainder.
 * @tparam Float 浮点类型 / Float type.
 * @param value 剩余规范化值；会原地减少 / Remaining normalized value; reduced in place.
 * @param scale 当前十进制权重 / Current decimal scale.
 * @return 返回提取出的十进制数字 / Returns the extracted decimal digit.
 */
template <typename Float>
uint8_t Writer::ExtractDigit(Float& value, Float scale)
{
  Float scaled = value / scale;
  // Bias to correct floating-point rounding when the true digit value is an
  // integer but division leaves it just below (e.g. 1.9999999... instead of 2).
  // 1e-12 is effective for double (epsilon ~2.2e-16) but effectively zero for
  // float (epsilon ~1.2e-7). For float, digit extraction may have up to 1-ULP
  // error in the last digit; this is a known limitation of the approach.
  // Do NOT increase this bias to fix float: values like 1.999999f have a
  // legitimate float representation ~9.5e-7 below 2.0, so any bias large
  // enough to "fix" float rounding would also cause false carry on such values.
  auto digit = static_cast<int>(scaled + static_cast<Float>(1e-12L));
  if (digit < 0)
  {
    digit = 0;
  }
  else if (digit > 9)
  {
    digit = 9;
  }

  value -= static_cast<Float>(digit) * scale;
  // Zero-clamp: clear tiny negative residuals that are FP rounding artifacts.
  // 1e-9 is effective for double but coarse for long double; for float it is
  // large enough to clear genuine residuals without clamping real fractional
  // remainders (float residuals after digit extraction are < epsilon * scale).
  Float epsilon = scale * static_cast<Float>(1e-9L);
  if (value < 0 && value > -epsilon)
  {
    value = 0;
  }

  return static_cast<uint8_t>(digit);
}
