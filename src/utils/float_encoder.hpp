#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace LibXR
{

/**
 * @brief 浮点数编码器，将浮点值映射到定长无符号整数。
 *        A generic float encoder mapping a float value to N-bit unsigned integer.
 *
 * 用于将连续区间 [min, max] 映射到整数区间 [0, 2^bits - 1]，
 * 可用于 IMU 数据、传感器值、CAN 打包等场景。
 *
 * @tparam Bits     使用的整数位数，例如 21 表示使用 21-bit 无符号整数。
 * @tparam Scalar   浮点数类型，默认为 float。
 */
template <int Bits, typename Scalar = float>
class FloatEncoder
{
 public:
  static_assert(Bits > 0 && Bits < 32, "Bits must be between 1 and 31");

  /**
   * @brief 获取最大可编码的整数值。
   *        Returns the maximum encodable unsigned integer.
   */
  static constexpr uint32_t MaxInt() { return (1u << Bits) - 1; }

  /**
   * @brief 构造函数，设置映射区间 [min, max]。
   *        Constructor specifying the float range [min, max].
   *
   * @param min 最小浮点值 / Minimum float value.
   * @param max 最大浮点值 / Maximum float value.
   */
  FloatEncoder(Scalar min, Scalar max) : min_(min), max_(max), range_(max - min) {}

  /**
   * @brief 编码：将浮点数映射为无符号整数。
   *        Encodes a float to unsigned integer in range [0, 2^Bits - 1].
   *
   * @param value 输入的浮点数值 / Input float value.
   * @return 对应的整数编码值 / Encoded unsigned integer.
   */
  uint32_t Encode(Scalar value) const
  {
    Scalar clamped = std::fmin(std::fmax(value, min_), max_);
    Scalar norm = (clamped - min_) / range_;
    return static_cast<uint32_t>(std::round(norm * MaxInt()));
  }

  /**
   * @brief 解码：将无符号整数还原为浮点值。
   *        Decodes an unsigned integer to float.
   *
   * @param encoded 编码后的整数 / Encoded unsigned integer.
   * @return 对应的浮点数值 / Decoded float value.
   */
  Scalar Decode(uint32_t encoded) const
  {
    Scalar norm = static_cast<Scalar>(encoded) / MaxInt();
    return min_ + norm * range_;
  }

 private:
  Scalar min_;    ///< 浮点区间最小值
  Scalar max_;    ///< 浮点区间最大值
  Scalar range_;  ///< 区间范围（max - min）
};

}  // namespace LibXR
