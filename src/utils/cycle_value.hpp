#pragma once

#include <cmath>

#include "libxr_def.hpp"

namespace LibXR
{

using DefaultScalar = LIBXR_DEFAULT_SCALAR;

/**
 * @brief 角度循环处理类，用于处理周期性角度计算。
 *        A cyclic angle handling class for periodic angle calculations.
 *
 * 该类用于处理 0 到 2π 之间的角度运算，确保角度始终保持在合法范围内，
 * 并提供加减运算和类型转换功能。
 * This class handles angle calculations within the range of 0 to 2π,
 * ensuring values stay within valid limits and supporting addition, subtraction, and type
 * conversion operations.
 *
 * @tparam Scalar 角度的存储类型，默认为 `DefaultScalar`。
 *                The storage type for angles, default is `DefaultScalar`.
 */
template <typename Scalar = DefaultScalar>
class CycleValue
{
 public:
  /**
   * @brief 赋值运算符重载。
   *        Overloaded assignment operator.
   *
   * 赋值运算符默认使用编译器生成的实现。
   * The assignment operator uses the default compiler-generated implementation.
   *
   * @return 返回赋值后的 `CycleValue` 对象。
   *         Returns the assigned `CycleValue` object.
   */
  CycleValue& operator=(const CycleValue&) = default;

  /**
   * @brief 计算角度值并归一化到 0 到 2π 之间。
   *        Computes and normalizes the angle value within the range of 0 to 2π.
   *
   * @param value 输入角度值。
   *              The input angle value.
   * @return 归一化后的角度值。
   *         The normalized angle value.
   */
  static constexpr Scalar Calculate(Scalar value)
  {
    value = std::fmod(value, LibXR::TWO_PI);
    if (value < 0)
    {
      value += LibXR::TWO_PI;
    }
    return value;
  }

  /**
   * @brief 使用给定值初始化 `CycleValue`。
   *        Initializes `CycleValue` with a given value.
   *
   * @param value 需要存储的角度值。
   *              The angle value to be stored.
   */
  CycleValue(const Scalar& value) : value_(Calculate(value)) {}

  /**
   * @brief 拷贝构造函数，确保角度值在合法范围内。
   *        Copy constructor ensuring the angle value remains within valid limits.
   *
   * @param value 另一个 `CycleValue` 对象。
   *              Another `CycleValue` object.
   */
  CycleValue(const CycleValue& value) : value_(value.value_)
  {
    while (value_ >= LibXR::TWO_PI)
    {
      value_ -= LibXR::TWO_PI;
    }

    while (value_ < 0)
    {
      value_ += LibXR::TWO_PI;
    }
  }

  /**
   * @brief 默认构造函数，初始化为 0。
   *        Default constructor initializing the angle to 0.
   */
  CycleValue() : value_(0.0f) {}

  /**
   * @brief 加法运算符重载。
   *        Overloaded addition operator.
   *
   * @param value 需要加上的角度值。
   *              The angle value to be added.
   * @return 返回新的 `CycleValue` 对象。
   *         Returns a new `CycleValue` object.
   */
  CycleValue operator+(const Scalar& value) const { return CycleValue(value + value_); }

  CycleValue operator+(const CycleValue& value) const
  {
    return CycleValue(value.value_ + value_);
  }

  /**
   * @brief 复合加法运算符重载。
   *        Overloaded compound addition operator.
   *
   * @param value 需要加上的角度值。
   *              The angle value to be added.
   * @return 返回自身的引用。
   *         Returns a reference to itself.
   */
  CycleValue& operator+=(const Scalar& value)
  {
    value_ = Calculate(value + value_);
    return *this;
  }

  CycleValue& operator+=(const CycleValue& value)
  {
    Scalar ans = value.value_ + value_;
    while (ans >= LibXR::TWO_PI)
    {
      ans -= LibXR::TWO_PI;
    }

    while (ans < 0)
    {
      ans += LibXR::TWO_PI;
    }

    value_ = ans;
    return *this;
  }

  /**
   * @brief 减法运算符重载，计算角度差值并归一化到 `-π` 到 `π` 之间。
   *        Overloaded subtraction operator, computing the angle difference and
   * normalizing it to `-π` to `π`.
   *
   * @param raw_value 需要减去的角度值。
   *                  The angle value to be subtracted.
   * @return 归一化后的角度差值。
   *         The normalized angle difference.
   */
  Scalar operator-(const Scalar& raw_value) const
  {
    Scalar value = Calculate(raw_value);
    Scalar ans = value_ - value;
    while (ans >= LibXR::PI)
    {
      ans -= LibXR::TWO_PI;
    }

    while (ans < -LibXR::PI)
    {
      ans += LibXR::TWO_PI;
    }

    return ans;
  }

  Scalar operator-(const CycleValue& value) const
  {
    Scalar ans = value_ - value.value_;
    while (ans >= LibXR::PI)
    {
      ans -= LibXR::TWO_PI;
    }

    while (ans < -LibXR::PI)
    {
      ans += LibXR::TWO_PI;
    }

    return ans;
  }

  /**
   * @brief 复合减法运算符重载。
   *        Overloaded compound subtraction operator.
   *
   * @param value 需要减去的角度值。
   *              The angle value to be subtracted.
   * @return 返回自身的引用。
   *         Returns a reference to itself.
   */
  CycleValue& operator-=(const Scalar& value)
  {
    value_ = Calculate(value_ - value);
    return *this;
  }

  CycleValue& operator-=(const CycleValue& value)
  {
    Scalar ans = value_ - value.value_;
    while (ans >= LibXR::TWO_PI)
    {
      ans -= LibXR::TWO_PI;
    }

    while (ans < 0)
    {
      ans += LibXR::TWO_PI;
    }

    value_ = ans;
    return *this;
  }

  /**
   * @brief 取反运算符重载，将角度转换到 `2π - value_`。
   *        Overloaded negation operator, converting the angle to `2π - value_`.
   *
   * @return 返回取反后的 `CycleValue`。
   *         Returns the negated `CycleValue`.
   */
  CycleValue operator-() const { return CycleValue(LibXR::TWO_PI - value_); }

  /**
   * @brief 类型转换操作符，将 `CycleValue` 转换为 `Scalar`。
   *        Type conversion operator to convert `CycleValue` to `Scalar`.
   *
   * @return 以 `Scalar` 形式返回角度值。
   *         Returns the angle value as `Scalar`.
   */
  operator Scalar() const { return this->value_; }

  /**
   * @brief 赋值运算符重载，更新角度值并归一化。
   *        Overloaded assignment operator, updating and normalizing the angle value.
   *
   * @param value 需要赋值的新角度值。
   *              The new angle value to be assigned.
   * @return 返回自身的引用。
   *         Returns a reference to itself.
   */
  CycleValue& operator=(const Scalar& value)
  {
    value_ = Calculate(value);
    return *this;
  }

  /**
   * @brief 获取当前的角度值。
   *        Retrieves the current angle value.
   *
   * @return 角度值。
   *         The angle value.
   */
  Scalar Value() const { return value_; }

 private:
  Scalar value_;  ///< 存储的角度值 / The stored angle value.
};

}  // namespace LibXR
