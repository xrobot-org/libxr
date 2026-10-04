#pragma once

#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#ifndef DEF2STR
#define XR_TO_STR(_arg) #_arg
#define DEF2STR(_arg) XR_TO_STR(_arg)
#endif

#ifndef UNUSED
/// \brief 用于抑制未使用变量的警告 / Macro to suppress unused variable warnings
#define UNUSED(_x) ((void)(_x))
#endif

#ifndef LIBXR_SINGLE_CORE
#define LIBXR_SINGLE_CORE 0
#endif

#if defined(_MSC_VER)
#define LIBXR_NOINLINE __declspec(noinline)
#define LIBXR_PACKED_BEGIN __pragma(pack(push, 1))
#define LIBXR_PACKED_END __pragma(pack(pop))
#define LIBXR_PACKED
#elif defined(__clang__) || defined(__GNUC__)
#define LIBXR_NOINLINE __attribute__((noinline))
#define LIBXR_PACKED_BEGIN _Pragma("pack(push, 1)")
#define LIBXR_PACKED_END _Pragma("pack(pop)")
#define LIBXR_PACKED __attribute__((packed))
#else
#warning "LibXR compiler compatibility macros fallback to no-op on unknown compiler"
#define LIBXR_NOINLINE
#define LIBXR_PACKED_BEGIN
#define LIBXR_PACKED_END
#define LIBXR_PACKED
#endif

namespace LibXR
{
/// \brief PI 常量 / PI constant
inline constexpr double PI = 3.14159265358979323846;

/// \brief 2PI 常量 / 2PI constant
inline constexpr double TWO_PI = 2.0 * PI;

/// \brief 标准重力加速度（m/s²） / Standard gravitational acceleration (m/s²)
inline constexpr double STANDARD_GRAVITY = 9.80665;

/// \brief 真实硬件缓存行大小（用于 DMA / cache coherency） / Hardware cache-line size
/// used for DMA/cache-coherency boundaries
inline constexpr size_t HW_CACHE_LINE_SIZE = (sizeof(void*) == 8) ? 64 : 32;

/// \brief 并发结构对齐粒度（用于降低多核伪共享） / Alignment policy used by
/// concurrency-oriented structures
#if LIBXR_SINGLE_CORE
inline constexpr size_t CONCURRENCY_ALIGNMENT = sizeof(size_t);
#else
inline constexpr size_t CONCURRENCY_ALIGNMENT = HW_CACHE_LINE_SIZE;
#endif

/// \brief 兼容旧代码的缓存行别名 / Backward-compatible cache-line alias for existing code
inline constexpr size_t CACHE_LINE_SIZE = HW_CACHE_LINE_SIZE;

/// \brief 平台自然对齐大小 / Native platform alignment size
inline constexpr size_t ALIGN_SIZE = sizeof(void*);

/**
 * @brief 指向非静态数据成员的成员指针 / Pointer to a non-static data member
 */
template <typename OwnerType, typename MemberType>
concept MemberObjectPointer = std::is_member_object_pointer_v<MemberType OwnerType::*>;

/**
 * @brief 具有公共类型且可比较大小的类型对 / Type pair with a common type and ordering
 */
template <typename LeftType, typename RightType>
concept CommonOrdered = std::common_with<LeftType, RightType> &&
                        requires(const LeftType& left, const RightType& right) {
                          { left < right } -> std::convertible_to<bool>;
                          { left > right } -> std::convertible_to<bool>;
                        };

/**
 * @brief 计算成员在宿主对象中的偏移量
 *        Computes the offset of a member within the owning object
 * @param member 指向成员的成员指针，如 `&Type::member` / Member pointer such as
 * `&Type::member`
 * @return 成员偏移量 / Member offset
 */
template <typename OwnerType, typename MemberType>
  requires MemberObjectPointer<OwnerType, MemberType>
[[nodiscard]] inline size_t OffsetOf(MemberType OwnerType::* member) noexcept
{
  return reinterpret_cast<size_t>(
      &(reinterpret_cast<const volatile OwnerType*>(0)->*member));
}

/**
 * @brief 通过成员指针恢复其所属对象指针
 *        Recover the owning object pointer from a member pointer
 * @param ptr 指向成员的指针 / Pointer to the member
 * @param member 指向成员的成员指针，如 `&Type::member` / Member pointer such as
 * `&Type::member`
 * @return 所属对象指针 / Pointer to the owning object
 */
template <typename OwnerType, typename MemberType>
  requires MemberObjectPointer<OwnerType, MemberType>
[[nodiscard]] inline OwnerType* ContainerOf(MemberType* ptr,
                                            MemberType OwnerType::* member) noexcept
{
  return reinterpret_cast<OwnerType*>(reinterpret_cast<std::byte*>(ptr) -
                                      OffsetOf(member));
}

template <typename OwnerType, typename MemberType>
  requires MemberObjectPointer<OwnerType, MemberType>
[[nodiscard]] inline const OwnerType* ContainerOf(const MemberType* ptr,
                                                  MemberType OwnerType::* member) noexcept
{
  return reinterpret_cast<const OwnerType*>(reinterpret_cast<const std::byte*>(ptr) -
                                            OffsetOf(member));
}

/**
 * @enum ErrorCode
 * @brief 定义错误码枚举 / Defines an enumeration for error codes
 */
enum class ErrorCode : int8_t
{
  PENDING = 1,        ///< 等待中 / Pending
  OK = 0,             ///< 操作成功 / Operation successful
  FAILED = -1,        ///< 操作失败 / Operation failed
  INIT_ERR = -2,      ///< 初始化错误 / Initialization error
  ARG_ERR = -3,       ///< 参数错误 / Argument error
  STATE_ERR = -4,     ///< 状态错误 / State error
  SIZE_ERR = -5,      ///< 尺寸错误 / Size error
  CHECK_ERR = -6,     ///< 校验错误 / Check error
  NOT_SUPPORT = -7,   ///< 不支持 / Not supported
  NOT_FOUND = -8,     ///< 未找到 / Not found
  NO_RESPONSE = -9,   ///< 无响应 / No response
  NO_MEM = -10,       ///< 内存不足 / Insufficient memory
  NO_BUFF = -11,      ///< 缓冲区不足 / Insufficient buffer
  TIMEOUT = -12,      ///< 超时 / Timeout
  EMPTY = -13,        ///< 为空 / Empty
  FULL = -14,         ///< 已满 / Full
  BUSY = -15,         ///< 忙碌 / Busy
  PTR_NULL = -16,     ///< 空指针 / Null pointer
  OUT_OF_RANGE = -17  ///< 超出范围 / Out of range
};

/**
 * @enum SizeLimitMode
 * @brief 定义尺寸限制模式 / Defines size limit modes
 */
enum class SizeLimitMode : uint8_t
{
  EQUAL = 0,  ///< 尺寸必须相等 / Size must be equal
  LESS = 1,   ///< 尺寸必须小于等于 / Size must be less than or equal
  MORE = 2,   ///< 尺寸必须大于等于 / Size must be greater than or equal
  NONE = 3    ///< 无限制 / No restriction
};

/**
 * @brief 尺寸约束的纯判断函数 / Pure predicate for size-limit comparisons
 *
 * 这个辅助函数只判断尺寸关系是否成立；
 * 它本身不决定调用方应该断言、强约束终止，还是返回错误码。
 * This helper only answers whether the requested size relation holds.
 * It does not decide whether the caller should assert, require, or return an
 * error code.
 */
[[nodiscard]] constexpr bool SizeLimitCheck(SizeLimitMode mode, size_t limit,
                                            size_t size) noexcept
{
  switch (mode)
  {
    case SizeLimitMode::EQUAL:
      return limit == size;
    case SizeLimitMode::LESS:
      return limit >= size;
    case SizeLimitMode::MORE:
      return limit <= size;
    case SizeLimitMode::NONE:
      return true;
  }
  return false;
}
}  // namespace LibXR

#ifdef DEV_ASSERT
#undef DEV_ASSERT
#endif

#ifdef DEV_ASSERT_FROM_CALLBACK
#undef DEV_ASSERT_FROM_CALLBACK
#endif

#ifdef REQUIRE
#undef REQUIRE
#endif

#ifdef REQUIRE_FROM_CALLBACK
#undef REQUIRE_FROM_CALLBACK
#endif

#ifdef ASSERT
#undef ASSERT
#endif

#ifdef ASSERT_FROM_CALLBACK
#undef ASSERT_FROM_CALLBACK
#endif

/**
 * @brief 固定配置和调用前提检查 / Fixed configuration and caller precondition check
 * @note LIBXR_DEBUG_BUILD 启用失败检查；关闭时仍求值。必要操作应在宏外执行。
 *       LIBXR_DEBUG_BUILD enables failure checking; otherwise the expression is still
 *       evaluated. Required operations belong outside the macro.
 * @param arg 要检查的条件 / Condition to check
 */
#ifdef LIBXR_DEBUG_BUILD
#define ASSERT(arg)                                 \
  do                                                \
  {                                                 \
    if (!(arg))                                     \
    {                                               \
      libxr_fatal_error(__FILE__, __LINE__, false); \
    }                                               \
  } while (0)

/**
 * @brief 回调/ISR 上下文中的 LibXR 用户断言
 *        Assertion macro for LibXR users in callback/ISR contexts
 *
 * @param arg    要检查的条件 / Condition to check
 * @param in_isr 当前是否在中断上下文 / Whether currently in ISR context
 */
#define ASSERT_FROM_CALLBACK(arg, in_isr)              \
  do                                                   \
  {                                                    \
    if (!(arg))                                        \
    {                                                  \
      libxr_fatal_error(__FILE__, __LINE__, (in_isr)); \
    }                                                  \
  } while (0)
#else
#define ASSERT(arg) (void(arg), (void)0)
#define ASSERT_FROM_CALLBACK(arg, in_isr) \
  do                                      \
  {                                       \
    (void)(arg);                          \
    (void)(in_isr);                       \
  } while (0)
#endif

/**
 * @brief 库内部实现的开发期检查 / Development check for library implementation
 * @note 默认禁用且不求值；仅在定义 LIBXR_DEV_ASSERT_BUILD 时启用。
 *       Disabled without evaluation unless LIBXR_DEV_ASSERT_BUILD is defined.
 *       条件不得包含必须执行的操作 / Conditions must not contain required operations.
 * @param arg 要检查的条件 / Condition to check
 */
#ifdef LIBXR_DEV_ASSERT_BUILD
#define DEV_ASSERT(arg)                             \
  do                                                \
  {                                                 \
    if (!(arg))                                     \
    {                                               \
      libxr_fatal_error(__FILE__, __LINE__, false); \
    }                                               \
  } while (0)

/**
 * @brief 仅供 LibXR 本体开发使用的回调/ISR 开发期断言
 *        Development-only callback/ISR assertion for LibXR maintainers
 *
 * @param arg    要检查的条件 / Condition to check
 * @param in_isr 当前是否在中断上下文 / Whether currently in ISR context
 */
#define DEV_ASSERT_FROM_CALLBACK(arg, in_isr)          \
  do                                                   \
  {                                                    \
    if (!(arg))                                        \
    {                                                  \
      libxr_fatal_error(__FILE__, __LINE__, (in_isr)); \
    }                                                  \
  } while (0)
#else
#define DEV_ASSERT(arg) ((void)0)
#define DEV_ASSERT_FROM_CALLBACK(arg, in_isr) ((void)0)
#endif

/**
 * @brief 始终生效的致命运行错误检查 / Always-enabled fatal runtime error check
 * @note 可恢复错误应通过接口返回，不应在此终止。
 *       Recoverable errors belong to the interface's error-return path.
 * @param arg 要检查的条件 / Condition to check
 */
#define REQUIRE(arg)                                \
  do                                                \
  {                                                 \
    if (!(arg))                                     \
    {                                               \
      libxr_fatal_error(__FILE__, __LINE__, false); \
    }                                               \
  } while (0)

/**
 * @brief 回调/ISR 上下文中的强约束检查
 *        Strong requirement check in callback/ISR contexts
 *
 * @param arg    要检查的条件 / Condition to check
 * @param in_isr 当前是否在中断上下文 / Whether currently in ISR context
 */
#define REQUIRE_FROM_CALLBACK(arg, in_isr)             \
  do                                                   \
  {                                                    \
    if (!(arg))                                        \
    {                                                  \
      libxr_fatal_error(__FILE__, __LINE__, (in_isr)); \
    }                                                  \
  } while (0)

/**
 * @brief 处理致命错误 / Handles fatal errors
 * @param file 出错的源文件 / Source file where the error occurred
 * @param line 出错的行号 / Line number where the error occurred
 * @param in_isr 是否发生在中断服务例程（ISR） / Whether it occurred in an ISR
 */
extern "C" void libxr_fatal_error(const char* file, uint32_t line, bool in_isr);

namespace LibXR
{
/**
 * @brief 计算两个数的最大值 / Computes the maximum of two numbers
 * @tparam LeftType 第一个数的类型 / Type of the first number
 * @tparam RightType 第二个数的类型 / Type of the second number
 * @param a 第一个数 / First number
 * @param b 第二个数 / Second number
 * @return 两数中的较大值 / The larger of the two numbers
 */
template <typename LeftType, typename RightType>
  requires CommonOrdered<LeftType, RightType>
constexpr auto max(LeftType a, RightType b) -> std::common_type_t<LeftType, RightType>
{
  return (a > b) ? a : b;
}

/**
 * @brief 计算两个数的最小值 / Computes the minimum of two numbers
 * @tparam LeftType 第一个数的类型 / Type of the first number
 * @tparam RightType 第二个数的类型 / Type of the second number
 * @param a 第一个数 / First number
 * @param b 第二个数 / Second number
 * @return 两数中的较小值 / The smaller of the two numbers
 */
template <typename LeftType, typename RightType>
  requires CommonOrdered<LeftType, RightType>
constexpr auto min(LeftType a, RightType b) -> std::common_type_t<LeftType, RightType>
{
  return (a < b) ? a : b;
}
}  // namespace LibXR
