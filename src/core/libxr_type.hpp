#pragma once

#include <cstring>
#include <string>
#include <string_view>
#include <type_traits>

#include "libxr_def.hpp"

namespace LibXR
{

namespace Detail
{
/**
 * @brief 仅裁掉数组末尾的一个 `\\0`；其余字节按原始数据保留。
 *        Trim at most one trailing `\\0` from a bounded char array and keep all
 *        preceding bytes untouched.
 */
template <size_t N>
[[nodiscard]] constexpr size_t TrailingNulTrimmedArraySize(char (&data)[N]) noexcept
{
  return (data[N - 1] == '\0') ? (N - 1) : N;
}

template <size_t N>
[[nodiscard]] constexpr size_t TrailingNulTrimmedArraySize(const char (&data)[N]) noexcept
{
  return (data[N - 1] == '\0') ? (N - 1) : N;
}

/**
 * @brief `std::string` 或 `std::string_view`（忽略 cv 和引用）。通用对象构造函数会取这类
 *        对象本身的字节而不是文本，因此它们不能隐式转换为 `RawData` / `ConstRawData`。
 *        `std::string` or `std::string_view`, ignoring cv and reference. The generic
 *        object constructors would view the object's own bytes instead of the text, so
 *        these types do not convert implicitly to `RawData` / `ConstRawData`.
 */
template <typename T>
concept StdStringOrView = std::is_same_v<std::remove_cvref_t<T>, std::string> ||
                          std::is_same_v<std::remove_cvref_t<T>, std::string_view>;
}  // namespace Detail

/**
 * @brief 可写原始数据视图 / Mutable raw data view
 *
 * @note 该类型不拥有数据，仅保存起始地址和字节数。调用方需要保证被引用对象在视图使用期间
 *       保持有效且可写。
 *       / This type does not own the data. It only stores the start address and
 *       byte count. The caller must keep the referenced object alive and writable
 *       while the view is in use.
 */
class RawData
{
 public:
  /**
   * @brief 使用指定地址和大小构造 `RawData` 对象。
   *        Constructs a `RawData` object with the specified address and size.
   *
   * @param addr 数据的起始地址。
   *             The starting address of the data.
   * @param size 数据的大小（字节）。
   *             The size of the data (in bytes).
   */
  RawData(void* addr, size_t size) : addr_(addr), size_(size) {}

  /**
   * @brief 默认构造函数，初始化为空数据。
   *        Default constructor initializing to empty data.
   */
  RawData() = default;

  /**
   * @brief 从可写对象构造视图，大小为 `sizeof(DataType)` / Construct a view from a
   *        writable object; the size is `sizeof(DataType)`
   * @tparam DataType 对象类型，不能是 `std::string` 或 `std::string_view` / Object type;
   *         must not be `std::string` or `std::string_view`
   * @param data 被引用对象 / Referenced object
   */
  template <typename DataType>
    requires(!std::is_const_v<DataType> &&
             !std::is_same_v<std::remove_cvref_t<DataType>, RawData> &&
             !Detail::StdStringOrView<DataType>)
  RawData(DataType& data) : addr_(&data), size_(sizeof(DataType))
  {
  }

  /**
   * @brief 禁止 `std::string` 隐式转换为 `RawData`；文本视图须显式写 `RawData(text)`。
   *        Implicit conversion from `std::string` is deleted; write `RawData(text)` for
   *        a text view.
   */
  template <typename Text>
    requires(std::is_same_v<Text, std::string>)
  RawData(Text&) = delete;  // 用 RawData(text) / Use RawData(text)

  /**
   * @brief 拷贝构造函数。
   *        Copy constructor.
   *
   * @param data 另一个 `RawData` 对象。
   *             Another `RawData` object.
   */
  RawData(const RawData& data) = default;

  /**
   * @brief 从 `char *` 指针构造 `RawData`，数据大小为字符串长度（不含 `\0`）。
   *        Constructs `RawData` from a `char *` pointer,
   *        with size set to the string length (excluding `\0`).
   *
   * @param data C 风格字符串指针。
   *             A C-style string pointer.
   */
  template <typename CharPtr>
    requires(std::is_same_v<std::remove_cvref_t<CharPtr>, char*>)
  RawData(CharPtr&& data) : addr_(data), size_(data != nullptr ? std::strlen(data) : 0)
  {
  }

  /**
   * @brief 从可写字符数组构造文本视图 / Construct a text view from a writable char array
   * @tparam N 数组长度 / Array length
   * @param data 被引用字符数组 / Referenced char array
   *
   * @note 若数组最后一个字符恰为 `\\0`，则仅裁掉这一个尾随终止符；否则保留整个数组长度。
   *       / If the last array element is `\\0`, only that trailing terminator is
   *       trimmed; otherwise the full array extent is kept.
   */
  template <size_t N>
  RawData(char (&data)[N]) : addr_(data), size_(Detail::TrailingNulTrimmedArraySize(data))
  {
  }

  /**
   * @brief 从可写字符串构造文本视图 / Construct a text view from a writable string
   * @param data 被引用字符串 / Referenced string
   */
  explicit RawData(std::string& data)
      : addr_(data.empty() ? nullptr : data.data()), size_(data.size())
  {
  }

  /**
   * @brief 赋值运算符重载。
   *        Overloaded assignment operator.
   *
   * @param data 另一个 `RawData` 对象。
   *             Another `RawData` object.
   * @return 返回赋值后的 `RawData` 对象引用。
   *         Returns a reference to the assigned `RawData` object.
   */
  RawData& operator=(const RawData& data) = default;

  void* addr_ = nullptr;  ///< 数据起始地址 / Data start address
  size_t size_ = 0;       ///< 数据字节数 / Data size in bytes
};

/**
 * @brief 只读原始数据视图 / Immutable raw data view
 *
 * @note 该类型不拥有数据，仅保存起始地址和字节数。调用方需要保证被引用对象在视图使用期间
 *       保持有效。
 *       / This type does not own the data. It only stores the start address and
 *       byte count. The caller must keep the referenced object alive while the
 *       view is in use.
 */
class ConstRawData
{
 public:
  /**
   * @brief 使用指定地址和大小构造 `ConstRawData` 对象。
   *        Constructs a `ConstRawData` object with the specified address and size.
   *
   * @param addr 数据的起始地址。
   *             The starting address of the data.
   * @param size 数据的大小（字节）。
   *             The size of the data (in bytes).
   */
  ConstRawData(const void* addr, size_t size) : addr_(addr), size_(size) {}

  /**
   * @brief 默认构造函数，初始化为空数据。
   *        Default constructor initializing to empty data.
   */
  ConstRawData() = default;

  /**
   * @brief 从任意对象构造只读视图，大小为 `sizeof(DataType)` / Construct a read-only
   *        view from any object; the size is `sizeof(DataType)`
   * @tparam DataType 对象类型，不能是指针、`std::string` 或 `std::string_view` / Object
   *         type; must not be a pointer, `std::string` or `std::string_view`
   * @param data 被引用对象 / Referenced object
   */
  template <typename DataType>
    requires(!std::is_pointer_v<std::remove_cvref_t<DataType>> &&
             !std::is_same_v<std::remove_cvref_t<DataType>, ConstRawData> &&
             !std::is_same_v<std::remove_cvref_t<DataType>, RawData> &&
             !Detail::StdStringOrView<DataType>)
  ConstRawData(const DataType& data)
      : addr_(reinterpret_cast<const DataType*>(&data)), size_(sizeof(DataType))
  {
  }

  /**
   * @brief 禁止 `std::string` / `std::string_view` 隐式转换为 `ConstRawData`；文本视图须
   *        显式写 `ConstRawData(text)`。
   *        Implicit conversion from `std::string` / `std::string_view` is deleted; write
   *        `ConstRawData(text)` for a text view.
   */
  template <typename Text>
    requires(Detail::StdStringOrView<Text>)
  ConstRawData(const Text&) = delete;  // 用 ConstRawData(text) / Use ConstRawData(text)

  /**
   * @brief 拷贝构造函数。
   *        Copy constructor.
   *
   * @param data 另一个 `ConstRawData` 对象。
   *             Another `ConstRawData` object.
   */
  ConstRawData(const ConstRawData& data) = default;

  /**
   * @brief 从 `RawData` 构造 `ConstRawData`，数据地址和大小保持不变。
   *        Constructs `ConstRawData` from `RawData`,
   *        keeping the same data address and size.
   *
   * @param data `RawData` 对象。
   *             A `RawData` object.
   */
  ConstRawData(const RawData& data) : addr_(data.addr_), size_(data.size_) {}

  /**
   * @brief 从 `char*` / `const char*` 文本指针构造
   * `ConstRawData`，数据大小为字符串长度（不含 `\0`）。
   * Constructs `ConstRawData` from a `char*` / `const char*` text pointer, with size set
   * to the string length (excluding `\0`).
   *
   * @param data C 风格字符串指针。
   *             A C-style string pointer.
   */
  template <typename CharPtr>
    requires(std::is_pointer_v<std::remove_cvref_t<CharPtr>> &&
             std::is_same_v<
                 std::remove_cv_t<std::remove_pointer_t<std::remove_cvref_t<CharPtr>>>,
                 char> &&
             !std::is_volatile_v<std::remove_pointer_t<std::remove_cvref_t<CharPtr>>>)
  ConstRawData(CharPtr&& data)
      : addr_(data),
        size_(data != nullptr ? std::strlen(static_cast<const char*>(data)) : 0)
  {
  }

  /**
   * @brief 从只读字符串构造文本视图 / Construct a text view from a read-only string
   * @param data 被引用字符串 / Referenced string
   */
  explicit ConstRawData(const std::string& data)
      : addr_(data.empty() ? nullptr : data.data()), size_(data.size())
  {
  }

  /**
   * @brief 从字符串视图构造文本视图 / Construct a text view from a string view
   * @param data 被引用字符串视图 / Referenced string view
   */
  explicit ConstRawData(std::string_view data)
      : addr_(data.empty() ? nullptr : data.data()), size_(data.size())
  {
  }

  /**
   * @brief 从字符数组构造 `ConstRawData`；若最后一个字符是 `\\0`，仅忽略这一尾随终止符。
   *        Constructs `ConstRawData` from a character array; if the last element
   *        is `\\0`, only that trailing terminator is ignored.
   *
   * @tparam N 数组大小。
   *           The array size.
   * @param data 需要存储的字符数组。
   *             The character array to be stored.
   */
  template <size_t N>
  ConstRawData(char (&data)[N])
      : addr_(reinterpret_cast<const void*>(data)),
        size_(Detail::TrailingNulTrimmedArraySize(data))
  {
  }

  template <size_t N>
  ConstRawData(const char (&data)[N])
      : addr_(reinterpret_cast<const void*>(data)),
        size_(Detail::TrailingNulTrimmedArraySize(data))
  {
  }

  /**
   * @brief 赋值运算符重载。
   *        Overloaded assignment operator.
   *
   * @param data 另一个 `ConstRawData` 对象。
   *             Another `ConstRawData` object.
   * @return 返回赋值后的 `ConstRawData` 对象引用。
   *         Returns a reference to the assigned `ConstRawData` object.
   */
  ConstRawData& operator=(const ConstRawData& data) = default;

  const void* addr_ = nullptr;  ///< 数据起始地址 / Data start address
  size_t size_ = 0;             ///< 数据字节数 / Data size in bytes
};

/**
 * @brief 类型标识符生成器 / RTTI-free type identifier generator
 */
class TypeID
{
 public:
  using ID = const void*;

  /**
   * @brief 获取类型的唯一标识符 / Get a unique identifier for type `T`
   * @tparam T 目标类型 / Target type
   * @return 类型唯一标识符指针 / Unique type identifier pointer
   */
  template <typename T>
  static ID GetID()
  {
    static char id;  // NOLINT
    return &id;
  }
};

}  // namespace LibXR
