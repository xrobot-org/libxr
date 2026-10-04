#pragma once

#include <concepts>
#include <cstring>
#include <tuple>
#include <type_traits>
#include <utility>

#include "libxr_def.hpp"

namespace LibXR
{

/**
 * @brief 可转换为精确回调函数指针的可调用对象
 *        Callable convertible to the exact callback function pointer
 */
template <typename CallableType, typename BoundArgType, typename... CallbackArgs>
concept CallbackFunctionCompatible = requires(CallableType callable) {
  static_cast<void (*)(bool, BoundArgType, CallbackArgs...)>(callable);
};

template <typename... Args>
struct CallbackBlockHeader
{
  using InvokeFunType = void (*)(void*, bool, Args...);

  InvokeFunType run_fun_ = nullptr;
};

/**
 * @brief 回调函数封装块，提供参数绑定与擦除调用入口 / Callback block with bound argument
 * and erased invoke entry
 *
 * @tparam ArgType 绑定的第一个参数类型 / Type of the first bound argument
 * @tparam Args 额外的参数类型列表 / Additional argument types
 */
template <typename ArgType, typename... Args>
class CallbackBlock : public CallbackBlockHeader<Args...>
{
 public:
  /**
   * @brief 回调函数类型定义 / Callback function type definition
   */
  using FunctionType = void (*)(bool, ArgType, Args...);

  /**
   * @brief 构造回调块，绑定回调函数与参数 / Construct a callback block with bound
   * function and argument
   *
   * @param fun 需要调用的回调函数 / Callback function to be invoked
   * @param arg 绑定的参数值 / Bound argument value
   */
  CallbackBlock(FunctionType fun, ArgType&& arg)
      : CallbackBlockHeader<Args...>{&InvokeThunk}, fun_(fun), arg_(std::move(arg))
  {
    ASSERT(fun_ != nullptr);
  }

  /**
   * @brief 禁用拷贝构造与拷贝赋值 / Copy construction and copy assignment are disabled
   */
  CallbackBlock(const CallbackBlock& other) = delete;
  CallbackBlock& operator=(const CallbackBlock& other) = delete;

  static void InvokeThunk(void* cb_block, bool in_isr, Args... args)
  {
    auto* cb = static_cast<CallbackBlock<ArgType, Args...>*>(cb_block);
    cb->Invoke(in_isr, std::forward<Args>(args)...);
  }

 protected:
  void Invoke(bool in_isr, Args... args)
  {
    fun_(in_isr, arg_, std::forward<Args>(args)...);
  }

  FunctionType fun_;  ///< 绑定的回调函数 / Bound callback function
  ArgType arg_;       ///< 绑定的参数 / Bound argument
};

template <typename ArgType, typename... Args>
class GuardedCallbackBlock : public CallbackBlock<ArgType, Args...>
{
 public:
  /**
   * @brief 带防重入保护的回调块 / Callback block with reentry guard
   *
   * @note 该保护层只负责压平同一回调的递归重入链路，
   *       不承担通用的跨上下文同步语义。
   *       This guard only flattens self-recursive callback chains and does not
   *       provide general cross-context synchronization semantics.
   */
  GuardedCallbackBlock(typename CallbackBlock<ArgType, Args...>::FunctionType fun,
                       ArgType&& arg)
      : CallbackBlock<ArgType, Args...>(fun, std::move(arg))
  {
    this->run_fun_ = &InvokeThunk;
  }

  static void InvokeThunk(void* cb_block, bool in_isr, Args... args)
  {
    auto* cb = static_cast<GuardedCallbackBlock<ArgType, Args...>*>(cb_block);

    if (!cb->running_)
    {
      cb->running_ = true;
      auto cur_args = std::tuple<std::decay_t<Args>...>{std::forward<Args>(args)...};
      do
      {
        cb->pending_ = false;
        std::apply([&](auto&... a) { cb->Invoke(in_isr, a...); }, cur_args);
        if (cb->pending_)
        {
          cur_args = std::move(cb->pending_args_);
        }
      } while (cb->pending_);
      cb->running_ = false;
      return;
    }

    // 重入时只保留最新一组参数，把递归调用压平成串行重放。
    // On reentry, keep only the latest argument pack so recursive callback chains are
    // flattened into serialized replay.
    cb->pending_args_ = std::tuple<std::decay_t<Args>...>{std::forward<Args>(args)...};
    cb->pending_ = true;
  }

 private:
  bool running_ = false;
  bool pending_ = false;
  std::tuple<std::decay_t<Args>...> pending_args_{};
};

/**
 * @brief 通用回调包装，支持动态参数传递 / Generic callback wrapper supporting dynamic
 * argument passing
 *
 * @tparam Args 额外的参数类型列表 / Additional argument types
 */
template <typename... Args>
class Callback
{
  static void FunctionDefault(void*, bool, Args...) {}
  inline static CallbackBlockHeader<Args...> empty_cb_block_ = {&FunctionDefault};

 public:
  /**
   * @brief 创建回调对象并绑定回调函数与参数 / Create a callback instance with bound
   * function and argument
   *
   * @tparam BoundArgType 绑定参数类型 / Bound argument type
   * @tparam CallableType 回调可调用对象类型 / Callback callable type
   * @param fun 需要绑定的回调函数 / Callback function to bind
   * @param arg 绑定的参数值 / Bound argument value
   * @return Callback 实例 / Created Callback instance
   *
   * @note 预期用法是初始化阶段创建并长期持有；
   *       运行时高频创建/销毁不属于设计目标。
   *       Intended usage is initialization-time creation with long-lived
   *       retention; runtime churn-style create/destroy patterns are outside
   *       the intended model.
   */
  template <typename BoundArgType, typename CallableType>
    requires CallbackFunctionCompatible<CallableType, BoundArgType, Args...>
  [[nodiscard]] static Callback Create(CallableType fun, BoundArgType arg)
  {
    using FunctionType = typename CallbackBlock<BoundArgType, Args...>::FunctionType;
    auto cb_block = new CallbackBlock<BoundArgType, Args...>(
        static_cast<FunctionType>(fun), std::move(arg));
    return Callback(cb_block);
  }

  /**
   * @brief 创建带防重入保护的回调 / Create a guarded callback
   *
   * @tparam BoundArgType 绑定参数类型 / Bound argument type
   * @tparam CallableType 回调可调用对象类型 / Callback callable type
   * @param fun 需要绑定的回调函数 / Callback function to bind
   * @param arg 绑定的参数值 / Bound argument value
   * @return Callback 实例 / Created Callback instance
   *
   * @note 该 guarded callback 预期用于单条逻辑回调链上的递归压平，
   *       不承担任意多上下文串行化。
   *       Guarded callbacks are meant for self-recursive flattening on one
   *       logical callback chain, not arbitrary multi-context serialization.
   */
  template <typename BoundArgType, typename CallableType>
    requires CallbackFunctionCompatible<CallableType, BoundArgType, Args...>
  [[nodiscard]] static Callback CreateGuarded(CallableType fun, BoundArgType arg)
  {
    using FunctionType = typename CallbackBlock<BoundArgType, Args...>::FunctionType;
    auto cb_block = new GuardedCallbackBlock<BoundArgType, Args...>(
        static_cast<FunctionType>(fun), std::move(arg));
    return Callback(cb_block);
  }

  /**
   * @brief 默认构造函数，创建空回调对象 / Default constructor creating an empty callback
   */
  Callback() : cb_block_(&empty_cb_block_) {}

  Callback(const Callback&) = default;
  Callback& operator=(const Callback&) = default;

  /**
   * @brief 移动构造函数，转移回调对象的所有权 / Move constructor transferring callback
   * ownership
   *
   * @param other 另一个 Callback 实例 / Another Callback instance
   */
  Callback(Callback&& other) noexcept
      : cb_block_(std::exchange(other.cb_block_, &empty_cb_block_))
  {
  }

  /**
   * @brief 移动赋值运算符，转移回调对象的所有权 / Move assignment operator transferring
   * callback ownership
   *
   * @param other 另一个 Callback 实例 / Another Callback instance
   * @return 当前对象引用 / Reference to the current object
   */
  Callback& operator=(Callback&& other) noexcept
  {
    if (this != &other)
    {
      cb_block_ = std::exchange(other.cb_block_, &empty_cb_block_);
    }
    return *this;
  }

  template <typename... PassArgs>
  void Run(bool in_isr, PassArgs&&... args) const
  {
    cb_block_->run_fun_(cb_block_, in_isr, std::forward<PassArgs>(args)...);
  }

  /**
   * @brief 检查回调是否为空 / Check whether the callback is empty
   *
   * @return true 回调为空 / Callback is empty
   * @return false 回调非空 / Callback is not empty
   */
  bool Empty() const { return cb_block_ == &empty_cb_block_; }

 private:
  /**
   * @brief 私有构造函数，仅用于内部创建回调实例 / Private constructor used internally to
   * create callback instances
   *
   * @param cb_block 回调块对象指针 / Pointer to the callback block
   */
  explicit Callback(CallbackBlockHeader<Args...>* cb_block)
      : cb_block_((cb_block != nullptr) ? cb_block : &empty_cb_block_)
  {
  }

  CallbackBlockHeader<Args...>* cb_block_ =
      &empty_cb_block_;  ///< 回调块指针 / Pointer to the callback block
};

}  // namespace LibXR
