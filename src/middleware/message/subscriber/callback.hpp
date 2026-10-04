#pragma once

#include <new>

#include "../topic.hpp"
#include "libxr_def.hpp"

namespace LibXR
{
/**
 * @class Topic::Callback
 * @brief 每次发布时直接执行函数的订阅句柄 / Subscription handle that runs a function on
 *        each publish
 */
class Topic::Callback
{
  /**
   * @struct FunctionTraits
   * @brief 把一个无捕获回调函数拆成返回类型和参数类型 / Split one callback function
   *        without captures into its return type and parameter types
   * @tparam Function 回调函数类型 / Callback function type
   */
  template <typename Function>
  struct FunctionTraits;

  /**
   * @struct FunctionTraits<Return (*)(Args...)>
   * @brief 普通函数指针版本的签名提取器 / Signature extractor specialization for plain
   *        function pointers
   * @tparam Return 回调返回类型 / Callback return type
   * @tparam Args 回调参数列表 / Callback argument list
   */
  template <typename Return, typename... Args>
  struct FunctionTraits<Return (*)(Args...)>
  {
    using ReturnType = Return;  ///< 回调返回类型 / Callback return type.
    static constexpr size_t ARITY = sizeof...(Args);  ///< 参数个数 / Number of arguments.

    /**
     * @brief 取出指定下标的参数类型 / Fetch the argument type at the given index
     * @tparam Index 参数下标 / Argument index
     */
    template <size_t Index>
    using Arg = std::tuple_element_t<Index, std::tuple<Args...>>;
  };

  /**
   * @struct MessageViewTraits
   * @brief 判断某个参数是否为 `MessageView<T>` / Tell whether one argument is
   *        `MessageView<T>`
   * @tparam T 待判断的参数类型 / Argument type to inspect
   */
  template <typename T>
  struct MessageViewTraits
  {
    static constexpr bool VALUE =
        false;  ///< 默认不是 `MessageView` / Not a `MessageView` by default.
  };

  /**
   * @struct MessageViewTraits<MessageView<Data>>
   * @brief 识别 `MessageView<T>` 并取出里面的数据类型 / Detect `MessageView<T>` and
   *        expose its payload type
   * @tparam Data `MessageView` 内部承载的数据类型 / Payload type carried by the
   *        `MessageView`
   */
  template <typename Data>
  struct MessageViewTraits<MessageView<Data>>
  {
    static constexpr bool VALUE =
        true;  ///< 该参数是 `MessageView` / This argument is a `MessageView`.
    using DataType =
        Data;  ///< `MessageView` 对应的数据类型 / Payload type behind the `MessageView`.
  };

  /**
   * @brief 去掉引用和 cv 限定后的规范类型 / Canonical type after removing reference and
   *        cv qualifiers
   * @tparam T 待规范化的类型 / Type to normalize
   */
  template <typename T>
  using RemoveCVRef = std::remove_cv_t<std::remove_reference_t<T>>;

  /**
   * @brief 是否为带时间戳的类型化消息视图参数 / Whether one callback payload argument
   *        is a timestamped typed message view
   * @tparam T 待判断的参数类型 / Argument type to inspect
   */
  template <typename T>
  static constexpr bool IS_MESSAGE_VIEW =
      MessageViewTraits<RemoveCVRef<T>>::VALUE;  ///< 是否为带时间戳的类型化消息视图参数
                                                 ///< Whether the argument is a
                                                 ///< timestamped typed message view.

  /**
   * @brief 是否为只读 raw payload 视图参数 / Whether one callback payload argument is a
   *        read-only raw payload view
   * @tparam T 待判断的参数类型 / Argument type to inspect
   */
  template <typename T>
  static constexpr bool IS_RAW_DATA_VIEW = std::same_as<RemoveCVRef<T>, ConstRawData> ||
                                           std::same_as<RemoveCVRef<T>, RawMessageView>;

  /**
   * @brief 是否把可写 `RawData` 当成 payload 参数 / Whether one callback payload
   *        argument is the writable `RawData` descriptor
   * @tparam T 待判断的参数类型 / Argument type to inspect
   */
  template <typename T>
  static constexpr bool IS_RAW_DATA = std::same_as<RemoveCVRef<T>, RawData>;

  /**
   * @brief 是否把 payload 直接当成一个对象接收 / Whether one callback payload argument
   *        receives the payload directly as one object
   * @tparam T 待判断的参数类型 / Argument type to inspect
   */
  template <typename T>
  static constexpr bool IS_TYPED_DATA =
      !IS_MESSAGE_VIEW<T> && !IS_RAW_DATA_VIEW<T> &&
      TopicPayload<RemoveCVRef<T>>;  ///< 是否把负载直接当成一个对象来接收 / Whether the
                                     ///< payload is received directly as one typed
                                     ///< object.

  /**
   * @brief 是否是回调接口允许的 payload 参数写法 / Whether one payload argument form is
   *        accepted by this callback interface
   * @tparam T 待判断的参数类型 / Argument type to inspect
   */
  template <typename T>
  static constexpr bool IS_CALLBACK_PAYLOAD =
      IS_MESSAGE_VIEW<T> || IS_RAW_DATA_VIEW<T> ||
      IS_TYPED_DATA<T>;  ///< 是否是这个回调接口允许的负载参数写法 / Whether this callback
                         ///< interface accepts this payload argument form.

  /**
   * @brief 是否错误地使用了可变 `MessageView<T>&` / Whether one payload argument is an
   *        invalid mutable `MessageView<T>&`
   * @tparam T 待判断的参数类型 / Argument type to inspect
   */
  template <typename T>
  static constexpr bool IS_MUTABLE_MESSAGE_VIEW_REF =
      IS_MESSAGE_VIEW<T> && std::is_lvalue_reference_v<T> &&
      !std::is_const_v<std::remove_reference_t<T>>;

  /**
   * @struct BlockHeader
   * @brief 回调句柄真正指向的那块数据的公共头 / Common header of the data block actually
   *        pointed to by one callback handle
   *
   * 一个 `Callback` 最终只保存一个 `BlockHeader*`。真正的回调函数、绑定参数和负载类型
   * 都放在派生块里；发布路径先看 `payload_type_id` 做契约检查，再通过 `run`
   * 调到那块具体数据。
   * One `Callback` finally stores only one `BlockHeader*`. The real callback
   * function, bound argument, and expected payload type live in derived blocks;
   * the publish path first checks `payload_type_id` and then jumps through `run`
   * into that concrete block.
   */
  struct BlockHeader
  {
    using RunFun = void (*)(const BlockHeader*, bool, MicrosecondTimestamp, void*,
                            size_t);  ///< 所有具体回调块共用的执行入口 / Shared execution
                                      ///< entry used by all concrete callback blocks.

    RunFun run = nullptr;  ///< 当前块的执行函数 / Execution function of the current
                           ///< block.
    TypeID::ID payload_type_id = nullptr;  ///< 该回调期望的主题负载类型标识 / Expected
                                           ///< topic payload type identifier.
    bool accepts_raw_payload = false;      ///< 是否按 raw payload 视图接收 / Whether this
                                           ///< callback receives a raw payload view.
  };

  /**
   * @brief 取回调 payload 参数对应的精确类型标识 / Get the exact type ID expected by
   *        one callback payload argument
   * @tparam PayloadArg 回调 payload 参数类型 / Callback payload argument type
   * @return 对应的精确类型标识 / Exact type ID expected by this callback payload argument
   */
  template <typename PayloadArg>
  static TypeID::ID PayloadTypeID()
  {
    static_assert(!IS_MUTABLE_MESSAGE_VIEW_REF<PayloadArg>,
                  "LibXR::Topic::Callback does not accept MessageView<T>&; "
                  "use MessageView<T> or const MessageView<T>& instead.");
    static_assert(!IS_RAW_DATA<PayloadArg>,
                  "LibXR::Topic::Callback does not accept RawData as the payload; "
                  "use the topic's payload type T (keeps the type check), or "
                  "ConstRawData for raw bytes.");

    if constexpr (IS_RAW_DATA_VIEW<PayloadArg>)
    {
      return nullptr;
    }
    else if constexpr (IS_MESSAGE_VIEW<PayloadArg>)
    {
      using View = MessageViewTraits<RemoveCVRef<PayloadArg>>;
      return TypeID::GetID<typename View::DataType>();
    }
    else
    {
      static_assert(IS_TYPED_DATA<PayloadArg>,
                    "LibXR::Topic::Callback payload must be Topic::MessageView<T>, "
                    "Topic::RawMessageView, ConstRawData, T, T&, or const T&.");
      return TypeID::GetID<RemoveCVRef<PayloadArg>>();
    }
  }

  /**
   * @brief 回调 payload 参数是否是 raw 字节视图 / Whether the callback payload
   *        argument is a raw byte view
   * @tparam PayloadArg 回调 payload 参数类型 / Callback payload argument type
   * @return 是 raw 字节视图则返回 true / Returns true for raw byte view payloads
   */
  template <typename PayloadArg>
  static constexpr bool AcceptsRawPayload()
  {
    return IS_RAW_DATA_VIEW<PayloadArg>;
  }

  /**
   * @brief 按回调声明的参数形式，把当前消息拼成实参再调函数 / Build the actual call
   *        arguments from the current message in the form declared by the
   *        callback, then invoke it
   * @tparam Function 回调函数类型 / Callback function type
   * @tparam BoundArg 绑定参数类型 / Bound argument type
   * @tparam PayloadArg 负载参数类型 / Payload argument type
   * @param fun 回调函数 / Callback function
   * @param arg 绑定参数 / Bound argument
   * @param in_isr 当前是否处于中断上下文 / Whether the current path runs in ISR context
   * @param timestamp 当前消息时间戳 / Current message timestamp
   * @param payload_addr 当前消息 payload 对象地址 / Address of the current payload
   *        object
   */
  template <typename Function, typename BoundArg, typename PayloadArg>
  static void InvokePayload(Function fun, BoundArg& arg, bool in_isr,
                            MicrosecondTimestamp timestamp, void* payload_addr,
                            size_t payload_size)
  {
    if constexpr (std::same_as<RemoveCVRef<PayloadArg>, ConstRawData>)
    {
      ConstRawData raw_payload{payload_addr, payload_size};
      fun(in_isr, arg, raw_payload);
    }
    else if constexpr (std::same_as<RemoveCVRef<PayloadArg>, RawMessageView>)
    {
      RawMessageView raw_message{timestamp, ConstRawData{payload_addr, payload_size}};
      fun(in_isr, arg, raw_message);
    }
    else if constexpr (IS_MESSAGE_VIEW<PayloadArg>)
    {
      using View = MessageViewTraits<RemoveCVRef<PayloadArg>>;
      using Data = typename View::DataType;
      fun(in_isr, arg,
          MessageView<Data>{timestamp, reinterpret_cast<Data*>(payload_addr)});
    }
    else
    {
      using Data = RemoveCVRef<PayloadArg>;
      fun(in_isr, arg, *reinterpret_cast<Data*>(payload_addr));
    }
  }

  /**
   * @struct PayloadOnlyBlock
   * @brief 只转发负载的回调块 / Callback block that forwards only the payload
   * @tparam Function 回调函数类型 / Callback function type
   * @tparam BoundArg 绑定参数类型 / Bound argument type
   * @tparam PayloadArg 负载参数类型 / Payload argument type
   */
  template <typename Function, typename BoundArg, typename PayloadArg>
  struct PayloadOnlyBlock : public BlockHeader
  {
    /**
     * @brief 构造一个只转发负载的具体回调块 / Construct one concrete callback block that
     *        forwards only the payload
     * @param fun 目标回调函数 / Target callback function
     * @param arg 绑定到回调中的参数 / Argument bound into the callback
     */
    PayloadOnlyBlock(Function fun, BoundArg&& arg)
        : BlockHeader{&Run, PayloadTypeID<PayloadArg>(), AcceptsRawPayload<PayloadArg>()},
          fun_(fun),
          arg_(std::move(arg))
    {
    }

    /**
     * @brief 根据块头指针取回具体块并执行回调 / Recover the concrete block from the block
     *        header pointer and run the callback
     * @param header 回调块头指针 / Callback block header pointer
     * @param in_isr 当前是否处于中断上下文 / Whether the current path runs in ISR context
     * @param timestamp 当前消息时间戳 / Current message timestamp
     * @param payload_addr 当前消息 payload 对象地址 / Address of the current payload
     *        object
     */
    static void Run(const BlockHeader* header, bool in_isr,
                    MicrosecondTimestamp timestamp, void* payload_addr,
                    size_t payload_size)
    {
      auto* block = static_cast<const PayloadOnlyBlock*>(header);
      InvokePayload<Function, BoundArg, PayloadArg>(
          block->fun_, const_cast<BoundArg&>(block->arg_), in_isr, timestamp,
          payload_addr, payload_size);
    }

    Function fun_;  ///< 目标回调函数 / Target callback function.
    BoundArg arg_;  ///< 绑定到回调中的参数副本 / Stored copy of the bound argument.
  };

  /**
   * @struct TimestampPayloadBlock
   * @brief 同时转发时间戳和负载的回调块 / Callback block that forwards both timestamp and
   *        payload
   * @tparam Function 回调函数类型 / Callback function type
   * @tparam BoundArg 绑定参数类型 / Bound argument type
   * @tparam PayloadArg 负载参数类型 / Payload argument type
   */
  template <typename Function, typename BoundArg, typename PayloadArg>
  struct TimestampPayloadBlock : public BlockHeader
  {
    /**
     * @brief 构造一个同时转发时间戳和负载的具体回调块 / Construct one concrete callback
     *        block that forwards both timestamp and payload
     * @param fun 目标回调函数 / Target callback function
     * @param arg 绑定到回调中的参数 / Argument bound into the callback
     */
    TimestampPayloadBlock(Function fun, BoundArg&& arg)
        : BlockHeader{&Run, PayloadTypeID<PayloadArg>(), AcceptsRawPayload<PayloadArg>()},
          fun_(fun),
          arg_(std::move(arg))
    {
    }

    /**
     * @brief 根据块头指针取回具体块并执行带时间戳回调 / Recover the concrete block from
     *        the block header pointer and run the timestamped callback
     * @param header 回调块头指针 / Callback block header pointer
     * @param in_isr 当前是否处于中断上下文 / Whether the current path runs in ISR context
     * @param timestamp 当前消息时间戳 / Current message timestamp
     * @param payload_addr 当前消息 payload 对象地址 / Address of the current payload
     *        object
     */
    static void Run(const BlockHeader* header, bool in_isr,
                    MicrosecondTimestamp timestamp, void* payload_addr,
                    size_t payload_size)
    {
      auto* block = static_cast<const TimestampPayloadBlock*>(header);
      if constexpr (std::same_as<RemoveCVRef<PayloadArg>, ConstRawData>)
      {
        ConstRawData raw_payload{payload_addr, payload_size};
        block->fun_(in_isr, block->arg_, timestamp, raw_payload);
      }
      else if constexpr (std::same_as<RemoveCVRef<PayloadArg>, RawMessageView>)
      {
        RawMessageView raw_message{timestamp, ConstRawData{payload_addr, payload_size}};
        block->fun_(in_isr, block->arg_, timestamp, raw_message);
      }
      else if constexpr (IS_MESSAGE_VIEW<PayloadArg>)
      {
        using View = MessageViewTraits<RemoveCVRef<PayloadArg>>;
        using Data = typename View::DataType;
        block->fun_(in_isr, block->arg_, timestamp,
                    MessageView<Data>{timestamp, reinterpret_cast<Data*>(payload_addr)});
      }
      else
      {
        using Data = RemoveCVRef<PayloadArg>;
        block->fun_(in_isr, block->arg_, timestamp,
                    *reinterpret_cast<Data*>(payload_addr));
      }
    }

    Function fun_;  ///< 目标回调函数 / Target callback function.
    BoundArg arg_;  ///< 绑定到回调中的参数副本 / Stored copy of the bound argument.
  };

  /**
   * @struct Factory
   * @brief 看回调有几个参数，决定分配哪种回调块 / Choose which callback block to
   *        allocate from the callback parameter count
   * @tparam Function 回调函数类型 / Callback function type
   * @tparam BoundArg 绑定参数类型 / Bound argument type
   * @tparam Arity 回调参数个数 / Number of callback arguments
   */
  template <typename Function, typename BoundArg, size_t Arity>
  struct Factory
  {
    static_assert(Arity == 3 || Arity == 4,
                  "LibXR::Topic::Callback function must be void(bool, Arg, Payload) "
                  "or void(bool, Arg, MicrosecondTimestamp, Payload).");
  };

  /**
   * @struct Factory<Function, BoundArg, 3>
   * @brief 三参数回调块工厂 / Factory for three-argument callback blocks
   * @tparam Function 回调函数类型 / Callback function type
   * @tparam BoundArg 绑定参数类型 / Bound argument type
   */
  template <typename Function, typename BoundArg>
  struct Factory<Function, BoundArg, 3>
  {
    using Traits = FunctionTraits<Function>;  ///< 回调签名信息 / Callback signature info.
    using PayloadArg = typename Traits::template Arg<2>;  ///< 第三个参数，即负载参数类型
                                                          ///< Third argument, namely the
                                                          ///< payload argument type.

    /**
     * @brief 创建只转发负载的回调块 / Create one callback block that forwards only the
     *        payload
     * @param fun 目标回调函数 / Target callback function
     * @param arg 绑定到回调中的参数 / Argument bound into the callback
     * @return 新分配的回调块头指针 / Returns the newly allocated callback block header
     */
    static BlockHeader* Create(Function fun, BoundArg&& arg)
    {
      static_assert(std::same_as<typename Traits::ReturnType, void>);
      static_assert(std::same_as<typename Traits::template Arg<0>, bool>);
      static_assert(std::same_as<typename Traits::template Arg<1>, BoundArg>);
      static_assert(IS_CALLBACK_PAYLOAD<PayloadArg>);
      return new PayloadOnlyBlock<Function, BoundArg, PayloadArg>(fun, std::move(arg));
    }
  };

  /**
   * @struct Factory<Function, BoundArg, 4>
   * @brief 四参数回调块工厂 / Factory for four-argument callback blocks
   * @tparam Function 回调函数类型 / Callback function type
   * @tparam BoundArg 绑定参数类型 / Bound argument type
   */
  template <typename Function, typename BoundArg>
  struct Factory<Function, BoundArg, 4>
  {
    using Traits = FunctionTraits<Function>;  ///< 回调签名信息 / Callback signature info.
    using TimestampArg =
        typename Traits::template Arg<2>;  ///< 第三个参数，即时间戳参数类型 / Third
                                           ///< argument, namely the
                                           ///< timestamp argument type.
    using PayloadArg = typename Traits::template Arg<3>;  ///< 第四个参数，即负载参数类型
                                                          ///< Fourth argument, namely the
                                                          ///< payload argument type.

    /**
     * @brief 创建同时转发时间戳和负载的回调块 / Create one callback block that forwards
     *        both timestamp and payload
     * @param fun 目标回调函数 / Target callback function
     * @param arg 绑定到回调中的参数 / Argument bound into the callback
     * @return 新分配的回调块头指针 / Returns the newly allocated callback block header
     */
    static BlockHeader* Create(Function fun, BoundArg&& arg)
    {
      static_assert(std::same_as<typename Traits::ReturnType, void>);
      static_assert(std::same_as<typename Traits::template Arg<0>, bool>);
      static_assert(std::same_as<typename Traits::template Arg<1>, BoundArg>);
      static_assert(std::same_as<RemoveCVRef<TimestampArg>, MicrosecondTimestamp>);
      static_assert(IS_CALLBACK_PAYLOAD<PayloadArg>);
      return new TimestampPayloadBlock<Function, BoundArg, PayloadArg>(fun,
                                                                       std::move(arg));
    }
  };

  /**
   * @brief 空回调块的执行函数 / Execution function of the empty callback block
   * @param header 空块头指针 / Empty block header pointer
   * @param in_isr 当前是否处于中断上下文 / Whether the current path runs in ISR context
   * @param timestamp 当前消息时间戳 / Current message timestamp
   * @param payload_addr 当前消息 payload 对象地址 / Address of the current payload
   *        object
   */
  static void EmptyRun(const BlockHeader* header, bool in_isr,
                       MicrosecondTimestamp timestamp, void* payload_addr,
                       size_t payload_size)
  {
    UNUSED(header);
    UNUSED(in_isr);
    UNUSED(timestamp);
    UNUSED(payload_addr);
    UNUSED(payload_size);
  }

  inline static BlockHeader empty_block_{
      &EmptyRun, nullptr, false};  ///< 空回调句柄共用的静态空执行块 / Shared static empty
                                   ///< execution block used by empty callback handles.

 public:
  /**
   * @brief 构造一个空回调句柄 / Construct one empty callback handle
   */
  Callback() = default;

  /**
   * @brief 拷贝构造回调句柄 / Copy-construct one callback handle
   * @param other 被拷贝的回调句柄 / Callback handle to copy from
   * @note 这里只复制句柄本身，不复制底层回调块；两个句柄会指向同一块回调数据 /
   *       This copies only the handle itself, not the underlying callback block;
   *       both handles point to the same callback data block
   */
  Callback(const Callback& other) = default;

  /**
   * @brief 拷贝赋值回调句柄 / Copy-assign one callback handle
   * @param other 被拷贝的回调句柄 / Callback handle to copy from
   * @return 当前回调句柄 / Returns the current callback handle
   * @note 这里只复制句柄本身，不复制底层回调块；赋值后两个句柄会指向同一块回调数据 /
   *       This copies only the handle itself, not the underlying callback block;
   *       after assignment both handles point to the same callback data block
   */
  Callback& operator=(const Callback& other) = default;

  /**
   * @brief 创建一个回调订阅句柄 / Create a callback subscription handle
   * @tparam BoundArg 绑定参数类型 / Bound argument type
   * @tparam Callable 回调可调用对象类型 / Callback callable type
   * @param fun 捕获为空的回调函数 / Capture-free callback function
   * @param arg 绑定到回调的参数 / Argument bound into the callback
   * @return 创建后的回调订阅句柄 / Returns the created callback subscription handle
   * @note 这里会把 `arg` 拷进新分配的回调块里；之后回调执行读写的就是这份块内副本 /
   *       This copies `arg` into the newly allocated callback block; later
   *       callback execution reads or writes that stored copy
   */
  template <typename BoundArg, typename Callable>
  [[nodiscard]] static Callback Create(Callable fun, BoundArg arg)
  {
    using Function = decltype(+std::declval<Callable>());
    using Traits = FunctionTraits<Function>;
    static_assert(Traits::ARITY == 3 || Traits::ARITY == 4,
                  "LibXR::Topic::Callback::Create expects a capture-free callable "
                  "with bool and bound-argument parameters.");
    return Callback(
        Factory<Function, BoundArg, Traits::ARITY>::Create(+fun, std::move(arg)));
  }

  /**
   * @brief 执行回调 / Run the callback
   * @param in_isr 当前是否处于中断上下文 / Whether the current path runs in ISR context
   * @param timestamp 当前消息时间戳 / Current message timestamp
   * @param payload_addr 当前消息 payload 对象地址 / Address of the current payload
   *        object
   */
  void Run(bool in_isr, MicrosecondTimestamp timestamp, void* payload_addr,
           size_t payload_size) const
  {
    block_->run(block_, in_isr, timestamp, payload_addr, payload_size);
  }

  /**
   * @brief 读取这个回调句柄期望的 payload 类型标识 / Read the payload type ID expected
   *        by this callback handle
   * @return payload 类型标识 / Payload type ID
   */
  TypeID::ID PayloadTypeID() const { return block_->payload_type_id; }

  /**
   * @brief 判断该回调是否按 raw payload 视图接收消息 / Tell whether this callback
   *        receives messages as a raw payload view
   * @return 是 raw payload 视图回调则返回 true / Returns true for raw payload view
   * callbacks
   */
  bool IsRawPayloadView() const { return block_->accepts_raw_payload; }

 private:
  /**
   * @brief 用指定执行块构造回调句柄 / Construct one callback handle from the given
   *        execution block
   * @param block 回调块公共头；为空时回退到静态空块 / Callback block header; falls back
   *        to the shared empty block when null
   */
  explicit Callback(BlockHeader* block) : block_(block ? block : &empty_block_) {}

  BlockHeader* block_ = &empty_block_;  ///< 当前回调句柄绑定的执行块 / Execution block
                                        ///< bound to the current callback handle.
};

/**
 * @struct Topic::CallbackBlock
 * @brief 挂在 topic 订阅链表里的回调记录 / Callback record stored in the topic subscriber
 *        list
 */
struct Topic::CallbackBlock : public Topic::SuberBlock
{
  /**
   * @brief 构造一个回调订阅块 / Construct one callback subscriber block
   * @param callback 要挂接的回调句柄 / Callback handle to attach
   * @param topic_payload_size 当前 topic 的固定 payload 字节数 / Fixed payload
   *        size of the topic this callback is registered to
   */
  CallbackBlock(Callback& callback, size_t topic_payload_size)
      : cb(callback), payload_size(topic_payload_size)
  {
    type = SuberType::CALLBACK;
  }

  /**
   * @brief 通过注册时绑定的 topic payload size 执行回调 / Run the callback with the
   *        topic payload size bound at registration time
   * @param in_isr 当前是否处于中断上下文 / Whether the current path runs in ISR context
   * @param timestamp 当前消息时间戳 / Current message timestamp
   * @param payload_addr 当前消息 payload 对象地址 / Address of the current payload object
   */
  void Run(bool in_isr, MicrosecondTimestamp timestamp, void* payload_addr) const
  {
    cb.Run(in_isr, timestamp, payload_addr, payload_size);
  }

  Callback cb;  ///< 订阅的回调句柄 / Subscribed callback handle.

  /// 注册到的 topic 固定 payload 字节数 / Fixed payload size of the subscribed topic.
  size_t payload_size = 0;
};

/**
 * @brief 注册回调函数 / Register a callback function
 * @param cb 需要注册的回调函数 / Callback function to register
 *
 * @note 包含初始化期动态内存分配，回调订阅应长期存在 / Contains initialization-time
 * dynamic allocation; callback subscriptions are expected to be long-lived
 * @note 回调在发布锁内运行，不应重入发布同一个主题 / The callback runs while the publish
 * lock is held and should not re-enter publishing on the same topic
 * @note 类型化回调的 payload 类型与 topic 不一致时，在所有构建类型中进入致命错误 /
 * A typed callback whose payload type differs from the topic raises a fatal error in
 * every build type
 */
inline void Topic::RegisterCallback(Callback& cb)
{
  if (!cb.IsRawPayloadView())
  {
    RequireCallbackType(block_, cb.PayloadTypeID());
  }

  auto node = new (std::align_val_t(LibXR::CONCURRENCY_ALIGNMENT))
      LockFreeList::Node<CallbackBlock>(cb, block_->data_.payload_size);
  block_->data_.subers.Add(*node);
}
}  // namespace LibXR
