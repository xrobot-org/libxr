#pragma once

#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "lockfree_list.hpp"
#include "rbt.hpp"

namespace LibXR
{

/**
 * @class Event
 * @brief 事件管理系统，允许基于事件 ID 注册和触发回调函数。
 *        Event management system that allows registration and triggering of callbacks
 * based on event IDs.
 */
class Event
{
 public:
  using Callback = LibXR::Callback<uint32_t>;

  /**
   * @brief 回调链表指针类型，用于事件触发时从 ISR 中安全调用。
   *        Pointer to the callback list, safe to use in ISR after acquired in
   * non-interrupt context.
   */
  using CallbackList = LockFreeList*;

  /**
   * @brief 构造函数，初始化用于存储事件的红黑树。
   *        Constructs an Event object with an empty red-black tree for event storage.
   */
  Event();

  /**
   * @brief 为特定事件注册回调函数。
   *        Registers a callback function for a specific event.
   * @param event 要注册回调的事件 ID / The event ID to register the callback for.
   * @param cb    事件触发时执行的回调函数。
   * The callback function to be executed when the event occurs.
   *
   * @note 包含动态内存分配。
   *       Contains dynamic memory allocation.
   */
  void Register(uint32_t event, const Callback& cb);

  /**
   * @brief 触发与特定事件关联的所有回调函数（非中断上下文）。
   *        Triggers all callbacks associated with a specific event (non-interrupt
   * context).
   * @param event 要激活的事件 ID / The event ID to activate.
   */
  void Active(uint32_t event);

  /**
   * @brief 从 callback-safe 路径触发与特定事件关联的所有回调函数。
   *        Triggers all callbacks associated with a specific event from a callback-safe
   * path.
   *
   * @param list 在非回调函数中获取的事件回调链表指针。
   * The event callback list pointer obtained from the non-callback function.
   * @param event 要激活的事件 ID / The event ID to activate.
   * @param in_isr 当前 callback-safe 路径是否实际位于 ISR。
   * Whether the current callback-safe path is actually in ISR context.
   *
   * @note 默认值为 true，用于兼容旧代码中“FromCallback 等价于 ISR=true”的调用习惯。
   *       The default remains true for backward compatibility with older callers that
   * treated FromCallback as ISR=true.
   */
  void ActiveFromCallback(CallbackList list, uint32_t event, bool in_isr = true);

  /**
   * @brief 获取指定事件的回调链表指针（必须在非中断上下文中调用）。
   *        Returns the callback list pointer for the given event (must be called outside
   * ISR).
   * @param event 要查询的事件 ID / The event ID to search.
   * @return 回调链表指针，如果未注册则主动创建。
   * The callback list pointer, if not registered, it is actively created.
   * @note 当前 Event 只支持“查找或创建”回调链表，不提供删除或替换某个事件链表的接口；
   *       因此在 Event 对象存活期间，这个函数返回的链表指针保持稳定。
   *       The current Event API only supports finding or creating a callback
   *       list; it does not provide any way to remove or replace one event's
   *       list, so the returned pointer stays stable for the lifetime of the
   *       Event object.
   */
  CallbackList GetList(uint32_t event);

  /**
   * @brief 将源事件绑定到当前事件实例中的目标事件。
   *        Binds an event from a source Event instance to a target event in the current
   * instance.
   * @param sources       包含原始事件的源 Event 实例 / The source Event instance.
   * @param source_event  源事件实例中的事件 ID / The source event ID.
   * @param target_event  当前实例中的目标事件 ID / The target event ID in this instance.
   *
   * @note 包含动态内存分配。
   *       Contains dynamic memory allocation.
   * @note 当前实现会在绑定时预先拿到目标事件的回调链表指针，因此后续 callback-safe
   *       触发路径不需要再为这个绑定做惰性查找或分配。
   *       The current implementation acquires the target event's callback-list
   *       pointer during binding, so later callback-safe activation does not
   *       need an extra lazy lookup or allocation for this binding.
   * @note 这依赖 `GetList()` 的稳定性约束：当前 Event 不支持删除或替换某个事件的回调
   *       链表，因此绑定后缓存的目标链表指针会一直指向同一个链表，直到目标 Event 对象
   *       自身结束生命周期。
   *       This relies on `GetList()` stability: Event does not support removing
   *       or replacing one event's callback list, so the cached target-list
   *       pointer keeps referring to the same list until the target Event
   *       object itself reaches the end of its lifetime.
   */
  void Bind(Event& sources, uint32_t source_event, uint32_t target_event);

 private:
  /**
   * @struct Block
   * @brief 用于存储事件回调的数据结构。
   *        Data structure for storing event callbacks.
   */
  struct Block
  {
    uint32_t event;  ///< 与该回调关联的事件 ID / Event ID associated with this callback.
    Callback
        cb;  ///< 关联该事件的回调函数 / Callback function associated with this event.
  };

  RBTree<uint32_t> rbt_;  ///< 用于管理已注册事件的红黑树 / Red-black tree for managing
                          ///< registered events.
};

}  // namespace LibXR
