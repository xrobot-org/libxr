#include <ti/devices/msp/msp.h>

static inline uint32_t atomic_enter(void)
{
  uint32_t primask_state = __get_PRIMASK();
  __disable_irq();
  return primask_state;
}

static inline void atomic_exit(uint32_t primask_state)
{
  if (!primask_state)
  {
    __enable_irq();
  }
}

#define UNUSED(x) (void)(x)

/**
 * @brief 提供单核 32 位强比较交换 / Provide single-core 32-bit strong compare-exchange.
 * @param ptr 目标字地址 / Target word address.
 * @param expected 预期值地址，失败时写回实际值 / Expected value, updated on failure.
 * @param desired 成功时写入的新值 / New value on success.
 * @param success_memorder 成功时内存顺序 / Success memory order.
 * @param failure_memorder 失败时内存顺序 / Failure memory order.
 * @return 比较相等时返回 true / True when the comparison matches.
 * @note GCC 定长运行时 ABI 为五个参数，没有 weak 参数；沿用单核中断保护。
 *       GCC 的内建声明带 weak 参数，与运行时 ABI 不同，因此只在这里关闭该诊断。
 *       The sized GCC runtime ABI has five arguments, without weak; uses the IRQ guard.
 *       GCC's built-in declaration has the weak argument, unlike the runtime ABI, so the
 *       diagnostic is turned off for this definition only.
 */
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wbuiltin-declaration-mismatch"
#endif
__attribute__((weak)) _Bool __atomic_compare_exchange_4(volatile void* ptr,
                                                        void* expected,
                                                        unsigned int desired,  // NOLINT
                                                        int success_memorder,
                                                        int failure_memorder)
{
  UNUSED(success_memorder);
  UNUSED(failure_memorder);

  volatile unsigned int* addr = (volatile unsigned int*)ptr;
  unsigned int expected_val = *(unsigned int*)expected;
  _Bool result = false;

  // 2. 使用安全的方式开关中断
  uint32_t primask = atomic_enter();

  if (*addr == expected_val)
  {
    *addr = desired;
    result = true;
  }
  else
  {
    *(unsigned int*)expected = *addr;
    result = false;
  }

  atomic_exit(primask);  // 恢复状态
  return result;
}
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif

/**
 * @brief 原子置位并返回原值 / Atomically set bits and return the previous value.
 * @param ptr 目标字地址 / Target word address.
 * @param val 待置位的掩码 / Bits to set.
 * @param memorder 内存顺序，本实现使用现有单核临界区 / Ordering; uses the single-core
 * guard.
 * @return 更新前的值 / Value before the update.
 */
__attribute__((weak)) unsigned int __atomic_fetch_or_4(volatile void* ptr,
                                                       unsigned int val, int memorder)
{
  UNUSED(memorder);
  volatile unsigned int* addr = (volatile unsigned int*)ptr;
  const uint32_t primask = atomic_enter();
  const unsigned int previous = *addr;
  *addr = previous | val;
  atomic_exit(primask);
  return previous;
}

/**
 * @brief  模拟实现 __atomic_store_4 函数 / Simulate the __atomic_store_4 function
 * @param  ptr 指向原子变量的指针 / Pointer to the atomic variable
 * @param  val 需要存储的新值 / The new value to be stored
 * @param  memorder 内存顺序标志 / Memory order (ignored)
 * @retval 无返回值 / None
 */
__attribute__((weak)) void __atomic_store_4(volatile void* ptr, unsigned int val,
                                            int memorder)  // NOLINT
{
  UNUSED(memorder);
  volatile unsigned int* addr = (volatile unsigned int*)ptr;

  uint32_t primask = atomic_enter();
  *addr = val;
  atomic_exit(primask);
}

/**
 * @brief  模拟实现 __atomic_load_4 函数 / Simulate the __atomic_load_4 function
 * @param  ptr 指向原子变量的指针 / Pointer to the atomic variable
 * @param  memorder 内存顺序标志 / Memory order (ignored)
 * @retval 当前值 / Returns the current value
 */
__attribute__((weak)) unsigned int __atomic_load_4(const volatile void* ptr,
                                                   int memorder)  // NOLINT
{
  UNUSED(memorder);
  volatile unsigned int* addr = (volatile unsigned int*)ptr;
  unsigned int val = 0;

  uint32_t primask = atomic_enter();
  val = *addr;
  atomic_exit(primask);
  return val;
}

/**
 * @brief  模拟实现 __atomic_exchange_4 函数 / Simulate the __atomic_exchange_4 function
 * @param  ptr 指向原子变量的指针 / Pointer to the atomic variable
 * @param  val 需要存储的新值 / The new value to be stored
 * @param  memorder 内存顺序标志 / Memory order (ignored)
 * @retval 交换前的旧值 / Returns the old value before exchange
 */
__attribute__((weak)) unsigned int __atomic_exchange_4(volatile void* ptr,
                                                       unsigned int val,
                                                       int memorder)  // NOLINT
{
  UNUSED(memorder);
  volatile unsigned int* addr = (volatile unsigned int*)ptr;
  unsigned int old_val = 0;

  uint32_t primask = atomic_enter();
  old_val = *addr;
  *addr = val;
  atomic_exit(primask);
  return old_val;
}

/**
 * @brief  模拟实现 __atomic_fetch_add_4 函数 / Simulate the __atomic_fetch_add_4 function
 * @param  ptr 指向原子变量的指针 / Pointer to the atomic variable
 * @param  val 需要相加的值 / The value to add
 * @param  memorder 内存顺序标志 / Memory order (ignored)
 * @retval 加法前的旧值 / Returns the old value before addition
 */
__attribute__((weak)) unsigned int __atomic_fetch_add_4(volatile void* ptr,
                                                        unsigned int val,
                                                        int memorder)  // NOLINT
{
  UNUSED(memorder);
  volatile unsigned int* addr = (volatile unsigned int*)ptr;
  unsigned int old_val = 0;

  uint32_t primask = atomic_enter();
  old_val = *addr;
  *addr = old_val + val;
  atomic_exit(primask);
  return old_val;
}

/**
 * @brief  模拟实现 __atomic_fetch_sub_4 函数 / Simulate the __atomic_fetch_sub_4 function
 * @param  ptr 指向原子变量的指针 / Pointer to the atomic variable
 * @param  val 需要相减的值 / The value to subtract
 * @param  memorder 内存顺序标志 / Memory order (ignored)
 * @retval 减法前的旧值 / Returns the old value before subtraction
 */
__attribute__((weak)) unsigned int __atomic_fetch_sub_4(volatile void* ptr,
                                                        unsigned int val,
                                                        int memorder)  // NOLINT
{
  UNUSED(memorder);
  volatile unsigned int* addr = (volatile unsigned int*)ptr;
  unsigned int old_val = 0;

  uint32_t primask = atomic_enter();
  old_val = *addr;
  *addr = old_val - val;
  atomic_exit(primask);
  return old_val;
}

/**
 * @brief  模拟实现 __atomic_exchange_1 函数 / Simulate the __atomic_exchange_1 function
 * @param  ptr 指向原子变量（1字节大小）的指针 / Pointer to the 1-byte atomic variable
 * @param  val 需要存储的新值 / The new value to be stored
 * @param  memorder 内存顺序标志（忽略） / Memory order (ignored)
 * @retval 交换前的旧值 / Returns the old value before exchange
 */
__attribute__((weak)) unsigned char __atomic_exchange_1(volatile void* ptr,
                                                        unsigned char val,
                                                        int memorder)  // NOLINT
{
  UNUSED(memorder);
  volatile unsigned char* addr = (volatile unsigned char*)ptr;
  unsigned char old_val = 0;

  uint32_t primask = atomic_enter();
  old_val = *addr;
  *addr = val;
  atomic_exit(primask);
  return old_val;
}

/**
 * @brief  模拟实现 __atomic_store_1 函数 / Simulate the __atomic_store_1 function
 * @param  ptr 指向原子变量（1字节大小）的指针 / Pointer to the 1-byte atomic variable
 * @param  val 需要存储的新值 / The new value to be stored
 * @param  memorder 内存顺序标志（忽略） / Memory order (ignored)
 * @retval 无返回值 / None
 */
__attribute__((weak)) void __atomic_store_1(volatile void* ptr, unsigned char val,
                                            int memorder)  // NOLINT
{
  UNUSED(memorder);
  volatile unsigned char* addr = (volatile unsigned char*)ptr;

  uint32_t primask = atomic_enter();
  *addr = val;
  atomic_exit(primask);
}

/**
 * @brief  模拟实现 __atomic_test_and_set 函数 / Simulate the __atomic_test_and_set
 * function
 * @param  ptr 指向原子标志位的指针 / Pointer to the atomic flag variable
 * @param  memorder 内存顺序标志（忽略） / Memory order (ignored)
 * @retval 返回之前的值 / Returns the previous value (0 or 1)
 */
#if !defined(__clang__)
__attribute__((weak)) _Bool __atomic_test_and_set(volatile void* ptr, int memorder)
{
  UNUSED(memorder);
  volatile unsigned char* addr = (volatile unsigned char*)ptr;
  _Bool old_val = false;

  uint32_t primask = atomic_enter();
  old_val = *addr;
  *addr = 1;
  atomic_exit(primask);

  return old_val;
}
#endif
