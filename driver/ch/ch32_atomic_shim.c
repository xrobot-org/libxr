#include <stdint.h>

#include "ch32_interrupt_guard.h"

#if !defined(__riscv_zaamo)
#error "CH32 QingKe V4 requires the RISC-V Zaamo extension"
#endif

#if defined(__riscv_zalrsc) || defined(__riscv_atomic)
#error "CH32 QingKe V4 must be compiled with Zaamo, not the full RISC-V A extension"
#endif

/* QingKe V4 的 LR/SC 不提供标准保留语义；字 AMO 使用原生指令，CAS 通过短暂屏蔽中断实现。
 * QingKe V4 LR/SC lacks reservation semantics. Keep native word AMOs and guard CAS
 * with a bounded single-core interrupt mask. */
typedef uint32_t libxr_atomic_fallback_guard_state_t;

static inline libxr_atomic_fallback_guard_state_t libxr_atomic_fallback_enter(void)
{
  const uint32_t interrupt_state = libxr_ch32_interrupt_save_and_disable();
  __asm volatile("fence rw, rw" ::: "memory");
  return interrupt_state;
}

static inline void libxr_atomic_fallback_exit(
    libxr_atomic_fallback_guard_state_t interrupt_state)
{
  __asm volatile("fence rw, rw" ::: "memory");
  libxr_ch32_interrupt_restore(interrupt_state);
}

/* GCC 定长运行时接口有五个参数，执行强 CAS。
 * The sized GCC runtime helper has five arguments and performs a strong CAS. */
__attribute__((used, noinline)) _Bool libxr_atomic_compare_exchange_4(
    volatile void* ptr, void* expected, unsigned int desired, int success_memorder,
    int failure_memorder) __asm__("__atomic_compare_exchange_4");

__attribute__((used, noinline)) _Bool
libxr_atomic_compare_exchange_4(volatile void* ptr, void* expected, unsigned int desired,
                                int success_memorder, int failure_memorder)
{
  (void)success_memorder;
  (void)failure_memorder;

  volatile unsigned int* const value = (volatile unsigned int*)ptr;
  unsigned int* const expected_value = (unsigned int*)expected;
  const libxr_atomic_fallback_guard_state_t interrupt_state =
      libxr_atomic_fallback_enter();
  const unsigned int observed = *value;
  const _Bool matched = observed == *expected_value;

  if (matched)
  {
    *value = desired;
  }
  else
  {
    *expected_value = observed;
  }

  libxr_atomic_fallback_exit(interrupt_state);
  return matched;
}
