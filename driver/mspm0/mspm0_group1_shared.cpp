#include "mspm0_group1_shared.hpp"

namespace LibXR::MSPM0Group1Shared
{
namespace
{
// INT_GROUP1 的成员共用一个中断号；优先级低于 SysTick 时基。
// The INT_GROUP1 members share one interrupt number; the priority stays below the
// SysTick timebase.
void EnableIRQ(IRQn_Type irqn)
{
  NVIC_SetPriority(irqn, 1U);
  NVIC_EnableIRQ(irqn);
}
}  // namespace

void EnableGroup1IRQ()
{
#if defined(GPIOA_BASE)
  EnableIRQ(GPIOA_INT_IRQn);
#elif defined(GPIOB_BASE)
  EnableIRQ(GPIOB_INT_IRQn);
#elif defined(GPIOC_BASE)
  EnableIRQ(GPIOC_INT_IRQn);
#elif defined(COMP0_BASE)
  EnableIRQ(COMP0_INT_IRQn);
#elif defined(COMP1_BASE)
  EnableIRQ(COMP1_INT_IRQn);
#elif defined(COMP2_BASE)
  EnableIRQ(COMP2_INT_IRQn);
#elif defined(TRNG_BASE)
  EnableIRQ(TRNG_INT_IRQn);
#endif
}
}  // namespace LibXR::MSPM0Group1Shared

// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void GROUP1_IRQHandler(void)
{
  uint32_t iidx = DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1);
  using namespace LibXR::MSPM0Group1Shared;

  switch (iidx)
  {
#if defined(GPIOA_BASE)
    case DL_INTERRUPT_GROUP1_IIDX_GPIOA:
      if (auto fn = gpioa_irq_cb)
      {
        fn();
      }
      break;
#endif
#if defined(GPIOB_BASE)
    case DL_INTERRUPT_GROUP1_IIDX_GPIOB:
      if (auto fn = gpiob_irq_cb)
      {
        fn();
      }
      break;
#endif
#if defined(COMP0_BASE)
    case DL_INTERRUPT_GROUP1_IIDX_COMP0:
      if (auto fn = comp0_irq_cb)
      {
        fn();
      }
      break;
#endif
#if defined(COMP1_BASE)
    case DL_INTERRUPT_GROUP1_IIDX_COMP1:
      if (auto fn = comp1_irq_cb)
      {
        fn();
      }
      break;
#endif
#if defined(COMP2_BASE)
    case DL_INTERRUPT_GROUP1_IIDX_COMP2:
      if (auto fn = comp2_irq_cb)
      {
        fn();
      }
      break;
#endif
#if defined(TRNG_BASE)
    case DL_INTERRUPT_GROUP1_IIDX_TRNG:
      if (auto fn = trng_irq_cb)
      {
        fn();
      }
      break;
#endif
#if defined(GPIOC_BASE)
    case DL_INTERRUPT_GROUP1_IIDX_GPIOC:
      if (auto fn = gpioc_irq_cb)
      {
        fn();
      }
      break;
#endif
    default:
      break;
  }
}
