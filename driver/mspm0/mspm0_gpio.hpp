#pragma once

#include <ti/driverlib/dl_gpio.h>

#include "gpio.hpp"

namespace LibXR
{

/// 本芯片的 GPIO 端口数（按 GPIOx_BASE 是否定义推得）/ Number of GPIO ports on this
/// device (derived from which GPIOx_BASE macros exist)
#ifdef GPIOC_BASE
constexpr uint8_t MAX_PORTS = 3;
#elif defined(GPIOB_BASE)
constexpr uint8_t MAX_PORTS = 2;
#else
constexpr uint8_t MAX_PORTS = 1;
#endif

/**
 * @brief MSPM0 GPIO / MSPM0 GPIO.
 *
 * @note 引脚中断由驱动开关；首次登记端口中断时开启 INT_GROUP1，并设为优先级 1。
 *       The driver enables and disables the pin interrupts; registering the first port
 *       interrupt enables INT_GROUP1 at priority 1.
 */
class MSPM0GPIO : public GPIO
{
 public:
  /**
   * @brief 构造 GPIO 对象并登记到端口中断分发表 / Construct a GPIO object and register
   *        it in the port interrupt dispatch table
   * @param port GPIO 端口寄存器（GPIOA/GPIOB/GPIOC）/ GPIO port registers
   * @param pin_mask 单个引脚的位掩码，只能有一位置位 / Single-pin bit mask, exactly one
   *        bit set
   * @param pincm 该引脚的 IOMUX PINCM 索引 / IOMUX PINCM index of the pin
   */
  explicit MSPM0GPIO(GPIO_Regs* port, uint32_t pin_mask, uint32_t pincm);

  bool Read() override;

  void Write(bool value) override;

  ErrorCode EnableInterrupt() override;

  ErrorCode DisableInterrupt() override;

  ErrorCode SetConfig(Configuration config) override;

  /**
   * @brief 端口中断入口：清除挂起位并回调对应引脚 / Port interrupt entry: clear the
   *        pending bits and run the callback of each pending pin
   * @param port 触发中断的 GPIO 端口 / GPIO port that raised the interrupt
   */
  static inline void OnInterrupt(GPIO_Regs* port)
  {
    const int idx = GetPortIndex(reinterpret_cast<uint32_t>(port));  // NOLINT
    OnInterruptDispatch(port, idx);
  }

 private:
  static void OnInterruptDispatch(GPIO_Regs* port, int port_idx);

  static constexpr int GetPortIndex(uint32_t base_addr)
  {
#ifdef GPIOA_BASE
    if (base_addr == GPIOA_BASE)
    {
      return 0;
    }
#endif
#ifdef GPIOB_BASE
    if (base_addr == GPIOB_BASE)
    {
      return 1;
    }
#endif
#ifdef GPIOC_BASE
    if (base_addr == GPIOC_BASE)
    {
      return 2;
    }
#endif
    return -1;
  }

  GPIO_Regs* port_;
  uint32_t pin_mask_;
  uint32_t pincm_;
  LibXR::GPIO::Direction current_direction_ = LibXR::GPIO::Direction::INPUT;

  // 安全的编译时nullptr数组初始化 / Safe compile-time initialization of nullptr array
  // NOLINTNEXTLINE(bugprone-dynamic-static-initializers)
  static MSPM0GPIO* instance_map_[MAX_PORTS][32];
};

}  // namespace LibXR
