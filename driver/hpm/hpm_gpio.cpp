#include "hpm_gpio.hpp"

#include <atomic>

#include "hpm_interrupt.h"
#include "hpm_ioc_regs.h"

using namespace LibXR;

/**
 * @brief GPIO 对象映射表 / GPIO object dispatch map.
 *
 * 按端口和引脚保存对象指针，供中断入口快速找到具体实例并触发回调。
 * Stores instance pointers by port/pin so IRQ entry can dispatch callbacks quickly.
 */
HPMGPIO* HPMGPIO::map[HPMGPIO::PORT_COUNT][HPMGPIO::PIN_COUNT] = {};
GPIO_Type* HPMGPIO::port_controller_map[HPMGPIO::PORT_COUNT] = {};
HPMGPIO::PortIrqRouteState HPMGPIO::port_irq_route_map[HPMGPIO::PORT_COUNT] = {};

namespace
{
// Serialize shared per-port IRQ routing updates against task/ISR races on the local core.
std::atomic_flag g_hpm_gpio_irq_route_lock = ATOMIC_FLAG_INIT;

bool HPM_GPIO_IsFastController(GPIO_Type* gpio)
{
#if defined(HPM_FGPIO)
  return gpio == HPM_FGPIO;
#else
  (void)gpio;
  return false;
#endif
}

class HPMGPIOIrqRouteGuard
{
 public:
  HPMGPIOIrqRouteGuard()
      : irq_state_(disable_global_irq(CSR_MSTATUS_MIE_MASK) & CSR_MSTATUS_MIE_MASK)
  {
    while (g_hpm_gpio_irq_route_lock.test_and_set(std::memory_order_acquire))
    {
    }
  }

  ~HPMGPIOIrqRouteGuard()
  {
    g_hpm_gpio_irq_route_lock.clear(std::memory_order_release);
    restore_global_irq(irq_state_);
  }

  HPMGPIOIrqRouteGuard(const HPMGPIOIrqRouteGuard&) = delete;
  HPMGPIOIrqRouteGuard& operator=(const HPMGPIOIrqRouteGuard&) = delete;

 private:
  uint32_t irq_state_;
};
}  // namespace

/**
 * @brief 将 PAD 复用切换为 GPIO 功能 / Route PAD mux to GPIO function.
 * @param gpio GPIO 控制器基地址 / GPIO controller base address.
 * @param port GPIO 端口号 / GPIO port index.
 * @param pad_index IOC PAD 编号 / IOC PAD index.
 * @param enable_loopback 是否使能 IOC loopback / Whether to enable IOC loopback.
 *
 * 仅处理 GPIO 复用，不涉及外设复用配置。
 * Only GPIO muxing is handled here; peripheral alternate-function setup is not included.
 */
static inline void HPM_GPIO_ConfigMuxToGPIO(GPIO_Type* gpio, uint32_t port,
                                            uint16_t pad_index,
                                            bool enable_loopback = true)
{
  uint32_t func_ctl = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0);
  if (enable_loopback)
  {
    func_ctl |= IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
  }
  HPM_IOC->PAD[pad_index].FUNC_CTL = func_ctl;

  // GPIOY 端口需要额外通过 PIOC 选择 SOC GPIO 信号 / GPIOY needs extra PIOC routing.
#if defined(GPIO_DI_GPIOY) && defined(HPM_PIOC)
  if (port == GPIO_DI_GPIOY && (gpio == HPM_GPIO0 || HPM_GPIO_IsFastController(gpio)))
  {
    HPM_PIOC->PAD[pad_index].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(3);
  }
#else
  (void)gpio;
  (void)port;
#endif
}

/**
 * @brief 构造 HPM GPIO 对象并注册到中断映射表 /
 * Construct HPM GPIO object and register into interrupt map.
 */
HPMGPIO::HPMGPIO(GPIO_Type* gpio, uint32_t port, uint8_t pin, uint32_t irq,
                 uint16_t pad_index)
    : gpio_(gpio),
      port_(port),
      pin_(pin),
      irq_(irq),
      pad_index_(pad_index == INVALID_PAD_INDEX ? ResolvePadIndex(gpio, port, pin)
                                                : pad_index)
{
  if (port_ < PORT_COUNT && pin_ < PIN_COUNT && !HPM_GPIO_IsFastController(gpio_))
  {
    map[port_][pin_] = this;
    if (port_controller_map[port_] == nullptr)
    {
      port_controller_map[port_] = gpio_;
    }
  }
}

/**
 * @brief 使能当前引脚中断 / Enable interrupt for current pin.
 *
 * 对于共享同一 port IRQ 的多个 pin：
 * - `gpio_enable_pin_interrupt()` 始终按 pin 开事件；
 * - PLIC 路由只在该 port 首次进入“已使能”状态时打开一次。
 * - FGPIO 没有 GPIO 中断能力，按完整 SDK GPIO 示例的约束返回 NOT_SUPPORT。
 * For multiple pins sharing one port IRQ:
 * - `gpio_enable_pin_interrupt()` always operates per pin;
 * - PLIC routing is enabled only once when the first pin on that port becomes active.
 * - FGPIO has no GPIO interrupt capability and returns NOT_SUPPORT according to the
 *   full SDK GPIO sample constraint.
 */
ErrorCode HPMGPIO::EnableInterrupt()
{
  if (irq_ == INVALID_IRQ || port_ >= PORT_COUNT || pin_ >= PIN_COUNT)
  {
    return ErrorCode::ARG_ERR;
  }
  if (HPM_GPIO_IsFastController(gpio_))
  {
    return ErrorCode::NOT_SUPPORT;
  }

  HPMGPIOIrqRouteGuard guard;
  if (interrupt_enabled_)
  {
    return ErrorCode::OK;
  }

  PortIrqRouteState& route = port_irq_route_map[port_];
  const bool needs_port_irq_enable = (route.enabled_pin_count == 0u);
  if (!needs_port_irq_enable && (route.controller != gpio_ || route.irq != irq_))
  {
    return ErrorCode::STATE_ERR;
  }

  gpio_enable_pin_interrupt(gpio_, port_, pin_);

  if (needs_port_irq_enable)
  {
    route.controller = gpio_;
    route.irq = irq_;
    intc_m_enable_irq_with_priority(irq_, 1);
  }

  ++route.enabled_pin_count;
  interrupt_enabled_ = true;
  return ErrorCode::OK;
}

/**
 * @brief 失能当前引脚中断 / Disable interrupt for current pin.
 *
 * 先关闭当前 pin 的 GPIO 事件；
 * 仅当该 port 最后一个已使能 pin 被释放时，才关闭共享的 PLIC 路由。
 * FGPIO 没有 GPIO 中断能力，按完整 SDK GPIO 示例的约束返回 NOT_SUPPORT。
 * Disable the current pin's GPIO event first; the shared PLIC route is disabled only
 * after the last enabled pin on the same port is released.
 * FGPIO has no GPIO interrupt capability and returns NOT_SUPPORT according to the
 * full SDK GPIO sample constraint.
 */
ErrorCode HPMGPIO::DisableInterrupt()
{
  if (irq_ == INVALID_IRQ || port_ >= PORT_COUNT || pin_ >= PIN_COUNT)
  {
    return ErrorCode::ARG_ERR;
  }
  if (HPM_GPIO_IsFastController(gpio_))
  {
    return ErrorCode::NOT_SUPPORT;
  }

  HPMGPIOIrqRouteGuard guard;
  if (!interrupt_enabled_)
  {
    return ErrorCode::OK;
  }

  gpio_disable_pin_interrupt(gpio_, port_, pin_);

  PortIrqRouteState& route = port_irq_route_map[port_];
  if (route.enabled_pin_count == 0u)
  {
    route = {};
    interrupt_enabled_ = false;
    return ErrorCode::STATE_ERR;
  }

  if (route.controller != gpio_ || route.irq != irq_)
  {
    interrupt_enabled_ = false;
    return ErrorCode::STATE_ERR;
  }

  --route.enabled_pin_count;
  if (route.enabled_pin_count == 0u)
  {
    const uint32_t route_irq = route.irq;
    route = {};
    intc_m_disable_irq(route_irq);
  }

  interrupt_enabled_ = false;
  return ErrorCode::OK;
}

/**
 * @brief 配置 GPIO 方向、中断模式与上下拉 / Configure GPIO direction, interrupt mode, and
 * pull.
 *
 * 与 STM32 HAL 的常见用法对齐：
 * - `Direction` 对应 GPIO 模式（输入/输出/中断边沿）
 * - `Pull` 对应上拉/下拉配置
 * Aligned with common STM32 HAL usage:
 * - `Direction` maps to GPIO mode (input/output/IRQ edge)
 * - `Pull` maps to pull-up / pull-down configuration
 */
ErrorCode HPMGPIO::SetConfig(Configuration config)
{
  if (port_ >= PORT_COUNT || pin_ >= PIN_COUNT)
  {
    return ErrorCode::ARG_ERR;
  }

  switch (config.pull)
  {
    case Pull::NONE:
    case Pull::UP:
    case Pull::DOWN:
      break;
    default:
      return ErrorCode::ARG_ERR;
  }

  const bool is_interrupt_direction =
      config.direction == Direction::FALL_INTERRUPT ||
      config.direction == Direction::RISING_INTERRUPT ||
      config.direction == Direction::FALL_RISING_INTERRUPT;
  if (is_interrupt_direction && HPM_GPIO_IsFastController(gpio_))
  {
    return ErrorCode::NOT_SUPPORT;
  }
  if (config.direction == Direction::FALL_RISING_INTERRUPT)
  {
#if !defined(GPIO_SOC_HAS_EDGE_BOTH_INTERRUPT) || (GPIO_SOC_HAS_EDGE_BOTH_INTERRUPT != 1)
    return ErrorCode::NOT_SUPPORT;
#endif
  }

  if (pad_index_ != INVALID_PAD_INDEX)
  {
    // 保持接口自洽：先确保 PAD 复用到 GPIO / Keep API self-contained: force GPIO mux
    // first.
    HPM_GPIO_ConfigMuxToGPIO(gpio_, port_, pad_index_);
  }

  switch (config.direction)
  {
    case Direction::INPUT:
      gpio_set_pin_input(gpio_, port_, pin_);
      break;
    case Direction::OUTPUT_PUSH_PULL:
    case Direction::OUTPUT_OPEN_DRAIN:
      gpio_set_pin_output(gpio_, port_, pin_);
      break;
    case Direction::FALL_INTERRUPT:
      gpio_set_pin_input(gpio_, port_, pin_);
      gpio_config_pin_interrupt(gpio_, port_, pin_, gpio_interrupt_trigger_edge_falling);
      break;
    case Direction::RISING_INTERRUPT:
      gpio_set_pin_input(gpio_, port_, pin_);
      gpio_config_pin_interrupt(gpio_, port_, pin_, gpio_interrupt_trigger_edge_rising);
      break;
    case Direction::FALL_RISING_INTERRUPT:
#if defined(GPIO_SOC_HAS_EDGE_BOTH_INTERRUPT) && (GPIO_SOC_HAS_EDGE_BOTH_INTERRUPT == 1)
      gpio_set_pin_input(gpio_, port_, pin_);
      gpio_config_pin_interrupt(gpio_, port_, pin_, gpio_interrupt_trigger_edge_both);
      break;
#else
      return ErrorCode::NOT_SUPPORT;
#endif
    default:
      return ErrorCode::ARG_ERR;
  }

  if (pad_index_ != INVALID_PAD_INDEX)
  {
    uint32_t pad_ctl = HPM_IOC->PAD[pad_index_].PAD_CTL;

    // 仅更新上下拉与开漏位，保留其他电气属性 / Update pull + open-drain bits only.
    pad_ctl &=
        ~(IOC_PAD_PAD_CTL_PE_MASK | IOC_PAD_PAD_CTL_PS_MASK | IOC_PAD_PAD_CTL_OD_MASK);

    switch (config.pull)
    {
      case Pull::NONE:
        break;
      case Pull::UP:
        pad_ctl |= IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
        break;
      case Pull::DOWN:
        pad_ctl |= IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(0);
        break;
    }

    if (config.direction == Direction::OUTPUT_OPEN_DRAIN)
    {
      pad_ctl |= IOC_PAD_PAD_CTL_OD_SET(1);
    }

    HPM_IOC->PAD[pad_index_].PAD_CTL = pad_ctl;
  }

  return ErrorCode::OK;
}

/**
 * @brief 将引脚切换为模拟高阻 / Put current pin into analog high-impedance mode.
 *
 * 用途：
 * - 该引脚仅作为外部网络连接点，不希望其数字输入/上下拉影响外部 PWM 信号；
 * - 例如 PB10 外部跳线到 PA10 时，PA10 需保持高阻避免“分流”。
 * Usage:
 * - Keep this pin from loading an externally driven signal (PWM/analog net);
 * - Example: PB10 externally jumpered to PA10, PA10 should stay high-Z.
 */
ErrorCode HPMGPIO::SetAnalogHighImpedance()
{
  if (port_ >= PORT_COUNT || pin_ >= PIN_COUNT)
  {
    return ErrorCode::ARG_ERR;
  }

  if (pad_index_ == INVALID_PAD_INDEX)
  {
    return ErrorCode::NOT_SUPPORT;
  }

  if (interrupt_enabled_)
  {
    ErrorCode ans = DisableInterrupt();
    if (ans != ErrorCode::OK)
    {
      return ans;
    }
  }
  else
  {
    gpio_disable_pin_interrupt(gpio_, port_, pin_);
  }

  gpio_set_pin_input(gpio_, port_, pin_);

  HPM_IOC->PAD[pad_index_].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;

  uint32_t pad_ctl = HPM_IOC->PAD[pad_index_].PAD_CTL;
  pad_ctl &= ~(IOC_PAD_PAD_CTL_PE_MASK | IOC_PAD_PAD_CTL_OD_MASK);
  HPM_IOC->PAD[pad_index_].PAD_CTL = pad_ctl;

  return ErrorCode::OK;
}

/**
 * @brief 扫描并分发指定端口的 GPIO 中断 / Scan and dispatch GPIO IRQ callbacks for one
 * port.
 * @param port GPIO 端口号 / GPIO port index.
 *
 * 建议在板级 IRQHandler 中仅调用一次该函数：
 * - 读取端口中断标志；
 * - 逐 pin 清标志；
 * - 触发对应对象的 LibXR 回调。
 * Recommended to call once from board IRQ handler:
 * - read port IRQ flags;
 * - clear flag per asserted pin;
 * - run corresponding LibXR callback.
 */
void HPMGPIO::CheckInterrupt(uint32_t port)
{
  if (port >= PORT_COUNT)
  {
    return;
  }

  GPIO_Type* controller = port_controller_map[port];
  if (controller == nullptr)
  {
    return;
  }

  const uint32_t flags = gpio_get_port_interrupt_flags(controller, port);
  if (flags == 0u)
  {
    return;
  }

  for (uint8_t pin = 0; pin < PIN_COUNT; ++pin)
  {
    if ((flags & (1u << pin)) == 0u)
    {
      continue;
    }

    gpio_clear_pin_interrupt_flag(controller, port, pin);
    if (auto* gpio = map[port][pin]; gpio != nullptr && gpio->interrupt_enabled_)
    {
      gpio->callback_.Run(true);
    }
  }
}

/**
 * @brief 推导指定 GPIO 对应的 IOC PAD 编号 / Resolve IOC PAD index for a GPIO pin.
 */
uint16_t HPMGPIO::ResolvePadIndex(GPIO_Type* gpio, uint32_t port, uint8_t pin)
{
  if (gpio != HPM_GPIO0 && !HPM_GPIO_IsFastController(gpio))
  {
    return INVALID_PAD_INDEX;
  }

  switch (port)
  {
#if defined(GPIO_DI_GPIOA) && defined(IOC_PAD_PA00)
    case GPIO_DI_GPIOA:
      return static_cast<uint16_t>(IOC_PAD_PA00 + pin);
#endif
#if defined(GPIO_DI_GPIOB) && defined(IOC_PAD_PB00)
    case GPIO_DI_GPIOB:
      return static_cast<uint16_t>(IOC_PAD_PB00 + pin);
#endif
#if defined(GPIO_DI_GPIOX) && defined(IOC_PAD_PX00)
    case GPIO_DI_GPIOX:
      return pin < 8u ? static_cast<uint16_t>(IOC_PAD_PX00 + pin) : INVALID_PAD_INDEX;
#endif
#if defined(GPIO_DI_GPIOY) && defined(IOC_PAD_PY00)
    case GPIO_DI_GPIOY:
      return pin < 8u ? static_cast<uint16_t>(IOC_PAD_PY00 + pin) : INVALID_PAD_INDEX;
#endif
    default:
      return INVALID_PAD_INDEX;
  }
}

/**
 * @brief GPIO 中断分发的 C 封装入口 / C wrapper entry for GPIO interrupt dispatch.
 * @param port GPIO 端口号 / GPIO port index.
 */
extern "C" void libxr_hpm_gpio_check_interrupt(uint32_t port)
{
  HPMGPIO::CheckInterrupt(port);
}

/*
 * GPIO 端口中断入口由驱动定义，应用无需再声明这些 IRQ 的 ISR。
 * The driver defines the GPIO port interrupt entries; the application does not declare
 * ISRs for these IRQs.
 */
#define LIBXR_HPM_GPIO_PORT_ISR(irq, port)         \
  SDK_DECLARE_EXT_ISR_M(irq, libxr_hpm_gpio_##irq) \
  void libxr_hpm_gpio_##irq(void) { HPMGPIO::CheckInterrupt(port); }

#if defined(IRQn_GPIO0_A) && defined(GPIO_DI_GPIOA)
LIBXR_HPM_GPIO_PORT_ISR(IRQn_GPIO0_A, GPIO_DI_GPIOA)
#endif
#if defined(IRQn_GPIO0_B) && defined(GPIO_DI_GPIOB)
LIBXR_HPM_GPIO_PORT_ISR(IRQn_GPIO0_B, GPIO_DI_GPIOB)
#endif
#if defined(IRQn_GPIO0_C) && defined(GPIO_DI_GPIOC)
LIBXR_HPM_GPIO_PORT_ISR(IRQn_GPIO0_C, GPIO_DI_GPIOC)
#endif
#if defined(IRQn_GPIO0_D) && defined(GPIO_DI_GPIOD)
LIBXR_HPM_GPIO_PORT_ISR(IRQn_GPIO0_D, GPIO_DI_GPIOD)
#endif
#if defined(IRQn_GPIO0_E) && defined(GPIO_DI_GPIOE)
LIBXR_HPM_GPIO_PORT_ISR(IRQn_GPIO0_E, GPIO_DI_GPIOE)
#endif
#if defined(IRQn_GPIO0_F) && defined(GPIO_DI_GPIOF)
LIBXR_HPM_GPIO_PORT_ISR(IRQn_GPIO0_F, GPIO_DI_GPIOF)
#endif
#if defined(IRQn_GPIO0_X) && defined(GPIO_DI_GPIOX)
LIBXR_HPM_GPIO_PORT_ISR(IRQn_GPIO0_X, GPIO_DI_GPIOX)
#endif
#if defined(IRQn_GPIO0_Y) && defined(GPIO_DI_GPIOY)
LIBXR_HPM_GPIO_PORT_ISR(IRQn_GPIO0_Y, GPIO_DI_GPIOY)
#endif
#if defined(IRQn_GPIO0_Z) && defined(GPIO_DI_GPIOZ)
LIBXR_HPM_GPIO_PORT_ISR(IRQn_GPIO0_Z, GPIO_DI_GPIOZ)
#endif
