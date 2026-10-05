#pragma once

/**
 * @file hpm_gpio.hpp
 * @brief HPM GPIO 驱动适配头文件 / Adapter header for the HPM GPIO driver.
 *
 * @details
 * 本文件在 `GPIO_Type` 上实现 `LibXR::GPIO`，通过 HPM SDK `hpm_gpio_drv`
 * 完成引脚读写、方向配置、中断触发模式配置和中断标志分发。当前实现使用同步
 * GPIO API；驱动为每个 GPIO 端口 IRQ 定义中断入口，入口按 port/pin 映射分发，
 * 应用无需声明这些 ISR。HPM5301 的 `GPIO0_A/B/X/Y` IRQ 是端口级 PLIC
 * 路由，因此同一 port 上多个 pin 会共享同一 IRQ enable/disable 计数。端口表大小
 * 按当前 SDK 头文件暴露的 `GPIO_DI_GPIO*` 宏推导，避免为不同 HPM 系列复制驱动文件。
 *
 * This file implements `LibXR::GPIO` on top of `GPIO_Type` using the HPM SDK
 * `hpm_gpio_drv` APIs for pin read/write, direction setup, interrupt trigger
 * setup, and interrupt-flag dispatch. The current implementation uses synchronous
 * GPIO APIs; the driver defines the interrupt entry of every GPIO port IRQ, which
 * dispatches by port/pin, so the application declares no ISR for these IRQs. HPM5301
 * routes `GPIO0_A/B/X/Y` IRQs per port through PLIC, so multiple pins on the same port
 * share one IRQ enable/disable reference count. The dispatch table size is derived from
 * the `GPIO_DI_GPIO*` macros exposed by the active SDK headers to avoid per-series driver
 * forks.
 */

#include "gpio.hpp"
#include "hpm_gpio_drv.h"
#include "hpm_plic_drv.h"
#include "hpm_soc.h"

namespace LibXR
{
/**
 * @class HPMGPIO
 * @brief HPM 平台 GPIO 驱动实现 / GPIO driver implementation for HPM platform.
 *
 * @details
 * 该类将 LibXR `GPIO` 抽象接口适配到 HPM SDK GPIO API。`Read()` 和
 * `Write()` 分别调用 `gpio_read_pin()` 与 `gpio_write_pin()`；`SetConfig()`
 * 调用 `gpio_set_pin_input()`、`gpio_set_pin_output()` 和
 * `gpio_config_pin_interrupt()`，并在可解析 PAD 时更新 IOC `FUNC_CTL`、
 * `PAD_CTL` 的 GPIO 复用、loopback、上下拉和开漏位。`EnableInterrupt()` 与
 * `DisableInterrupt()` 只支持普通 `HPM_GPIO0` 中断路径；完整 SDK GPIO 示例说明
 * FGPIO 不支持中断，因此 `HPM_FGPIO` 中断请求返回 `ErrorCode::NOT_SUPPORT`，并且
 * 不会注册到中断分发表。
 *
 * This class adapts the LibXR `GPIO` abstraction to HPM SDK GPIO APIs. `Read()`
 * and `Write()` call `gpio_read_pin()` and `gpio_write_pin()` respectively.
 * `SetConfig()` calls `gpio_set_pin_input()`, `gpio_set_pin_output()`, and
 * `gpio_config_pin_interrupt()`, and updates IOC `FUNC_CTL` / `PAD_CTL` GPIO mux,
 * loopback, pull, and open-drain bits when the PAD can be resolved.
 * `EnableInterrupt()` and `DisableInterrupt()` support only the normal `HPM_GPIO0`
 * IRQ path; the full SDK GPIO sample states that FGPIO does not support interrupts,
 * so interrupt requests on `HPM_FGPIO` return `ErrorCode::NOT_SUPPORT` and are not
 * registered in the interrupt dispatch map.
 *
 * 中断回调由 `CheckInterrupt()` 统一分发。
 * Interrupt callbacks are dispatched through `CheckInterrupt()`.
 */
class HPMGPIO final : public GPIO
{
 public:
  /**
   * @brief 构造 HPM GPIO 对象 / Construct an HPM GPIO object.
   * @param gpio GPIO 控制器基地址 / GPIO controller base address.
   * @param port GPIO 端口号 / GPIO port index.
   * @param pin GPIO 引脚号 / GPIO pin index.
   * @param irq 当前 port 对应的 PLIC IRQ 号；没有中断需求时可保持默认值 /
   * PLIC IRQ number for the current port. Keep the default when interrupt support
   * is not needed.
   * @param pad_index IOC PAD 编号，默认值表示自动由 `(gpio, port, pin)` 推导；无法
   * 推导时仍可读写 GPIO，但 PAD 复用和模拟高阻配置会受限 /
   * IOC PAD index. The default auto-resolves it from `(gpio, port, pin)`. If it
   * cannot be resolved, GPIO read/write can still work, while PAD mux and analog
   * high-impedance configuration are limited.
   */
  HPMGPIO(GPIO_Type* gpio, uint32_t port, uint8_t pin, uint32_t irq = INVALID_IRQ,
          uint16_t pad_index = INVALID_PAD_INDEX);

  /**
   * @brief 读取引脚电平 / Read current pin level.
   * @return `true` 高电平，`false` 低电平 / `true` for high level, `false` for low level.
   */
  inline bool Read() override
  {
    const uint32_t level = gpio_read_pin(gpio_, port_, pin_);
    return level == 1u;
  }

  /**
   * @brief 写引脚电平 / Write output pin level.
   * @param value `true` 高电平，`false` 低电平 / `true` for high level, `false` for low
   * level.
   */
  inline void Write(bool value) override
  {
    gpio_write_pin(gpio_, port_, pin_, value ? 1u : 0u);
  }

  /**
   * @brief 使能当前引脚中断 / Enable GPIO interrupt for current pin.
   * @return 成功返回 `ErrorCode::OK`；IRQ/端口参数无效返回 `ErrorCode::ARG_ERR`；
   * `HPM_FGPIO` 返回 `ErrorCode::NOT_SUPPORT`；同 port 存在冲突 IRQ 路由返回
   * `ErrorCode::STATE_ERR` /
   * Returns `ErrorCode::OK` on success, `ErrorCode::ARG_ERR` for invalid irq/port/pin,
   * `ErrorCode::NOT_SUPPORT` for `HPM_FGPIO`, and `ErrorCode::STATE_ERR` when the
   * same port is already routed to another IRQ.
   *
   * @note HPM SDK `gpio_enable_pin_interrupt()` 只设置 GPIO `IE` 位；PLIC 路由由
   * `intc_m_enable_irq_with_priority()` 打开，且同一 port 只在首个 pin enable 时调用 /
   * HPM SDK `gpio_enable_pin_interrupt()` only sets the GPIO `IE` bit; PLIC routing
   * is opened through `intc_m_enable_irq_with_priority()` only when the first pin on
   * the same port is enabled.
   */
  ErrorCode EnableInterrupt() override;

  /**
   * @brief 失能当前引脚中断 / Disable GPIO interrupt for current pin.
   * @return 成功返回 `ErrorCode::OK`；IRQ/端口参数无效返回 `ErrorCode::ARG_ERR`；
   * `HPM_FGPIO` 返回 `ErrorCode::NOT_SUPPORT`；端口路由状态异常返回
   * `ErrorCode::STATE_ERR` /
   * Returns `ErrorCode::OK` on success, `ErrorCode::ARG_ERR` for invalid irq/port/pin,
   * `ErrorCode::NOT_SUPPORT` for `HPM_FGPIO`, and `ErrorCode::STATE_ERR` when shared
   * port-route state is inconsistent.
   *
   * @note HPM SDK `gpio_disable_pin_interrupt()` 只清当前 pin 的 GPIO `IE` 位；
   * 共享 PLIC 路由只在同 port 最后一个已启用 pin disable 后关闭 /
   * HPM SDK `gpio_disable_pin_interrupt()` only clears the current pin's GPIO `IE`
   * bit; the shared PLIC route is closed only after the last enabled pin on the same
   * port is disabled.
   */
  ErrorCode DisableInterrupt() override;

  /**
   * @brief 配置当前引脚模式 / Configure GPIO mode for current pin.
   * @param config GPIO 配置参数 / GPIO configuration.
   * @return 成功返回 `ErrorCode::OK`；port/pin 或枚举值无效返回 `ErrorCode::ARG_ERR`；
   * `HPM_FGPIO` 的中断方向或 SDK 未提供双沿枚举时返回 `ErrorCode::NOT_SUPPORT` /
   * Returns `ErrorCode::OK` on success, `ErrorCode::ARG_ERR` for invalid port/pin
   * or enum values, or `ErrorCode::NOT_SUPPORT` for interrupt directions on
   * `HPM_FGPIO` or when the SDK does not provide the both-edge enum.
   * @note 该接口会将 PAD 复用强制切换为 GPIO，不负责外设复用配置 /
   * This API forces pad mux to GPIO function (no peripheral alternate-function setup).
   * @note 在可解析到 PAD 时会自动开启 IOC loopback，用于输出模式下直接通过 `Read()`
   * 读取实际引脚电平 / When PAD can be resolved, IOC loopback is enabled so `Read()` can
   * return the actual pad level even in output mode.
   */
  ErrorCode SetConfig(Configuration config) override;

  /**
   * @brief 将当前 PAD 配置为模拟高阻 / Configure current pad to analog high-impedance.
   * @return 成功返回 `ErrorCode::OK`；port/pin 无效返回 `ErrorCode::ARG_ERR`；
   * PAD 不可解析返回 `ErrorCode::NOT_SUPPORT` /
   * Returns `ErrorCode::OK` on success, `ErrorCode::ARG_ERR` for invalid port/pin,
   * or `ErrorCode::NOT_SUPPORT` when PAD is unavailable.
   *
   * @note 该接口直接写 IOC `FUNC_CTL` 的 `ANALOG` 位并清除上下拉和开漏位，未上板前
   * 仍需用实际电气连接确认高阻效果 /
   * This API writes the IOC `FUNC_CTL` `ANALOG` bit and clears pull/open-drain bits.
   * The high-impedance effect still needs board-level electrical verification.
   * @note 如果当前 pin 已经通过 `EnableInterrupt()` 进入共享 port IRQ 路由，本接口会先
   * 走 `DisableInterrupt()` 维护引用计数；否则只清当前 pin 的 GPIO `IE` 位 /
   * If the pin is already routed through `EnableInterrupt()`, this API first calls
   * `DisableInterrupt()` to keep the shared port IRQ reference count consistent;
   * otherwise it only clears the current pin's GPIO `IE` bit.
   */
  ErrorCode SetAnalogHighImpedance();

  /**
   * @brief 分发某一端口的 GPIO 中断回调 / Dispatch GPIO interrupt callbacks for one port.
   * @param port GPIO 端口号 / GPIO port index.
   *
   * 由驱动定义的 GPIO 端口中断入口调用。
   * Called by the GPIO port interrupt entries that the driver defines.
   *
   * 同时会清除触发引脚的中断标志位。
   * It also clears interrupt flags for triggered pins.
   */
  static void CheckInterrupt(uint32_t port);

 private:
#if defined(GPIO_DI_GPIOZ)
  static constexpr uint32_t PORT_COUNT =
      GPIO_DI_GPIOZ + 1u;  ///< 支持的端口数量 / Supported port count.
#elif defined(GPIO_DI_GPIOY)
  static constexpr uint32_t PORT_COUNT =
      GPIO_DI_GPIOY + 1u;  ///< 支持的端口数量 / Supported port count.
#elif defined(GPIO_DI_GPIOX)
  static constexpr uint32_t PORT_COUNT =
      GPIO_DI_GPIOX + 1u;  ///< 支持的端口数量 / Supported port count.
#elif defined(GPIO_DI_GPIOF)
  static constexpr uint32_t PORT_COUNT =
      GPIO_DI_GPIOF + 1u;  ///< 支持的端口数量 / Supported port count.
#elif defined(GPIO_DI_GPIOE)
  static constexpr uint32_t PORT_COUNT =
      GPIO_DI_GPIOE + 1u;  ///< 支持的端口数量 / Supported port count.
#elif defined(GPIO_DI_GPIOD)
  static constexpr uint32_t PORT_COUNT =
      GPIO_DI_GPIOD + 1u;  ///< 支持的端口数量 / Supported port count.
#elif defined(GPIO_DI_GPIOC)
  static constexpr uint32_t PORT_COUNT =
      GPIO_DI_GPIOC + 1u;  ///< 支持的端口数量 / Supported port count.
#else
  static constexpr uint32_t PORT_COUNT = 15u;  ///< 支持的端口数量 / Supported port count.
#endif
  static constexpr uint32_t PIN_COUNT = 32;  ///< 每个端口引脚数 / Pins per port.
  static constexpr uint32_t INVALID_IRQ =
      0xFFFFFFFFu;  ///< 无效 IRQ 标记 / Invalid IRQ marker.
  static constexpr uint16_t INVALID_PAD_INDEX =
      0xFFFFu;  ///< 无效 PAD 标记 / Invalid PAD marker.

  /**
   * @brief 端口级 IRQ 路由状态 / Shared port-level IRQ routing state.
   *
   * HPM5301 GPIO IRQ 以 port 为单位路由到 PLIC，因此多个 pin 共用同一个 IRQ 时，
   * 只能在“首个 pin 使能”时打开路由，在“最后一个 pin 失能”时关闭路由。
   * HPM5301 routes GPIO IRQs per port, so a shared IRQ must only be enabled on
   * the first pin user and disabled after the last pin user is released.
   */
  struct PortIrqRouteState
  {
    GPIO_Type* controller = nullptr;
    uint32_t irq = INVALID_IRQ;
    uint32_t enabled_pin_count = 0u;
  };

  /**
   * @brief 根据控制器与端口引脚推导 IOC PAD 编号 /
   * Resolve IOC PAD index from controller/port/pin tuple.
   * @param gpio GPIO 控制器基地址 / GPIO controller base address.
   * @param port GPIO 端口号 / GPIO port index.
   * @param pin GPIO 引脚号 / GPIO pin index.
   * @return 有效 PAD 编号，无法推导时返回 `INVALID_PAD_INDEX` /
   * Resolved PAD index, or `INVALID_PAD_INDEX` if unavailable.
   */
  static uint16_t ResolvePadIndex(GPIO_Type* gpio, uint32_t port, uint8_t pin);

  ///< 端口-引脚到对象实例的静态映射，用于中断分发 /
  ///< Static port-pin to object map for interrupt dispatch.
  static HPMGPIO* map[PORT_COUNT][PIN_COUNT];
  static GPIO_Type* port_controller_map[PORT_COUNT];
  static PortIrqRouteState port_irq_route_map[PORT_COUNT];

  GPIO_Type* gpio_;     ///< GPIO 控制器实例 / GPIO controller instance.
  uint32_t port_;       ///< GPIO 端口号 / GPIO port index.
  uint8_t pin_;         ///< GPIO 引脚号 / GPIO pin index.
  uint32_t irq_;        ///< 当前 port 对应 IRQ 号 / IRQ number for the current port.
  uint16_t pad_index_;  ///< IOC PAD 编号 / IOC PAD index.
  bool interrupt_enabled_ =
      false;  ///< 当前 pin IRQ 使能状态 / Per-instance IRQ enabled flag.
};

}  // namespace LibXR

/**
 * @brief GPIO 中断分发的 C 接口 / C entry for GPIO interrupt dispatch.
 * @note 驱动已定义 GPIO 端口中断入口；保留此接口供 C 代码使用。
 *       The driver already defines the GPIO port interrupt entries; this entry is kept
 *       for C code.
 * @param port GPIO 端口号 / GPIO port index.
 */
extern "C" void libxr_hpm_gpio_check_interrupt(uint32_t port);
