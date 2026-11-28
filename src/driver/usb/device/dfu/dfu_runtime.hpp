#pragma once

#include <atomic>
#include <cstdint>

#include "dfu_def.hpp"

namespace LibXR::USB
{

/**
 * @class DfuRuntimeClass
 * @brief 运行时 DFU 类：处理 DETACH/GETSTATUS/GETSTATE 并超时跳转 / Runtime DFU class:
 *        handles DETACH/GETSTATUS/GETSTATE and jumps after timeout
 *
 * 有跳转函数时功能描述符声明 bitWillDetach：主机发 DETACH 后不复位，设备在 DETACH
 * 的状态阶段完成、再过超时之后由 Process() 跳转。没有跳转函数时 DETACH 被拒绝。
 * With a jump function the functional descriptor sets bitWillDetach: the host does not
 * reset after DETACH, and Process() jumps once the DETACH status stage has completed and
 * the timeout has passed. Without a jump function DETACH is refused.
 *
 * @note 除 Process() 外只在设备的 owner 中运行。
 *       Apart from Process(), runs in the device's owner only.
 */
class DfuRuntimeClass : public DfuInterfaceClassBase
{
 public:
  /// 跳到 bootloader 的函数，参数为 jump_ctx / Function that jumps to the bootloader;
  /// its argument is jump_ctx
  using JumpFun = void (*)(void*);
  /// 默认接口字符串 / Default interface string
  static constexpr const char* DEFAULT_INTERFACE_STRING = "XRUSB DFU RT";
  /// bmAttributes 的 bitWillDetach / bitWillDetach of bmAttributes
  static constexpr uint8_t ATTR_WILL_DETACH = 0x08U;

  /**
   * @brief 构造 / Construct
   * @param jump_to_bootloader 跳转函数；nullptr 时不声明 bitWillDetach、拒绝 DETACH /
   *        Jump function; with nullptr bitWillDetach is not set and DETACH is refused
   * @param jump_ctx 传给跳转函数的参数 / Argument passed to the jump function
   * @param detach_timeout_ms 默认超时，毫秒；也写进 wDetachTimeOut / Default timeout in
   *        milliseconds; also written to wDetachTimeOut
   * @param interface_string 接口字符串 / Interface string
   * @param webusb_landing_page_url WebUSB 落地页；nullptr 不启用 / WebUSB landing page;
   *        nullptr disables it
   * @param webusb_vendor_code WebUSB 厂商请求码 / WebUSB vendor request code
   */
  explicit DfuRuntimeClass(
      JumpFun jump_to_bootloader, void* jump_ctx = nullptr,
      uint16_t detach_timeout_ms = 50U,
      const char* interface_string = DEFAULT_INTERFACE_STRING,
      const char* webusb_landing_page_url = nullptr,
      uint8_t webusb_vendor_code = WebUsb::WEBUSB_VENDOR_CODE_DEFAULT);

  /**
   * @brief 超时到了就跳到 bootloader / Jump to the bootloader once the timeout expired
   * @note 应用主循环周期调用，一个上下文。超时从 DETACH 的状态阶段完成时算起；跳转在
   *       调用者的上下文里执行，只执行一次。
   *       Called periodically by the application loop, from one context. The timeout
   *       counts from the completion of the DETACH status stage; the jump runs in the
   *       caller's context, once.
   */
  void Process();

 protected:
  /// @name DeviceClass 的实现 / DeviceClass implementation
  /// @{

  /// 记下接口号，回到 APP_IDLE；总是 OK / Record the interface number, back to
  /// APP_IDLE; always OK
  [[nodiscard]] ErrorCode BindEndpoints(EndpointPool& pool, uint8_t first_interface,
                                        bool in_isr) override;

  /// 回到 APP_IDLE，取消尚未被 Process() CAS 认领的跳转 / Back to APP_IDLE, cancel
  /// a jump not yet claimed by Process() CAS
  void UnbindEndpoints(EndpointPool& pool, bool in_isr) override;

  /**
   * @brief DETACH、GETSTATUS、GETSTATE / DETACH, GETSTATUS, GETSTATE
   * @return DETACH 只在 APP_IDLE 接受（否则 ARG_ERR），没有跳转函数时 NOT_SUPPORT；
   *         其他请求 ARG_ERR。
   *         DETACH is accepted in APP_IDLE only (ARG_ERR otherwise) and NOT_SUPPORT
   *         without a jump function; other requests ARG_ERR.
   */
  ErrorCode OnControlRequest(bool in_isr, const SetupPacket& setup,
                             ConstRawData& in_data) override;

  /// DETACH 的状态阶段完成：开始计时 / DETACH status stage completed: start the timer
  void OnControlComplete(bool in_isr, const SetupPacket& setup) override;

  /// DETACH 没有完成：回到 APP_IDLE / DETACH did not complete: back to APP_IDLE
  void OnControlAbort(bool in_isr, const SetupPacket& setup) override;
  /// @}

 private:
  /// 跳转阶段：IDLE 为未等待，非零奇数令牌标识一次具体的 DETACH 等待 / Jump phase:
  /// IDLE when inactive; a non-zero odd generation token identifies one armed DETACH
  static constexpr uint32_t IDLE = 0U;

  /// @name 构造时固定 / Fixed at construction
  /// @{
  JumpFun jump_to_bootloader_;          ///< 跳转函数 / Jump function
  void* jump_ctx_;                      ///< 跳转参数 / Jump argument
  uint16_t default_detach_timeout_ms_;  ///< wValue 为 0 时的超时 / Timeout for wValue 0
  /// @}

  /// @name 仅 owner / Owner only
  /// @{
  /// APP_IDLE 或 APP_DETACH；DETACH 置 APP_DETACH，中止、绑定、解绑置 APP_IDLE /
  /// APP_IDLE or APP_DETACH; DETACH sets APP_DETACH, abort, bind and unbind APP_IDLE
  DFUState state_ = DFUState::APP_IDLE;
  /**
   * @brief 递增奇数等待代数令牌（由 owner 推进） / Monotonic odd generation token
   * @note 模 2^32 加 2 步进（初值 1，首个发布值 3，经 0xFFFFFFFF 回到 1 并跳过
   *       IDLE=0），周期为 2^31。受限于 32 位原子存储规则，本协议要求单次 Process()
   *       在读出令牌后至执行 CAS 前的挂起窗口内，owner 连续重挂不得达到或超过 2^31 次。
   *       Advances +2 mod 2^32 (starts at 1, first published 3, wraps 0xFFFFFFFF to 1,
   *       skipping IDLE=0) with a 2^31 period. Under the 32-bit atomic constraint,
   *       requires fewer than 2^31 re-arms during one Process() pause window.
   */
  uint32_t next_token_ = 1U;
  /// GETSTATUS 的应答，到下一个 GETSTATUS 前不变 / GETSTATUS answer, unchanged until
  /// the next GETSTATUS
  DFUStatusResponse status_response_{};
  /// GETSTATE 的应答 / GETSTATE answer
  uint8_t state_response_ = 0U;
  /// @}

  /// @name owner 与 Process() 之间 / Between the owner and Process()
  /// @{
  /**
   * @brief 跳转阶段与代数令牌 / Jump phase and generation token
   * @note owner 写非零奇数令牌（DETACH 完成）和 IDLE（中止、绑定、解绑）；Process()
   *       仅把与已检超时对应的同一令牌换成 IDLE 后跳转。
   *       The owner writes a non-zero odd token (DETACH completed) and IDLE (abort, bind,
   *       unbind); Process() swaps only that same token for IDLE and then jumps.
   */
  std::atomic<uint32_t> launch_phase_{IDLE};
  /**
   * @brief 跳转时刻（毫秒） / Jump time in milliseconds
   * @note owner 在发布奇数令牌前以 release 写；Process() 以 acquire 读，GETSTATUS
   *       仅在 owner 中以 relaxed 读。
   *       Release-stored by owner before publishing the token; acquire-loaded by
   *       Process(), and relaxed-loaded by GETSTATUS inside owner context.
   */
  std::atomic<uint32_t> detach_deadline_ms_{0U};
  /// @}
};

}  // namespace LibXR::USB
