#pragma once

#include "libxr_def.hpp"
#include DEF2STR(LIBXR_CH32_CONFIG_FILE)

/**
 * @brief 当前器件是否具备并启用 OTG HS 控制器 / Whether this part exposes usable USBHS
 * @note V30x 公共头文件在单 USBFS 的 D8（V303）上也定义 USBHSD，但未定义 USBHS_IRQn。
 *       The shared V30x header also defines USBHSD on USBFS-only D8 (V303), without
 *       USBHS_IRQn.
 */
#if defined(USBHSD) && !defined(CH32V30x_D8)
#define LIBXR_CH32_HAS_USBHS 1
#else
#define LIBXR_CH32_HAS_USBHS 0
#endif

namespace LibXR
{

/**
 * @brief CH32 USB 控制器编号 / CH32 USB controller identifiers
 */
typedef enum : uint8_t
{
#if defined(USBFSD)
  CH32_USB_OTG_FS,
#endif
#if LIBXR_CH32_HAS_USBHS
  CH32_USB_OTG_HS,
#endif
  CH32_USB_DEV_ID_NUM
} ch32_usb_dev_id_t;

}  // namespace LibXR
