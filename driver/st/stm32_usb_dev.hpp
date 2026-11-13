#pragma once

#include "main.h"

#if defined(USB_OTG_FS) || defined(USB_OTG_HS)
// OTG（DWC2）控制器：不使用 PCD 句柄的寄存器运行期。
// OTG (DWC2) controllers: register runtime without the PCD handle.
#include "stm32_usb_otg.hpp"
#elif defined(USB) || defined(USB_DRD_FS)
// 经典 FSDEV 与 DRD（PMA）控制器：共用的寄存器运行期。
// Classic FSDEV and DRD (PMA) controllers: the shared register runtime.
#include "stm32_usb_fsdev.hpp"
#endif
