#pragma once

#include "main.h"

#if defined(USB_OTG_FS) || defined(USB_OTG_HS)
// OTG (DWC2) controllers: register runtime without the PCD handle.
#include "stm32_usb_otg.hpp"
#elif defined(USB) || defined(USB_DRD_FS)
// Classic FSDEV and DRD (PMA) controllers: the shared register runtime.
#include "stm32_usb_fsdev.hpp"
#endif
