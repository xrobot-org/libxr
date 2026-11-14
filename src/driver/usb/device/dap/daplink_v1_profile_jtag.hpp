#pragma once

#ifndef LIBXR_USB_DAPLINK_V1_BUILD_CONFIG_DEFINED
#define LIBXR_USB_DAPLINK_V1_BUILD_CONFIG_DEFINED
namespace LibXR::USB::DapLinkV1BuildConfig
{
static constexpr bool ENABLE_JTAG = true;
}
#else
static_assert(LibXR::USB::DapLinkV1BuildConfig::ENABLE_JTAG,
              "Conflicting DAPLink v1 build profile");
#endif

#include "daplink_v1.hpp"
