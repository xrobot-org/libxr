#pragma once

/**
 * @file
 * @brief DAP v2 的编译配置：只有 SWD；在 daplink_v2.hpp 之前包含 / DAP v2 build
 *        configuration: SWD only; include it before daplink_v2.hpp
 */

#ifndef LIBXR_USB_DAPLINK_V2_BUILD_CONFIG_DEFINED
#define LIBXR_USB_DAPLINK_V2_BUILD_CONFIG_DEFINED
namespace LibXR::USB::DapLinkV2BuildConfig
{
/// 不编入 JTAG 支持，只有 SWD / JTAG support left out, SWD only
static constexpr bool ENABLE_JTAG = false;
}  // namespace LibXR::USB::DapLinkV2BuildConfig
#else
static_assert(!LibXR::USB::DapLinkV2BuildConfig::ENABLE_JTAG,
              "Conflicting DAPLink v2 build profile");
#endif

#include "daplink_v2.hpp"
