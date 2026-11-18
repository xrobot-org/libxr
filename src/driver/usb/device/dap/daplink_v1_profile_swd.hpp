#pragma once

/**
 * @file
 * @brief DAP v1 的编译配置：只有 SWD；在 daplink_v1.hpp 之前包含 / DAP v1 build
 *        configuration: SWD only; include it before daplink_v1.hpp
 */

#ifndef LIBXR_USB_DAPLINK_V1_BUILD_CONFIG_DEFINED
#define LIBXR_USB_DAPLINK_V1_BUILD_CONFIG_DEFINED
namespace LibXR::USB::DapLinkV1BuildConfig
{
/// 不编入 JTAG 支持，只有 SWD / JTAG support left out, SWD only
static constexpr bool ENABLE_JTAG = false;
}  // namespace LibXR::USB::DapLinkV1BuildConfig
#else
static_assert(!LibXR::USB::DapLinkV1BuildConfig::ENABLE_JTAG,
              "Conflicting DAPLink v1 build profile");
#endif

#include "daplink_v1.hpp"
