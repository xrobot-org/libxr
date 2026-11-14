#pragma once

// CH32 USB 驱动共用的头文件入口：外设库的配置头文件由 LIBXR_CH32_CONFIG_FILE 指定。
// Common header entry of the CH32 USB drivers: LIBXR_CH32_CONFIG_FILE names the
// configuration header of the peripheral library.
#include "libxr_def.hpp"
#include DEF2STR(LIBXR_CH32_CONFIG_FILE)
