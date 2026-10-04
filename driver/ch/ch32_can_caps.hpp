#pragma once

#include "libxr_def.hpp"
#include DEF2STR(LIBXR_CH32_CONFIG_FILE)

// V30x 公共头文件在单 CAN 的 V303（D8）上也定义 CAN2；仅 D8C 为双 CAN。
// The V30x common header also defines CAN2 on single-CAN V303 (D8).
// Only D8C (V305/V307/V317) has two CAN controllers in this SDK family.
#if defined(CH32V30x_D8C)
#define LIBXR_CH32_HAS_CAN2 1
#else
#define LIBXR_CH32_HAS_CAN2 0
#endif
