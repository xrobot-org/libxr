// 故意不加 #pragma once：每个用到 Endpoint::Type::INTERRUPT 的头文件都包含本文件，
// 无论 FreeRTOS 在它之前还是之后才被包含，宏都会在用到之前被取消。
// Intentionally no #pragma once: every header that uses Endpoint::Type::INTERRUPT
// includes this file, so the macro is removed before use no matter whether FreeRTOS was
// included before or after.

// Xtensa 的 xtensa/config/specreg.h（经 FreeRTOS 间接包含，传统 ESP32 上有）把 INTERRUPT
// 定义成特殊寄存器号，与 Endpoint::Type::INTERRUPT 冲突。
// Xtensa's xtensa/config/specreg.h (pulled in through FreeRTOS; present on the classic
// ESP32) defines INTERRUPT as a special-register number, which collides with
// Endpoint::Type::INTERRUPT.
#if defined(INTERRUPT)
#undef INTERRUPT
#endif
