#pragma once

// ESP 驱动的公共定义与头文件整理入口。把 ESP-IDF/FreeRTOS 的集成细节集中在这里，
// 使其余 ESP 驱动头文件只需包含这一个干净的公共头文件。
// Shared ESP driver definitions/header hygiene entry point.
// Keep ESP-IDF/FreeRTOS integration quirks localized here so the
// rest of the ESP driver headers can include one clean common header.
#include <FreeRTOS.h>

#include "sdkconfig.h"
