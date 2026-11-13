#pragma once

// Shared ESP driver definitions/header hygiene entry point.
// Keep ESP-IDF/FreeRTOS integration quirks localized here so the
// rest of the ESP driver headers can include one clean common header.
#include <FreeRTOS.h>

#include "sdkconfig.h"
