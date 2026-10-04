#include "ch32_power.hpp"

using namespace LibXR;

// CH32 的复位 / 待机接口只是对标准库入口做薄封装。
// CH32 reset / standby hooks are thin wrappers over the vendor library entry points.
void CH32PowerManager::Reset() { NVIC_SystemReset(); }

void CH32PowerManager::Shutdown() { PWR_EnterSTANDBYMode(); }
