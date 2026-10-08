#pragma once

#include "hpm_soc.h"
#include "power.hpp"

namespace LibXR
{

/**
 * @brief HPM 电源管理实现 / HPM power manager implementation
 */
class HPMPowerManager : public PowerManager
{
 public:
  explicit HPMPowerManager();

  void Reset() override;

  void Shutdown() override;

  void JumpToBootloader() override;
};

}  // namespace LibXR
