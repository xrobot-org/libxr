#pragma once

#include "power.hpp"
#include "ti_msp_dl_config.h"

namespace LibXR
{

/**
 * @brief MSPM0 电源管理实现 / MSPM0 power manager implementation
 */
class MSPM0PowerManager : public PowerManager
{
 public:
  explicit MSPM0PowerManager();

  void Reset() override;

  void Shutdown() override;

  void JumpToBootloader() override;
};

}  // namespace LibXR
