#include "hpm_dma.hpp"

using namespace LibXR;

namespace
{

// Callers run in thread/startup context, so a plain flag is enough.
bool dma_mgr_initialized = false;

}  // namespace

void HPMDmaManager::EnsureInitialized()
{
  if (dma_mgr_initialized)
  {
    return;
  }

  dma_mgr_init();
  dma_mgr_initialized = true;
}
