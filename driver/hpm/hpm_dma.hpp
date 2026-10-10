#pragma once

/**
 * @file hpm_dma.hpp
 * @brief HPM DMA 管理内部 helper / Internal HPM DMA management helper.
 *
 * @details
 * LibXR 的 HPM 驱动统一通过 HPM SDK `dma_mgr` 组件申请 DMA 通道，并共享
 * `dma_mgr` 安装的 HDMA/XDMA 中断入口。`dma_mgr` 是必需依赖：应用需在
 * `find_package(hpm-sdk)` 之前 `set(CONFIG_DMA_MGR 1)`，使 SDK 编译
 * `components/dma_mgr` 并导出 `hpm_dma_mgr.h`；本地化 SDK 需保留该组件目录。缺少
 * `hpm_dma_mgr.h` 时本头文件以 `#error` 终止编译。
 * LibXR HPM drivers request DMA channels through the HPM SDK `dma_mgr` component and
 * share the HDMA/XDMA interrupt entries installed by `dma_mgr`. `dma_mgr` is a
 * required dependency: the application must `set(CONFIG_DMA_MGR 1)` before
 * `find_package(hpm-sdk)` so the SDK builds `components/dma_mgr` and exports
 * `hpm_dma_mgr.h`; a localized SDK must keep that component directory. Without
 * `hpm_dma_mgr.h`, this header stops the build with `#error`.
 */

#if !__has_include("hpm_dma_mgr.h")
#error "LibXR HPM 驱动需要 SDK dma_mgr：在 find_package(hpm-sdk) 前 set(CONFIG_DMA_MGR 1)"
#error \
    "LibXR HPM drivers need dma_mgr: set(CONFIG_DMA_MGR 1) before find_package(hpm-sdk)"
#endif

#include "hpm_dma_mgr.h"

namespace LibXR
{

/**
 * @class HPMDmaManager
 * @brief HPM SDK `dma_mgr` 一次性初始化入口 / One-time initialization entry for the
 * HPM SDK `dma_mgr`.
 *
 * @details
 * `dma_mgr_init()` 每个程序只能执行一次；使用 DMA 的 LibXR HPM 驱动在构造函数中调用
 * EnsureInitialized()，不得自行调用 `dma_mgr_init()`。应用代码若也使用
 * `dma_mgr`，同样应调用 EnsureInitialized() 而不是 `dma_mgr_init()`。
 * `dma_mgr_init()` must run exactly once per program. LibXR HPM drivers that use DMA
 * call EnsureInitialized() from their constructors and never call `dma_mgr_init()`
 * directly. Application code that also uses `dma_mgr` should call
 * EnsureInitialized() instead of `dma_mgr_init()` as well.
 */
class HPMDmaManager
{
 public:
  HPMDmaManager() = delete;

  /**
   * @brief 确保 `dma_mgr_init()` 已执行且只执行一次 / Ensure `dma_mgr_init()` has
   * run, exactly once.
   *
   * @note 仅在线程或启动上下文（如驱动构造函数）调用，不可在中断中调用 /
   * Call only from thread or startup context such as driver constructors, never
   * from an ISR.
   */
  static void EnsureInitialized();
};

}  // namespace LibXR
