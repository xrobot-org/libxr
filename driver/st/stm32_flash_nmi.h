#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @brief 在 NMI 处理函数中处理 Flash 的双位 ECC 错误
   *        Handle the Flash double ECC error in the NMI handler.
   * @return 双位 ECC 错误标志已置位并已清除时返回 `true`，NMI 来自其他原因时返回
   *         `false`
   *         `true` when the double ECC error flag was set and has been cleared,
   *         `false` when the NMI has another cause.
   * @note 编程或擦除被掉电打断的单元 ECC 校验不过。G0、G4、L4、L5、WB、WL、U0、U5、
   *       WBA、H5、U3 读到这样的单元时产生 NMI，NMI 不可屏蔽，而 CubeMX 生成的
   *       `NMI_Handler` 是死循环，于是每次启动都停在里面。在它的用户代码区调用本函数：
   *       A unit whose program or erase was cut by a power loss fails its ECC check.
   *       On G0, G4, L4, L5, WB, WL, U0, U5, WBA, H5 and U3 reading such a unit
   *       raises an NMI, which cannot be masked, and the `NMI_Handler` CubeMX
   *       generates is an endless loop, so every start ends there. Call this
   *       function in its user code section:
   * @code
   * void NMI_Handler(void)
   * {
   *   // USER CODE BEGIN NonMaskableInt_IRQn 0
   *   if (libxr_stm32_flash_ecc_nmi())
   *   {
   *     return;
   *   }
   *   // USER CODE END NonMaskableInt_IRQn 0
   *   ...
   * }
   * @endcode
   * @note 返回 `true` 后被打断的读操作继续执行，读到的是该单元写了一半的内容，由调用方
   *       （如 `DatabaseRaw`）按未完成的写入处理；真实损坏造成的双位错误同样返回，
   *       不作区分。单位错误标志一并清除：它置位期间，后续的双位错误不再产生 NMI。
   *       H5 上同时使指令缓存失效。没有这种 NMI 的系列上本函数恒返回 `false`。
   *       After `true` the interrupted read goes on and delivers the half-written
   *       contents of the unit, which the caller (`DatabaseRaw`, for example)
   *       handles as an unfinished write; a double error caused by real damage
   *       returns the same way and is not told apart. The single error flag is
   *       cleared as well: while it is set, a later double error raises no NMI. On
   *       H5 the instruction cache is invalidated too. On families without this NMI
   *       the function always returns `false`.
   */
  bool libxr_stm32_flash_ecc_nmi(void);

#ifdef __cplusplus
}
#endif
