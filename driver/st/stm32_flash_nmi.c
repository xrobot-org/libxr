#include "stm32_flash_nmi.h"

#include "main.h"

bool libxr_stm32_flash_ecc_nmi(void)
{
#if defined(FLASH_ECCR_OTP_ECC) && defined(FLASH_ECCR_ECCD) && defined(FLASH_ECCR_ECCC)
  /* H5：检测寄存器和纠正寄存器分开，位名沿用 FLASH_ECCR_。
   * H5: separate detection and correction registers; the bit names keep FLASH_ECCR_. */
  if ((FLASH->ECCDETR & FLASH_ECCR_ECCD) == 0U)
  {
    return false;
  }
  FLASH->ECCDETR |= FLASH_ECCR_ECCD;
  FLASH->ECCCORR |= FLASH_ECCR_ECCC;
#if defined(ICACHE) && defined(ICACHE_CR_CACHEINV)
  ICACHE->CR |= ICACHE_CR_CACHEINV;
#endif
  return true;
#elif defined(FLASH_ECCDR_ECCD) && defined(FLASH_ECCCR_ECCC)
  /* U3：检测寄存器 ECCDR，纠正寄存器 ECCCR。
   * U3: detection register ECCDR, correction register ECCCR. */
  if ((FLASH->ECCDR & FLASH_ECCDR_ECCD) == 0U)
  {
    return false;
  }
  FLASH->ECCDR |= FLASH_ECCDR_ECCD;
  FLASH->ECCCR |= FLASH_ECCCR_ECCC;
  return true;
#elif defined(FLASH_ECCR_ECCD) && defined(FLASH_ECCR_ECCC)
  /* G0、G4、L4、L5、WB、WL、U0、U5、WBA：两个标志在同一个 ECCR 里，写 1 清除；部分型号
   * 另有 128 位读取高半部分的标志（ECCD2/ECCC2）或第二个 bank 的寄存器（ECC2R）。
   * G0, G4, L4, L5, WB, WL, U0, U5, WBA: both flags are in one ECCR and are cleared by
   * writing 1; some parts add the flags of the upper half of a 128-bit read (ECCD2/ECCC2)
   * or a register for the second bank (ECC2R). */
  bool handled = false;
#if defined(FLASH_ECCR_ECCD2) && defined(FLASH_ECCR_ECCC2)
  const uint32_t detected = FLASH_ECCR_ECCD | FLASH_ECCR_ECCD2;
  const uint32_t flags = detected | FLASH_ECCR_ECCC | FLASH_ECCR_ECCC2;
#else
  const uint32_t detected = FLASH_ECCR_ECCD;
  const uint32_t flags = detected | FLASH_ECCR_ECCC;
#endif
  if ((FLASH->ECCR & detected) != 0U)
  {
    FLASH->ECCR |= flags;
    handled = true;
  }
#if defined(FLASH_ECC2R_ECCD) && defined(FLASH_ECC2R_ECCC)
  if ((FLASH->ECC2R & FLASH_ECC2R_ECCD) != 0U)
  {
    FLASH->ECC2R |= FLASH_ECC2R_ECCD | FLASH_ECC2R_ECCC;
    handled = true;
  }
#endif
  return handled;
#else
  return false;
#endif
}
