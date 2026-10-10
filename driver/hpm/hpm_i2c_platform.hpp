#pragma once

/**
 * @file hpm_i2c_platform.hpp
 * @brief HPM I2C 平台资源解析 helper / Platform resource resolver for HPM I2C.
 *
 * @details
 * 本内部 helper 仅依赖 HPM SDK 暴露的实例、IRQ 和 DMA request 宏，将
 * `HPM_I2Cn`、`IRQn_I2Cn` 与 `HPM_DMA_SRC_I2Cn` 统一维护在同一个表中。
 * 缺少某个 IRQ 或 DMA request 宏时，该字段保持无效值，由调用者返回
 * `NOT_SUPPORT`，不按系列名硬编码。
 *
 * This internal helper only relies on HPM SDK exposed instance, IRQ, and DMA
 * request macros. It keeps `HPM_I2Cn`, `IRQn_I2Cn`, and `HPM_DMA_SRC_I2Cn` in one
 * table. If an IRQ or DMA request macro is absent, that field remains invalid and
 * the caller reports `NOT_SUPPORT` without hard-coding SoC series names.
 */

#include <cstddef>
#include <cstdint>

#include "hpm_i2c.hpp"

#if LIBXR_HPM_I2C_SUPPORTED
namespace LibXR
{
namespace HPMI2CPlatform
{

constexpr uint8_t kInvalidDmaSource = 0xFFU;
constexpr int32_t kInvalidIndex = -1;
constexpr int32_t kInvalidIrq = -1;

struct Resource
{
  I2C_Type* base;
  int32_t index;
  uint8_t dma_source;
  int32_t irq;
};

#if defined(HPM_DMA_SRC_I2C0)
#define LIBXR_HPM_I2C_DMA_SRC_0 HPM_DMA_SRC_I2C0
#else
#define LIBXR_HPM_I2C_DMA_SRC_0 kInvalidDmaSource
#endif
#if defined(HPM_DMA_SRC_I2C1)
#define LIBXR_HPM_I2C_DMA_SRC_1 HPM_DMA_SRC_I2C1
#else
#define LIBXR_HPM_I2C_DMA_SRC_1 kInvalidDmaSource
#endif
#if defined(HPM_DMA_SRC_I2C2)
#define LIBXR_HPM_I2C_DMA_SRC_2 HPM_DMA_SRC_I2C2
#else
#define LIBXR_HPM_I2C_DMA_SRC_2 kInvalidDmaSource
#endif
#if defined(HPM_DMA_SRC_I2C3)
#define LIBXR_HPM_I2C_DMA_SRC_3 HPM_DMA_SRC_I2C3
#else
#define LIBXR_HPM_I2C_DMA_SRC_3 kInvalidDmaSource
#endif
#if defined(HPM_DMA_SRC_I2C4)
#define LIBXR_HPM_I2C_DMA_SRC_4 HPM_DMA_SRC_I2C4
#else
#define LIBXR_HPM_I2C_DMA_SRC_4 kInvalidDmaSource
#endif
#if defined(HPM_DMA_SRC_I2C5)
#define LIBXR_HPM_I2C_DMA_SRC_5 HPM_DMA_SRC_I2C5
#else
#define LIBXR_HPM_I2C_DMA_SRC_5 kInvalidDmaSource
#endif
#if defined(HPM_DMA_SRC_I2C6)
#define LIBXR_HPM_I2C_DMA_SRC_6 HPM_DMA_SRC_I2C6
#else
#define LIBXR_HPM_I2C_DMA_SRC_6 kInvalidDmaSource
#endif
#if defined(HPM_DMA_SRC_I2C7)
#define LIBXR_HPM_I2C_DMA_SRC_7 HPM_DMA_SRC_I2C7
#else
#define LIBXR_HPM_I2C_DMA_SRC_7 kInvalidDmaSource
#endif

#if defined(IRQn_I2C0)
#define LIBXR_HPM_I2C_IRQ_0 IRQn_I2C0
#else
#define LIBXR_HPM_I2C_IRQ_0 kInvalidIrq
#endif
#if defined(IRQn_I2C1)
#define LIBXR_HPM_I2C_IRQ_1 IRQn_I2C1
#else
#define LIBXR_HPM_I2C_IRQ_1 kInvalidIrq
#endif
#if defined(IRQn_I2C2)
#define LIBXR_HPM_I2C_IRQ_2 IRQn_I2C2
#else
#define LIBXR_HPM_I2C_IRQ_2 kInvalidIrq
#endif
#if defined(IRQn_I2C3)
#define LIBXR_HPM_I2C_IRQ_3 IRQn_I2C3
#else
#define LIBXR_HPM_I2C_IRQ_3 kInvalidIrq
#endif
#if defined(IRQn_I2C4)
#define LIBXR_HPM_I2C_IRQ_4 IRQn_I2C4
#else
#define LIBXR_HPM_I2C_IRQ_4 kInvalidIrq
#endif
#if defined(IRQn_I2C5)
#define LIBXR_HPM_I2C_IRQ_5 IRQn_I2C5
#else
#define LIBXR_HPM_I2C_IRQ_5 kInvalidIrq
#endif
#if defined(IRQn_I2C6)
#define LIBXR_HPM_I2C_IRQ_6 IRQn_I2C6
#else
#define LIBXR_HPM_I2C_IRQ_6 kInvalidIrq
#endif
#if defined(IRQn_I2C7)
#define LIBXR_HPM_I2C_IRQ_7 IRQn_I2C7
#else
#define LIBXR_HPM_I2C_IRQ_7 kInvalidIrq
#endif

#define LIBXR_HPM_I2C_RESOURCE_ENTRY(index_value)             \
  {HPM_I2C##index_value, static_cast<int32_t>(index_value),   \
   static_cast<uint8_t>(LIBXR_HPM_I2C_DMA_SRC_##index_value), \
   static_cast<int32_t>(LIBXR_HPM_I2C_IRQ_##index_value)}

static const Resource kResources[] = {
#if defined(HPM_I2C0)
    LIBXR_HPM_I2C_RESOURCE_ENTRY(0),
#endif
#if defined(HPM_I2C1)
    LIBXR_HPM_I2C_RESOURCE_ENTRY(1),
#endif
#if defined(HPM_I2C2)
    LIBXR_HPM_I2C_RESOURCE_ENTRY(2),
#endif
#if defined(HPM_I2C3)
    LIBXR_HPM_I2C_RESOURCE_ENTRY(3),
#endif
#if defined(HPM_I2C4)
    LIBXR_HPM_I2C_RESOURCE_ENTRY(4),
#endif
#if defined(HPM_I2C5)
    LIBXR_HPM_I2C_RESOURCE_ENTRY(5),
#endif
#if defined(HPM_I2C6)
    LIBXR_HPM_I2C_RESOURCE_ENTRY(6),
#endif
#if defined(HPM_I2C7)
    LIBXR_HPM_I2C_RESOURCE_ENTRY(7),
#endif
    {nullptr, kInvalidIndex, kInvalidDmaSource, kInvalidIrq},
};

#undef LIBXR_HPM_I2C_RESOURCE_ENTRY
#undef LIBXR_HPM_I2C_IRQ_7
#undef LIBXR_HPM_I2C_IRQ_6
#undef LIBXR_HPM_I2C_IRQ_5
#undef LIBXR_HPM_I2C_IRQ_4
#undef LIBXR_HPM_I2C_IRQ_3
#undef LIBXR_HPM_I2C_IRQ_2
#undef LIBXR_HPM_I2C_IRQ_1
#undef LIBXR_HPM_I2C_IRQ_0
#undef LIBXR_HPM_I2C_DMA_SRC_7
#undef LIBXR_HPM_I2C_DMA_SRC_6
#undef LIBXR_HPM_I2C_DMA_SRC_5
#undef LIBXR_HPM_I2C_DMA_SRC_4
#undef LIBXR_HPM_I2C_DMA_SRC_3
#undef LIBXR_HPM_I2C_DMA_SRC_2
#undef LIBXR_HPM_I2C_DMA_SRC_1
#undef LIBXR_HPM_I2C_DMA_SRC_0

static const Resource* Find(I2C_Type* i2c)
{
  if (i2c == nullptr)
  {
    return nullptr;
  }

  for (size_t i = 0U; i < (sizeof(kResources) / sizeof(kResources[0])); ++i)
  {
    if (i2c == kResources[i].base)
    {
      return &kResources[i];
    }
  }
  return nullptr;
}

static uint8_t ResolveI2cDmaSource(I2C_Type* i2c)
{
  const Resource* resource = Find(i2c);
  return resource != nullptr ? resource->dma_source : kInvalidDmaSource;
}

static int32_t ResolveI2cIndex(I2C_Type* i2c)
{
  const Resource* resource = Find(i2c);
  return resource != nullptr ? resource->index : kInvalidIndex;
}

static int32_t ResolveI2cIrq(I2C_Type* i2c)
{
  const Resource* resource = Find(i2c);
  return resource != nullptr ? resource->irq : kInvalidIrq;
}

}  // namespace HPMI2CPlatform
}  // namespace LibXR
#endif
