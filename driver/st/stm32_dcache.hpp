#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "main.h"

namespace LibXR
{
/**
 * @brief D-Cache API 可直接接受 `void*` / D-Cache API accepts `void*`
 */
template <typename FunctionType>
concept DCacheFunctionAcceptsVoidPtr = std::is_invocable_v<FunctionType, void*, int32_t>;

/**
 * @brief D-Cache API 可直接接受 `volatile void*` / D-Cache API accepts `volatile void*`
 */
template <typename FunctionType>
concept DCacheFunctionAcceptsVolatileVoidPtr =
    std::is_invocable_v<FunctionType, volatile void*, int32_t>;

/**
 * @brief 按当前工具链接受的指针类型调用 CMSIS D-Cache 接口
 *        Calls the CMSIS D-Cache helper with the pointer type accepted by the
 *        current toolchain
 */
template <typename FunctionType>
  requires DCacheFunctionAcceptsVoidPtr<FunctionType>
inline void STM32_CallDCacheByAddr(FunctionType function, void* addr, int32_t dsize)
{
  function(addr, dsize);
}

/**
 * @brief 按 `volatile void*` 指针类型调用 CMSIS D-Cache 接口
 *        Calls the CMSIS D-Cache helper accepting a `volatile void*` pointer.
 * @tparam FunctionType CMSIS D-Cache 函数指针/可调用类型 / CMSIS D-Cache callable type.
 * @param function CMSIS D-Cache 操作函数 / CMSIS D-Cache function.
 * @param addr 目标内存起始地址 / Target memory base address.
 * @param dsize 操作字节数 / Byte size.
 */
template <typename FunctionType>
  requires(!DCacheFunctionAcceptsVoidPtr<FunctionType> &&
           DCacheFunctionAcceptsVolatileVoidPtr<FunctionType>)
inline void STM32_CallDCacheByAddr(FunctionType function, void* addr, int32_t dsize)
{
  function(reinterpret_cast<volatile void*>(addr), dsize);
}

/**
 * @brief 按 `uint32_t*` 指针类型调用 CMSIS D-Cache 接口
 *        Calls the CMSIS D-Cache helper accepting a `uint32_t*` pointer.
 * @tparam FunctionType CMSIS D-Cache 函数指针/可调用类型 / CMSIS D-Cache callable type.
 * @param function CMSIS D-Cache 操作函数 / CMSIS D-Cache function.
 * @param addr 目标内存起始地址 / Target memory base address.
 * @param dsize 操作字节数 / Byte size.
 */
template <typename FunctionType>
  requires(!DCacheFunctionAcceptsVoidPtr<FunctionType> &&
           !DCacheFunctionAcceptsVolatileVoidPtr<FunctionType>)
inline void STM32_CallDCacheByAddr(FunctionType function, void* addr, int32_t dsize)
{
  function(reinterpret_cast<uint32_t*>(addr), dsize);
}

/**
 * @brief 清理指定内存范围覆盖的 D-Cache cache line
 *        Cleans D-Cache lines covering the specified memory range
 */
inline void STM32_CleanDCacheByAddr(const void* addr, size_t size)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  auto* raw = const_cast<void*>(addr);
  const auto dsize = static_cast<int32_t>(size);
  STM32_CallDCacheByAddr(&SCB_CleanDCache_by_Addr, raw, dsize);
#else
  (void)addr;
  (void)size;
#endif
}

/**
 * @brief 失效指定内存范围覆盖的 D-Cache cache line
 *        Invalidates D-Cache lines covering the specified memory range
 */
inline void STM32_InvalidateDCacheByAddr(const void* addr, size_t size)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  auto* raw = const_cast<void*>(addr);
  const auto dsize = static_cast<int32_t>(size);
  STM32_CallDCacheByAddr(&SCB_InvalidateDCache_by_Addr, raw, dsize);
#else
  (void)addr;
  (void)size;
#endif
}
}  // namespace LibXR
