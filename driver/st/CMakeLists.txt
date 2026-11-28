file(GLOB ${PROJECT_NAME}_DRIVER_CPP_SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cpp")

file(GLOB ${PROJECT_NAME}_DRIVER_C_SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.c")

target_sources(
  ${PROJECT_NAME}
  PRIVATE ${${PROJECT_NAME}_DRIVER_CPP_SOURCES}
  PRIVATE ${${PROJECT_NAME}_DRIVER_C_SOURCES}
)

# 原子补丁提供编译期可能新生成的 __atomic_* 符号，须关闭 LTO 以保持原生目标文件可见。
# Atomic shims satisfy compiler-synthesized __atomic_* calls during LTO codegen; keep
# them as native objects with -fno-lto.
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  set_property(
    SOURCE "${CMAKE_CURRENT_LIST_DIR}/stm32_atomic_shim.c"
    APPEND
    PROPERTY COMPILE_OPTIONS "-fno-lto"
  )
endif()

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})

# USB: the device stack wraps HAL_PCD_Init and HAL_PCD_IRQHandler so the BSP's CubeMX
# code stays as generated (stm32_usb_wrap.cpp covers both OTG and FSDEV/DRD).
target_link_options(
  ${PROJECT_NAME} INTERFACE "-Wl,--wrap=HAL_PCD_Init" "-Wl,--wrap=HAL_PCD_IRQHandler"
)

# Bare metal only: the HAL tick drives DriverAsyncCondition::OnTick (__wrap_HAL_IncTick,
# see stm32_timebase.hpp). Under FreeRTOS or ThreadX the system layer provides the tick
# and HAL_IncTick is not wrapped.
if(_xr_system STREQUAL "none")
  target_link_options(${PROJECT_NAME} INTERFACE "-Wl,--wrap=HAL_IncTick")
  xr_keep_sources_out_of_lto("stm32[a-z0-9]+xx_hal\\.c$")
  xr_keep_sources_out_of_lto("driver/st/stm32_hal_tick\\.cpp$")
endif()

# The HAL source that defines both wrapped PCD functions and the wrapper translation unit
# stay out of link-time optimization (see cmake/keep_out_of_lto.cmake).
xr_keep_sources_out_of_lto("stm32[a-z0-9]+xx_hal_pcd\\.c$")
xr_keep_sources_out_of_lto("driver/st/stm32_usb_wrap\\.cpp$")
