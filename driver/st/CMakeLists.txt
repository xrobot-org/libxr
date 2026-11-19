file(GLOB ${PROJECT_NAME}_DRIVER_CPP_SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cpp")

file(GLOB ${PROJECT_NAME}_DRIVER_C_SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.c")

target_sources(
  ${PROJECT_NAME}
  PRIVATE ${${PROJECT_NAME}_DRIVER_CPP_SOURCES}
  PRIVATE ${${PROJECT_NAME}_DRIVER_C_SOURCES})

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
endif()

# The HAL source that defines both wrapped PCD functions stays out of link-time
# optimization (see cmake/keep_out_of_lto.cmake).
xr_keep_sources_out_of_lto("stm32[a-z0-9]+xx_hal_pcd\\.c$")
