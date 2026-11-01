file(GLOB ${PROJECT_NAME}_DRIVER_CPP_SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cpp")

file(GLOB ${PROJECT_NAME}_DRIVER_C_SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.c")

target_sources(
  ${PROJECT_NAME}
  PRIVATE ${${PROJECT_NAME}_DRIVER_CPP_SOURCES}
  PRIVATE ${${PROJECT_NAME}_DRIVER_C_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})

# GCC 14 reports the atomics reached through the device core's pool reference in this
# file as writes to address zero (-Wstringop-overflow); the pointers are checked non-null.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set_source_files_properties(
    "${CMAKE_CURRENT_LIST_DIR}/stm32_usb_otg.cpp" PROPERTIES COMPILE_OPTIONS
                                                             "-Wno-stringop-overflow"
  )
endif()

# USB: the device stack wraps HAL_PCD_Init and HAL_PCD_IRQHandler so the BSP's CubeMX
# code stays as generated (stm32_usb_wrap.cpp covers both OTG and FSDEV/DRD).
target_link_options(
  ${PROJECT_NAME} INTERFACE "-Wl,--wrap=HAL_PCD_Init" "-Wl,--wrap=HAL_PCD_IRQHandler"
)
