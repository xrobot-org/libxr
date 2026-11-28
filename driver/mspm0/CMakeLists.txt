set(_xr_mspm0_driver_sources
    ${CMAKE_CURRENT_LIST_DIR}/mspm0_group1_shared.cpp
    ${CMAKE_CURRENT_LIST_DIR}/mspm0_gpio.cpp
    ${CMAKE_CURRENT_LIST_DIR}/mspm0_pwm.cpp
    ${CMAKE_CURRENT_LIST_DIR}/mspm0_timebase.cpp
    ${CMAKE_CURRENT_LIST_DIR}/mspm0_power.cpp
    ${CMAKE_CURRENT_LIST_DIR}/mspm0_spi.cpp
    ${CMAKE_CURRENT_LIST_DIR}/mspm0_i2c.cpp
    ${CMAKE_CURRENT_LIST_DIR}/mspm0_uart.cpp
    ${CMAKE_CURRENT_LIST_DIR}/mspm0_atomic_shim.c
    ${CMAKE_CURRENT_LIST_DIR}/mspm0_syscalls.c
)

target_sources(${PROJECT_NAME} PRIVATE ${_xr_mspm0_driver_sources})

# 原子补丁提供编译期可能新生成的 __atomic_* 符号，须关闭 LTO 以保持原生目标文件可见。
# Atomic shims satisfy compiler-synthesized __atomic_* calls during LTO codegen; keep
# them as native objects with -fno-lto.
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  set_property(
    SOURCE "${CMAKE_CURRENT_LIST_DIR}/mspm0_atomic_shim.c"
    APPEND
    PROPERTY COMPILE_OPTIONS "-fno-lto"
  )
endif()
