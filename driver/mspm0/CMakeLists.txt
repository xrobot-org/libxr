set(_xr_mspm0_driver_sources
    ${CMAKE_CURRENT_LIST_DIR}/mspm0_group1_shared.cpp
    ${CMAKE_CURRENT_LIST_DIR}/mspm0_gpio.cpp
    ${CMAKE_CURRENT_LIST_DIR}/mspm0_pwm.cpp
    ${CMAKE_CURRENT_LIST_DIR}/mspm0_timebase.cpp
    ${CMAKE_CURRENT_LIST_DIR}/mspm0_spi.cpp
    ${CMAKE_CURRENT_LIST_DIR}/mspm0_i2c.cpp
    ${CMAKE_CURRENT_LIST_DIR}/mspm0_uart.cpp
    ${CMAKE_CURRENT_LIST_DIR}/mspm0_atomic_shim.c
)

target_sources(${PROJECT_NAME} PRIVATE ${_xr_mspm0_driver_sources})
