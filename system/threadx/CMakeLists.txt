file(
  GLOB ${PROJECT_NAME}_SYSTEM_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
)
list(SORT ${PROJECT_NAME}_SYSTEM_SOURCES)

target_sources(${PROJECT_NAME}
  PRIVATE ${${PROJECT_NAME}_SYSTEM_SOURCES}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC ${CMAKE_CURRENT_LIST_DIR}
)

target_compile_definitions(${PROJECT_NAME}
  PUBLIC UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE=1
)

# Tick source of DriverAsyncCondition (tick.cpp, see libxr_system.hpp). The wrapped
# function is defined in assembly, so LTO does not apply.
target_link_options(${PROJECT_NAME} INTERFACE "-Wl,--wrap=_tx_timer_interrupt")
