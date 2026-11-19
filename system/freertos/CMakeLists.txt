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

# Tick source of DriverAsyncCondition (tick.cpp, see libxr_system.hpp). ESP-IDF uses the
# IDF tick hook instead and is not wrapped.
if(NOT _xr_driver STREQUAL "esp")
  target_link_options(${PROJECT_NAME} INTERFACE "-Wl,--wrap=xTaskIncrementTick")
  # tasks.c defines the wrapped function (see cmake/keep_out_of_lto.cmake).
  xr_keep_sources_out_of_lto("FreeRTOS.*/tasks\\.c$")
endif()
