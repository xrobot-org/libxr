file(GLOB ${PROJECT_NAME}_DRIVER_SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cpp")

target_sources(${PROJECT_NAME} PRIVATE ${${PROJECT_NAME}_DRIVER_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})

find_package(PkgConfig QUIET)

find_library(WPA_CLIENT_LIB wpa_client)

if(WPA_CLIENT_LIB AND PkgConfig_FOUND)
  message(STATUS "wpa_client found.")
  target_compile_definitions(xr PUBLIC HAVE_WPA_CLIENT)
  target_link_libraries(xr PUBLIC ${WPA_CLIENT_LIB})
  pkg_check_modules(LIBNM REQUIRED libnm)
  target_compile_options(xr PUBLIC ${LIBNM_CFLAGS_OTHER})
  target_include_directories(xr PUBLIC ${LIBNM_INCLUDE_DIRS})
  target_link_libraries(xr PUBLIC ${LIBNM_LIBRARIES})
else()
  message(WARNING "wpa_client not found.")
  message(
    WARNING
      "Use `sudo apt install libwpa-client-dev libnm-dev` to install wpa_client and libnm."
  )
endif()

if(PkgConfig_FOUND)
  pkg_check_modules(LIBUDEV QUIET libudev)
endif()
if(LIBUDEV_FOUND)
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${LIBUDEV_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${LIBUDEV_LIBRARIES})
else()
  message(STATUS "libudev development files not found; Linux SPI/GPIO build continues without udev linkage.")
endif()
