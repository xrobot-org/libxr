# LibXR target selection for ESP-IDF
set(LIBXR_SYSTEM freertos)
set(LIBXR_DRIVER esp)
set(LIBXR_STATIC_BUILD ON)

# Preserve an existing policy; otherwise follow the active ESP-IDF core configuration.
if(NOT DEFINED LIBXR_SINGLE_CORE)
  if(NOT DEFINED CONFIG_SOC_CPU_CORES_NUM)
    message(FATAL_ERROR "ESP-IDF did not publish CONFIG_SOC_CPU_CORES_NUM")
  endif()

  if(CONFIG_SOC_CPU_CORES_NUM GREATER 1 AND NOT CONFIG_FREERTOS_UNICORE)
    set(LIBXR_SINGLE_CORE OFF)
  else()
    set(LIBXR_SINGLE_CORE ON)
  endif()
endif()

if(NOT TARGET xr)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/.. ${CMAKE_BINARY_DIR}/libxr_build)
endif()

target_compile_definitions(xr
  PUBLIC ESP_PLATFORM=1
)

# Shared ESP driver definitions header.
# Keep the global include so generic FreeRTOS-based translation units also
# see the same ESP-specific macro hygiene as the driver headers.
target_compile_options(xr
  PRIVATE
  -include
  esp_def.hpp
)

# IDF's own headers (e.g. hal/adc_ll.h on ESP32-P4) use designated initializers
# that trip -Werror=missing-field-initializers under GCC >= 15. Keep them
# non-fatal instead of weakening the project-wide -Werror policy.
target_compile_options(xr
  PRIVATE
  -Wno-error=missing-field-initializers
)

# Prefer official ESP-IDF component targets so headers/libs are managed by IDF.
set(_libxr_idf_components
  freertos
  driver
  hal
  usb
  esp_hal_uart
  esp_hal_gpspi
  esp_hal_timg
  esp_hal_wdt
  esp_hal_systimer
  esp_driver_uart
  esp_driver_spi
  esp_hw_support
  esp_timer
  esp_event
  esp_netif
  esp_wifi
  esp_adc
  nvs_flash
)

set(_libxr_idf_targets "")
foreach(_component IN LISTS _libxr_idf_components)
  if(TARGET idf::${_component})
    list(APPEND _libxr_idf_targets idf::${_component})
  endif()
endforeach()

# Newer IDF versions split driver components; link them when available.
foreach(_component IN ITEMS esp_driver_gpio esp_driver_ledc)
  if(TARGET idf::${_component})
    list(APPEND _libxr_idf_targets idf::${_component})
  endif()
endforeach()
list(REMOVE_DUPLICATES _libxr_idf_targets)
list(SORT _libxr_idf_targets)

if(NOT TARGET idf::freertos)
  message(FATAL_ERROR "libxr esp32.cmake must be used under ESP-IDF (missing idf::freertos)")
endif()

target_link_libraries(xr
  PUBLIC ${_libxr_idf_targets}
)

# IDF exports FreeRTOS include roots, but LibXR uses `#include "FreeRTOS.h"`
# (without `freertos/` prefix). Keep a narrow compatibility fallback so this
# integration works across IDF versions/layout changes.
set(_libxr_idf_path "")
if(DEFINED IDF_PATH)
  set(_libxr_idf_path "${IDF_PATH}")
elseif(DEFINED ENV{IDF_PATH})
  set(_libxr_idf_path "$ENV{IDF_PATH}")
endif()

if(_libxr_idf_path)
  set(_libxr_freertos_compat_dirs
    "${_libxr_idf_path}/components/freertos/include/freertos"
    "${_libxr_idf_path}/components/freertos/FreeRTOS-Kernel/include/freertos"
  )

  foreach(_dir IN LISTS _libxr_freertos_compat_dirs)
    if(EXISTS "${_dir}/FreeRTOS.h")
      target_include_directories(xr
        PUBLIC "${_dir}"
      )
    endif()
  endforeach()
endif()

target_link_libraries(${COMPONENT_LIB}
  PUBLIC xr
)
