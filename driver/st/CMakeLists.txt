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

# --wrap replaces a symbol when the linker resolves it. With link-time optimization the
# compiler resolves a call to a function whose definition it can see before the linker
# does: ST's function is inlined into the CubeMX code and the wrapper is never called.
# The HAL source that defines both wrapped functions is therefore compiled without LTO
# in every target of the project; its callers and LibXR itself can stay in LTO.
function(_xr_stm32_keep_hal_pcd_out_of_lto)
  if(NOT CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    return()
  endif()

  # Every directory of the project, starting at the top.
  set(_dirs "${CMAKE_SOURCE_DIR}")
  set(_pending "${CMAKE_SOURCE_DIR}")
  while(_pending)
    list(GET _pending 0 _dir)
    list(REMOVE_AT _pending 0)
    get_property(_subdirs DIRECTORY "${_dir}" PROPERTY SUBDIRECTORIES)
    list(APPEND _dirs ${_subdirs})
    list(APPEND _pending ${_subdirs})
  endwhile()

  # The HAL PCD source, in whichever target the CubeMX project compiles it.
  set(_files "")
  foreach(_dir IN LISTS _dirs)
    get_property(_targets DIRECTORY "${_dir}" PROPERTY BUILDSYSTEM_TARGETS)
    foreach(_target IN LISTS _targets)
      get_target_property(_type ${_target} TYPE)
      set(_sources "")
      if(NOT _type STREQUAL "INTERFACE_LIBRARY")
        get_target_property(_own ${_target} SOURCES)
        if(_own)
          list(APPEND _sources ${_own})
        endif()
      endif()
      get_target_property(_interface ${_target} INTERFACE_SOURCES)
      if(_interface)
        list(APPEND _sources ${_interface})
      endif()
      foreach(_source IN LISTS _sources)
        if(_source MATCHES "stm32[a-z0-9]+xx_hal_pcd\\.c$")
          get_filename_component(_file "${_source}" ABSOLUTE BASE_DIR "${_dir}")
          list(APPEND _files "${_file}")
        endif()
      endforeach()
    endforeach()
  endforeach()

  if(_files)
    list(REMOVE_DUPLICATES _files)
    set_property(
      SOURCE ${_files}
      DIRECTORY ${_dirs}
      APPEND
      PROPERTY COMPILE_OPTIONS "-fno-lto"
    )
  endif()
endfunction()

# The source property of another directory needs CMake 3.18; run at the end of the
# configuration (CMake 3.19) so that targets created after LibXR are seen as well.
if(CMAKE_VERSION VERSION_LESS 3.18)
  if(CMAKE_INTERPROCEDURAL_OPTIMIZATION OR "${CMAKE_C_FLAGS}" MATCHES "-flto")
    message(
      FATAL_ERROR
        "LibXR takes over the STM32 USB controller with --wrap, which link-time "
        "optimization bypasses. Use CMake 3.18 or newer, or compile "
        "stm32*_hal_pcd.c with -fno-lto."
    )
  endif()
elseif(CMAKE_VERSION VERSION_LESS 3.19)
  _xr_stm32_keep_hal_pcd_out_of_lto()
else()
  cmake_language(DEFER DIRECTORY "${CMAKE_SOURCE_DIR}" CALL _xr_stm32_keep_hal_pcd_out_of_lto)
endif()
