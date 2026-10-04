# XRobot integration of a BSP: the Module list, the generated entry header and its
# freshness check. Included by the LibXR CMakeLists.txt after the xr target exists.

if(XROBOT_MODULES_DIR)
  get_filename_component(_xr_bsp_root "${XROBOT_MODULES_DIR}" DIRECTORY)

  # The entry header location is fixed; reject the removed per-BSP header list.
  if(DEFINED XROBOT_MAIN_HEADERS)
    message(
      FATAL_ERROR
        "[XRobot] XROBOT_MAIN_HEADERS is no longer supported. The generated entry is "
        "always ${_xr_bsp_root}/User/xrobot_main.hpp; remove XROBOT_MAIN_HEADERS "
        "from the BSP CMake."
    )
  endif()

  # cmake_language(DEFER) below needs CMake 3.19.
  if(CMAKE_VERSION VERSION_LESS 3.19)
    message(
      FATAL_ERROR
        "[XRobot] XRobot integration requires CMake 3.19 or newer (found ${CMAKE_VERSION})."
    )
  endif()

  if(NOT EXISTS "${XROBOT_MODULES_DIR}/CMakeLists.txt")
    message(
      FATAL_ERROR
        "[XRobot] ${XROBOT_MODULES_DIR}/CMakeLists.txt is missing. "
        "Run `xrobot setup` (xrobot 1.0 or newer) in ${_xr_bsp_root}. "
        "A project without XRobot does not set XROBOT_MODULES_DIR; for an STM32CubeMX "
        "project, `libxr stm32 setup` (libxr 6.0 or newer) removes it."
    )
  endif()
  include("${XROBOT_MODULES_DIR}/CMakeLists.txt")

  get_property(_xr_module_deps GLOBAL PROPERTY XR_MODULE_DEPS)
  if(_xr_module_deps)
    list(REMOVE_DUPLICATES _xr_module_deps)
    list(SORT _xr_module_deps)
    target_link_libraries(${PROJECT_NAME} PUBLIC ${_xr_module_deps})
  endif()

  # User/xrobot_main.hpp names its inputs in "// xrobot:" lines: exactly one
  # `config "<path>"`, then `entry "<path>"` and `depends "<path>"` lines, and a
  # `digest <sha256>` line. Paths are relative to the header directory unless absolute.
  set(_xr_main_header "${_xr_bsp_root}/User/xrobot_main.hpp")
  if(NOT EXISTS "${_xr_main_header}")
    message(
      FATAL_ERROR
        "[XRobot] ${_xr_main_header} is missing. Run `xrobot setup` in ${_xr_bsp_root}, "
        "or `xrobot gen -c <config>` there to select a product."
    )
  endif()
  set_property(
    DIRECTORY
    APPEND
    PROPERTY CMAKE_CONFIGURE_DEPENDS "${_xr_main_header}"
  )

  get_filename_component(_xr_main_header_dir "${_xr_main_header}" DIRECTORY)
  file(STRINGS "${_xr_main_header}" _xr_directives REGEX "^// xrobot:" ENCODING UTF-8)
  set(_xr_config "")
  set(_xr_depends "")
  foreach(_xr_directive IN LISTS _xr_directives)
    if(_xr_directive MATCHES "^// xrobot: digest [0-9a-f]+$")
      continue()
    endif()
    if(NOT _xr_directive MATCHES "^// xrobot: (config|entry|depends) \"([^\"]+)\"$")
      message(
        FATAL_ERROR "[XRobot] ${_xr_main_header}: unrecognized line: ${_xr_directive}"
      )
    endif()
    set(_xr_kind "${CMAKE_MATCH_1}")
    set(_xr_path "${CMAKE_MATCH_2}")
    if(NOT IS_ABSOLUTE "${_xr_path}")
      set(_xr_path "${_xr_main_header_dir}/${_xr_path}")
    endif()
    get_filename_component(_xr_path "${_xr_path}" ABSOLUTE)
    if(_xr_kind STREQUAL "config")
      if(NOT _xr_config STREQUAL "")
        message(FATAL_ERROR "[XRobot] ${_xr_main_header}: more than one config line.")
      endif()
      set(_xr_config "${_xr_path}")
    else()
      list(APPEND _xr_depends "${_xr_path}")
    endif()
  endforeach()
  if(_xr_config STREQUAL "")
    message(
      FATAL_ERROR
        "[XRobot] ${_xr_main_header}: missing `// xrobot: config` line. "
        "Run `xrobot gen -c <config>` in ${_xr_bsp_root}."
    )
  endif()
  if(NOT _xr_depends)
    message(
      FATAL_ERROR
        "[XRobot] ${_xr_main_header}: missing `// xrobot: depends` line. "
        "Run `xrobot gen -c <config>` in ${_xr_bsp_root}."
    )
  endif()

  file(RELATIVE_PATH _xr_config_arg "${_xr_bsp_root}" "${_xr_config}")
  message(STATUS "XRobot product: ${_xr_config_arg}")

  # Every build compares the content of the header's inputs with the digest it records,
  # before compiling xr. It reports, never regenerates.
  add_custom_target(
    xrobot_freshness_check ALL
    COMMAND
      ${CMAKE_COMMAND} "-DXROBOT_MAIN_HEADER=${_xr_main_header}"
      "-DXROBOT_BSP_ROOT=${_xr_bsp_root}" -P
      "${CMAKE_CURRENT_LIST_DIR}/XRobotFreshness.cmake"
    COMMENT "Checking that the generated XRobot entry is up to date"
    VERBATIM
  )
  add_dependencies(${PROJECT_NAME} xrobot_freshness_check)

  # After each build of the top-level executable (STM32 CubeMX BSPs name it
  # ${CMAKE_PROJECT_NAME}), print the product it was built for. The target may be
  # created after this point, so the check runs at the end of the top-level directory.
  # CMake attaches POST_BUILD commands only in the directory that created the target.
  function(_xrobot_report_product product)
    if(NOT TARGET "${CMAKE_PROJECT_NAME}")
      return()
    endif()
    get_target_property(_xr_type "${CMAKE_PROJECT_NAME}" TYPE)
    get_target_property(_xr_source_dir "${CMAKE_PROJECT_NAME}" SOURCE_DIR)
    if(_xr_type STREQUAL "EXECUTABLE" AND _xr_source_dir STREQUAL "${CMAKE_SOURCE_DIR}")
      add_custom_command(
        TARGET "${CMAKE_PROJECT_NAME}"
        POST_BUILD
        COMMAND
          ${CMAKE_COMMAND} -E echo
          "$<TARGET_FILE_NAME:${CMAKE_PROJECT_NAME}> built for product: ${product}"
        VERBATIM
      )
    endif()
  endfunction()
  get_filename_component(_xr_product "${_xr_config}" NAME_WLE)
  # Deferred call arguments are evaluated when the call runs; bind the value now.
  cmake_language(
    EVAL
    CODE
    "cmake_language(DEFER DIRECTORY [[${CMAKE_SOURCE_DIR}]] CALL _xrobot_report_product [[${_xr_product}]])"
  )
endif()
