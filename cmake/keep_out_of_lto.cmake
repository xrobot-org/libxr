# --wrap replaces a symbol when the linker resolves it. With link-time optimization the
# compiler can bind a call to the wrapped function before the linker sees it (a direct
# call or inlined code), and the wrapper is skipped for that call; which calls are
# affected changes with the project and the optimization level. Both the source that
# defines a wrapped function and the __wrap_* wrapper translation unit itself are
# compiled without LTO in every target of the project, leaving every reference and the
# wrap glue to the linker.
#
# xr_keep_sources_out_of_lto(<regex>): every source of the project whose normalized
# absolute path matches <regex> (case-insensitively) gets -fno-lto. The patterns are
# collected and applied once, at the end of the configuration (CMake 3.19), so that
# targets created after LibXR are seen as well.

function(_xr_apply_no_lto)
  if(NOT CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    return()
  endif()
  get_property(_patterns GLOBAL PROPERTY XR_NO_LTO_PATTERNS)
  if(NOT _patterns)
    return()
  endif()

  # Every directory of the project, starting at the top.
  set(_dirs "${CMAKE_SOURCE_DIR}")
  set(_pending "${CMAKE_SOURCE_DIR}")
  while(_pending)
    list(GET _pending 0 _dir)
    list(REMOVE_AT _pending 0)
    get_property(
      _subdirs
      DIRECTORY "${_dir}"
      PROPERTY SUBDIRECTORIES
    )
    list(APPEND _dirs ${_subdirs})
    list(APPEND _pending ${_subdirs})
  endwhile()

  # The matching sources, in whichever target the project compiles them.
  set(_files "")
  foreach(_dir IN LISTS _dirs)
    get_property(
      _targets
      DIRECTORY "${_dir}"
      PROPERTY BUILDSYSTEM_TARGETS
    )
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
        if(_source MATCHES "^\\$<")
          continue()
        endif()
        get_filename_component(_file "${_source}" ABSOLUTE BASE_DIR "${_dir}")
        string(REPLACE "\\" "/" _file_norm "${_file}")
        string(TOLOWER "${_file_norm}" _file_lower)
        foreach(_pattern IN LISTS _patterns)
          string(TOLOWER "${_pattern}" _pat_lower)
          if(_file_lower MATCHES "${_pat_lower}")
            list(APPEND _files "${_file}")
          endif()
        endforeach()
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

function(xr_keep_sources_out_of_lto pattern)
  # The source property of another directory needs CMake 3.18.
  if(CMAKE_VERSION VERSION_LESS 3.18)
    if(CMAKE_INTERPROCEDURAL_OPTIMIZATION OR "${CMAKE_C_FLAGS}" MATCHES "-flto")
      message(
        FATAL_ERROR
          "LibXR wraps functions with --wrap, which link-time optimization bypasses. "
          "Use CMake 3.18 or newer, or compile the sources matching '${pattern}' with "
          "-fno-lto."
      )
    endif()
    return()
  endif()
  set_property(GLOBAL APPEND PROPERTY XR_NO_LTO_PATTERNS "${pattern}")
  if(CMAKE_VERSION VERSION_LESS 3.19)
    _xr_apply_no_lto()
    return()
  endif()
  get_property(_deferred GLOBAL PROPERTY XR_NO_LTO_DEFERRED)
  if(NOT _deferred)
    set_property(GLOBAL PROPERTY XR_NO_LTO_DEFERRED TRUE)
    cmake_language(DEFER DIRECTORY "${CMAKE_SOURCE_DIR}" CALL _xr_apply_no_lto)
  endif()
endfunction()
