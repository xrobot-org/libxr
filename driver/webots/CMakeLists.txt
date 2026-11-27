file(
  GLOB ${PROJECT_NAME}_DRIVER_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
)
list(SORT ${PROJECT_NAME}_DRIVER_SOURCES)

target_sources(${PROJECT_NAME}
  PRIVATE ${${PROJECT_NAME}_DRIVER_SOURCES}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC ${CMAKE_CURRENT_LIST_DIR}
)

foreach(_xr_mem_var LIBXR_FASTCOPY_POLICY LIBXR_FASTMOVE_POLICY
                    LIBXR_FASTSET_POLICY LIBXR_FASTCMP_POLICY)
  if(NOT DEFINED ${_xr_mem_var} OR "${${_xr_mem_var}}" STREQUAL "")
    set(${_xr_mem_var} STD_LIBC)
  endif()
endforeach()
