cmake_minimum_required(VERSION 3.12)

project(xr LANGUAGES C CXX ASM)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(LIBXR_DEV_ASSERT_BUILD "Enable LibXR maintainer-only development assertions" OFF)
if(LIBXR_SHARED_BUILD)
  add_library(${PROJECT_NAME} SHARED)
elseif(LIBXR_STATIC_BUILD)
  add_library(${PROJECT_NAME} STATIC)
elseif(LIBXR_OBJECT_BUILD)
  add_library(${PROJECT_NAME} OBJECT)
else()
  add_library(${PROJECT_NAME} STATIC)
endif()

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

target_compile_options(
  ${PROJECT_NAME}
  PUBLIC -Wno-psabi
  PUBLIC -fmacro-prefix-map=${CMAKE_SOURCE_DIR}=.
)

include(cmake/config.cmake)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_definitions(${PROJECT_NAME} PUBLIC LIBXR_DEBUG_BUILD)
endif()

if(LIBXR_DEV_ASSERT_BUILD)
  target_compile_definitions(${PROJECT_NAME} PUBLIC LIBXR_DEV_ASSERT_BUILD)
endif()

target_compile_definitions(${PROJECT_NAME} PUBLIC LIBXR_SINGLE_CORE=$<BOOL:${LIBXR_SINGLE_CORE}>)

file(GLOB ${PROJECT_NAME}_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)
list(SORT ${PROJECT_NAME}_SOURCES)

file(GLOB ${PROJECT_NAME}_CORE_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/core/*.cpp"
)
list(SORT ${PROJECT_NAME}_CORE_SOURCES)

file(GLOB ${PROJECT_NAME}_SYSTEM_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/system/*.cpp"
)
list(SORT ${PROJECT_NAME}_SYSTEM_SOURCES)

file(GLOB ${PROJECT_NAME}_DRIVER_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/driver/*.cpp"
)
list(SORT ${PROJECT_NAME}_DRIVER_SOURCES)

file(GLOB ${PROJECT_NAME}_UTILS_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/utils/*.cpp"
)
list(SORT ${PROJECT_NAME}_UTILS_SOURCES)

file(GLOB ${PROJECT_NAME}_STRUCTURE_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/structure/*.cpp"
)
list(SORT ${PROJECT_NAME}_STRUCTURE_SOURCES)

file(GLOB ${PROJECT_NAME}_STRUCTURE_SUBDIR_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/structure/*/*.cpp"
)
list(SORT ${PROJECT_NAME}_STRUCTURE_SUBDIR_SOURCES)

file(GLOB ${PROJECT_NAME}_MIDDLEWARE_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/middleware/*.cpp"
)
list(SORT ${PROJECT_NAME}_MIDDLEWARE_SOURCES)

file(GLOB ${PROJECT_NAME}_MIDDLEWARE_SUBDIR_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/middleware/*/*.cpp"
)
list(SORT ${PROJECT_NAME}_MIDDLEWARE_SUBDIR_SOURCES)

file(GLOB ${PROJECT_NAME}_MIDDLEWARE_NESTED_SUBDIR_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/middleware/*/*/*.cpp"
)
list(SORT ${PROJECT_NAME}_MIDDLEWARE_NESTED_SUBDIR_SOURCES)

target_sources(
  ${PROJECT_NAME}
  PRIVATE ${${PROJECT_NAME}_SOURCES}
  PRIVATE ${${PROJECT_NAME}_CORE_SOURCES}
  PRIVATE ${${PROJECT_NAME}_SYSTEM_SOURCES}
  PRIVATE ${${PROJECT_NAME}_DRIVER_SOURCES}
  PRIVATE ${${PROJECT_NAME}_SYSTEM_EXT_SOURCES}
  PRIVATE ${${PROJECT_NAME}_UTILS_SOURCES}
  PRIVATE ${${PROJECT_NAME}_STRUCTURE_SOURCES}
  PRIVATE ${${PROJECT_NAME}_STRUCTURE_SUBDIR_SOURCES}
  PRIVATE ${${PROJECT_NAME}_MIDDLEWARE_SOURCES}
  PRIVATE ${${PROJECT_NAME}_MIDDLEWARE_SUBDIR_SOURCES}
  PRIVATE ${${PROJECT_NAME}_MIDDLEWARE_NESTED_SUBDIR_SOURCES}
)

file(GLOB_RECURSE _xr_force_o3_sources CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*_o3.cpp"
)
list(SORT _xr_force_o3_sources)

file(GLOB_RECURSE _xr_force_os_sources CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*_os.cpp"
)
list(SORT _xr_force_os_sources)

if(_xr_force_o3_sources)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    set_property(SOURCE ${_xr_force_o3_sources} APPEND PROPERTY COMPILE_OPTIONS "/O2")
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR
         CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR
         CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    set_property(SOURCE ${_xr_force_o3_sources} APPEND PROPERTY COMPILE_OPTIONS "-O3")
  else()
    message(WARNING
            "LibXR file-level optimized sources are not configured for compiler: "
            "${CMAKE_CXX_COMPILER_ID}")
  endif()
endif()

if(_xr_force_os_sources)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    set_property(SOURCE ${_xr_force_os_sources} APPEND PROPERTY COMPILE_OPTIONS "/O1")
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR
         CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR
         CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    set_property(SOURCE ${_xr_force_os_sources} APPEND PROPERTY COMPILE_OPTIONS "-Os")
  else()
    message(WARNING
            "LibXR file-level size-optimized sources are not configured for compiler: "
            "${CMAKE_CXX_COMPILER_ID}")
  endif()
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/src/core/CMakeLists.txt)

# The current tree still relies on basename and short subtree includes across major layers.
# Keep the exported roots explicit here until the include-style migration lands.
set(_xr_public_include_dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/core
    ${CMAKE_CURRENT_SOURCE_DIR}/src/system
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/src/structure
    ${CMAKE_CURRENT_SOURCE_DIR}/src/middleware
    ${CMAKE_CURRENT_SOURCE_DIR}/src/driver
    ${CMAKE_CURRENT_SOURCE_DIR}/system/${_xr_system}
)

file(GLOB _xr_core_SUBDIRS CONFIGURE_DEPENDS LIST_DIRECTORIES true
     "${CMAKE_CURRENT_SOURCE_DIR}/src/core/*"
)
foreach(_xr_core_SUBDIR IN LISTS _xr_core_SUBDIRS)
  if(IS_DIRECTORY "${_xr_core_SUBDIR}")
    get_filename_component(_xr_core_NAME "${_xr_core_SUBDIR}" NAME)
    if(EXISTS "${_xr_core_SUBDIR}/${_xr_core_NAME}.hpp" OR
       EXISTS "${_xr_core_SUBDIR}/libxr_${_xr_core_NAME}.hpp")
      list(APPEND _xr_public_include_dirs "${_xr_core_SUBDIR}")
    endif()
  endif()
endforeach()

file(GLOB _xr_middleware_SUBDIRS CONFIGURE_DEPENDS LIST_DIRECTORIES true
     "${CMAKE_CURRENT_SOURCE_DIR}/src/middleware/*"
)
file(GLOB _xr_structure_SUBDIRS CONFIGURE_DEPENDS LIST_DIRECTORIES true
     "${CMAKE_CURRENT_SOURCE_DIR}/src/structure/*"
)
foreach(_xr_structure_SUBDIR IN LISTS _xr_structure_SUBDIRS)
  if(IS_DIRECTORY "${_xr_structure_SUBDIR}")
    get_filename_component(_xr_structure_NAME "${_xr_structure_SUBDIR}" NAME)
    if(EXISTS "${_xr_structure_SUBDIR}/${_xr_structure_NAME}.hpp")
      list(APPEND _xr_public_include_dirs "${_xr_structure_SUBDIR}")
    endif()
  endif()
endforeach()

foreach(_xr_middleware_SUBDIR IN LISTS _xr_middleware_SUBDIRS)
  if(IS_DIRECTORY "${_xr_middleware_SUBDIR}")
    get_filename_component(_xr_middleware_NAME "${_xr_middleware_SUBDIR}" NAME)
    if(EXISTS "${_xr_middleware_SUBDIR}/${_xr_middleware_NAME}.hpp")
      list(APPEND _xr_public_include_dirs "${_xr_middleware_SUBDIR}")
    endif()
  endif()
endforeach()

target_include_directories(${PROJECT_NAME} PUBLIC ${_xr_public_include_dirs})

if(_xr_driver)
  target_sources(${PROJECT_NAME} PRIVATE ${${PROJECT_NAME}_DRIVER_C_SOURCES})

  target_include_directories(
    ${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/driver/${_xr_driver}
  )
endif()

# XRUSB
if(_xr_driver)
  include(${CMAKE_CURRENT_SOURCE_DIR}/src/driver/usb/CMakeLists.txt)
endif()

if(_xr_system MATCHES "webots")
  target_link_directories(${PROJECT_NAME} PUBLIC ${WEBOTS_HOME}/lib/controller)

  target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC ${LIBRARIES}
    PUBLIC ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
    PUBLIC ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX}
  )

  target_include_directories(
    ${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC ${WEBOTS_HOME}/include/controller/c
    PUBLIC ${WEBOTS_HOME}/include/controller/cpp
    PUBLIC ${WEBOTS_HOME}/lib/controller
  )
endif()

if(LIBXR_NO_EIGEN)
  target_compile_definitions(${PROJECT_NAME} PUBLIC LIBXR_NO_EIGEN)
else()
  add_subdirectory(lib/Eigen)

  target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/Eigen)

  target_compile_definitions(${PROJECT_NAME} PUBLIC EIGEN_NO_IO)
endif()

if(XROBOT_MODULES_DIR)
  if(NOT EXISTS "${XROBOT_MODULES_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
      "XRobot source list is missing. Resolve modules before configuring the BSP.")
  endif()
  include("${XROBOT_MODULES_DIR}/CMakeLists.txt")

  # BSP toolchains may omit NDEBUG even in Release. Export the actual native
  # build profile to the generated entry without changing global optimization.
  target_compile_definitions(${PROJECT_NAME} PUBLIC
    "XROBOT_OPTIMIZED_BUILD=$<IF:$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>,$<CONFIG:MinSizeRel>>,1,0>")

  get_property(_xr_module_deps GLOBAL PROPERTY XR_MODULE_DEPS)
  if(_xr_module_deps)
    list(REMOVE_DUPLICATES _xr_module_deps)
    list(SORT _xr_module_deps)
    target_link_libraries(${PROJECT_NAME} PUBLIC ${_xr_module_deps})
  endif()
endif()

if(LIBXR_TEST_BUILD)
  enable_testing()
  add_subdirectory(test)
endif()
