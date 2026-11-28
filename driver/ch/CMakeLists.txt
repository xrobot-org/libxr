if(DEFINED LIBXR_CH32_CONFIG_FILE)
  target_compile_definitions(
    ${PROJECT_NAME} PUBLIC LIBXR_CH32_CONFIG_FILE=${LIBXR_CH32_CONFIG_FILE}
  )
else()
  message(
    FATAL_ERROR
      "Please set LIBXR_CH32_CONFIG_FILE. e.g. set(LIBXR_CH32_CONFIG_FILE=ch32v30x_conf.h)"
  )
endif()

file(GLOB ${PROJECT_NAME}_DRIVER_CPP_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
)
list(SORT ${PROJECT_NAME}_DRIVER_CPP_SOURCES)

file(GLOB ${PROJECT_NAME}_DRIVER_C_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_LIST_DIR}/*.c"
)
list(SORT ${PROJECT_NAME}_DRIVER_C_SOURCES)

target_sources(
  ${PROJECT_NAME}
  PRIVATE ${${PROJECT_NAME}_DRIVER_CPP_SOURCES}
  PRIVATE ${${PROJECT_NAME}_DRIVER_C_SOURCES}
)

# 原子补丁提供编译期可能新生成的 __atomic_* 符号，须关闭 LTO 以保持原生目标文件可见。
# Atomic shims satisfy compiler-synthesized __atomic_* calls during LTO codegen; keep
# them as native objects with -fno-lto.
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  set_property(
    SOURCE "${CMAKE_CURRENT_LIST_DIR}/ch32_atomic_shim.c"
    APPEND
    PROPERTY COMPILE_OPTIONS "-fno-lto"
  )
endif()

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})

if(NOT DEFINED LIBXR_FASTCOPY_POLICY OR "${LIBXR_FASTCOPY_POLICY}" STREQUAL "")
  set(LIBXR_FASTCOPY_POLICY SHIFTED_ALL)
endif()
