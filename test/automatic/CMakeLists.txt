if(NOT _xr_system STREQUAL "linux")
  message(FATAL_ERROR "Automatic tests require LIBXR_SYSTEM=linux.")
endif()

add_compile_options(-g)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/..")

add_library(
  test_assert_disabled OBJECT
  core/assert/libxr_assert/assert_disabled_headers.cpp ../../src/core/rw/operation.cpp
  ../../src/core/rw/read_port.cpp ../../src/core/rw/write_port.cpp
  ../../src/core/rw/write_stream.cpp
)
target_include_directories(
  test_assert_disabled PRIVATE $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(
  test_assert_disabled PRIVATE $<TARGET_PROPERTY:xr,INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(
  test_assert_disabled PRIVATE -ULIBXR_DEBUG_BUILD -ULIBXR_DEV_ASSERT_BUILD -Wall -Werror
                               -Wunused-parameter
)

foreach(debug RANGE 0 1)
  foreach(dev RANGE 0 1)
    set(mode_target test_assert_${debug}_${dev})
    add_executable(${mode_target} core/assert/libxr_assert/assert_modes.cpp)
    target_include_directories(${mode_target} PRIVATE . .. ../../src/core)
    if(debug)
      target_compile_definitions(${mode_target} PRIVATE LIBXR_DEBUG_BUILD)
    endif()
    if(dev)
      target_compile_definitions(${mode_target} PRIVATE LIBXR_DEV_ASSERT_BUILD)
    endif()
    add_test(NAME ${mode_target} COMMAND ${mode_target})
    set_tests_properties(${mode_target} PROPERTIES TIMEOUT 10)
  endforeach()
endforeach()

include(CheckCXXSourceCompiles)

string(
  CONCAT _xr_unaligned_probe_src
         "#if defined(__ARM_FEATURE_UNALIGNED) || defined(__x86_64__) || "
         "defined(__i386__) || defined(_M_X64) || defined(_M_IX86) || "
         "defined(__aarch64__) || defined(_M_ARM64) || "
         "defined(__riscv_misaligned_fast)\n"
         "int main() { return 0; }\n"
         "#else\n"
         "#error no_fast_unaligned_word\n"
         "#endif\n"
)
check_cxx_source_compiles("${_xr_unaligned_probe_src}" _xr_has_fast_unaligned_word)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU"
   OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang"
   OR CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang"
)
  set_property(
    SOURCE ../../src/core/libxr_mem_o3.cpp ../../src/core/libxr_mem_cmp.cpp
    APPEND
    PROPERTY COMPILE_OPTIONS "-fno-strict-aliasing"
  )

  set(_xr_saved_required_flags "${CMAKE_REQUIRED_FLAGS}")
  set(_xr_saved_required_link_options "${CMAKE_REQUIRED_LINK_OPTIONS}")
  set(CMAKE_REQUIRED_FLAGS "-O3 -flto")
  set(CMAKE_REQUIRED_LINK_OPTIONS "-O3" "-flto")
  check_cxx_source_compiles("int main() { return 0; }" _xr_mem_supports_lto)
  set(CMAKE_REQUIRED_FLAGS "${_xr_saved_required_flags}")
  set(CMAKE_REQUIRED_LINK_OPTIONS "${_xr_saved_required_link_options}")
else()
  set(_xr_mem_supports_lto FALSE)
endif()

set(_xr_mem_policy_matrix
    "shifted_odd_32|LIBXR_FASTCOPY_POLICY_SHIFTED_ODD|LIBXR_MEM_POLICY_BUILTIN|LIBXR_MEM_POLICY_BUILTIN|LIBXR_MEM_POLICY_BUILTIN|4U"
    "shifted_all_32|LIBXR_FASTCOPY_POLICY_SHIFTED_ALL|LIBXR_MEM_POLICY_BUILTIN|LIBXR_MEM_POLICY_BUILTIN|LIBXR_MEM_POLICY_BUILTIN|4U"
    "shifted_odd_64|LIBXR_FASTCOPY_POLICY_SHIFTED_ODD|LIBXR_MEM_POLICY_BUILTIN|LIBXR_MEM_POLICY_BUILTIN|LIBXR_MEM_POLICY_BUILTIN|8U"
    "shifted_all_64|LIBXR_FASTCOPY_POLICY_SHIFTED_ALL|LIBXR_MEM_POLICY_BUILTIN|LIBXR_MEM_POLICY_BUILTIN|LIBXR_MEM_POLICY_BUILTIN|8U"
    "std_libc|LIBXR_FASTCOPY_POLICY_STD_LIBC|LIBXR_MEM_POLICY_STD_LIBC|LIBXR_MEM_POLICY_STD_LIBC|LIBXR_MEM_POLICY_STD_LIBC|DEFAULT"
    "mixed_esp_xtensa|LIBXR_FASTCOPY_POLICY_STD_LIBC|LIBXR_MEM_POLICY_BUILTIN|LIBXR_MEM_POLICY_STD_LIBC|LIBXR_MEM_POLICY_BUILTIN|4U"
)
if(_xr_has_fast_unaligned_word)
  list(
    APPEND
    _xr_mem_policy_matrix
    "unaligned_word|LIBXR_FASTCOPY_POLICY_UNALIGNED_WORD|LIBXR_MEM_POLICY_BUILTIN|LIBXR_MEM_POLICY_BUILTIN|LIBXR_MEM_POLICY_BUILTIN|DEFAULT"
  )
endif()
foreach(_xr_mem_case IN LISTS _xr_mem_policy_matrix)
  string(REPLACE "|" ";" _xr_mem_parts "${_xr_mem_case}")
  list(GET _xr_mem_parts 0 _xr_mem_name)
  list(GET _xr_mem_parts 1 _xr_copy_policy)
  list(GET _xr_mem_parts 2 _xr_move_policy)
  list(GET _xr_mem_parts 3 _xr_set_policy)
  list(GET _xr_mem_parts 4 _xr_cmp_policy)
  list(GET _xr_mem_parts 5 _xr_mem_word_size)
  set(_xr_mem_target "test_mem_policy_${_xr_mem_name}")
  add_executable(
    ${_xr_mem_target} core/libxr_mem/test_mem_policy.cpp ../../src/core/libxr_mem_o3.cpp
                      ../../src/core/libxr_mem_cmp.cpp
  )
  target_include_directories(${_xr_mem_target} PRIVATE .. ../../src/core)
  target_compile_features(${_xr_mem_target} PRIVATE cxx_std_20)
  target_compile_definitions(
    ${_xr_mem_target}
    PRIVATE LIBXR_FASTCOPY_POLICY=${_xr_copy_policy}
            LIBXR_FASTMOVE_POLICY=${_xr_move_policy}
            LIBXR_FASTSET_POLICY=${_xr_set_policy} LIBXR_FASTCMP_POLICY=${_xr_cmp_policy}
  )
  if(NOT "${_xr_mem_word_size}" STREQUAL "DEFAULT")
    target_compile_definitions(
      ${_xr_mem_target} PRIVATE LIBXR_MEM_WORD_SIZE=${_xr_mem_word_size}
    )
  endif()
  target_compile_options(${_xr_mem_target} PRIVATE -O3 -Wall -Wextra -Werror)
  add_test(NAME ${_xr_mem_target} COMMAND ${_xr_mem_target})
  set_tests_properties(${_xr_mem_target} PROPERTIES TIMEOUT 10)
endforeach()

if(_xr_mem_supports_lto)
  foreach(_xr_mem_lto_bits IN ITEMS 32 64)
    if(_xr_mem_lto_bits EQUAL 32)
      set(_xr_mem_lto_word_size "4U")
    else()
      set(_xr_mem_lto_word_size "8U")
    endif()
    set(_xr_mem_lto_target "test_mem_policy_lto_${_xr_mem_lto_bits}")
    add_executable(
      ${_xr_mem_lto_target}
      core/libxr_mem/test_mem_policy.cpp ../../src/core/libxr_mem_o3.cpp
      ../../src/core/libxr_mem_cmp.cpp
    )
    target_include_directories(${_xr_mem_lto_target} PRIVATE .. ../../src/core)
    target_compile_features(${_xr_mem_lto_target} PRIVATE cxx_std_20)
    target_compile_definitions(
      ${_xr_mem_lto_target}
      PRIVATE LIBXR_FASTCOPY_POLICY=LIBXR_FASTCOPY_POLICY_SHIFTED_ODD
              LIBXR_FASTMOVE_POLICY=LIBXR_MEM_POLICY_BUILTIN
              LIBXR_FASTSET_POLICY=LIBXR_MEM_POLICY_BUILTIN
              LIBXR_FASTCMP_POLICY=LIBXR_MEM_POLICY_BUILTIN
              LIBXR_MEM_WORD_SIZE=${_xr_mem_lto_word_size}
    )
    target_compile_options(${_xr_mem_lto_target} PRIVATE -O3 -flto -Wall -Wextra -Werror)
    target_link_options(${_xr_mem_lto_target} PRIVATE -O3 -flto)
    add_test(NAME ${_xr_mem_lto_target} COMMAND ${_xr_mem_lto_target})
    set_tests_properties(${_xr_mem_lto_target} PROPERTIES TIMEOUT 10)
  endforeach()
endif()

# 用实际裸机后端验证 Semaphore 的 atomic_ref 计数与空闲定时刷新；时钟由测试控制，不链接 Linux xr 库。
# Test bare-metal Semaphore atomic_ref counting and idle timer refresh with a controlled clock.
add_executable(
  test_none_semaphore
  system/semaphore/test_none_semaphore.cpp ../../src/system/timer.cpp
  ../../src/structure/lockfree_list.cpp ../../system/none/libxr_system.cpp
  ../../system/none/semaphore.cpp ../../system/none/thread.cpp
)
target_include_directories(
  test_none_semaphore PRIVATE . .. ../../system/none
                              $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
)
get_target_property(_xr_none_sem_test_definitions xr INTERFACE_COMPILE_DEFINITIONS)
list(FILTER _xr_none_sem_test_definitions EXCLUDE REGEX
     "^LIBXR_SYSTEM_|^LIBXR_SINGLE_CORE="
)
target_compile_definitions(
  test_none_semaphore PRIVATE ${_xr_none_sem_test_definitions}
                              LIBXR_NOT_SUPPORT_MUTI_THREAD=1 LIBXR_SINGLE_CORE=1
)
target_compile_features(test_none_semaphore PRIVATE cxx_std_20)
find_package(Threads REQUIRED)
target_link_libraries(test_none_semaphore PRIVATE Threads::Threads)
add_test(NAME test_none_semaphore COMMAND test_none_semaphore)
set_tests_properties(test_none_semaphore PROPERTIES TIMEOUT 10)

# 用可脚本化的确定性 Semaphore 覆盖全部 5 处 WaitClaimedNotification 回收与 REQUIRE 分支。
# Deterministically test all 5 WaitClaimedNotification call sites, ThreadX FAILED, and REQUIRE.
add_executable(
  test_wait_claimed_notification
  core/rw/operation/test_wait_claimed_notification.cpp
  ../../src/core/libxr_mem_cmp.cpp
  ../../src/core/libxr_mem_o3.cpp
  ../../src/core/rw/operation.cpp
  ../../src/core/rw/read_port.cpp
  ../../src/core/rw/write_port.cpp
  ../../src/middleware/message/publish.cpp
  ../../src/middleware/message/topic.cpp
  ../../src/structure/lockfree_list.cpp
  ../../src/structure/queue/queue_base.cpp
  ../../src/system/timer.cpp
  ../../src/utils/crc_o3.cpp
  ../../system/none/libxr_system.cpp
  ../../system/none/mutex.cpp
  ../../system/none/thread.cpp
)
target_include_directories(
  test_wait_claimed_notification
  PRIVATE . .. ../../system/none $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(
  test_wait_claimed_notification
  PRIVATE ${_xr_none_sem_test_definitions} LIBXR_NOT_SUPPORT_MUTI_THREAD=1
          LIBXR_SINGLE_CORE=1
)
target_compile_features(test_wait_claimed_notification PRIVATE cxx_std_20)
target_link_libraries(test_wait_claimed_notification PRIVATE Threads::Threads)
add_test(NAME test_wait_claimed_notification COMMAND test_wait_claimed_notification)
set_tests_properties(test_wait_claimed_notification PROPERTIES TIMEOUT 10)

# 直接编译生产 STM32CANFD / STM32CAN / CH32CAN 驱动验证 TX 串行化、重配与 Bus-Off 恢复。
# Compile real production STM32CANFD / STM32CAN / CH32CAN drivers to test TX recovery paths.
add_library(
  test_can_prod_stm32_canfd OBJECT ../../driver/st/stm32_canfd.cpp
                                   driver/can/test_stm32_canfd_prod.cpp
)
target_include_directories(
  test_can_prod_stm32_canfd PRIVATE driver/can/hal_stubs/stm32_canfd ../../driver/st . ..
                                    $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(
  test_can_prod_stm32_canfd PRIVATE $<TARGET_PROPERTY:xr,INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_features(test_can_prod_stm32_canfd PRIVATE cxx_std_20)

add_library(
  test_can_prod_stm32_can OBJECT ../../driver/st/stm32_can.cpp
                                 driver/can/test_stm32_can_prod.cpp
)
target_include_directories(
  test_can_prod_stm32_can PRIVATE driver/can/hal_stubs/stm32_can ../../driver/st . ..
                                  $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(
  test_can_prod_stm32_can PRIVATE $<TARGET_PROPERTY:xr,INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_features(test_can_prod_stm32_can PRIVATE cxx_std_20)

add_library(
  test_can_prod_ch32_can OBJECT
  ../../driver/ch/ch32_can.cpp ../../driver/ch/ch32_can_def.cpp
  ../../driver/ch/ch32_usbcan_shared.cpp driver/can/test_ch32_can_prod.cpp
)
target_include_directories(
  test_can_prod_ch32_can PRIVATE driver/can/hal_stubs/ch32_can ../../driver/ch . ..
                                 $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(
  test_can_prod_ch32_can PRIVATE $<TARGET_PROPERTY:xr,INTERFACE_COMPILE_DEFINITIONS>
                                 LIBXR_CH32_CONFIG_FILE=ch32_can_test_conf.h
)
target_compile_options(test_can_prod_ch32_can PRIVATE -Wno-attributes)
target_compile_features(test_can_prod_ch32_can PRIVATE cxx_std_20)

add_executable(
  test_can_tx_recovery
  driver/can/test_can_tx_recovery.cpp $<TARGET_OBJECTS:test_can_prod_stm32_canfd>
  $<TARGET_OBJECTS:test_can_prod_stm32_can> $<TARGET_OBJECTS:test_can_prod_ch32_can>
)
target_link_libraries(test_can_tx_recovery PRIVATE xr Threads::Threads)
target_compile_features(test_can_tx_recovery PRIVATE cxx_std_20)
add_test(NAME test_can_tx_recovery COMMAND test_can_tx_recovery)
set_tests_properties(test_can_tx_recovery PROPERTIES TIMEOUT 20)

# 可控时钟 DFU 运行时防 ABA 与弱内存序验证；不链接 Linux xr 库。
# Controlled-clock DFU runtime ABA prevention and weak-memory test; standalone link.
add_executable(
  test_dfu_runtime_aba
  driver/usb/test_dfu_runtime_aba.cpp
  ../../src/core/libxr_mem_o3.cpp
  ../../src/driver/usb/core/ep.cpp
  ../../src/driver/usb/core/ep_pool.cpp
  ../../src/driver/usb/core/ep_slots.cpp
  ../../src/driver/usb/core/usb_service.cpp
  ../../src/driver/usb/device/device_class.cpp
  ../../src/driver/usb/device/dfu/dfu_def.cpp
  ../../src/driver/usb/device/dfu/dfu_runtime.cpp
)
target_include_directories(
  test_dfu_runtime_aba
  PRIVATE . .. ../../src/driver/usb/core ../../src/driver/usb/device
          ../../src/driver/usb/device/bos ../../src/driver/usb/device/dfu
          $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(
  test_dfu_runtime_aba PRIVATE $<TARGET_PROPERTY:xr,INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_features(test_dfu_runtime_aba PRIVATE cxx_std_20)
add_test(NAME test_dfu_runtime_aba COMMAND test_dfu_runtime_aba)
set_tests_properties(test_dfu_runtime_aba PROPERTIES TIMEOUT 10)

# USB 核心构造与前置契约 REQUIRE 拒绝测试（fork 隔离，不启动 PlatformInit）。
# USB core construction REQUIRE contract tests (fork-isolated without PlatformInit).
add_executable(test_usb_core_contracts driver/usb/test_usb_core_contracts.cpp)
target_include_directories(
  test_usb_core_contracts
  PRIVATE . .. ../../src/driver/usb/core ../../src/driver/usb/device
          ../../src/driver/usb/device/bos
)
target_link_libraries(test_usb_core_contracts PRIVATE xr)
add_test(NAME test_usb_core_contracts COMMAND test_usb_core_contracts)
set_tests_properties(test_usb_core_contracts PROPERTIES TIMEOUT 10)

# LinuxCANFD 的自动化测试：默认创建临时 vcan 接口，需要 CAP_NET_ADMIN；
# 无法创建时以退出码 77 跳过（CTest 的 SKIP_RETURN_CODE）。也可用
# --a can0 --b can1 在真实互连接口上运行。
# Automated LinuxCANFD test: creates a temporary vcan interface (CAP_NET_ADMIN) and
# exits 77 when that is unavailable, or runs on real interfaces via --a/--b.
if(_xr_driver STREQUAL "linux")
  find_package(Threads REQUIRED)
  add_executable(test_linux_can driver/can/linux_can_vcan.cpp)
  target_include_directories(
    test_linux_can PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/.."
  )
  target_link_libraries(test_linux_can PRIVATE xr Threads::Threads)
  add_test(NAME test_linux_can COMMAND test_linux_can)
  set_tests_properties(test_linux_can PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)

  # 无硬件协议错误解码回归；即使 vcan 不可用也必须运行。
  # Hardware-independent decoding regression, including when vcan is unavailable.
  add_executable(test_linux_can_error driver/can/linux_can_error.cpp)
  target_include_directories(
    test_linux_can_error PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/.."
  )
  target_link_libraries(test_linux_can_error PRIVATE xr)
  add_test(NAME test_linux_can_error COMMAND test_linux_can_error)
  set_tests_properties(test_linux_can_error PROPERTIES TIMEOUT 10)
endif()

# USB 仿真后端和 DAP 仿真（driver/usb/sim/usb_sim.cmake）。
# The USB simulation backend and the DAP simulation (driver/usb/sim/usb_sim.cmake).
include(driver/usb/sim/usb_sim.cmake)

add_executable(
  libxr_test
  main.cpp
  core/assert/libxr_assert/test_assert.cpp
  core/libxr_cb/test_cb.cpp
  core/libxr_color/test_color.cpp
  core/libxr_def/test_def.cpp
  core/libxr_mem/test_mem.cpp
  core/libxr_pipe/test_pipe.cpp
  core/libxr_string/test_string.cpp
  core/libxr_time/test_time.cpp
  core/libxr_type/test_type.cpp
  core/print/format/format_frontend_parser/test_format_frontend.cpp
  core/print/print/test_print.cpp
  core/print/print_api/test_print_api.cpp
  core/print/printf/printf_frontend_parser/test_printf_frontend.cpp
  core/print/writer/test_print_failure.cpp
  core/rw/operation/test_operation.cpp
  core/rw/read_port/test_read_port.cpp
  core/rw/stdio/test_linux_stdio_print.cpp
  core/rw/write_port/test_write_port.cpp
  core/rw/write_stream/test_write_stream.cpp
  driver/debug/jtag_general_gpio/test_jtag_general_gpio.cpp
  driver/debug/jtag_protocol/test_jtag_protocol.cpp
  driver/debug/swd_general_gpio/test_swd_general_gpio.cpp
  driver/debug/swd_protocol/test_swd_protocol.cpp
  driver/driver_async_condition/test_driver_async_condition.cpp
  driver/power/test_power.cpp
  driver/uart/test_uart.cpp
  driver/usb/test_usb_ep_slots.cpp
  driver/usb/test_usb_ep.cpp
  driver/usb/test_usb_ep_pool.cpp
  driver/usb/test_usb_dev_core.cpp
  driver/usb/test_cdc.cpp
  driver/usb/test_uac.cpp
  driver/usb/test_hid.cpp
  driver/usb/test_dfu.cpp
  driver/usb/test_gsusb.cpp
  driver/usb/test_dap.cpp
  driver/usb/test_usb_detach_timer.cpp
  driver/usb/test_usb_fsdev.cpp
  driver/timebase/test_timebase.cpp
  middleware/database/database/test_database.cpp
  middleware/database/raw/raw/test_linux_database_raw.cpp
  middleware/database/raw_sequential/test_linux_database_sequential.cpp
  middleware/event/test_event.cpp
  middleware/logger/logger/test_logger.cpp
  middleware/message/packet/packet/test_packet.cpp
  middleware/message/topic/bench_latency.cpp
  middleware/message/topic/bench_modes.cpp
  middleware/message/topic/bench_overload.cpp
  middleware/message/topic/bench_standard.cpp
  middleware/message/topic/test_attach_queue.cpp
  middleware/message/topic/test_balance_rr.cpp
  middleware/message/topic/test_cross_process.cpp
  middleware/message/topic/test_lifecycle.cpp
  middleware/message/topic/test_linux_shm_topic.cpp
  middleware/message/topic/test_mixed_modes.cpp
  middleware/message/topic/test_runtime.cpp
  middleware/message/topic/test_topic.cpp
  middleware/ramfs/ramfs/test_ramfs.cpp
  middleware/terminal/command/test_command.cpp
  middleware/terminal/display/test_display.cpp
  middleware/terminal/input/test_input.cpp
  middleware/terminal/terminal/test_integration.cpp
  structure/double_buffer/test_double_buffer.cpp
  structure/latest_snapshot/test_latest_snapshot.cpp
  structure/lockfree_list/test_lockfree_list.cpp
  structure/object_pool/test_object_pool.cpp
  structure/queue/mpmc_queue/test_mpmc_queue.cpp
  structure/queue/queue/test_queue.cpp
  structure/queue/spsc_queue/test_spsc_prefix.cpp
  structure/queue/spsc_queue/test_spsc_queue.cpp
  structure/rbt/test_rbt.cpp
  structure/slot_pool/test_slot_pool.cpp
  system/thread/test_thread.cpp
  system/mutex/test_mutex.cpp
  system/semaphore/test_semaphore.cpp
  system/timer/test_timer.cpp
  utils/crc/test_crc.cpp
  utils/cycle_value/test_cycle_value.cpp
  utils/float_encoder/test_encoder.cpp
  utils/inertia/test_inertia.cpp
  utils/kinematic/test_kinematic.cpp
  utils/lpfilter/test_lpfilter.cpp
  utils/pid/test_pid.cpp
  utils/serialized_service/test_serialized_service.cpp
  utils/transform/test_transform.cpp
  utils/transform/test_transform_euler_orders.cpp
)
target_link_libraries(libxr_test PRIVATE xr)
target_include_directories(
  libxr_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/.."
)

find_program(XR_TEST_SCRIPT_EXECUTABLE script)
if(NOT XR_TEST_SCRIPT_EXECUTABLE)
  message(FATAL_ERROR "Automatic tests require the util-linux script program for a PTY.")
endif()
add_test(NAME libxr_automatic COMMAND "${XR_TEST_SCRIPT_EXECUTABLE}" -q -e -c
                                      "\"$<TARGET_FILE:libxr_test>\"" /dev/null
)
set_tests_properties(
  libxr_automatic PROPERTIES TIMEOUT 300 RUN_SERIAL TRUE WORKING_DIRECTORY
                             "${PROJECT_SOURCE_DIR}"
)

add_library(
  test_print_config_matrix OBJECT
  core/print/print/test_float_disabled.cpp
  core/print/print/test_float_fixed_only.cpp
  core/print/print/test_float_precision_limit.cpp
  core/print/print/test_integer_base_disabled.cpp
  core/print/print/test_integer_text_disabled.cpp
  core/print/print/test_pointer_64bit_disabled.cpp
  core/print/print/test_syntax_switches_disabled.cpp
)
add_dependencies(test_print_config_matrix xr)
target_compile_features(test_print_config_matrix PRIVATE cxx_std_20)
target_compile_definitions(
  test_print_config_matrix PRIVATE $<TARGET_PROPERTY:xr,INTERFACE_COMPILE_DEFINITIONS>
)
target_include_directories(
  test_print_config_matrix
  PRIVATE core/print/print $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
          "${CMAKE_CURRENT_SOURCE_DIR}/../../lib/Eigen"
)

# XRobot 生成头文件的内容检查（cmake/XRobotFreshness.cmake）；只运行 CMake 脚本，不编译代码。
# The content check of the XRobot generated header (cmake/XRobotFreshness.cmake); it runs a
# CMake script and compiles nothing.
add_test(
  NAME xrobot_freshness
  COMMAND
    "${CMAKE_COMMAND}" "-DWORK=${CMAKE_CURRENT_BINARY_DIR}/xrobot_freshness"
    "-DCHECK=${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/XRobotFreshness.cmake" -P
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/xrobot_freshness.cmake"
)
set_tests_properties(xrobot_freshness PROPERTIES TIMEOUT 60)
