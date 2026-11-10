if(NOT _xr_system STREQUAL "linux")
  message(FATAL_ERROR "Automatic tests require LIBXR_SYSTEM=linux.")
endif()

add_compile_options(-g)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/..")

add_library(
  test_assert_disabled OBJECT
  core/assert/libxr_assert/assert_disabled_headers.cpp ../../src/core/rw/read_port.cpp
  ../../src/core/rw/write_port.cpp ../../src/core/rw/write_stream.cpp
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

# 用实际裸机后端验证协作式 ASync；时钟由测试控制，不链接 Linux xr 库。
# Test cooperative ASync with the actual bare-metal backend and a controlled clock.
add_executable(
  test_async_cooperative
  system/async/test_async_cooperative.cpp
  ../../src/system/async.cpp
  ../../src/system/timer.cpp
  ../../src/structure/lockfree_list.cpp
  ../../system/none/libxr_system.cpp
  ../../system/none/semaphore.cpp
  ../../system/none/thread.cpp
)
target_include_directories(
  test_async_cooperative PRIVATE . .. ../../system/none
                                 $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
)
get_target_property(_xr_async_test_definitions xr INTERFACE_COMPILE_DEFINITIONS)
list(FILTER _xr_async_test_definitions EXCLUDE REGEX "^LIBXR_SYSTEM_|^LIBXR_SINGLE_CORE=")
target_compile_definitions(
  test_async_cooperative PRIVATE ${_xr_async_test_definitions}
                                 LIBXR_NOT_SUPPORT_MUTI_THREAD=1 LIBXR_SINGLE_CORE=1
)
target_compile_features(test_async_cooperative PRIVATE cxx_std_20)
add_test(NAME test_async_cooperative COMMAND test_async_cooperative)
set_tests_properties(test_async_cooperative PROPERTIES TIMEOUT 10)

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
  core/rw/read_port/test_read_port.cpp
  core/rw/stdio/test_linux_stdio_print.cpp
  core/rw/write_port/test_write_port.cpp
  core/rw/write_stream/test_write_stream.cpp
  driver/power/test_power.cpp
  driver/uart/test_uart.cpp
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
  structure/list/test_list.cpp
  structure/lockfree_list/test_lockfree_list.cpp
  structure/object_pool/test_object_pool.cpp
  structure/queue/mpmc_queue/test_mpmc_queue.cpp
  structure/queue/queue/test_queue.cpp
  structure/queue/spsc_queue/test_spsc_prefix.cpp
  structure/queue/spsc_queue/test_spsc_queue.cpp
  structure/rbt/test_rbt.cpp
  structure/slot_pool/test_slot_pool.cpp
  structure/stack/test_stack.cpp
  system/async/test_async.cpp
  system/thread/test_thread.cpp
  system/mutex/test_mutex.cpp
  system/semaphore/test_semaphore.cpp
  system/timer/test_timer.cpp
  utils/crc/test_crc.cpp
  utils/cycle_value/test_cycle_value.cpp
  utils/flag/test_flag.cpp
  utils/float_encoder/test_encoder.cpp
  utils/inertia/test_inertia.cpp
  utils/kinematic/test_kinematic.cpp
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

# Stack::Insert 对不可平凡复制的元素类型必须编译失败。目标不参与默认构建，由测试单独构建，
# 并检查输出中有 static_assert 的报错文字。
# Stack::Insert must not compile for an element type that is not trivially copyable. The
# target is left out of the default build; the test builds it and checks the output for
# the static_assert message.
add_library(
  test_stack_insert_nontrivial OBJECT EXCLUDE_FROM_ALL
  structure/stack/stack_insert_nontrivial.cpp
)
target_link_libraries(test_stack_insert_nontrivial PRIVATE xr)
add_test(NAME stack_insert_nontrivial
         COMMAND "${CMAKE_COMMAND}" --build "${CMAKE_BINARY_DIR}" --config $<CONFIG>
                 --target test_stack_insert_nontrivial
)
set_tests_properties(
  stack_insert_nontrivial PROPERTIES PASS_REGULAR_EXPRESSION
                                     "requires a trivially copyable element type" TIMEOUT 120
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
# Real Endpoint/CDC/RW code; only the hardware segment is controlled by the test.
add_executable(test_usb_endpoint driver/usb/core/test_endpoint.cpp)
target_link_libraries(test_usb_endpoint PRIVATE xr)
target_include_directories(test_usb_endpoint PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
add_test(NAME usb_endpoint COMMAND test_usb_endpoint)
set_tests_properties(usb_endpoint PROPERTIES TIMEOUT 30)

# The shared FSDEV/CH32 DevFS register runtime against a fake controller.
add_executable(test_usb_fsdev driver/usb/core/test_fsdev.cpp)
target_link_libraries(test_usb_fsdev PRIVATE xr)
target_include_directories(test_usb_fsdev PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
add_test(NAME usb_fsdev COMMAND test_usb_fsdev)
set_tests_properties(usb_fsdev PROPERTIES TIMEOUT 30)

# DriverAsyncCondition driven by OnTick() calls from the test instead of a timebase.
add_executable(
  test_driver_async_condition driver/async_condition/test_driver_async_condition.cpp
)
target_link_libraries(test_driver_async_condition PRIVATE xr)
target_include_directories(
  test_driver_async_condition PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/.."
)
add_test(NAME driver_async_condition COMMAND test_driver_async_condition)
set_tests_properties(driver_async_condition PROPERTIES TIMEOUT 30)

# HID device class on the slot model: input reports, OUT leases, LED and idle rate.
add_executable(test_usb_hid driver/usb/device/hid/test_hid.cpp)
target_link_libraries(test_usb_hid PRIVATE xr)
target_include_directories(test_usb_hid PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
add_test(NAME usb_hid COMMAND test_usb_hid)
set_tests_properties(usb_hid PROPERTIES TIMEOUT 30)

# DAP v1 device class: commands run in Step(), leases, rebind and SWD ownership.
add_executable(test_usb_daplink_v1 driver/usb/device/dap/test_daplink_v1.cpp)
target_link_libraries(test_usb_daplink_v1 PRIVATE xr)
target_include_directories(test_usb_daplink_v1 PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
add_test(NAME usb_daplink_v1 COMMAND test_usb_daplink_v1)
set_tests_properties(usb_daplink_v1 PROPERTIES TIMEOUT 30)
# DAP v2 device class: borrow-mode receive and submit, response depth, delays in Step.
add_executable(test_usb_daplink_v2 driver/usb/device/dap/test_daplink_v2.cpp)
target_link_libraries(test_usb_daplink_v2 PRIVATE xr)
target_include_directories(test_usb_daplink_v2 PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
add_test(NAME usb_daplink_v2 COMMAND test_usb_daplink_v2)
set_tests_properties(usb_daplink_v2 PROPERTIES TIMEOUT 30)

# SwdGeneralGPIO: the SWDIO level during idle cycles.
add_executable(test_swd_general_gpio driver/debug/test_swd_general_gpio.cpp)
target_link_libraries(test_swd_general_gpio PRIVATE xr)
target_include_directories(test_swd_general_gpio PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
add_test(NAME swd_general_gpio COMMAND test_swd_general_gpio)
set_tests_properties(swd_general_gpio PROPERTIES TIMEOUT 30)

# SWD packet request and ACK encoding against the ADIv5 packet format.
add_executable(test_swd_protocol driver/debug/test_swd_protocol.cpp)
target_link_libraries(test_swd_protocol PRIVATE xr)
target_include_directories(test_swd_protocol PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
add_test(NAME swd_protocol COMMAND test_swd_protocol)
set_tests_properties(swd_protocol PROPERTIES TIMEOUT 30)

# DFU device classes: core-owned status stage, DNLOAD data stage and receive capacity.
add_executable(test_usb_dfu driver/usb/device/dfu/test_dfu.cpp)
target_link_libraries(test_usb_dfu PRIVATE xr)
target_include_directories(test_usb_dfu PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
add_test(NAME usb_dfu COMMAND test_usb_dfu)
set_tests_properties(usb_dfu PROPERTIES TIMEOUT 30)

# CH32 USBHS IN endpoint against a fake register block: ACKed packets that INT_ST
# reports as busy NAKs are found by their toggle.
add_executable(
  test_ch32_usbhs_in driver/usb/ch32/test_ch32_usbhs_in.cpp
                     "${CMAKE_CURRENT_SOURCE_DIR}/../../driver/ch/ch32_usb_endpoint_otghs.cpp"
)
target_link_libraries(test_ch32_usbhs_in PRIVATE xr)
target_include_directories(
  test_ch32_usbhs_in PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/.."
                             "${CMAKE_CURRENT_SOURCE_DIR}/driver/usb/ch32"
                             "${CMAKE_CURRENT_SOURCE_DIR}/../../driver/ch"
)
target_compile_definitions(
  test_ch32_usbhs_in PRIVATE LIBXR_CH32_CONFIG_FILE=fake_ch32v30x_usbhs.h
)
add_test(NAME ch32_usbhs_in COMMAND test_ch32_usbhs_in)
set_tests_properties(ch32_usbhs_in PROPERTIES TIMEOUT 30)

# CH32 USBFS endpoints against a fake register block: the shared bidirectional block, the
# IN packets below 64 bytes and the resend of an OUT packet.
add_executable(
  test_ch32_usbfs driver/usb/ch32/test_ch32_usbfs.cpp
                  "${CMAKE_CURRENT_SOURCE_DIR}/../../driver/ch/ch32_usb_endpoint_otgfs.cpp"
)
target_link_libraries(test_ch32_usbfs PRIVATE xr)
target_include_directories(
  test_ch32_usbfs PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/.."
                          "${CMAKE_CURRENT_SOURCE_DIR}/driver/usb/ch32"
                          "${CMAKE_CURRENT_SOURCE_DIR}/../../driver/ch"
)
target_compile_definitions(
  test_ch32_usbfs PRIVATE LIBXR_CH32_CONFIG_FILE=fake_ch32v30x_usbfs.h
)
add_test(NAME ch32_usbfs COMMAND test_ch32_usbfs)
set_tests_properties(ch32_usbfs PROPERTIES TIMEOUT 30)

# STM32 OTG backend against a model of the DWC2 device controller: the real backend
# source with a fake CubeMX main.h, its register accesses trapped at the OTG addresses
# (Linux x86-64 only).
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
  add_executable(
    test_stm32_usb_otg
    driver/usb/dwc2/stm32/test_stm32_usb_otg.cpp driver/usb/dwc2/stm32/fake_stm32_hal.cpp
    "${CMAKE_CURRENT_SOURCE_DIR}/../../driver/st/stm32_usb_otg.cpp"
  )
  target_link_libraries(test_stm32_usb_otg PRIVATE xr)
  target_include_directories(
    test_stm32_usb_otg
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/.." "${CMAKE_CURRENT_SOURCE_DIR}/driver/usb/dwc2"
            "${CMAKE_CURRENT_SOURCE_DIR}/driver/usb/dwc2/stm32"
            "${CMAKE_CURRENT_SOURCE_DIR}/../../driver/st"
  )
  add_test(NAME stm32_usb_otg COMMAND test_stm32_usb_otg)
  set_tests_properties(stm32_usb_otg PROPERTIES TIMEOUT 120)

  # ESP32-S2/S3 USB backend against the same controller model: the real backend source
  # with fake ESP-IDF headers for an ESP32-S3. The backend stores a pointer in a 32-bit
  # DMA address register, which a 64-bit compiler only accepts with -fpermissive; the
  # register bit-fields are accessed as whole words, as ESP-IDF builds them; sdkconfig.h
  # comes first, as it does through the platform headers on the target.
  set(_esp_usb_backend
      "${CMAKE_CURRENT_SOURCE_DIR}/../../driver/esp/esp_usb.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/../../driver/esp/esp_usb_dev.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/../../driver/esp/esp_usb_ep.cpp"
  )
  set_source_files_properties(
    ${_esp_usb_backend} PROPERTIES COMPILE_OPTIONS
                                   "-fpermissive;-fstrict-volatile-bitfields;-include;sdkconfig.h"
  )
  add_executable(
    test_esp_usb driver/usb/dwc2/esp/test_esp_usb.cpp driver/usb/dwc2/esp/fake_esp_idf.cpp
                 ${_esp_usb_backend}
  )
  target_link_libraries(test_esp_usb PRIVATE xr)
  target_include_directories(
    test_esp_usb
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/.." "${CMAKE_CURRENT_SOURCE_DIR}/driver/usb/dwc2"
            "${CMAKE_CURRENT_SOURCE_DIR}/driver/usb/dwc2/esp"
            "${CMAKE_CURRENT_SOURCE_DIR}/../../driver/esp"
  )
  add_test(NAME esp_usb COMMAND test_esp_usb)
  set_tests_properties(esp_usb PROPERTIES TIMEOUT 120)
endif()

# cdc_test classes: lease return, DTR-triggered packet and the read re-arm.
add_executable(test_usb_cdc_test driver/usb/device/cdc/test_cdc_test.cpp)
target_link_libraries(test_usb_cdc_test PRIVATE xr)
target_include_directories(test_usb_cdc_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
add_test(NAME usb_cdc_test COMMAND test_usb_cdc_test)
set_tests_properties(usb_cdc_test PROPERTIES TIMEOUT 30)
# CDCUart / CDCToUart: SET_LINE_CODING is always acknowledged, GET returns the applied
# coding.
add_executable(test_usb_cdc_uart driver/usb/device/cdc/test_cdc_uart.cpp)
target_link_libraries(test_usb_cdc_uart PRIVATE xr)
target_include_directories(test_usb_cdc_uart PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
add_test(NAME usb_cdc_uart COMMAND test_usb_cdc_uart)
set_tests_properties(usb_cdc_uart PROPERTIES TIMEOUT 30)
# gs_usb device class: per-channel queues, echo and RX alternation, backpressure and
# USER_ID.
add_executable(test_usb_gsusb driver/usb/device/gsusb/test_gsusb.cpp)
target_link_libraries(test_usb_gsusb PRIVATE xr)
target_include_directories(test_usb_gsusb PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
add_test(NAME usb_gsusb COMMAND test_usb_gsusb)
set_tests_properties(usb_gsusb PROPERTIES TIMEOUT 30)
# UAC1 microphone on the slot model: alt switching, SOF pacing, FAILED recovery.
add_executable(test_uac_mic driver/usb/device/uac/test_uac_mic.cpp)
target_link_libraries(test_uac_mic PRIVATE xr)
target_include_directories(test_uac_mic PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
add_test(NAME usb_uac_mic COMMAND test_uac_mic)
set_tests_properties(usb_uac_mic PROPERTIES TIMEOUT 30)

# Simulation USB backend: backend contract tests on the real endpoint core and DeviceCore,
# driven packet by packet by a simulated host.
add_executable(test_usb_sim_contract driver/usb/sim/test_usb_sim_contract.cpp)
target_link_libraries(test_usb_sim_contract PRIVATE xr)
target_include_directories(test_usb_sim_contract PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
add_test(NAME usb_sim_contract COMMAND test_usb_sim_contract)
set_tests_properties(usb_sim_contract PROPERTIES TIMEOUT 60)

# Hardware profiles of the simulation backend (FSDEV, STM32 OTG FS, ESP32-S3, CH32 OTG FS):
# the BSP compositions, the controller limits and the frame interrupt.
add_executable(test_usb_sim_profiles driver/usb/sim/test_usb_sim_profiles.cpp)
target_link_libraries(test_usb_sim_profiles PRIVATE xr)
target_include_directories(
  test_usb_sim_profiles PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/.."
                                "${CMAKE_CURRENT_SOURCE_DIR}/driver/debug/sim")
add_test(NAME usb_sim_profiles COMMAND test_usb_sim_profiles)
set_tests_properties(usb_sim_profiles PROPERTIES TIMEOUT 120)

# The simulated device over USB/IP for a real host stack (manual and nightly use on a
# Linux machine with vhci-hcd; not a ctest).
add_executable(usb_sim_usbip driver/usb/sim/usb_sim_usbip.cpp)
target_link_libraries(usb_sim_usbip PRIVATE xr pthread)
target_include_directories(usb_sim_usbip PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")

# HID, gs_usb, DFU and UAC over USB/IP for the kernel drivers (manual and nightly use).
add_executable(usb_sim_classes driver/usb/sim/usb_sim_classes.cpp)
target_link_libraries(usb_sim_classes PRIVATE xr pthread)
target_include_directories(usb_sim_classes PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")

# SWD target simulation: ADIv5 SW-DP and MEM-AP at the pin level, through the Swd
# interface and through SwdGeneralGPIO with simulated pins.
add_executable(test_swd_target driver/debug/sim/test_swd_target.cpp)
target_link_libraries(test_swd_target PRIVATE xr)
target_include_directories(test_swd_target PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
add_test(NAME swd_target COMMAND test_swd_target)
set_tests_properties(swd_target PROPERTIES TIMEOUT 30)

# remote_bitbang server for the simulated SWD target, so that real OpenOCD can drive it
# (manual and nightly use; not a ctest).
add_executable(swd_target_bitbang driver/debug/sim/swd_target_bitbang.cpp)
target_link_libraries(swd_target_bitbang PRIVATE xr)
target_include_directories(swd_target_bitbang PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")

# End to end: a real debugger over USB/IP to the DAP v2 class, the SwdGeneralGPIO engine
# and the simulated SWD target (manual and nightly use; not a ctest).
add_executable(dap_sim_usbip driver/debug/sim/dap_sim_usbip.cpp)
target_link_libraries(dap_sim_usbip PRIVATE xr pthread)
target_include_directories(dap_sim_usbip PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")

# DAP v2 randomised test: random commands, packetisation, depth, read timing, injected
# WAITs and unplugs, through the USB simulation backend to the simulated SWD target.
add_executable(test_dap_fuzz driver/debug/sim/test_dap_fuzz.cpp)
target_link_libraries(test_dap_fuzz PRIVATE xr)
target_include_directories(test_dap_fuzz PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
add_test(NAME dap_fuzz COMMAND test_dap_fuzz)
set_tests_properties(dap_fuzz PROPERTIES TIMEOUT 300)

# Replay of DAP sessions recorded from OpenOCD, pyOCD and probe-rs (dap_record.sh).
add_executable(test_dap_replay driver/debug/sim/test_dap_replay.cpp)
target_link_libraries(test_dap_replay PRIVATE xr)
target_include_directories(test_dap_replay PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
target_compile_definitions(
  test_dap_replay
  PRIVATE DAP_REPLAY_DIR="${CMAKE_CURRENT_SOURCE_DIR}/driver/debug/sim/replay")
add_test(NAME dap_replay COMMAND test_dap_replay)
set_tests_properties(dap_replay PROPERTIES TIMEOUT 120)
