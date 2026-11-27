# USB 仿真与 DAP 仿真自动测试：SWD/JTAG 目标仿真及 DAP 随机与回放测试进 ctest。
# USB and DAP simulation automatic tests: SWD/JTAG target simulation and DAP fuzz/replay
# tests registered in ctest.

set(USB_SIM_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(DAP_SIM_DIR "${CMAKE_CURRENT_LIST_DIR}/../../debug/sim")
set(USB_SIM_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/.." "${USB_SIM_DIR}" "${DAP_SIM_DIR}")

function(usb_sim_program name source)
  add_executable(${name} ${source})
  target_link_libraries(${name} PRIVATE xr pthread)
  target_include_directories(${name} PRIVATE ${USB_SIM_INCLUDES})
endfunction()

# SWD 目标仿真：经 Swd 接口和经 SwdGeneralGPIO 接仿真引脚 / The SWD target model through
# the Swd interface and through SwdGeneralGPIO on simulated pins
usb_sim_program(test_swd_target "${DAP_SIM_DIR}/test_swd_target.cpp")
add_test(NAME swd_target COMMAND test_swd_target)
set_tests_properties(swd_target PROPERTIES TIMEOUT 60)

# JTAG 目标仿真：经 JtagDp 和经 JtagGeneralGPIO 接仿真引脚 / The JTAG target model through
# JtagDp and through JtagGeneralGPIO on simulated pins
usb_sim_program(test_jtag_target "${DAP_SIM_DIR}/test_jtag_target.cpp")
add_test(NAME jtag_target COMMAND test_jtag_target)
set_tests_properties(jtag_target PROPERTIES TIMEOUT 60)

# DAP v2 随机测试，经 USB 仿真后端到 SWD 目标仿真 / Randomised DAP v2 test through the USB
# simulation backend to the simulated SWD target
usb_sim_program(test_dap_fuzz "${DAP_SIM_DIR}/test_dap_fuzz.cpp")
add_test(NAME dap_fuzz COMMAND test_dap_fuzz)
set_tests_properties(dap_fuzz PROPERTIES TIMEOUT 300)

# 回放 OpenOCD、pyOCD、probe-rs 的录制会话 / Replay of sessions recorded from OpenOCD,
# pyOCD and probe-rs
usb_sim_program(test_dap_replay "${DAP_SIM_DIR}/test_dap_replay.cpp")
target_compile_definitions(test_dap_replay PRIVATE DAP_REPLAY_DIR="${DAP_SIM_DIR}/replay")
add_test(NAME dap_replay COMMAND test_dap_replay)
set_tests_properties(dap_replay PROPERTIES TIMEOUT 120)
