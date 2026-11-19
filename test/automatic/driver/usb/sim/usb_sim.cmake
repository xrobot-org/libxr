# USB 仿真后端：真实的端点核心、设备核心和设备类跑在仿真控制器上。契约、档位、SWD 目标、
# DAP 随机和回放测试进 ctest；USB/IP 和 remote_bitbang 程序要 root、vhci 或外部工具，由
# 脚本手动运行，不进 ctest。
# The USB simulation backend: the real endpoint core, device core and device classes on
# a simulated controller. The contract, profile, SWD target, DAP fuzz and replay tests
# are in ctest; the USB/IP and remote_bitbang programs need root, vhci or outside tools,
# are run by scripts by hand and stay out of ctest.

set(USB_SIM_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(DAP_SIM_DIR "${CMAKE_CURRENT_LIST_DIR}/../../debug/sim")
set(USB_SIM_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/.." "${USB_SIM_DIR}" "${DAP_SIM_DIR}")

function(usb_sim_program name source)
  add_executable(${name} ${source})
  target_link_libraries(${name} PRIVATE xr pthread)
  target_include_directories(${name} PRIVATE ${USB_SIM_INCLUDES})
endfunction()

# 后端契约 / The backend contract
usb_sim_program(test_usb_sim_contract "${USB_SIM_DIR}/test_usb_sim_contract.cpp")
add_test(NAME usb_sim_contract COMMAND test_usb_sim_contract)
set_tests_properties(usb_sim_contract PROPERTIES TIMEOUT 120)

# 硬件档位上的真实设备类 / Real device classes on the hardware profiles
usb_sim_program(test_usb_sim_profiles "${USB_SIM_DIR}/test_usb_sim_profiles.cpp")
add_test(NAME usb_sim_profiles COMMAND test_usb_sim_profiles)
set_tests_properties(usb_sim_profiles PROPERTIES TIMEOUT 120)

# SWD 目标仿真：经 Swd 接口和经 SwdGeneralGPIO 接仿真引脚 / The SWD target model through
# the Swd interface and through SwdGeneralGPIO on simulated pins
usb_sim_program(test_swd_target "${DAP_SIM_DIR}/test_swd_target.cpp")
add_test(NAME swd_target COMMAND test_swd_target)
set_tests_properties(swd_target PROPERTIES TIMEOUT 60)

# DAP v2 随机测试，经 USB 仿真后端到 SWD 目标仿真 / Randomised DAP v2 test through the USB
# simulation backend to the simulated SWD target
usb_sim_program(test_dap_fuzz "${DAP_SIM_DIR}/test_dap_fuzz.cpp")
add_test(NAME dap_fuzz COMMAND test_dap_fuzz)
set_tests_properties(dap_fuzz PROPERTIES TIMEOUT 300)

# 回放 OpenOCD、pyOCD、probe-rs 的录制会话（dap_record.sh）/ Replay of sessions recorded
# from OpenOCD, pyOCD and probe-rs (dap_record.sh)
usb_sim_program(test_dap_replay "${DAP_SIM_DIR}/test_dap_replay.cpp")
target_compile_definitions(test_dap_replay PRIVATE DAP_REPLAY_DIR="${DAP_SIM_DIR}/replay")
add_test(NAME dap_replay COMMAND test_dap_replay)
set_tests_properties(dap_replay PROPERTIES TIMEOUT 120)

# 手动运行的程序 / Programs run by hand
usb_sim_program(usb_sim_usbip "${USB_SIM_DIR}/usb_sim_usbip.cpp")
usb_sim_program(usb_sim_classes "${USB_SIM_DIR}/usb_sim_classes.cpp")
usb_sim_program(dap_sim_usbip "${DAP_SIM_DIR}/dap_sim_usbip.cpp")
usb_sim_program(swd_target_bitbang "${DAP_SIM_DIR}/swd_target_bitbang.cpp")
