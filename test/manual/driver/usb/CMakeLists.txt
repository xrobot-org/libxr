# 手动 USB/IP 端到端仿真程序（EXCLUDE_FROM_ALL：默认不参与 CI 构建，按需显式指定目标编译）。
# Manual USB/IP end-to-end simulation programs (EXCLUDE_FROM_ALL: omitted from default CI
# builds; build explicitly by target name when needed).

function(usb_manual_sim_program name source)
  add_executable(${name} EXCLUDE_FROM_ALL ${source})
  set_target_properties(
    ${name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../../.."
  )
  target_link_libraries(${name} PRIVATE xr pthread)
  target_include_directories(
    ${name}
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/../../.."
            "${CMAKE_CURRENT_SOURCE_DIR}/../../../automatic/driver/usb/sim"
            "${CMAKE_CURRENT_SOURCE_DIR}/../../../automatic/driver/debug/sim"
  )
endfunction()

usb_manual_sim_program(usb_sim_usbip "${CMAKE_CURRENT_SOURCE_DIR}/usb_sim_usbip.cpp")
usb_manual_sim_program(usb_sim_classes "${CMAKE_CURRENT_SOURCE_DIR}/usb_sim_classes.cpp")
usb_manual_sim_program(
  dap_sim_usbip "${CMAKE_CURRENT_SOURCE_DIR}/../debug/dap_sim_usbip.cpp"
)
