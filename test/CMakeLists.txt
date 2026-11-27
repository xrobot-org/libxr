add_subdirectory(automatic)
if(_xr_system STREQUAL "linux")
  add_subdirectory(manual/driver/usb)
endif()
