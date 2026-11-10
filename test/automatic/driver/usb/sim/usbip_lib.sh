#!/usr/bin/env bash
# usbip_lib.sh: attach and detach helpers for the simulated device, sourced by
# classes_e2e.sh and plug_cycle.sh. 挂接和摘除仿真设备的函数。
#
# The host has other USB devices, some of them with the same VID:PID as a simulated one
# (a real gs_usb adapter). The helpers therefore never name a vhci port or a device by
# VID:PID alone: the port is found from the remote URL the simulated server was imported
# from (usbip://127.0.0.1:<tcp port>/<busid>), and a device is the one whose sysfs path
# lies under the vhci host controller.
# 主机上还有别的 USB 设备，有的和仿真设备同 VID:PID（真实的 gs_usb 适配器）。所以这里
# 从不只凭端口号或 VID:PID 选设备：vhci 端口按导入时的远端 URL 找，设备按 sysfs 路径在
# vhci 主机控制器之下判定。
# 只用 sudo -n 运行 usbip（attach、detach、port）和 modprobe。
# sudo -n is used for the usbip tool (attach, detach, port) and modprobe only.

SIM_TCP_PORT=${SIM_TCP_PORT:-3240}
SIM_BUSID=${SIM_BUSID:-1-1}
SIM_PORT=""

# Print the vhci port the simulated server is imported on, empty if none.
sim_find_port() {
  sudo -n usbip port 2>/dev/null | awk -v url="usbip://127.0.0.1:${SIM_TCP_PORT}/${SIM_BUSID}" '
    /^Port [0-9]+:/ { sub(":", "", $2); port = $2 + 0 }
    index($0, url) { print port; exit }'
}

# Attach the simulated server and remember its vhci port in SIM_PORT.
sim_attach() {
  sudo -n usbip --tcp-port "$SIM_TCP_PORT" attach -r 127.0.0.1 -b "$SIM_BUSID" \
    >/dev/null 2>&1 || return 1
  SIM_PORT=""
  for _ in $(seq 1 50); do
    SIM_PORT=$(sim_find_port)
    [ -n "$SIM_PORT" ] && return 0
    sleep 0.1
  done
  return 1
}

# Detach the port of the simulated server, nothing else.
sim_detach() {
  local port
  port=$(sim_find_port)
  [ -n "$port" ] && sudo -n usbip detach -p "$port" >/dev/null 2>&1
  SIM_PORT=""
  return 0
}

# True if the sysfs path of a device lies under a vhci host controller.
sim_is_vhci() {
  readlink -f "$1" | grep -q "/vhci_hcd\."
}

# The sysfs directory of the simulated device (under vhci) with this VID:PID, if any.
# 仿真设备（在 vhci 之下）中 VID:PID 相符者的 sysfs 目录。
sim_usb_device() {
  local d
  for d in /sys/bus/usb/devices/*; do
    [ -f "$d/idVendor" ] || continue
    [ "$(cat "$d/idVendor")" = "$1" ] && [ "$(cat "$d/idProduct")" = "$2" ] || continue
    sim_is_vhci "$d" && { echo "$d"; return 0; }
  done
  return 1
}

# Like sim_usb_device, but fails unless the host has exactly one device with this
# VID:PID and it is the one under vhci. Tools that can only pick by VID:PID and serial
# (OpenOCD, pyOCD, probe-rs) are started only after this holds.
# 同 sim_usb_device，但主机上该 VID:PID 的设备必须恰有一个且在 vhci 之下才成功。只能按
# VID:PID 和序列号选设备的工具（OpenOCD、pyOCD、probe-rs）在这一条成立之后才启动。
sim_usb_device_unique() {
  local d count=0 found=""
  for d in /sys/bus/usb/devices/*; do
    [ -f "$d/idVendor" ] || continue
    [ "$(cat "$d/idVendor")" = "$1" ] && [ "$(cat "$d/idProduct")" = "$2" ] || continue
    count=$((count + 1))
    sim_is_vhci "$d" && found=$d
  done
  [ "$count" -eq 1 ] && [ -n "$found" ] && echo "$found"
}
