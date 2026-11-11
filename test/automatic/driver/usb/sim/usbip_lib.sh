#!/usr/bin/env bash
# usbip_lib.sh: start, attach and detach helpers for the simulated device, sourced by
# classes_e2e.sh, plug_cycle.sh, dap_e2e.sh and dap_record.sh. 启动、挂接和摘除仿真设备
# 的函数。
#
# The host may have other USB devices, some of them with the same VID:PID as a simulated
# one (a real gs_usb adapter, a CMSIS-DAP probe), and it may run a usbipd of its own. The
# helpers therefore never use the stock USB/IP port (3240) or busid (1-1) and never name a
# vhci port or a device by VID:PID alone:
# - sim_start_server starts the simulated server itself, on a free TCP port, with a random
#   busid, and waits until the server's own log says it listens on that port with that
#   busid (a bind failure shows as a dead process and another port is tried);
# - sim_attach refuses to attach unless that server process is alive and the port answers a
#   device list with the random busid, which no other server has;
# - the vhci port is found from the remote URL the server was imported from
#   (usbip://127.0.0.1:<tcp port>/<busid>), and a device is the one whose sysfs path lies
#   under the vhci host controller.
# 主机上可能有别的 USB 设备，有的和仿真设备同 VID:PID（真实的 gs_usb 适配器、CMSIS-DAP
# 探针），也可能运行着自己的 usbipd。所以这里从不使用 USB/IP 的默认端口（3240）和 busid
# （1-1），也从不只凭端口号或 VID:PID 选设备：
# - sim_start_server 自己启动仿真服务端，用空闲的 TCP 端口和随机的 busid，并等到服务端自己
#   的日志说它在该端口以该 busid 监听（绑定失败表现为进程退出，换一个端口重试）；
# - sim_attach 只有在该服务端进程还活着、且该端口对设备列表请求返回随机 busid 时才挂接，
#   别的服务端不会有这个 busid；
# - vhci 端口按导入时的远端 URL 找，设备按 sysfs 路径在 vhci 主机控制器之下判定。
# sudo -n is used for the usbip tool (attach, detach, port) only. No script loads or
# unloads a kernel module: when vhci_hcd is missing they report COULD NOT RUN.
# 只用 sudo -n 运行 usbip（attach、detach、port）。没有脚本加载或卸载内核模块：缺 vhci_hcd
# 时报告 COULD NOT RUN。
#
# Exit status convention of the scripts that source this file: 0 everything passed, 1 a
# check failed, 3 nothing failed but something could not run (a missing tool, privilege,
# kernel module or device node).
# 引用本文件的脚本的退出码约定：0 全部通过，1 有检查失败，3 没有失败但有东西无法运行（缺
# 工具、权限、内核模块或设备节点）。

SIM_TCP_PORT=""
SIM_BUSID=""
SIM_SERVER_PID=""
SIM_PORT=""
SIM_COULD_NOT_RUN=3

# Print "COULD NOT RUN: <reason>" and return 3 unless every named command exists.
sim_need_tools() {
  local tool
  for tool in "$@"; do
    command -v "$tool" >/dev/null 2>&1 && continue
    echo "COULD NOT RUN: $tool is not installed or not in PATH"
    return $SIM_COULD_NOT_RUN
  done
  return 0
}

# Succeeds if the host can run the simulated devices over USB/IP: python3, the usbip tool,
# sudo without a password and the vhci_hcd kernel module. Otherwise prints what is missing
# as COULD NOT RUN and returns 3. Nothing is loaded or changed.
sim_check_host() {
  sim_need_tools python3 usbip sudo || return $SIM_COULD_NOT_RUN
  if ! sudo -n true 2>/dev/null; then
    echo "COULD NOT RUN: sudo -n does not work here (usbip attach and detach need it)"
    return $SIM_COULD_NOT_RUN
  fi
  if [ ! -d /sys/module/vhci_hcd ] && [ ! -d /sys/devices/platform/vhci_hcd.0 ]; then
    echo "COULD NOT RUN: the vhci_hcd kernel module is not loaded (these scripts never load or unload modules)"
    return $SIM_COULD_NOT_RUN
  fi
  if ! sudo -n usbip port >/dev/null 2>&1; then
    echo "COULD NOT RUN: 'sudo -n usbip port' fails, so usbip cannot reach the vhci driver"
    return $SIM_COULD_NOT_RUN
  fi
  return 0
}

# A free TCP port on the loopback address.
sim_free_port() {
  python3 -c 'import socket; s = socket.socket(); s.bind(("127.0.0.1", 0)); print(s.getsockname()[1])'
}

sim_server_alive() {
  [ -n "$SIM_SERVER_PID" ] && kill -0 "$SIM_SERVER_PID" 2>/dev/null
}

sim_stop_server() {
  if [ -n "$SIM_SERVER_PID" ]; then
    kill "$SIM_SERVER_PID" 2>/dev/null
    wait "$SIM_SERVER_PID" 2>/dev/null
  fi
  SIM_SERVER_PID=""
}

# sim_start_server <log> <command> [arguments...]: start the simulated server in the
# background on a free port and with a random busid, its output in <log>. "@PORT@" in an
# argument stands for the TCP port. The server is only reported started when its own log
# line says it listens on 127.0.0.1:<port> with the busid chosen here.
# 在后台以空闲端口和随机 busid 启动仿真服务端，输出写进 <log>。参数里的 "@PORT@" 代表 TCP
# 端口。只有服务端自己的日志写明它在 127.0.0.1:<端口> 以这里选的 busid 监听，才算启动成功。
sim_start_server() {
  local log=$1 attempt arg
  local -a args
  shift
  sim_stop_server
  SIM_BUSID="sim-$$-$RANDOM"
  for attempt in 1 2 3 4 5; do
    SIM_TCP_PORT=$(sim_free_port) || { echo "no free TCP port" >&2; return 1; }
    args=()
    for arg in "$@"; do args+=("${arg//@PORT@/$SIM_TCP_PORT}"); done
    env -u USBIP_BIND USBIP_BUSID="$SIM_BUSID" "${args[@]}" >"$log" 2>&1 &
    SIM_SERVER_PID=$!
    for _ in $(seq 1 50); do
      if grep -qxF "usbip: listening on 127.0.0.1:${SIM_TCP_PORT} busid ${SIM_BUSID}" "$log" 2>/dev/null &&
         sim_server_alive; then
        return 0
      fi
      sim_server_alive || break
      sleep 0.1
    done
    sim_stop_server
  done
  echo "the simulated USB/IP server did not start: $(tr '\n' ' ' < "$log" | cut -c1-200)" >&2
  return 1
}

# Print the vhci port the simulated server is imported on, empty if none.
sim_find_port() {
  [ -n "$SIM_TCP_PORT" ] || return 0
  sudo -n usbip port 2>/dev/null | awk -v url="usbip://127.0.0.1:${SIM_TCP_PORT}/${SIM_BUSID}" '
    /^Port [0-9]+:/ { sub(":", "", $2); port = $2 + 0 }
    index($0, url) { print port; exit }'
}

# Attach the simulated server and remember its vhci port in SIM_PORT. Refuses (returns 1)
# unless the server process started by sim_start_server is alive and the TCP port answers
# a device list with this run's random busid.
sim_attach() {
  if ! sim_server_alive; then
    echo "the simulated USB/IP server is not running; not attaching" >&2
    return 1
  fi
  if ! usbip --tcp-port "$SIM_TCP_PORT" list -r 127.0.0.1 2>/dev/null |
       grep -qE "^ *${SIM_BUSID}:"; then
    echo "127.0.0.1:${SIM_TCP_PORT} does not export busid ${SIM_BUSID}: not the server started here; not attaching" >&2
    return 1
  fi
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

# The sysfs directory of the device imported from this run's server (the local bus-port that
# `usbip port` lists for its URL), empty if there is none yet. Another simulated device with the
# same VID:PID under vhci, from another run, is not this one.
# 从本次运行的服务端导入的设备的 sysfs 目录（`usbip port` 在它的 URL 那一行列出的本地总线端口），
# 还没有时为空。vhci 下别的运行留下的同 VID:PID 的仿真设备不是它。
sim_our_device() {
  local busport
  [ -n "$SIM_TCP_PORT" ] || return 1
  busport=$(sudo -n usbip port 2>/dev/null | awk -v url="usbip://127.0.0.1:${SIM_TCP_PORT}/${SIM_BUSID}" '
    index($0, url) { print $1; exit }')
  [ -n "$busport" ] && [ -d "/sys/bus/usb/devices/$busport" ] &&
    echo "/sys/bus/usb/devices/$busport"
}

# True if the sysfs path $1 lies at or below the directory $2.
sim_is_under() {
  case "$(readlink -f "$1")/" in "$(readlink -f "$2")/"*) return 0;; esac
  return 1
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
